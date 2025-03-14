/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <pangolin/pangolin.h>


namespace ORB_SLAM2
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale, const bool bCorrectLoop):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0), mbCorrectLoop(bCorrectLoop)
{
    mnCovisibilityConsistencyTh = 3;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
// Loop Closing entry point. 
// Retrieve the last keyframe and try to close loops.
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
            // Add a bool here to only detect but not optimize
            // Save detected loop

               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               if(ComputeSim3())
               {   
                if (mbCorrectLoop){
                    // Perform loop fusion and pose graph optimization
                   CorrectLoop();
                }
                else{
                    // Save detecetd loop only
                }
                   
               }
            }
        }       

        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {   
        // Save Loop
        // SaveLoop(mvpEnoughConsistentCandidates);
        return true;
    }

    mpCurrentKF->SetErase();
    return false;
}

bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3
    // nInitialCandidates, number of loop candidates
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    // for each loop candidate, compute the sim3 (R|t,s=scale)
    for(int i=0; i<nInitialCandidates; i++)
    {   
        // mvpEnoughConsistentCandidates -> Detected loops (vector of KeyFrame*)
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        // matching current KF with Loop candidate
        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        // if num of matches > 20, create a Sim3Solver for compute the sim3 transformation
        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {   
            // geometric verification init. with RANSAC
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);
            // set the solver in the vector of solvers
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    // This process may ends early if there are a candidate with enough inliers
    // Therefore, a potentially best candidates might be discarded
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore; // default false

            Sim3Solver* pSolver = vpSim3Solvers[i];
            // Sim3 transformation from current KF to the matched KF
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                // gScm -> Sim3 transformation from current KF to the matched KF
                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                mScm = Converter::toCvMat(gScm);
                mg2oScm = gScm;
                cout << "Sim3 Transformation: " << endl << mScm << endl;
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);
                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }
    
    // if no match, discard all candidates
    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    // Verifying consistency in the neighbourhood of the matched points
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {   
        // add an extra condition to check the loop;
        // What's the difference between nTotalMatches >= 40
        cout << "Loop Detected and Passed the Geometric Verification!" << endl;
        cout << "Current KF ID: " << mpCurrentKF->mnId << endl;
        cout << "Matched KF ID: " << mpMatchedKF->mnId << endl;
        cout << "Total Matches: " << nTotalMatches << endl;
        cout << "Loop Transform from Matched KF to Current KF " << endl << mScm << endl;
        cout << "Now Conduct Trajectory Similarity Verification!" << endl;

        if (ComputeTrajSim(mpMap, mpCurrentKF->mnId, mpMatchedKF->mnId, mg2oScm)){
            cout << "Trajectory Similarity Verification Passed!" << endl;
            cout << "Loop Detected!" << endl;
            cout << "Current KF ID: " << mpCurrentKF->mnId << endl;
            cout << "Matched KF ID: " << mpMatchedKF->mnId << endl;
            cout << "Total Matches: " << nTotalMatches << endl;
            cout << "Loop Transform from Matched KF to Current KF " << endl << mScm << endl;

            mpMatchedKF->AddLoopEdge(mpCurrentKF, nTotalMatches, mScm);
            mpCurrentKF->AddLoopEdge(mpMatchedKF, nTotalMatches, Converter::computeInverseSimTransform(mScm));

            for(int i=0; i<nInitialCandidates; i++)
                if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                    mvpEnoughConsistentCandidates[i]->SetErase();
            return true;
        }
        else{
            for(int i=0; i<nInitialCandidates; i++)
                mvpEnoughConsistentCandidates[i]->SetErase();
            mpCurrentKF->SetErase();
            return false;
        }
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}

bool LoopClosing::ComputeTrajSim(Map* pMap, const int fromId, const int toId, const g2o::Sim3 &gScm)
{   
    cout << "Start Trajectory Similarity Verification!" << endl;
    // For each detected loop candidate we try to compute a trajectory similarity between the original trajectory and the using the loop candidate optimized trajectory 
    
    // Setup Sim3 optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    // create a pose graph with vertices of all keyframes
    // retrieve all KeyFrames
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const unsigned int nMaxKFid = pMap->GetMaxKFid(); // max keyframe id
    // create a vector of Sim3 poses
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1); // poses of KFs
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1); // corrected poses of KFs
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1); // vertices of KFs
    // vector<cv::Mat,Eigen::aligned_allocator<cv::Mat> > vSwc(nMaxKFid+1); // poses of KFs

    // Set KeyFrame vertices
    cout << "Set KeyFrame Vertices!" << endl;
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();
        const int nIDi = pKF->mnId; // KF's id
        // vSwc[nIDi] = pKF->GetPoseInverse(); // camera to world transformation
        cv::Mat Tcw = pKF->GetPose();
        Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(Tcw.rowRange(0,3).colRange(0,3));
        Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(Tcw.rowRange(0,3).col(3));
        g2o::Sim3 Scw(Rcw,tcw,1.0);
        vScw[nIDi] = Scw;
        VSim3->setEstimate(Scw);
        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = mbFixScale;
        optimizer.addVertex(VSim3);
        vpVertices[nIDi]=VSim3;
        if (i == 0){
            VSim3->setFixed(true);
        }
            
        // LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        // if(it!=CorrectedSim3.end())
        // {   
        //     vScw[nIDi] = it->second; // camera to world transformation
        //     VSim3->setEstimate(it->second);
        // }
        // else
        // {
        // Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation().t());
        // Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
        // g2o::Sim3 Siw(Rcw,tcw,1.0);
        // vScw[nIDi] = Siw;
        // VSim3->setEstimate(Siw);
        // }

        // if(pKF==pLoopKF)
        //     // Set the Detected Loop Closure Frame as fixed 
        //     VSim3->setFixed(true);

        // VSim3->setId(nIDi);
        // VSim3->setMarginalized(false);
        // VSim3->_fix_scale = mbFixScale;

        // optimizer.addVertex(VSim3);

        // vpVertices[nIDi]=VSim3;
    }

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
    // set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    // // insert loop candidate edge
    // mScm = 
    // g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);

    g2o::EdgeSim3* e = new g2o::EdgeSim3();
    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(toId)));
    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(fromId)));
    e->setMeasurement(gScm);
    e->information() = matLambda;
    optimizer.addEdge(e);

    cout << "Loop Transform: " << endl << Converter::toCvMat(gScm) << endl;
    // cout << "Original Transform" << endl << vScw[fromId] << endl;

    cout << "Set Edges!" << endl;
    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++){
        KeyFrame* pKF = vpKFs[i];
        const int nIDi = pKF->mnId;
        g2o::Sim3 Swi;

        // LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        // if(iti!=NonCorrectedSim3.end())
        //     Swi = (iti->second).inverse();
        // else
        Swi = vScw[nIDi].inverse();
        KeyFrame* pParentKF = pKF->GetParent();
        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;
            g2o::Sim3 Sjw;
            Sjw = vScw[nIDj];
            g2o::Sim3 Sji = Sjw * Swi;
            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);
            e->information() = matLambda;
            optimizer.addEdge(e);
        }
    }
   
    // debugging
    cout << "Number of KeyFrames: " << vpKFs.size() << endl;
    cout << "Number of Edges: " << optimizer.edges().size() << endl;
    cout << "Start Optimization!" << endl;

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(20);
    
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
    }

    // double ate;
    vector<Eigen::Matrix4d> vGt;
    vector<Eigen::Matrix4d> vEs;
    vector<Eigen::Vector3d> points1, points2, points2_aligned;
    double ate;
    

    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        const int nIDi = pKFi->mnId;

        cv::Mat Twc =  Converter::toCvMat(vScw[nIDi].inverse());
        cv::Mat Twc_corrected =  Converter::toCvMat(vCorrectedSwc[nIDi]);

        vGt.push_back(Converter::toMatrix4d(Twc));
        vEs.push_back(Converter::toMatrix4d(Twc_corrected));

        Eigen::Vector3d pt1;
        Eigen::Vector3d pt2;
        pt1 << Twc.at<float>(0,3), Twc.at<float>(1,3), Twc.at<float>(2,3);
        pt2 << Twc_corrected.at<float>(0,3), Twc_corrected.at<float>(1,3), Twc_corrected.at<float>(2,3);

        points1.push_back(pt1);
        points2.push_back(pt2);
    }

    std::string filename_odom = "/ORB_SLAM2/ROS_output/test/odom" + std::to_string(mfiletimer) + ".txt";
    std::string filename_optimized = "/ORB_SLAM2/ROS_output/test/optimized" + std::to_string(mfiletimer) + ".txt";
    mfiletimer++;
    // output vGt and vEs to separate files
    WriteMatricesToFile(vGt, filename_odom);
    WriteMatricesToFile(vEs, filename_optimized);

    // Perform Umeyama alignment
    Eigen::MatrixXd x = vectorToMatrix(points1);
    Eigen::MatrixXd y = vectorToMatrix(points2);

    auto result = umeyamaAlignment(x, y, true);
    Eigen::Matrix3d R = result.rotation * result.scale;
    Eigen::Quaterniond q(R);
    Eigen::Vector3d t = result.translation;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    Eigen::Matrix4d T_inv = T.inverse();
    Eigen::Matrix3d R_inv = T_inv.block<3,3>(0,0);
    Eigen::Vector3d t_inv = T_inv.block<3,1>(0,3);
    for (const auto& p : points2){
        Eigen::Vector3d p2_align = R_inv * p + t_inv;
        points2_aligned.push_back(p2_align);
    }

    ate = computeRMSE(points1, points2_aligned);
    cout << "FromId: " << fromId << " ToId: " << toId << endl;
    cout << "ATE: " << ate << endl;

    // ate = AlignTrajectory(vGt, vEs);
    // DrawTrajectory(vGt, vEs);
    // cout << "ATE: " << ate << endl;

    // align the corrected and original poses
    // compute the similarity transformation
    // compute the error
    // if the error is less than a threshold, return true
    // else return false
    if (ate < 0.3){
        return true;
    }
    else{
        return false;
    }
    // return false;   
}
// // Add save loop
// void LoopClosing::SaveLoop(){
    
//     cout << "Save Loop" << endl;
// }

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    // Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > >
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    // CorrectedSim3 -> Corrected Sim3 pose
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    // Twc -> Current keyframe pose
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    mpMap->InformNewBigChange();

    // Add loop edge
    // mpMatchedKF->AddLoopEdge(mpCurrentKF, 0);
    // mpCurrentKF->AddLoopEdge(mpMatchedKF, 0);

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId;   
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);
            }
        }
    }
}


void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    int idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }            

            mpMap->InformNewBigChange();

            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

LoopClosing::UmeyamaResult LoopClosing::umeyamaAlignment(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y, bool with_scale) {
    checkMatrixDimensions(x, y);

    // Means
    Eigen::VectorXd mean_x = x.rowwise().mean();
    Eigen::VectorXd mean_y = y.rowwise().mean();

    // Variance
    double sigma_x = (x.colwise() - mean_x).squaredNorm() / x.cols();

    // Covariance matrix
    Eigen::MatrixXd cov_xy = Eigen::MatrixXd::Zero(x.rows(), x.rows());
    for (int i = 0; i < x.cols(); ++i) {
        cov_xy += (y.col(i) - mean_y) * (x.col(i) - mean_x).transpose();
    }
    cov_xy /= x.cols();

    // Singular Value Decomposition (SVD)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov_xy, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd u = svd.matrixU();
    Eigen::MatrixXd v = svd.matrixV();

    // S matrix
    Eigen::MatrixXd s = Eigen::MatrixXd::Identity(x.rows(), x.rows());
    if (u.determinant() * v.determinant() < 0.0) {
        s(x.rows() - 1, x.rows() - 1) = -1.0;
    }

    // Rotation matrix
    Eigen::MatrixXd rotation = u * s * v.transpose();

    // Scale and translation
    double scale = with_scale ? (svd.singularValues().dot(s.diagonal()) / sigma_x) : 1.0;
    Eigen::VectorXd translation = mean_y - scale * rotation * mean_x;

    return {rotation, translation, scale};
}

Eigen::MatrixXd LoopClosing::vectorToMatrix(const std::vector<Eigen::Vector3d>& points) {
    if (points.empty()) {
        throw std::invalid_argument("Input vector is empty.");
    }

    Eigen::MatrixXd matrix(3, points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        matrix.col(i) = points[i];
    }

    return matrix;
}

void LoopClosing::checkMatrixDimensions(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y) {
    if (x.rows() != y.rows() || x.cols() != y.cols()) {
        throw std::invalid_argument("Data matrices must have the same shape.");
    }
}

// double LoopClosing::AlignTrajectory(vector<Eigen::Matrix4d> gt, vector<Eigen::Matrix4d> es){
//     // Align the trajectory using dynamic time warping
//     // gt -> ground truth trajectory
//     // es -> estimated trajectory
//     // return the error
//     // the error is the sum of the distance between the two trajectories
//     // the distance is
//     // d = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
//     // the error is the sum of the distance between the two trajectories
//     cout << "Aligning traectories ..." << std::endl;
    
//     vector<Eigen::Matrix4d> vGt;
//     vector<Eigen::Matrix4d> vEs;

//     if (gt.size() != es.size())
//     {
//         std::cout << "size of groundtruth poses: " << gt.size() << std::endl; 
//         std::cout << "size of estimated poses: " << es.size() << std::endl; 
//         std::cerr << "for no association, size of estimated and ground truth trajectories must be equal." << std::endl;
//         return -1.0;
//     }
//     else
//     {
//         for(std::vector<Eigen::Matrix4d>::iterator it = es.begin(); it != es.end(); ++it)
//                 vEs.push_back(*it);

//         for(std::vector<Eigen::Matrix4d>::iterator it = gt.begin(); it != gt.end(); ++it)
//                 vGt.push_back(*it);

//         return CalculateATE(vGt, vEs);
//     }


// }

// double LoopClosing::CalculateATE(vector<Eigen::Matrix4d> gt, vector<Eigen::Matrix4d> es)
// {

// // convert pose vectors to Eigen matrices
//     double ate;
//     int N = gt.size();
//     Eigen::MatrixXd gtMat(3,N);
//     for (int i = 0; i < N; i++)
//     {
//         gtMat(0,i) = gt.at(i)(0,3);
//         gtMat(1,i) = gt.at(i)(1,3);
//         gtMat(2,i) = gt.at(i)(2,3);
//     }


//     int M = gt.size();
//     Eigen::MatrixXd esMat(3,N);
//     for (int i = 0; i < M; i++)
//     {
//         esMat(0,i) = es.at(i)(0,3);
//         esMat(1,i) = es.at(i)(1,3);
//         esMat(2,i) = es.at(i)(2,3);
//     }

//     // calculate the mean pose to zero-shift the poses
//     Eigen::Vector3d gtMean = gtMat.rowwise().mean();
//     Eigen::Vector3d esMean = esMat.rowwise().mean();

//     Eigen::MatrixXd gtZeroMat(3,N);
//     Eigen::MatrixXd esZeroMat(3,N);

//     gtZeroMat = gtMat.colwise() - gtMean;
//     esZeroMat = esMat.colwise() - esMean;

//     // rotation, translation, scale, and absoulte trajector error (ate) 
//     Eigen::Matrix3d rotation    = ATERotation(gtZeroMat, esZeroMat);
//     double   scale       = ATEScale(gtZeroMat, esZeroMat, rotation);
//     Eigen::Vector3d translation = ATETranslation(gtMat, esMat, scale, rotation, ate);

//     cout << "Rotation is:    "<<  endl << rotation << endl<<endl;
//     cout << "Scale is:       "<<  endl << scale << endl<<endl; 
//     cout << "Translation is: "<<  endl << translation << endl<<endl; 
//     cout << "Error is:       "<<  endl << ate << endl<<endl; 

//     // Eigen::Matrix4d Mat;
//     // Mat = Eigen::MatrixXd::Identity(4,4);

//     // Mat.block<3,3>(0,0) = scale*rotation;
//     // Mat.block<3,1>(0,3) = translation;
//     // Mat.block<1,4>(3,0) << 0,0,0,1;

//     return ate;

// }

// Eigen::Vector3d LoopClosing::ATETranslation(Eigen::MatrixXd model, Eigen::MatrixXd data, double scale, Eigen::MatrixXd rotation, double& ate)
// {
//     int N = model.cols();
//     Eigen::Vector3d translation = data.rowwise().mean() - (scale*rotation)*(model.rowwise().mean());
//     Eigen::MatrixXd rotatedModel(3,N);
//     rotatedModel = (scale*rotation)*model;

//     // error matrix E = [E1, E2, ...]
//     Eigen::MatrixXd errorMat(3, N);
//     errorMat = (rotatedModel.colwise() + translation) - data;
//     // errorMat = model.colwise() - data;

//     // Absoute Trajectory Error (ATE) = |||E1|| + ||E2||+ ... = \sum(||Ei||)
//     for (int i = 0; i < N; i++)
//         ate = ate + errorMat.col(i).norm();

//     ate = ate/N;
//     return translation;
// }

// double LoopClosing::ATEScale(Eigen::MatrixXd model, Eigen::MatrixXd data, Eigen::MatrixXd rotation)
// {
//     int cols = model.cols();
//     Eigen::MatrixXd rotatedModel;
//     rotatedModel = rotation * model;

//     double dots = 0.0;
//     double norms = 0.0;
//     double normi = 0.0;

//     //Model = [M0, M1, ...], Rotated Data = [R0, R1, ...]
//     // W = M0.D0' + M1.D1' + ...  = \sum{Mi.Di}
//     for (int i = 0; i < cols; i++)
//     {
//        Eigen::Vector3d v1 = data.col(i);
//        Eigen::Vector3d v2 = rotatedModel.col(i);
//        Eigen::Vector3d v3 = model.col(i);

//        dots = dots + v1.transpose()*v2;
//        normi = v3.norm();
//        norms = norms + normi*normi;
//     }

//     // scale
// //    return  1/(dots/norms);
//     return  (dots/norms);
//     //return 1;
// }

// Eigen::MatrixXd LoopClosing::ATERotation(Eigen::MatrixXd model, Eigen::MatrixXd data)
// {
//     Eigen::MatrixXd w;
//     w = Eigen::MatrixXd::Identity(3,3);

//     int cols = model.cols();

//     //Model = [M0, M1, ...], Data = [D0, D1, ...]
//     // W = M0*D0' + M1*D1' + ...  = \sum{Mi*Di}
//     for (int i = 0; i < cols; i++)
//         w = w + model.col(i) * data.col(i).transpose();

//     Eigen::JacobiSVD<Eigen::MatrixXd> svd(w.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);

//     Eigen::Matrix3d U = svd.matrixU();
//     Eigen::Matrix3d V = svd.matrixV();
//     float detV = V.determinant();
//     float detU = U.determinant();

//     Eigen::MatrixXd S;
//     S = Eigen::MatrixXd::Identity(3,3);

//     if(detU * detV < 0)
//         S(2,2) = -1;

//     Eigen::MatrixXd rot;
//     rot = U * S * V.transpose();

//     return rot;
// }

// void LoopClosing::DrawTrajectory(vector<Eigen::Matrix4d> poses1,
//                     vector<Eigen::Matrix4d> poses2) {
//     if (poses1.empty()&&poses2.empty()) {
//         cerr << "Trajectory is empty!" << endl;
//         return;
//     }

//     pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
//     glEnable(GL_DEPTH_TEST);
//     glEnable(GL_BLEND);
//     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//     pangolin::OpenGlRenderState s_cam(
//             pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
//             pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
//     );

//     pangolin::View &d_cam = pangolin::CreateDisplay()
//             .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
//             .SetHandler(new pangolin::Handler3D(s_cam));


//     while (pangolin::ShouldQuit() == false) {
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//         d_cam.Activate(s_cam);
//         glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

//         glLineWidth(2);
//         for (size_t i = 0; i < poses1.size()-1; i++) {    //因为没有构成回环，这里减2更好，不然会连成一条直线，若为回环就-1,也可以用pop_back()
//             glColor3f(1.0f, 0.0f, 0.0f);
//             glBegin(GL_LINES);
//             auto p1 = poses1[i], p2 = poses1[i + 1];
//             // , p2 = poses1[i + 1];
//             glVertex3d(p1(4,0), p1(4,1), p1(4,2));
//             glVertex3d(p2(4,0), p2(4,1), p2(4,2));
//             glEnd();
//         }
//         for (size_t i = 0; i < poses2.size(); i++) {
//             glColor3f(0.0f, 0.0f, 1.0f);
//             glBegin(GL_LINES);
//             auto p1 = poses2[i], p2 = poses2[i + 1];
//             // , p2 = poses2[i + 1];
//             glVertex3d(p1(4,0), p1(4,1), p1(4,2));
//             glVertex3d(p2(4,0), p2(4,1), p2(4,2));
//             glEnd();
//         }
//         pangolin::FinishFrame();
//         usleep(5000);   // sleep 5 ms
//     }

// }

void LoopClosing::WriteMatricesToFile(const std::vector<Eigen::Matrix4d>& vEs, const std::string& filename) {
    // Open the file for writing
    std::ofstream outFile(filename);
    
    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }
    
    // Iterate through the vector of matrices
    for (size_t i = 0; i < vEs.size(); ++i) {
        const Eigen::Matrix4d& mat = vEs[i]; // Get the current matrix in world frame
        // convert the 
        cv::Mat cvMat = Converter::toCvMat(mat);
        cv::Mat R = cvMat.rowRange(0,3).colRange(0,3);
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = cvMat.rowRange(0,3).col(3);
        // Write the matrix to the file
        outFile << i << " " << setprecision(7) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        // outFile << "Matrix " << i + 1 << ":" << std::endl; // Optional: Label the matrix
        // outFile << i << " "; // Label the matrix
        // for (int row = 0; row < mat.rows() - 1; ++row) {
        //     for (int col = 0; col < mat.cols(); ++col) {
        //         if (row == 3 && col == 4){
        //         outFile << mat(row, col) << std::endl; // Write element with a newline
        //         }
        //         else{
        //             outFile << mat(row, col) << " "; // Write element with a space
        //         }
        //     }
        //     // outFile << std::endl; // End of row
        // }
        // outFile << std::endl; // Separate matrices with a blank line
    }
    
    // Close the file
    outFile.close();
    std::cout << "Matrices successfully written to " << filename << std::endl;
}

double LoopClosing::computeRMSE(const std::vector<Eigen::Vector3d>& poses1, const std::vector<Eigen::Vector3d>& poses2){
    if (poses1.size() != poses2.size()) {
        throw std::invalid_argument("Pose vectors must have the same size.");
    }

    double error_sum = 0.0;
    for (size_t i = 0; i < poses1.size(); ++i) {
        Eigen::Vector3d diff = poses1[i] - poses2[i];
        error_sum += diff.squaredNorm();
    }
    return std::sqrt(error_sum / poses1.size());
}



} //namespace ORB_SLAM
