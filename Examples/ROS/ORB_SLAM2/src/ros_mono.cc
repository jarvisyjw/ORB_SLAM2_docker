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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
// #include <image_transport/image_transport.h>



using namespace std;

ros::Publisher pose_pub;
ros::Publisher path_pub;
ros::Publisher image_pub;
ros::Publisher tracked_points_pub;
// image_transport::ImageTransport image_transport;
// ros::Publisher image_pub;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void PublishPoseStamped(const cv::Mat &Tcw);

    void PublishRenderedImage(cv::Mat &image);

    ORB_SLAM2::System* mpSLAM;


protected:

    std::vector<float> mvq;
    std::vector<float> mvt;
    cv::Mat Rcb;

    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose_msg;
    sensor_msgs::ImagePtr rendered_image_msg;
};


std::vector<float> toQuatVec(const cv::Mat &M)
{   
    // Check that the input is a 3x3 matrix
    if (M.rows != 3 || M.cols != 3) {
        throw std::runtime_error("Input cv::Mat must be a 3x3 rotation matrix.");
    }

    // Check that the type is CV_32F (float)
    if (M.type() != CV_32F) {
        throw std::runtime_error("Input cv::Mat must have type CV_32F (float).");
    }

    // Convert cv::Mat to Eigen::Matrix3d
    Eigen::Matrix3d eigRot;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            eigRot(i, j) = static_cast<double>(M.at<float>(i, j));
        }
    }

    // Create Eigen quaternion from the rotation matrix
    Eigen::Quaterniond egtQuat(eigRot);

    // Convert the quaternion to a std::vector<float>
    std::vector<float> quatVec(4);
    quatVec[0] = static_cast<float>(egtQuat.x());
    quatVec[1] = static_cast<float>(egtQuat.y());
    quatVec[2] = static_cast<float>(egtQuat.z());
    quatVec[3] = static_cast<float>(egtQuat.w());

    return quatVec;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::NodeHandle private_nh("~");
    // ros::NodeHandle nh;

    // pose publisher
    std::string voc_file, settings_file, kf_traj_file, map_file, loop_file;

    private_nh.param<std::string>("voc_file", voc_file, "./Vocabulary/ORBvoc.txt");
    private_nh.param<std::string>("settings_file", settings_file, "./Examples/ROS/ORB_SLAM2/realsense.yaml");
    private_nh.param<std::string>("kf_traj_file", kf_traj_file, "./KeyFrameTrajectory.txt");
    private_nh.param<std::string>("map_file", map_file, "./Map.bin");
    private_nh.param<std::string>("loop_file", loop_file, "./loop.txt");

    ORB_SLAM2::System SLAM(voc_file, settings_file, ORB_SLAM2::System::MONOCULAR, true);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    // image_transport::ImageTransport it(nh);
    ros::Subscriber sub = nodeHandler.subscribe("/cam0/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/pose", 10);
    path_pub = nodeHandler.advertise<nav_msgs::Path>("/trajectory", 10); // Add path publisher
    image_pub = nodeHandler.advertise<sensor_msgs::Image>("/rendered_image", 1);

    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(kf_traj_file);

    // Save loop closure edges
    SLAM.SaveLoopClosureEdges(loop_file);

    // Save map
    SLAM.SaveMap(map_file);

    // ros::Duration(5.0).sleep();
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    cv::Mat Tcw = mpSLAM->GetCurrentPosition();

    PublishPoseStamped(Tcw);

    cv::Mat rendered_image = mpSLAM->DrawCurrentFrame();
    
    PublishRenderedImage(rendered_image);

}

void ImageGrabber::PublishPoseStamped(const cv::Mat &Tcw){
    
    if (!Tcw.empty()) {
    // ROS_WARN("Publishing PoseStamped");
    cv::Mat Twc = Tcw.inv();

    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
    cv::Mat twc = Twc.rowRange(0,3).col(3);
    
    mvt.resize(3);
    mvq.resize(4);

    mvt[0] = twc.at<float>(0);
    mvt[1] = twc.at<float>(1);
    mvt[2] = twc.at<float>(2);

    mvq = toQuatVec(Rwc);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "camera_init";
    
    pose_msg.pose.position.x = mvt[0];
    pose_msg.pose.position.y = mvt[1];
    pose_msg.pose.position.z = mvt[2];
    
    pose_msg.pose.orientation.x = mvq[0];
    pose_msg.pose.orientation.y = mvq[1];
    pose_msg.pose.orientation.z = mvq[2];
    pose_msg.pose.orientation.w = mvq[3];

    pose_pub.publish(pose_msg);

    // ROS_INFO("Published PoseStamped: Position(%.2f, %.2f, %.2f) Orientation(%.2f, %.2f, %.2f, %.2f)",
    //         pose_msg.pose.position.x,
    //         pose_msg.pose.position.y,
    //         pose_msg.pose.position.z,
    //         pose_msg.pose.orientation.x,
    //         pose_msg.pose.orientation.y,
    //         pose_msg.pose.orientation.z,
    //         pose_msg.pose.orientation.w);

    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "camera_init";
    path_msg.poses.push_back(pose_msg); // Append the current pose to the path

    // Publish the Path message
    path_pub.publish(path_msg);

    }
}

void ImageGrabber::PublishRenderedImage(cv::Mat &image)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "camera_init";
  rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  image_pub.publish(rendered_image_msg);
}

