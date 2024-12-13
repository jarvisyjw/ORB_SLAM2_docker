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

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::NodeHandle private_nh("~");

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
    ros::Subscriber sub = nodeHandler.subscribe("/cam0/image_raw", 1, &ImageGrabber::GrabImage,&igb);

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
}


