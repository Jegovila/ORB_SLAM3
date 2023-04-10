/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../../include/System.h"
#include "../../../../include/common.h"
#include "common.cc"


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    
    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/tello/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    
    
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string node_name = ros::this_node::getName();
    image_transport::ImageTransport image_transport(nodeHandler);
    nodeHandler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    nodeHandler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    bool enable_pangolin;
    nodeHandler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);


    // World frame orientation
    Eigen::Vector3d rpy_rad;
    std::string angle_names[3] = {"roll", "pitch", "yaw"};
    for (int i = 0; i < 3; i++)
    {
        nodeHandler.param<double>(node_name + "/world_" + angle_names[i], rpy_rad(i), 0);
    }
    
    setup_ros_publishers(nodeHandler, image_transport, rpy_rad);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

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

    //mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
            // ORB-SLAM3 runs in TrackMonocular()
    Sophus::SE3f Tcc0 = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
    Sophus::SE3f Twc = (Tcc0 * Tc0w).inverse();

    ros::Time msg_time = msg->header.stamp;

    publish_ros_camera_pose(Twc, msg_time);
    publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
    publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
    
}
