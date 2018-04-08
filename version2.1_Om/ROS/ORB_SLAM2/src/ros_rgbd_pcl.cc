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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/point_cloud2_iterator.h>
//#include <octomap/octomap.h>
//#include"../../../include/octomap_ros/conversions.h"
#include <tf/transform_datatypes.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}	//call mpSLAM's constructor

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

    //create global compare vector
    //vector<float> GlobalTrajectory;

};

    //create global octree
    //octomap::OcTree tree;
    //shared_ptr<octomap::OcTree> mpOctree = make_shared<octomap::OcTree>(0.1);
    //octomap::OcTree *mpOctree = new octomap::OcTree(0.1);
    //locker
    //mutex uptreeMutex;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/zed/left/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/zed/depth/depth_registered", 1);
    // Recieve PointCloud
    //message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/zed/point_cloud/cloud_registered", 1);
    // add pointcloud msg, sync the three msg
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);	//sync
    // add the third parameter, and call back ImageGrabber, when get sync then callback
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));		//register callback
    //first is callback, second is the object, and then is parameter

    ros::spin();
    cout<<"begin to finish"<<endl;

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("MyKeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

//add cloudpoint2 msg
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
/*    
    //get the newest trajectory
    vector<float> NewTrajectory(mpSLAM->GetCurrentKeyFrameTrajectory());	//copy the return vector to NewTrajectory	

    if(NewTrajectory!=GlobalTrajectory)	//if is have new trajectory, then update the octomap
    {    	
	cout<<NewTrajectory[0]<<" "<<NewTrajectory[1]<<" "<<NewTrajectory[2]<<" "<<NewTrajectory[3]<<" "<<NewTrajectory[4]<<" "<<NewTrajectory[5]<<" "<<NewTrajectory[6]<<endl;

	

	//transfer the pointcloud
	octomap::Pointcloud octomap_cloud;
	octomap::pointCloud2ToOctomap(local_Point2, octomap_cloud);

	//transfer trajectory to Pose6D
	octomath::Vector3 newest_xyz(NewTrajectory[0], NewTrajectory[1], NewTrajectory[2]);
	octomath::Quaternion newest_rot(NewTrajectory[6], NewTrajectory[3], NewTrajectory[4], NewTrajectory[5]);
	octomath::Pose6D newest_Pose6D(newest_xyz, newest_rot);

	//transfer pointcloud by real-time trajectory
	octomap_cloud.transform(newest_Pose6D);

	//lock other thread	
    	//mpSLAM->LockOtherThread();

	//unique_lock<mutex> lcktree(uptreeMutex);

	//stop mapping
	//mpSLAM->mpLocalMapper->RequestStop();
	// Wait until Local Mapping has effectively stopped
        //while(!mpSLAM->mpLocalMapper->isStopped())
        //{
        //    usleep(1000);
        //}
	//mpSLAM->mpTracker->InformOnlyTracking(true);

	//insert pointcloud to the tree
	mpOctree->insertPointCloud(octomap_cloud, newest_xyz);

	//restart mapping
	//mpSLAM->mpTracker->InformOnlyTracking(false);
        //mpSLAM->mpLocalMapper->Release();

	//unlock other thread	
    	//mpSLAM->UnlockOtherThread();
    }
        
    GlobalTrajectory=NewTrajectory;	//when the recall is done, rewrite the newest trajectory
*/
}


