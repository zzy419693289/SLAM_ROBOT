/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include "Converter.h"
#include <octomap/octomap.h>
//#include "localNavigation.h"

#include<mutex>
#include<thread>

namespace ORB_SLAM2
{

PointCloudMapping::PointCloudMapping(double resolution_)
{
    mpOctree = make_shared<octomap::OcTree>(resolution_);
    mbFinishRequested = false;
    mbFinished = false;
    mutexOfPCL = false;   //lock it
    kfnum = 0;

    cout << "Octomap have been initial!" << endl;
}

void PointCloudMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool PointCloudMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void PointCloudMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool PointCloudMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void PointCloudMapping::Reset()
{
	unique_lock<mutex> lock(ResetMutex);
	//clean key frame we have, and clean octomap
	lastKeyframeSize = 0;
	keyframes.clear();
    colorImgs.clear();
    depthImgs.clear();
	mpOctree->clear();
}

void PointCloudMapping::ExportTPose(Eigen::Isometry3d & T_Pose1,Eigen::Isometry3d & T_Pose2)	//output the new pose
{
    unique_lock<mutex> lck(poseMutex);
    T_Pose2 = T_Pose1;
}

void PointCloudMapping::ExportAKeyFrame(long unsigned int& kfnum1,long unsigned int& kfnum2)	//output the new key frame
{
    unique_lock<mutex> lck(keyframeMutex);
    kfnum2 = kfnum1;
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    
    unique_lock<mutex> lck(keyframeMutex);
    kfnum = kf->mnId+1;		//it means total key frames we have
    keyframes.push_back(kf);
    colorImgs.push_back(color.clone());
    depthImgs.push_back(depth.clone());

    mutexOfPCL=true;    //give one signal then call to start octomap
}

pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp(new PointCloud());	//this Ptr is boost::shared_ptr, do not need to free the memory
    // point cloud is null ptr. m and n will decide the poindcloud's accuracy.1280/m * 720/n
    for (int m=0; m<depth.rows; m+=10)	//the spare's point which have depth is standed for the point have reflectivity  
    {
        for (int n=0; n<depth.cols; n+=10)
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>5)
                continue;
            PointT p;
            p.z = d;
            p.x = (n - kf->cx) * p.z / kf->fx;
            p.y = (m - kf->cy) * p.z / kf->fy;           
             
	    if (p.x < -5 || p.x>5)     //pointcloud filter, 5m is farest boundary
                continue;	
            if (p.y < -5 || p.y>5)
                continue;   
            tmp->points.push_back(p);
        }
    }
    
    T_Pose = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud(*tmp, *cloud, T_Pose.inverse().matrix());	//inverse is []^-1, because TG=L, TPg=Pl, Pg=T^-1*Pl
    cloud->is_dense = false;
    
    //cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;

    return cloud;
}


void PointCloudMapping::Run()
{
    while(1)
    {
        if(mutexOfPCL)
		{
			//lock when we need restart.
			unique_lock<mutex> lock(ResetMutex);

            //keyframe is updated ,then count how much keyframes we have
            size_t N=0;
            {
            	unique_lock<mutex> lck(keyframeMutex);	//lock the keyframe number we have, in case error when the keyframe is inserting
            	N = keyframes.size();		//N is the total keyframes number
            }
        
			//if have new key frame, then update octomap
            for( size_t i=lastKeyframeSize; i<N ; i++ )
            {
            	PointCloud::Ptr p = generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i]);	   //generate which have been transformed
            	for (auto ptp:p->points)
	    		{
                    //update the global map    
		    		mpOctree->updateNode(octomap::point3d(ptp.x, ptp.y, ptp.z), true );
            	}    	
            }
            lastKeyframeSize = N;	//lastkeyframesize is used to generate pointcloud from the last time's keyframe to the newest end keyframe
	    	mpOctree->updateInnerOccupancy();
		}
		mutexOfPCL=false;

		if(CheckFinish())
            break;
		usleep(1000);	//1ms a turn to make sure whether have the new frame
	}
    mpOctree->writeBinary("mymap.bt");
    cout<<endl<<"Pointcloud mapping is done!"<<endl;

    SetFinish();
}

}
