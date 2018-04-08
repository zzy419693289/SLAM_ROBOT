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

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
//#include "localNavigation.h"

#include <thread>
#include <mutex>

namespace ORB_SLAM2
{

//class LocalNavigating;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    
    PointCloudMapping(double resolution_);
    
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    
    void Run();

    void RequestFinish();

    bool isFinished();

    //void SetLocalNavigator(LocalNavigating *pLocalNavigator);

    //octomap
    std::shared_ptr<octomap::OcTree> mpOctree;

    //output the last keyframe.if it is not NULL
    void ExportAKeyFrame(long unsigned int& kfnum1,long unsigned int& kfnum2);

    //record the key frame number
    long unsigned int kfnum;

    //keyframe set
    vector<KeyFrame*>       keyframes;
    
protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);   
    
    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mutexOfPCL;	//when true, will stop the thread 
    
    // data to generate point clouds
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    std::mutex              keyframeMutex;
    uint16_t                lastKeyframeSize =0;

    //pose matrix
    Eigen::Isometry3d T_Pose;
};

}

#endif // POINTCLOUDMAPPING_H
