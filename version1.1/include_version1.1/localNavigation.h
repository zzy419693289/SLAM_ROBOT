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

#ifndef LOCALNAVIGATION_H
#define LOCALNAVIGATION_H

#include "System.h"

#include <octomap/octomap.h>
#include "pointcloudmapping.h"
#include "AStar_TX2.h"
#include "uart_TX2.h"
#include "globalNavigation.h"
#include <KeyFrame.h>

#include <thread>
#include <mutex>
#include <vector>
#include <math.h>

namespace ORB_SLAM2
{

class PointCloudMapping;

class AStarPoint;

class AStar;

class Communicating;

class GlobalNavigating;

class Tracking;

class KeyFrame;

class LocalNavigating
{
public:
    
    LocalNavigating();
       
    void Run();

    void RequestFinish();

    bool isFinished();

    void SetPointCloudMapper(PointCloudMapping *pPointCloudMapper);

    void SetCommunicator(Communicating *pCommunicator);

    void SetGlobalNavigator(GlobalNavigating *pGlobalNavigator);

    void ImportNewPose(Eigen::Isometry3d& t_Pose);
    
protected:  
    
    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    //global map way, from 0-360
    float GlobalWay;

    //robot way, from 0-360
    float LocalWay;

    //Origin Local way, record the map way
    float OriginLocalWay;

    //pointcloud
    PointCloudMapping* mpPointCloudMapper;	

    //communicator
    Communicating* mpCommunicator;

    //Global Navigator
    GlobalNavigating* mpGlobalNavigator;

    //octomap
    std::shared_ptr<octomap::OcTree> mpOctree;

    //local keyframe
    long unsigned int kfnum;

    //pose matrix
    Eigen::Isometry3d T_Pose;
    
    std::vector<localGuide> LocalGuideSeq;

    //plan a front way, when the delta of globalway and localway between at -90~90
    std::vector<localGuide> PlanAFaceWay(octomap::OcTree tOctree,Eigen::Isometry3d t_Pose,float GlobalWay,float LocalWay,float OriginLocalWay);

    //car operate
    char Car_Operate[6];
};

}

#endif // LOCALNAVIGATITION_H
