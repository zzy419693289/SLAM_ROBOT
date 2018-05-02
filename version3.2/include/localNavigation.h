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
#include "Tracking.h"

#include <thread>
#include <mutex>
#include <vector>
#include <math.h>

namespace ORB_SLAM2
{
struct Send_Message
{
    char Data1;	//gait
    char Data2; //motion
    char Data3; //neg or pos
    char Data4; //speed
};

struct Angle_data;

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

    bool needRestarted();
	
    void UsedRestart();

    void SetPointCloudMapper(PointCloudMapping *pPointCloudMapper);

    void SetCommunicator(Communicating *pCommunicator);

    void SetGlobalNavigator(GlobalNavigating *pGlobalNavigator);

    void SetTracker(Tracking *pTracker);

    void ImportNewPose(Eigen::Isometry3d& t_Pose);

    //operate sequence
    std::vector<Send_Message> SendMessageSeq;
    bool Send_Restart; 	//false no need, true need

    void ExportSendMesSeq(vector<Send_Message>& SendSeqLocal, vector<Send_Message>& SendSeqUart);

    void CheckSendType(bool& SendTypeLocal, bool& SendTypeUart);

    void Reset();
    
protected:  
    
    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    void SetRestart();
    bool mbRestarted;
    std::mutex mMutexRestart;

    std::mutex ShareSeqMutex;
    std::mutex ResetMutex;

    //angle structrue
    Angle_data* our_Angle_Ptr;

    //global map way, from 0-360
    float GlobalWay;
    bool GlobalWayEffective;

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

    //tracker
    Tracking* mpTracker;

    //octomap
    std::shared_ptr<octomap::OcTree> mpOctree;

    //local keyframe
    long unsigned int kfnum;

    //pose matrix
    Eigen::Isometry3d T_Pose;
    
    std::vector<localGuide> LocalGuideSeq;

    //get tracking's mstate, if lost return false
    bool GetState(int TrackState);

    //plan a front way, when the delta of globalway and localway between at -90~90
    std::vector<localGuide> PlanAFaceWay(octomap::OcTree tOctree,Eigen::Isometry3d t_Pose,float GlobalWay,float LocalWay,float OriginLocalWay);

    //calculate the real control sequence, A Xi
    vector<Send_Message> GoSendMesXi(vector<localGuide> LocalGuideSeq);

    //move parallely instruction, A Xi
    vector<Send_Message> ParaSendMesXi();
	
    //turn around instruction, A Xi
    vector<Send_Message> TurnSendMesXi(float globalWay, float localWay);

    //stop A Xi
    vector<Send_Message> StopSendMesXi();

    //calculate the real control sequence, Omni. If degree<+-90
    vector<Send_Message> GoSendMesOm(vector<localGuide> LocalGuideSeq);

    //move backward instruction, Omni. If degree<+-90 and no way
    vector<Send_Message> BackSendMesOm();
	
    //turn around instruction, Omni
    vector<Send_Message> TurnSendMesOm(float globalWay, float localWay);

    //stop Omni
    vector<Send_Message> StopSendMesOm();

};

}

#endif // LOCALNAVIGATITION_H
