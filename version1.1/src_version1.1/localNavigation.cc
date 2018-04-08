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

#include "localNavigation.h"
#include <opencv2/highgui/highgui.hpp>
#include <octomap/octomap.h>
#include "pointcloudmapping.h"
#include <math.h>
#include "uart_TX2.h"
#include "Converter.h"
#include <KeyFrame.h>

#include<mutex>
#include<thread>

namespace ORB_SLAM2
{
#define MAXMAPM 61
#define MAXMAPN 61
#define Nsub	1.5
#define ExpandCoef 1
#define Inner45 1.5
#define InnerFix 20
#define Outter45 0.75
#define OutterCoef 2/3
#define FrontNodetect 0

LocalNavigating::LocalNavigating()
{  
    mbFinishRequested = false;
    mbFinished = false;
    GlobalWay = 0;
    LocalWay = 0;
    OriginLocalWay = 0;
    T_Pose.translation()[0] = 0;
    T_Pose.translation()[1] = 0;
    T_Pose.translation()[2] = 0;
    Car_Operate[0] = 0xAA;
    Car_Operate[1] = 0x30;
    Car_Operate[2] = 0x30;
    Car_Operate[3] = 0x30;
    Car_Operate[4] = 0x30;
    Car_Operate[5] = 0xBB;
    kfnum = 0;
    cout << "Local Navigator have been initial!" << endl;
}

void LocalNavigating::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalNavigating::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalNavigating::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LocalNavigating::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LocalNavigating::SetPointCloudMapper(PointCloudMapping *pPointCloudMapper)
{
    mpPointCloudMapper = pPointCloudMapper;
    mpOctree = pPointCloudMapper->mpOctree;		//import the octree	
    cout << "Octree have been set!" << endl;
}

void LocalNavigating::SetCommunicator(Communicating *pCommunicator)
{
    mpCommunicator = pCommunicator;		
    cout << "Local Communicator have been set!" << endl;
}

void LocalNavigating::SetGlobalNavigator(GlobalNavigating *pGlobalNavigator)
{
    mpGlobalNavigator = pGlobalNavigator;		
    cout << "Global Navigator have been set!" << endl;
}

vector<localGuide> LocalNavigating::PlanAFaceWay(octomap::OcTree tOctree,Eigen::Isometry3d t_Pose,float GlobalWay,float LocalWay,float OriginLocalWay)	//add front go
{
    vector<localGuide> tLocalGuideSeq;
    tLocalGuideSeq.clear();
    float treeHitLog = tOctree.getProbHitLog();
    double costhis,sinthis;
    costhis = cos((LocalWay-OriginLocalWay)*3.1415926535897932/180);
    sinthis = sin((LocalWay-OriginLocalWay)*3.1415926535897932/180);
    bool NavigatorMat[MAXMAPM][MAXMAPN] = {0};	//used to create a 0-1 map, and initial false

    for(int m=ExpandCoef;m<MAXMAPM-ExpandCoef;m++)	//n == x, m == z
	for(int n=ExpandCoef;n<MAXMAPN-ExpandCoef;n++)  //ignore the outer ring 
	{
	    //detect. If this point have been occupied by expand, continue the circle directly
	    if(NavigatorMat[m][n] == true)
     		continue;

	    //calculate the coordinate we need to go through
	    float x_one = n*0.05 + t_Pose.translation()[0] - Nsub;	//3m/2=1.5
	    float z_one = m*0.05 + t_Pose.translation()[2] + FrontNodetect;	//zed can not recognize less than 0.5m
	    float x_two = x_one * costhis - z_one*sinthis;
	    float z_two = x_one * sinthis + z_one*costhis;

            bool IfObs = false;
	    
	    //go through this x_two and z_two whether 3D map have obstacle
	    for(int l=0;l<5;l++)	//from y to y+0.25m have no obstacle
	    {
		octomap::OcTreeNode* pOctreeNode = tOctree.search(x_two, t_Pose.translation()[1]+l*0.05, z_two);
		if(pOctreeNode == NULL)
		{
		    continue;		//if have no this point, continue next point
		}
		if(pOctreeNode->getLogOdds() > treeHitLog)	//if the Node is occupied, then break, and site it.
		{					
		    IfObs = true;
		    break;				
		}
	    }
	    //save the output in navigator map
	    NavigatorMat[m][n] = IfObs;
	    if(IfObs)	//expand! When the expansive range is bigger, we will save more time. There the range is 5cm on each side.
	    {
		for(int exm=- ExpandCoef;exm<=ExpandCoef;exm++)
		    for(int exn=- ExpandCoef;exn<=ExpandCoef;exn++)
		    {
			NavigatorMat[m+exm][n+exn]=IfObs;
		    }
	    }
	}

    //print the 2D-map
    for ( auto p = end(NavigatorMat)-1 ; p >= begin(NavigatorMat); --p) 
    {    
    	for ( auto q = begin(*p); q != end(*p); ++q ) 
	{  
	    cout<<*q<<' ';  
        }  
        cout<<endl;  
    }  
 
    //use the navigator map to navigator
    AStar *Amaze = new AStar(NavigatorMat);	
    AStarPoint *Astart = new AStarPoint(0, (MAXMAPN-1)/2);
    int endm=0;
    int endn=0;
    if((GlobalWay-LocalWay>-45) && (GlobalWay-LocalWay<45))
    {
	endm=MAXMAPM-1-InnerFix;
        endn=(GlobalWay-LocalWay+45)/Inner45;
    }
    if(GlobalWay-LocalWay<=-45)
    {
	endm=(GlobalWay-LocalWay+90)*OutterCoef/Outter45;
        endn=0;
    }
    if(GlobalWay-LocalWay>=45)
    {
	endm=(LocalWay-GlobalWay+90)*OutterCoef/Outter45;
        endn=MAXMAPN-1;
    }
    AStarPoint *Aend = new AStarPoint(endm, endn);   
    std::vector<localGuide> localGuideList = Amaze->FindPath(Astart, Aend);  
    
    //whether can arrive the end point, have different operate.
    if (!localGuideList.empty())
    {
	std::vector<localGuide>::reverse_iterator _iter;
	for (_iter = localGuideList.rbegin(); _iter != localGuideList.rend(); ++_iter)	
	{
	    localGuide point = *_iter;
	    tLocalGuideSeq.push_back(point);
	}
    }
    
    delete Aend;
    delete Astart;
    delete Amaze;	    

    return tLocalGuideSeq;
}

void LocalNavigating::Run()
{
    //wait for initial
    sleep(1);
    Eigen::Isometry3d tT_Pose;
/*
    while(!mpCommunicator->fisrtRobotWay)	//wait until we have first angle
    {
	if(CheckFinish())
            break;
	usleep(1000);
    }
*/
    mpCommunicator->Angle_Export(mpCommunicator->RobotNowWay, OriginLocalWay);

    while(kfnum == 0)		//wait until first key frame
    	mpPointCloudMapper->ExportAKeyFrame(mpPointCloudMapper->kfnum,kfnum);	//import keyframe

    while(1)
    {     
	    
	mpGlobalNavigator->GlobalWay_Export(mpGlobalNavigator->GlobalWay, GlobalWay);	//import the newest global way
	    
        cout<<"The newest global way is "<<GlobalWay<<" degree."<<endl;	 

	if(kfnum != mpPointCloudMapper->kfnum)	//when the robot have new key frame, then run local navigator
	{
	    mpPointCloudMapper->ExportAKeyFrame(mpPointCloudMapper->kfnum,kfnum);	//import keyframe
            T_Pose = ORB_SLAM2::Converter::toSE3Quat(mpPointCloudMapper->keyframes[kfnum-1]->GetPose());
	    cout<<"The newest trajectory is "<<T_Pose.translation()[0]<<" "<<T_Pose.translation()[1]<<" "<<T_Pose.translation()[2]<<endl;

	    mpCommunicator->Angle_Export(mpCommunicator->RobotNowWay, LocalWay);	//import now earth angle

	    if(GlobalWay-LocalWay>=-90 && GlobalWay-LocalWay<=90)	//we can go
	    {
	    	LocalGuideSeq = PlanAFaceWay(*mpOctree,T_Pose,GlobalWay,LocalWay,OriginLocalWay);	//use tree value, avoid the tree changed.
		//go!
	    	if (!LocalGuideSeq.empty())	//make sure whether localguideseq is empty
	    	{
		    std::vector<localGuide>::iterator _iter;
		    for (_iter = LocalGuideSeq.begin(); _iter != LocalGuideSeq.end(); ++_iter)	
		    {
	    	    	localGuide point = *_iter;
		    	std::cout << point.n << "   " << point.m << std::endl;
	    	    }
		}
		else      
   	    	    std::cout << "No Way!" << std::endl;
	    }
	    else	//turn around
	    {
		;//turn around!
	    }
	    
	}

	//calculate the local map way

	if(CheckFinish())
            break;
	sleep(3);	//1s a update turn
	
    }

    cout<<endl<<"local navigation is done!"<<endl;

    SetFinish();
}

}

