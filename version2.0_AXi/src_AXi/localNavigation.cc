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

#include<math.h>
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
#define TDResolution 0.05

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
    kfnum = 0;
    Send_Restart = false;  
    our_Angle_Ptr = new Angle_data;	//give memory to this struct
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

void LocalNavigating::ExportSendMesSeq(vector<Send_Message>& SendSeqLocal, vector<Send_Message>& SendSeqUart)
{
    unique_lock<mutex> lck(ShareSeqMutex);
    SendSeqUart.assign(SendSeqLocal.begin(), SendSeqLocal.end());	
}

void LocalNavigating::CheckSendType(bool& SendTypeLocal, bool& SendTypeUart)
{
    unique_lock<mutex> lck(ShareSeqMutex);
    if(SendTypeLocal==true)
    {
		SendTypeUart=true;
		SendTypeLocal=false;	//after let uart send restart, take self false 
    }
    else
		SendTypeUart=false;
}

vector<localGuide> LocalNavigating::PlanAFaceWay(octomap::OcTree tOctree,Eigen::Isometry3d t_Pose,float GlobalWay,float LocalWay,float OriginLocalWay)	//add front go
{
    vector<localGuide> tLocalGuideSeq;
    tLocalGuideSeq.clear();
    float treeHitLog = tOctree.getProbHitLog();
    double costhis,sinthis;
    costhis = cos((LocalWay-OriginLocalWay)*3.1415926535897932/180);
    sinthis = sin((LocalWay-OriginLocalWay)*3.1415926535897932/180);
    int NavigatorMat[MAXMAPM][MAXMAPN] = {0};	//used to create a 0-1 map, and initial false

    for(int m=ExpandCoef;m<MAXMAPM-ExpandCoef;m++)	//n == x, m == z
	for(int n=ExpandCoef;n<MAXMAPN-ExpandCoef;n++)  //ignore the outer ring 
	{
	    //detect. If this point have been occupied by expand, continue the circle directly
	    if(NavigatorMat[m][n] == 1)
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

	    if(IfObs)	//expand! When the expansive range is bigger, we will save more time. There the range is 5cm on each side.
	    {
			NavigatorMat[m][n] = 1; //save the output in navigator map
			for(int exm=- ExpandCoef;exm<=ExpandCoef;exm++)
		    for(int exn=- ExpandCoef;exn<=ExpandCoef;exn++)
		    {
				NavigatorMat[m+exm][n+exn]=1;
		    }
	    }
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
	    	NavigatorMat[point.m][point.n] = 2;	//site the right way
	    	tLocalGuideSeq.push_back(point);
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
    }
    
    delete Aend;
    delete Astart;
    delete Amaze;	    

    return tLocalGuideSeq;
}

vector<Send_Message> LocalNavigating::GoSendMesXi(vector<localGuide> LocalGuideSeq)
{
    unsigned int SeqCounter;
    vector<Send_Message> RealGuideSeq;
    Send_Message This_Step;
    This_Step.Data1 = 0x30;
    This_Step.Data2 = 0x30; //stop
    This_Step.Data3 = 0x30; //+
    This_Step.Data4 = 0x43; //0.158m/s
    float LastWay = 0;
    for(SeqCounter = 0;SeqCounter+1<LocalGuideSeq.size();SeqCounter+=1)
    {
        float tangle = atan2(LocalGuideSeq[SeqCounter+1].n-LocalGuideSeq[SeqCounter].n, LocalGuideSeq[SeqCounter+1].m-LocalGuideSeq[SeqCounter].m)*180/3.14159;
		int outangle = (int)(tangle - LastWay);	//- is left , + is right
		if(outangle == 0)	//go straight
		{
	   		This_Step.Data2 = 0x31;
	    	This_Step.Data3 = 0x30;
	    	This_Step.Data4 = 0x43;
        	RealGuideSeq.push_back(This_Step);
		}
		else			// need turn around
		{
	    	This_Step.Data2 = 0x32;
	    	if(outangle > 0)
				This_Step.Data3 = 0x30;
	    	else
	    	{
				This_Step.Data3 = 0x31;
				outangle *= -1;
	    	}
	    	This_Step.Data4 = (char)(outangle & 0xff) + 0x30;	//turn to char
	    	RealGuideSeq.push_back(This_Step);
	    	This_Step.Data2 = 0x31;
	    	This_Step.Data3 = 0x30;
	    	This_Step.Data4 = 0x43;
        	RealGuideSeq.push_back(This_Step);
		}
		LastWay = tangle;
    }
    return RealGuideSeq;
}

vector<Send_Message> LocalNavigating::ParaSendMesXi()
{
    vector<Send_Message> RealGuideSeq;
    Send_Message This_Step;
    This_Step.Data1 = 0x30;
    This_Step.Data2 = 0x32; //turn 90
    This_Step.Data3 = 0x30; //+
    This_Step.Data4 = 0x8A; //90
    RealGuideSeq.push_back(This_Step);
    This_Step.Data2 = 0x31; //go straight
    This_Step.Data3 = 0x30; //+
    This_Step.Data4 = 0x43; 
    RealGuideSeq.push_back(This_Step);	//3 times
    RealGuideSeq.push_back(This_Step);
    RealGuideSeq.push_back(This_Step);
    std::cout << "No Way!" << std::endl;
    return RealGuideSeq;
}

vector<Send_Message> LocalNavigating::TurnSendMesXi(float globalWay, float localWay)
{
    vector<Send_Message> RealGuideSeq;
    Send_Message This_Step;//turn around!
    This_Step.Data1 = 0x30;
    This_Step.Data2 = 0x32;
    int detGLWay = globalWay-localWay;
    if(detGLWay<-90)
    {
		This_Step.Data3 = 0x31;
		detGLWay*=-1;
    }
    if(detGLWay>90)
		This_Step.Data3 = 0x30;
    This_Step.Data4 = (char)(detGLWay & 0xff) + 0x30;
    RealGuideSeq.push_back(This_Step);
    std::cout << "Turn around!" << std::endl;
    return RealGuideSeq;
}

vector<Send_Message> LocalNavigating::GoSendMesOm(vector<localGuide> LocalGuideSeq)
{
    unsigned int SeqCounter;
    vector<Send_Message> RealGuideSeq;
    Send_Message This_Step;
    This_Step.Data1 = 0x30; //Go pattern
    This_Step.Data2 = 0x31; //Go straight model
    This_Step.Data3 = 0x30; //0 degree
    This_Step.Data4 = 0x30; //0m
    for(SeqCounter = 0;SeqCounter+1<LocalGuideSeq.size();SeqCounter+=1)
    {
		//depend real map, calculate real distance
        float tdistance = (abs(LocalGuideSeq[SeqCounter+1].n-LocalGuideSeq[SeqCounter].n)+abs(LocalGuideSeq[SeqCounter+1].m-LocalGuideSeq[SeqCounter].m))*TDResolution;	
		int tdistance1 = floor(tdistance/0.03);	//calculate distance's gear
		This_Step.Data4 = 0x30 + tdistance1;
		//depend n and n+1 point, calculate the go way
		int tGoDegree = (LocalGuideSeq[SeqCounter+1].m-LocalGuideSeq[SeqCounter].m)*10+(LocalGuideSeq[SeqCounter+1].n-LocalGuideSeq[SeqCounter].n);		
		switch(tGoDegree)
		{
			case  10:This_Step.Data3 = 0x30;break;
			case  11:This_Step.Data3 = 0x31;break;
			case   1:This_Step.Data3 = 0x32;break;
			case  -9:This_Step.Data3 = 0x33;break;
			case -10:This_Step.Data3 = 0x34;break;
			case -11:This_Step.Data3 = 0x35;break;
			case  -1:This_Step.Data3 = 0x36;break;
			case   9:This_Step.Data3 = 0x37;break;
		}

		RealGuideSeq.push_back(This_Step);
    }
    return RealGuideSeq;
}

vector<Send_Message> LocalNavigating::BackSendMesOm()
{
    vector<Send_Message> RealGuideSeq;
    Send_Message This_Step;
    This_Step.Data1 = 0x30; //Go pattern
    This_Step.Data2 = 0x31; //Go straight model
    This_Step.Data3 = 0x34; //180 degree
    This_Step.Data4 = 0x30+30; //0.9m
    RealGuideSeq.push_back(This_Step);
    std::cout << "No Way!" << std::endl;
    return RealGuideSeq;
}

vector<Send_Message> LocalNavigating::TurnSendMesOm(float globalWay, float localWay)
{
    vector<Send_Message> RealGuideSeq;
    Send_Message This_Step;//turn around!
    This_Step.Data1 = 0x30; //Go pattern
    This_Step.Data2 = 0x32; //turn model
    int detGLWay = globalWay-localWay;
	This_Step.Data3 = 0x30 + detGLWay/100;	//hundred bit 
    This_Step.Data4 = 0x30 + detGLWay%100;	//a bit and ten bit
    RealGuideSeq.push_back(This_Step);
    std::cout << "Rotate "<<detGLWay<<" degrees!"<< std::endl;
    return RealGuideSeq;
}

void LocalNavigating::Run()
{
    //wait for initial
    sleep(1);
    Eigen::Isometry3d tT_Pose;

    mpCommunicator->Angle_Export(&(mpCommunicator->our_Angle), our_Angle_Ptr);
/*    while(!our_Angle_Ptr->fisrtWayGet)	//wait until we have first angle
    {										//if we have compass, we will use this code
	if(CheckFinish())
            break;
	usleep(1000);
	mpCommunicator->Angle_Export(&(mpCommunicator->our_Angle), our_Angle_Ptr);
    }*/
    OriginLocalWay=our_Angle_Ptr->RobotNowWay;

    while(kfnum == 0)		//wait until first key frame
    {
    	mpPointCloudMapper->ExportAKeyFrame(mpPointCloudMapper->kfnum,kfnum);	//import keyframe
		if(CheckFinish())
            break;
		usleep(1000);
    }

    while(1)
    {     
		//import the newest global way
		mpGlobalNavigator->GlobalWay_Export(mpGlobalNavigator->GlobalWay, GlobalWay);		 
		//when the robot not arrived the target, then run local navigator.import the newest pose
		mpPointCloudMapper->ExportAKeyFrame(mpPointCloudMapper->kfnum, kfnum);	
        T_Pose = ORB_SLAM2::Converter::toSE3Quat(mpPointCloudMapper->keyframes[kfnum-1]->GetPose());
		cout<<"The newest trajectory is "<<T_Pose.translation()[0]<<" "<<T_Pose.translation()[1]<<" "<<T_Pose.translation()[2]<<endl;
		//import now earth angle
		mpCommunicator->Angle_Export(&(mpCommunicator->our_Angle), our_Angle_Ptr);	
		LocalWay=our_Angle_Ptr->RobotNowWay;
		cout<<"The newest local way is "<<LocalWay<<" degree."<<endl;	 

		if(GlobalWay-LocalWay>=-90 && GlobalWay-LocalWay<=90)	//we can go
		{
	    	LocalGuideSeq = PlanAFaceWay(*mpOctree,T_Pose,GlobalWay,LocalWay,OriginLocalWay);	//use tree value, avoid the tree changed.		
	    	if (!LocalGuideSeq.empty())	//make sure whether localguideseq is empty, if no way move backly
	    	{
				//go!
	        	unique_lock<mutex> lck(ShareSeqMutex);
				SendMessageSeq.clear();
				SendMessageSeq = GoSendMesXi(LocalGuideSeq);
    			Send_Restart = true;
	    	}
	    	else  
	    	{
				//move backly   
				unique_lock<mutex> lck(ShareSeqMutex);
				SendMessageSeq.clear();		    
				SendMessageSeq = ParaSendMesXi();	
				Send_Restart = true;
	    	}
		}
		else	//turn around
		{
	    	unique_lock<mutex> lck(ShareSeqMutex);
	    	SendMessageSeq.clear();
	    	SendMessageSeq=TurnSendMesXi(GlobalWay, LocalWay);
	    	Send_Restart = true;
		}    

		if(CheckFinish())
        	break;
		sleep(3);	//1s a update turn	
    }

    delete our_Angle_Ptr;	//free this struct

    cout<<endl<<"local navigation is done!"<<endl;

    SetFinish();
}

}

