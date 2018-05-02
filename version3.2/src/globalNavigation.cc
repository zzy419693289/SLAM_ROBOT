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

#include <opencv2/highgui/highgui.hpp>
#include "localNavigation.h"
#include "uart_TX2.h"
#include "globalNavigation.h"

#include<math.h>
#include<mutex>
#include<thread>

namespace ORB_SLAM2
{
#define MAXGPSERROR 0.00005

GlobalNavigating::GlobalNavigating(float longitude_, float latitude_)
{  
    mbFinishRequested = false;
    mbFinished = false;
    GlobalWay = 0;
    GlobalWayEffective = false;
    End_Longitude = longitude_;
    End_Latitude = latitude_;
    Now_Longitude = 0;
    Now_Latitude = 0;
    our_GPS_Ptr = new GPS_data;	//give memory to this struct
    cout << "Global navigator have been initial!" << endl;
}

void GlobalNavigating::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool GlobalNavigating::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void GlobalNavigating::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool GlobalNavigating::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void GlobalNavigating::SetLocalNavigator(LocalNavigating *pLocalNavigator)
{
    mpLocalNavigator = pLocalNavigator;		//import the Global navigator	
    cout << "Local navigator have been set!" << endl;
}

void GlobalNavigating::SetCommunicator(Communicating *pCommunicator)
{
    mpCommunicator = pCommunicator;		//import the octree	
    cout << "Global Communicator have been set!" << endl;
}

void GlobalNavigating::GlobalWay_Export(float& GlobalWay1, float& GlobalWay2, bool& Effectiveness1, bool& Effectiveness2)
{
    unique_lock<mutex> lck(GlobalWayMutex);	
    GlobalWay2 = GlobalWay1;
    Effectiveness2 = Effectiveness1;
}

void GlobalNavigating::Trans_HardMap(GPS_data* GPS_Ptr, float& Latitude, float& Longitude)
{
    Latitude = (GPS_Ptr->N_latitude[0]-0x30)*10+(GPS_Ptr->N_latitude[1]-0x30)+((GPS_Ptr->N_latitude[2]-0x30)*10+(GPS_Ptr->N_latitude[3]-0x30)+(GPS_Ptr->N_latitude[5]-0x30)*0.1+(GPS_Ptr->N_latitude[6]-0x30)*0.01+(GPS_Ptr->N_latitude[7]-0x30)*0.001+(GPS_Ptr->N_latitude[8]-0x30)*0.0001)/60;
    Longitude = (GPS_Ptr->E_longitude[0]-0x30)*100+(GPS_Ptr->E_longitude[1]-0x30)*10+(GPS_Ptr->E_longitude[2]-0x30)+((GPS_Ptr->E_longitude[3]-0x30)*10+(GPS_Ptr->E_longitude[4]-0x30)+(GPS_Ptr->E_longitude[6]-0x30)*0.1+(GPS_Ptr->E_longitude[7]-0x30)*0.01+(GPS_Ptr->E_longitude[8]-0x30)*0.001+(GPS_Ptr->E_longitude[9]-0x30)*0.0001)/60;
}

bool GlobalNavigating::HaveArrivedNode(float Now_Longitude, float Now_Latitude, float End_Longitude, float End_Latitude)
{
	float GPSError = sqrt(pow(End_Longitude-Now_Longitude, 2)+pow(End_Latitude-Now_Latitude, 2));
	if(GPSError>MAXGPSERROR)	//not arrived
		return false;
	else
		return true;
}

float GlobalNavigating::Calculate_angle(float Now_Longitude, float Now_Latitude, float End_Longitude, float End_Latitude)
{
    float tangle = atan2(End_Longitude-Now_Longitude, End_Latitude-Now_Latitude)*180/3.14159;
    if(tangle<0)
	tangle = tangle+360;
    return tangle;
}

void GlobalNavigating::Run()
{
    sleep(5);

    while(1)
    {    
    	//calculate the newest map way
	if(mbFinishRequested == false)
	{
		mpCommunicator->GPS_Export(&(mpCommunicator->our_GPS), our_GPS_Ptr);
	}
	
	if (our_GPS_Ptr->Effective)	//global effect every 10 second update
	{	
	    	Trans_HardMap(our_GPS_Ptr, Now_Latitude, Now_Longitude);
		printf("North latitude is %f\r\n",Now_Latitude);
		printf("East longitude is %f\r\n",Now_Longitude);
		//judge whether arrived end point
		bool Arrived = HaveArrivedNode(Now_Longitude, Now_Latitude, End_Longitude, End_Latitude);
		if(!Arrived)	//if not arrived
		{
			unique_lock<mutex> lck(GlobalWayMutex);
			//calculate the global way
			GlobalWay = Calculate_angle(Now_Longitude, Now_Latitude, End_Longitude, End_Latitude);
			GlobalWayEffective = true;
			cout<<"The newest global way is "<<GlobalWay<<" degree."<<endl;
		}
		else	//if arrived
		{
			unique_lock<mutex> lck(GlobalWayMutex);
			GlobalWayEffective = false;	//stop the car
			cout<<"We have arrived the destination!"<<endl;
		}		
	}
	else
	{
		unique_lock<mutex> lck(GlobalWayMutex);
		GlobalWayEffective = false;
		cout<<"GPS have been lost!"<<endl;
	}
	
	if(CheckFinish())
            break;
	sleep(10);	//10s a update turn	
    }

    delete our_GPS_Ptr;	//free this struct

    cout<<endl<<"global navigation is done!"<<endl;

    SetFinish();
}

}

