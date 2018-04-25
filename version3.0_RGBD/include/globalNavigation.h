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

#ifndef GLOBALNAVIGATION_H
#define GLOBALNAVIGATION_H

#include "System.h"

#include "uart_TX2.h"
#include "localNavigation.h"

#include <thread>
#include <mutex>

namespace ORB_SLAM2
{
struct GPS_data;

class LocalNavigating;

class Communicating;

class GlobalNavigating
{
public:
    
    GlobalNavigating(float longitude_, float latitude_);
       
    void Run();

    void RequestFinish();

    bool isFinished();

    void SetLocalNavigator(LocalNavigating *pLocalNavigator);

    void SetCommunicator(Communicating *pCommunicator);

    void GlobalWay_Export(float& GlobalWay1, float& GlobalWay2, bool& Effectiveness1, bool& Effectiveness2);

    float GlobalWay;	//the Way of global map

    bool GlobalWayEffective;	//the effectiveness of global way
    
protected:  
    void Trans_Baidu(GPS_data* GPS_Ptr, float& Latitude, float& Longitude);

    bool HaveArrivedNode(float Now_Longitude, float Now_Latitude, float End_Longitude, float End_Latitude);	//judge whether the car is near end point

    float Calculate_angle(float Now_Longitude, float Now_Latitude, float End_Longitude, float End_Latitude);	//from 0-360, North is zero    

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    std::mutex GlobalWayMutex;

    LocalNavigating* mpLocalNavigator;

    //communicator
    Communicating* mpCommunicator;

    GPS_data* our_GPS_Ptr;

    float End_Longitude;
    float End_Latitude;

    float Now_Longitude;
    float Now_Latitude;

};

}

#endif // LOCALNAVIGATITION_H
