#ifndef UART_TX2_H
#define UART_TX2_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "localNavigation.h"
#include <thread>
#include <mutex>

namespace ORB_SLAM2
{
#define BAUDRATE B115200 
#define Car_Long 6

class LocalNavigating; 

struct GPS_data
{
	char N_latitude[10];	
	char E_longitude[11];	
	bool Effective;
};

struct Angle_data
{
	float RobotNowWay;	//robot's earth way
	bool Effective;	
};

class Communicating
{
public:
	Communicating();	

	int Write_Car(int fd, char* car_operate);

	void Run();

	void RequestFinish();

    	bool isFinished();

	void GPS_Export(GPS_data* GPS1_ptr, GPS_data* GPS2_ptr);

	GPS_data our_GPS;	//GPS data

	void Angle_Export(Angle_data* Angle1_ptr, Angle_data* Angle2_ptr);

	Angle_data our_Angle;	//angle data

	void SetLocalNavigator(LocalNavigating *pLocalNavigator);

protected:
	bool CheckFinish();
    	void SetFinish();
    	bool mbFinishRequested;
    	bool mbFinished;
    	std::mutex mMutexFinish;

	std::mutex GPSMutex;
    	std::mutex AngleMutex;

	int Initial_ttyTHS2(void);	//return fd

	void Close_ttyTHS2(int fd);

	int Read_Uart(int fd, char* buffer);		//read GPS one time

	bool Check_GPSdata(char* buffer, GPS_data* our_GPS_ptr);		//check whether data can use

	bool Check_Earthdata(char* buffer, Angle_data* our_Angle_ptr);
	
	int fd;	//serial port fd 

	//Local Navigator
    	LocalNavigating* mpLocalNavigator;

	//operate car
	char car_operate[6];
};

}

#endif
