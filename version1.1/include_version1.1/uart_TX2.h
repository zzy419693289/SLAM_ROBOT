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

#include <thread>
#include <mutex>

#define BAUDRATE B9600 
#define Car_Long 6
namespace ORB_SLAM2
{
 
struct GPS_data
{
	char N_latitude[10];	
	char E_longitude[11];	
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

	void GPS_Export(GPS_data& GPS1, GPS_data& GPS2);

	GPS_data our_GPS;	//GPS data

	void Angle_Export(float& Angle1, float& Angle2);

	float RobotNowWay;	//robot's earth way

	bool fisrtRobotWay;	//record whether have fisrt way

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

	bool Check_Earthdata(char* buffer, float& RobotNowWay);
	
	int fd;	//serial port fd
};

}

#endif
