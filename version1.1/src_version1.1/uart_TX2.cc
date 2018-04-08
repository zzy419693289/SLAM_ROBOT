#include "uart_TX2.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>   
#include "System.h"

#include<mutex>
#include<thread>

namespace ORB_SLAM2
{
Communicating::Communicating() 
{
    mbFinishRequested = false;
    mbFinished = false;
    RobotNowWay = 0;
    fisrtRobotWay = false;
    our_GPS.N_latitude[0] = '0';
    our_GPS.N_latitude[1] = '0';
    our_GPS.N_latitude[2] = '0';
    our_GPS.N_latitude[3] = '0';
    our_GPS.N_latitude[4] = '.';
    our_GPS.N_latitude[5] = '0';
    our_GPS.N_latitude[6] = '0';
    our_GPS.N_latitude[7] = '0';
    our_GPS.N_latitude[8] = '0';
    our_GPS.N_latitude[9] = '\0';
    our_GPS.E_longitude[0] = '0';
    our_GPS.E_longitude[1] = '0';
    our_GPS.E_longitude[2] = '0';
    our_GPS.E_longitude[3] = '0';
    our_GPS.E_longitude[4] = '0';
    our_GPS.E_longitude[5] = '.';
    our_GPS.E_longitude[6] = '0';
    our_GPS.E_longitude[7] = '0';
    our_GPS.E_longitude[8] = '0';
    our_GPS.E_longitude[9] = '0';
    our_GPS.E_longitude[10] = '\0';
    our_GPS.Effective = true;
    fd = Initial_ttyTHS2();	//open serial port ttyTHS2
    cout << "Communicator have been initial!" << endl;
}

void Communicating::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Communicating::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Communicating::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Communicating::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

int Communicating::Initial_ttyTHS2(void)
{
	/*open serial*/
	int fd;
	fd = open("/dev/ttyTHS2", O_RDWR | O_NDELAY);
	if (fd == -1)
		perror("Can not open Serial_Port 1/n");
	/*set the serial mode*/
	struct termios opt;	//defined in termios.h
	tcgetattr(fd, &opt);
	opt.c_cflag |= CLOCAL | CREAD;
	opt.c_cflag &= ~CSIZE;
	opt.c_cflag |= CS8;
	opt.c_cflag &= ~PARENB;
	opt.c_cflag &= ~CSTOPB;
	opt.c_lflag |= ICANON;
	cfsetispeed(&opt, BAUDRATE);
	cfsetospeed(&opt, BAUDRATE);  	//9600, 8bit, 1stop
	opt.c_cc[VTIME] = 0;
	opt.c_cc[VMIN] = 0;
	tcflush(fd, TCIOFLUSH);		//clear data flash
	tcsetattr(fd, TCSANOW, &opt);	//reflash when have new data immediately. And set the serial mode

	return fd;
}

int Communicating::Write_Car(int fd, char* car_operate)	//don't need fd, just input string
{
	int write_r = write(fd, car_operate, Car_Long);
	return write_r;
}

int Communicating::Read_Uart(int fd, char* buffer)
{
	char buffer_one;
	//tcflush(fd, TCIFLUSH);	//clear the input buffer
	int read_data = 0;
	int read_count = 0; 	//count how much times we read.
	while (1)
	{
		read_data = read(fd, &buffer_one, 1);	//$
		read_data = read_data;
		
		if (buffer_one == 0xAA)
		{
			read_data = read(fd, &buffer_one, 1);
			
			if(buffer_one == 0x30)	//recieved the operate
			{
				read_data = read(fd, buffer, 3);	//angle 2, stop 0xBB
				return 0;
			}

			if(buffer_one == 0x31)	//operate is done
			{
				read_data = read(fd, buffer, 3);
				return 1;
			}

			if(buffer_one == 0x32)	//recieved the earth angle
			{
				read_data = read(fd, buffer, 3);
				return 2;
			}

			if(buffer_one == 0x33)	//over time
			{
				read_data = read(fd, buffer, 3);
				return 3;
			}
		
			if(buffer_one == 0x34)	//GPS
			{
				read_data = read(fd, buffer, 20);
				return 4;
			}
		}
		//if have no data too many time , return -1.
		read_count++;
		usleep(200);
		if (read_count > 5000)
			return -1;
	}
}

bool Communicating::Check_GPSdata(char* buffer, GPS_data* our_GPS_ptr)
{
	if(buffer[19]==0xBB)	//stop byte
	{
	    unique_lock<mutex> lck(GPSMutex);
	    memcpy(our_GPS_ptr->N_latitude, buffer, 9);
	    memcpy(our_GPS_ptr->E_longitude, buffer+9, 10);
	    return true;
	}
	else		
	    return false;	//avoid warning
}

void Communicating::GPS_Export(GPS_data& GPS1, GPS_data& GPS2)
{
	unique_lock<mutex> lck(GPSMutex);	
	memcpy(GPS2.N_latitude, GPS1.N_latitude, 10);
	memcpy(GPS2.E_longitude, GPS1.E_longitude, 11);
	GPS2.Effective=GPS1.Effective;
}

bool Communicating::Check_Earthdata(char* buffer, float& RobotNowWay)
{
	if(buffer[2]==0xBB)	//stop byte
	{
	    unique_lock<mutex> lck(AngleMutex);
	    RobotNowWay = (buffer[0]-0x30)*256+buffer[1]-0x30;
	    return true;
	}
	else		
	    return false;	//avoid warning
}

void Communicating::Angle_Export(float& Angle1, float& Angle2)
{
	unique_lock<mutex> lck(AngleMutex);	
	Angle2=Angle1;
}

void Communicating::Close_ttyTHS2(int fd)
{
	close(fd);
}

void Communicating::Run()
{
    // Initial the ttyTHS2
    char buffer[30];
    int dataNum;	//represent different instruction
    bool dataEffect;

    while(1)
    {    
    	//calculate the newest map way
	dataNum = Read_Uart(fd, buffer);

	if(dataNum == 2)	//recieve earth angle
	{
	    dataEffect = Check_Earthdata(buffer, RobotNowWay);
	    fisrtRobotWay = true;
	}

	if(dataNum == 4)	//recieve GPS data
	    dataEffect = Check_GPSdata(buffer, &our_GPS);
	
	if(CheckFinish())
            break;
	usleep(1000);	//1ms a message	
    }

    dataEffect = dataEffect;	//avoid warning

    Close_ttyTHS2(fd);	//close serial port

    cout<<endl<<"Communication is done!"<<endl;

    SetFinish();
}

}
