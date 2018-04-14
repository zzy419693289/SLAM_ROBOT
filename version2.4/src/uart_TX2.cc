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
    our_Angle.RobotNowWay = 0;
    our_Angle.fisrtWayGet = false;
    our_Angle.Effective = false;
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
//    our_GPS.Effective = false;	//really use!!!
    our_GPS.Effective = true;	//test use!!!
    car_operate[0] = 0xAA;
    car_operate[5] = 0xBB;
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
	fd = open("/dev/ttyTHS2", O_RDWR);
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
	opt.c_lflag &= ~ICANON;
	cfsetispeed(&opt, BAUDRATE);
	cfsetospeed(&opt, BAUDRATE);  	//115200, 8bit, 1stop
	opt.c_cc[VTIME] = 1;
	opt.c_cc[VMIN] = 1;
	tcflush(fd, TCIOFLUSH);		//clear data flash
	tcsetattr(fd, TCSANOW, &opt);	//reflash when have new data immediately. And set the serial mode

	return fd;
}

void Communicating::SetLocalNavigator(LocalNavigating *pLocalNavigator)
{
    mpLocalNavigator = pLocalNavigator;		
    cout << "Local navigator have been set!" << endl;
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
				read_data = read(fd, buffer, 22);
				return 4;
			}
		}
		//if have no data too many time , return -1.
		read_count++;
		if (read_count > 10)
			return -1;
	}
}

bool Communicating::Check_GPSdata(char* buffer, GPS_data* our_GPS_ptr)
{
	if(buffer[21]==0xBB)	//stop byte,total have 24 chars
	{
	    unique_lock<mutex> lck(GPSMutex);
	    memcpy(our_GPS_ptr->N_latitude, buffer, 9);
	    memcpy(our_GPS_ptr->E_longitude, buffer+10, 10);
	    our_GPS_ptr->Effective = true;
	    return true;
	}
	else		
	    return false;	//avoid warning
}

void Communicating::GPS_Export(GPS_data* GPS1_ptr, GPS_data* GPS2_ptr)
{
	unique_lock<mutex> lck(GPSMutex);	
	memcpy(GPS2_ptr->N_latitude, GPS1_ptr->N_latitude, 10);
	memcpy(GPS2_ptr->E_longitude, GPS1_ptr->E_longitude, 11);
	GPS2_ptr->Effective=GPS1_ptr->Effective;
	//GPS1_ptr->Effective = false;	//real use!!!
	GPS1_ptr->Effective = true;	//test use!!!
}

bool Communicating::Check_Earthdata(char* buffer, Angle_data* our_Angle_ptr)	//uart GPS every 5 second
{
	if(buffer[2]==0xBB)	//stop byte
	{
	    unique_lock<mutex> lck(AngleMutex);
	    our_Angle_ptr->RobotNowWay=(buffer[1]-0x30)*100+buffer[0]-0x30;
	    our_Angle_ptr->Effective = true;
	    our_Angle_ptr->fisrtWayGet = true;
	    return true;
	}
	else		
	    return false;	//avoid warning
}

void Communicating::Angle_Export(Angle_data* Angle1_ptr, Angle_data* Angle2_ptr)
{
	unique_lock<mutex> lck(AngleMutex);	
	Angle2_ptr->RobotNowWay=Angle1_ptr->RobotNowWay;
	Angle2_ptr->Effective=Angle1_ptr->Effective;
	Angle2_ptr->fisrtWayGet=Angle1_ptr->fisrtWayGet;
	Angle1_ptr->Effective = false;
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
    unsigned int Send_times = 0;	//record how many operates we have send
    unsigned int Recieve_times = 0;	//record how many operates the car have recieved
    bool last_op_down = true;		//record last instruction whether have been executed, if done---true, 
    int Send_Count = 0;	//used to count send message time
    unsigned int Seq_Num = 0;	//record last time which we send
    vector<Send_Message> SendMesSeqUart;
    bool SendTypeUart;

    while(1)
    {    
    	//calculate the newest map way
	dataNum = Read_Uart(fd, buffer);	//if return -1, means not recieved

	if(dataNum == 0)	//recieved reflect 
	    Recieve_times++;

	if(dataNum == 1)	//recieve last operate down signal
	    last_op_down = true;

	if(dataNum == 2)	//recieve earth angle
	    dataEffect = Check_Earthdata(buffer, &our_Angle);

	if(dataNum == 4)	//recieve GPS data
	    dataEffect = Check_GPSdata(buffer, &our_GPS);

	Send_Count++;
	//if(Send_Count>19)	//50*20=1s
	if(last_op_down && (Send_Count>9))	//if last instruction have been executed, then send next instruction.if have yet no instruction, we will circle wait new instruction sequences arrived. make sure 2 instruction have 0.5 second delay
	{
	    //write
	    Send_Count = 0;
	    mpLocalNavigator->CheckSendType(mpLocalNavigator->Send_Restart, SendTypeUart);	//if need start new, then take SendTypeUart=1, other SendTypeUart=0.
	    if(SendTypeUart)
            {
		Seq_Num = 0;
		mpLocalNavigator->ExportSendMesSeq(mpLocalNavigator->SendMessageSeq, SendMesSeqUart);
	    }    
	    if(Seq_Num<SendMesSeqUart.size())	//if not the guide sequence end, we write to car
	    {
		car_operate[1] = SendMesSeqUart[Seq_Num].Data1;
    		car_operate[2] = SendMesSeqUart[Seq_Num].Data2;
		car_operate[3] = SendMesSeqUart[Seq_Num].Data3;
    		car_operate[4] = SendMesSeqUart[Seq_Num].Data4;
		std::cout << car_operate[2] << "   " << car_operate[3] << "   " << (car_operate[4]-0x30)/10+0x30<<(car_operate[4]-0x30)%10+0x30 << std::endl;
		dataEffect = Write_Car(fd, car_operate);
		Seq_Num++;

		last_op_down = false;
		Send_times++;
	    }
	}
			
	if(CheckFinish())
            break;
	usleep(50000);	//50ms check one message	
    }

    dataEffect = dataEffect;	//avoid warning

    //stop the robot
    car_operate[1] = 0x30;
    car_operate[2] = 0x30;
    car_operate[3] = 0x30;
    car_operate[4] = 0x30;
    dataEffect = Write_Car(fd, car_operate);  
    sleep(1);	//wait a minute to support time to stop

    //close serial port
    Close_ttyTHS2(fd);	

    cout<<endl<<"Communication is done!"<<endl;

    SetFinish();
}

}
