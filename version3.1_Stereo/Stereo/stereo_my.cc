/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <stdio.h>
#include <string.h>
#include <termios.h>  
#include <unistd.h>  
#include <fcntl.h> 

#include<opencv2/core/core.hpp>

// ZED include
#include <sl/Camera.hpp>

#include<System.h>

using namespace std;

int kbhit(void);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./stereo_my path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

	/*******************************************open ZED************************************************/
	 // Create a ZED camera object
    sl::Camera zed;
    // Set configuration parameters
    sl::InitParameters init_params;
	init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
    init_params.camera_resolution = sl::RESOLUTION_VGA; // Use 672*376  mode
    //init_params.camera_resolution = sl::RESOLUTION_HD720; // Use 1280*720  mode
    //init_params.camera_resolution = sl::RESOLUTION_HD1080; // Use 1920*1080  mode
    init_params.coordinate_units = sl::UNIT_METER;	//unit is m
    init_params.camera_fps = 15; // SLAM is 15 fps
	// Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::SUCCESS)
	{
		zed.close();
        exit(-1);
	}
    // Print camera information
    printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
    printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Main loop
    sl::Mat zed_leftimage(zed.getResolution(), sl::MAT_TYPE_8U_C4);
	sl::Mat zed_rightimage(zed.getResolution(), sl::MAT_TYPE_8U_C4);
	sl::Mat zed_depthimage(zed.getResolution(), sl::MAT_TYPE_32F_C1);
	//cv_Mat use zed_Mat's memory
    cv::Mat imleftRGB = cv::Mat((int) zed_leftimage.getHeight(), (int) zed_leftimage.getWidth(), CV_8UC4, zed_leftimage.getPtr<sl::uchar1>(sl::MEM_CPU));
	cv::Mat imrightRGB = cv::Mat((int) zed_rightimage.getHeight(), (int) zed_rightimage.getWidth(), CV_8UC4, zed_rightimage.getPtr<sl::uchar1>(sl::MEM_CPU));
	cv::Mat imD = cv::Mat((int) zed_depthimage.getHeight(), (int) zed_depthimage.getWidth(), CV_32FC1, zed_depthimage.getPtr<sl::float1>(sl::MEM_CPU));
	double sec_timestamp;

    while(1)
    {
    	if (zed.grab() == sl::SUCCESS) 
		{
            // A new image is available if grab() returns SUCCESS
			zed.retrieveImage(zed_leftimage, sl::VIEW_LEFT,sl::MEM_CPU); // Get the left image, TX2 have no GPU memory
			zed.retrieveImage(zed_rightimage, sl::VIEW_RIGHT,sl::MEM_CPU); // Get the left image, TX2 have no GPU memory
			zed.retrieveMeasure(zed_depthimage, sl::MEASURE_DEPTH,sl::MEM_CPU); 	//get depth image, use GPU
            sec_timestamp = zed.getCameraTimestamp()/1000000000; // Get the timestamp at the time the image was captured

			SLAM.TrackStereo( imleftRGB, imrightRGB, imD, sec_timestamp );

            if(kbhit())	//press 'q' to end slam
			{
				char ch=getchar();
				if(ch=='q') 
					break;
			}
        }
		else
		{
			if(kbhit())	//press 'q' to end slam
			{
				char ch=getchar();
				if(ch=='q') 
					break;
			}
		}     
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("MyKeyFrameTrajectory.txt");

	// Close the camera
	zed_leftimage.free(sl::MEM_CPU);
	zed_rightimage.free(sl::MEM_CPU);
	zed_depthimage.free(sl::MEM_CPU);
    zed.close();

    return 0;
}

//background check press key
int kbhit(void)  
{  
  struct termios oldt, newt;  
  int ch;  
  int oldf;  
  tcgetattr(STDIN_FILENO, &oldt);  
  newt = oldt;  
  newt.c_lflag &= ~(ICANON | ECHO);  
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);  
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  
  ch = getchar();  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
  fcntl(STDIN_FILENO, F_SETFL, oldf);  
  if(ch != EOF)  
  {  
    ungetc(ch, stdin);  
    return 1;  
  }  
  return 0;  
}  
