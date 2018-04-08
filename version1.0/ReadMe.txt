Version1.0:
在ORB_SLAM2算法的基础上，新开一个线程用于进行octomap的构建。ZED_Depth.yaml为系统参数配置文件。系统适用于ROS系统下，配合ZED摄像头使用。启动命令如下：
roscore
roslaunch zed_wrapper zed.launch
rosrun ORB_SLAM2 RGBD /home/nvidia/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/nvidia/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Zed_Depth.yaml
使用ctrl+c退出系统。使用octomap带有的octovis进行对.bt地图文件进行观察。