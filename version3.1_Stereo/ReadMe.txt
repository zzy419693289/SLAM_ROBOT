Verision3.1：
由于RGB-D模式下，在室外开阔环境中很容易出现匹配丢失的情况，原因是由于RGB-D在低分辨率下深度测量范围有限。从现版本开始，系统使用Stereo模式进行特征点跟踪，然后使用ZED提供的深度图进行局部地图建模。现在的启动指令如下：
/home/nvidia/ORB_SLAM2/Examples/Stereo/stereo_my /home/nvidia/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/nvidia/ORB_SLAM2/Examples/Stereo/Zed_Stereo.yaml
具体模式设定为672x376分辨率下，1000个特征点提取。
词典二进制化：
我们现在将ORB词典从txt读入格式，改为从bin格式读入。这样系统启动的速度提高了几倍。
显示界面的精简化：
现在系统启动后，只会留有current frame窗口以及控制台，便于我们进行监控。而原来的map viewer窗口由于对于调试用于不大，被我们屏蔽掉了。
修复BUG：
修复GPS信息与地图坐标转换的BUG。
