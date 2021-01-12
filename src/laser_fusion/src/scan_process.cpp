// /*
// 本代码是从LOAM中剥离出来,用于实现雷达点云数据前后帧融合,维护一个相对稠密的局部地图,
// 用于之后的障碍物识别和检测

// 融合方案: 1. 融合最新的固定帧数的雷达点云数据,剔除旧的数据
//                     2. 融合固定距离的雷达点云数据

// designed by Shubin
// */


#include<cmath>
#include<vector>

#include<opencv/cv.h>
#include<ros/ros.h>
#include <nav_msgs/Odometry.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include </home/sishubin/SLAMcodes/My_Local_Repo/Laser_fusion/src/laser_fusion/include/laser_fusion/common.h>


using namespace std;
//using namespace ros;

/****雷达的基本参数设置****/
//点云扫描周期,一般10Hz
const double scanPeriod = 0.1;
//激光雷达线数
const int N_SCANS = 16;

//初始化控制变量
const int systemDelay = 20;//弃用前20帧初始数据
int systemInitCount = 0;
bool systemInited = false;

//接收到点云数据后调用的回调函数
void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloudmsg)
{
    if(!systemInited)
    {
        systemInitCount++;
        if (systemInitCount >= systemDelay) {
           systemInited = true;
        }
    return;
    }

    //记录每个scan有曲率的点的开始和结束索引
    std::vector<int> scanStartInd(N_SCANS,0);
    std::vector<int> scanEndInd(N_SCANS,0);

    //当前的点云时间
    double timeScanCur = laserCloudmsg->header.stamp.toSec();
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    //消息转换成pcl数据存放
    pcl::fromROSMsg(*laserCloudmsg, laserCloudIn);
    std::vector<int> indices;
    //剔除Nan点
    pcl::removeNaNFromPointCloud(laserCloudIn,laserCloudIn,indices);
    //点云中点的数量
    int cloudSize = laserCloudIn.points.size();
    //lidar scan开始点的旋转角,atan2范围[-pi,+pi],计算旋转角时取负号是因为velodyne是顺时针旋转
    float startOri = -atan2(laserCloudIn.points[0].y,laserCloudIn.points[0].x);
    //lidar scan结束点的旋转角，加2*pi使点云旋转周期为2*pi
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                        laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;
    //判断结束方位角和起始角范围是否合理
    //结束方位角与开始方位角差值控制在(PI,3*PI)范围，允许lidar不是一个圆周扫描
    //正常情况下在这个范围内：pi < endOri - startOri < 3*pi，异常则修正
    if(endOri - startOri  >  3*M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if(endOri - startOri < M_PI)
    {
        endOri += endOri + 2*M_PI; 
    }
    //判断雷达是否旋转过半
    bool halfPassed = false;
    int count = cloudSize;
    PointType point_;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for(int i=0;i<cloudSize;i++)
    {
        //坐标轴交换，velodyne lidar的坐标系也转换到z轴向前，x轴向左的右手坐标系
        point_.x = laserCloudIn.points[i].y;
        point_.y = laserCloudIn.points[i].z;
        point_.z = laserCloudIn.points[i].x;

        //计算点的仰角,然后根据仰角排列激光号,velodyne/robosense每两个scan之间间隔是2度(垂直方向)
        //仰角的计算公式: atan(y/sqrt(x^2+z^2)),单位:角度
        float angle = atan2(point_.y , sqrt(point_.x*point_.x + point_.z*point_.z)) * 180 / M_PI;
        int scanID;
        //对仰角取整,四舍五入
        int roundedAngle = int(angle+(angle<0.0? -0.5:+0.5));
        //记录点所在的scanID, VLD-16/robosense中ID范围:0~15,共16线
        // VLD-16/robosense中仰角从15度->-15度时,ID从15递减至0
        if(roundedAngle > 0)
        {
            scanID = roundedAngle;
        }
        else
        {
            scanID = roundedAngle + (N_SCANS - 1);
        }
        //过滤掉scanID不正确的点
        if (scanID > (N_SCANS - 1) || scanID < 0 )
        {
            count--;
            continue;
         }
        //计算该点的旋转角,旋转角定义为空间中的点与z轴方向的夹角
        float ori = -atan2(point_.x,point_.z);
        //?????????旋转角的修正部分没看懂,loam中旋转角是如何定义的??
        if(!halfPassed)
        {
            //扫描未过半
            //根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，从而进行补偿
            //确保-pi/2 < ori - startOri < 3*pi/2
            //??????为啥不是0< ori - startOri < 2*pi
            if(ori < startOri - M_PI/2)
            {
                ori += 2*M_PI;
            }
            else if(ori > startOri + M_PI*3/2)
            {
                ori -= 2*M_PI;
            }
            //旋转超过半圈
            if(ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            //旋转超过半圈
            //????????超过半圈后为啥要加2*pi
            ori += 2 * M_PI;
            //确保-3*pi/2 < ori - endOri < pi/2
            if (ori < endOri - M_PI * 3 / 2) 
            {
                ori += 2 * M_PI;
            } 
            else if (ori > endOri + M_PI / 2)
             {
                ori -= 2 * M_PI;
            }
        }
        

        





    }






    



    
    
    



 }




int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_process");
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points",2,laserCloudHandler);

    
    return 0;
}

