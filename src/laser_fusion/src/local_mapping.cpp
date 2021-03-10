//Local_mapping
//功能:(1)构建维护局部地图; (2)根据局部地图优化Lidar位姿

#include <math.h>
#include </home/sishubin/SLAMcodes/My_Local_Repo/Laser_fusion/src/laser_fusion/include/laser_fusion/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//扫描周期
const float scanPeriod = 0.1;

//控制接收到的点云数据，每隔几帧处理一次
const int stackFrameNum = 1;

//控制处理得到的点云map，每隔几次publish给rviz显示
const int mapFrameNum = 5;

//时间戳
double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

//接收标志
bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;
bool newLaserOdometry = false;

//这三个参数啥意思???
int laserCloudCenWidth = 10;
int laserCloudCenHeight = 5;
int laserCloudCenDepth = 10;

//LOAM中维护的点云地图的大小,在空间中由很多个立方体构成
const int laserCloudWidth = 21;
const int laserCloudHeight = 11;
const int laserCloudDepth = 21;
//维护点云的立方体数量: 21*11*21=4851
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

//odometry计算得到的到世界坐标系下的转移矩阵
float transformSum[6] = {0};


int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;
double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};



//最新接收到的边沿点
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
//最新接收到的平面点
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
//点云全部点
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());


//接收角点
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
{
    timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();
    laserCloudCornerLast->clear();
    pcl::fromROSMsg(*laserCloudCornerLast2,*laserCloudCornerLast);
    newLaserCloudCornerLast = true;
}

//接收平面点
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr&  laserCloudSurfLast2)
{
    timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();
    laserCloudSurfLast->clear();
    pcl::fromROSMsg(*laserCloudSurfLast2,*laserCloudSurfLast);
    newLaserCloudSurfLast = true;
}

//接收点云全部点
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
    timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
    laserCloudFullRes->clear();
    pcl::fromROSMsg(*laserCloudFullRes2,*laserCloudFullRes);
    newLaserCloudFullRes = true;
}

//接收里程计信息
void laserOdometryHandler(const nav_msgs::OdometryConstPtr& laserOdometry)
{
    timeLaserOdometry = laserOdometry->header.stamp.toSec();
    double roll, pitch, yaw;

    //四元数转换为欧拉角
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    //下面代码中四元数的顺序???
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll,pitch,yaw);

    transformSum[0] = -pitch;
    transformSum[1] = -yaw;
    transformSum[2] = roll;

    transformSum[3] = laserOdometry->pose.pose.position.x;
    transformSum[4] = laserOdometry->pose.pose.position.y;
    transformSum[5] = laserOdometry->pose.pose.position.z;

    newLaserOdometry = true;
}

//接收原始IMU消息,只用了横滚和俯仰角
//这里为啥要用原始的imu信息呢???
void imuHandler(const sensor_msgs::ImuConstPtr& imuIn)
{
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    //将四元数转化成tf形式
    tf::quaternionMsgToTF(imuIn->orientation,orientation);
    //获得欧拉角
    tf::Matrix3x3(orientation).getRPY(roll,pitch,yaw);
    //imu最新收到的数据在imu数组中的位置
    imuPointerLast = (imuPointerLast + 1) % imuQueLength;

    imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
}




int main(int argc, char** argv)
{
    ROS_INFO("Local_mapping...");

    ros::init(argc,argv,"local_mapping");
    ros::NodeHandle nh;

    //接收的消息
    ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last",2,laserCloudCornerLastHandler);

    ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last",2,laserCloudSurfLastHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/laser_point_cloud_odom",2,laserCloudFullResHandler);

    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init",5,laserOdometryHandler);

    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data",50,imuHandler);

    //发布的消息
    !!!!!!!!!!!


    return 0;
}