//Laser_odometry
//该线程作用:(1)实现lidar里程计的功能;(2)完成前后帧点云数据的融合

#include <cmath>
#include </home/sishubin/SLAMcodes/My_Local_Repo/Laser_fusion/src/laser_fusion/include/laser_fusion/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


//一个点云周期
const float scanPeriod = 0.1;

//跳帧数，控制发给laserMapping的频率
const int skipFrameNum = 1;
bool systemInited = false;

//时间戳信息
double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;
double timeImuTrans = 0;

//消息接收标志
bool newCornerPointsSharp = false;
bool newCornerPointsLessSharp = false;
bool newSurfPointsFlat = false;
bool newSurfPointsLessFlat = false;
bool newLaserCloudFullRes = false;
bool newImuTrans = false;

//receive sharp points
pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
//receive less sharp points
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
//receive flat points
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
//receive less flat points
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());
//receive all points
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
//receive imu info
pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>());
//less sharp points of last frame
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
//less flat points of last frame
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
//kd-tree built by less sharp points of last frame
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<PointType>());
//kd-tree built by less flat points of last frame
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<PointType>());
//保存前一个节点发过来的未经处理过的特征点
pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());


int laserCloudCornerLastNum;//默认初始化为0
int laserCloudSurfLastNum;

//点云第一个点对应的RPY
float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
//点云最后一个点对应的RPY
float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
//点云最后一个点相对于第一个点的畸变位移
float imuShiftFromStartX = 0, imuShiftFromStartY = 0, imuShiftFromStartZ = 0;
//点云最后一个点相对于第一个点的畸变速度
float imuVeloFromStartX = 0, imuVeloFromStartY = 0, imuVeloFromStartZ = 0;

//当前帧相对上一帧的状态转移量，in the local frame
//存放的参数依次是:俯仰角, , 横滚角和位移XYZ
float transform[6] = {0};
//当前帧相对于第一帧的状态转移量，in the global frame
float transformSum[6] = {0};


void laserCloudSharpHandler(const sensor_msgs::PointCloud2::ConstPtr& cornerPointsSharp2)
{
    timeCornerPointsSharp = cornerPointsSharp2->header.stamp.toSec();
    cornerPointsSharp->clear();
    //将点云的ros消息转换成pcl格式
    pcl::fromROSMsg(*cornerPointsSharp2,*cornerPointsSharp);
    std::vector<int>indices;
    //去除Nan点
    pcl::removeNaNFromPointCloud(*cornerPointsSharp,*cornerPointsSharp,indices);
    newCornerPointsSharp = true;
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2::ConstPtr& cornerPointsLessSharp2)
{
    timeCornerPointsLessSharp = cornerPointsLessSharp2->header.stamp.toSec();
    cornerPointsLessSharp->clear();
    pcl::fromROSMsg(*cornerPointsLessSharp2, *cornerPointsLessSharp);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsLessSharp,*cornerPointsLessSharp, indices);
    newCornerPointsLessSharp = true;
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2::ConstPtr& surfPointsFlat2)
{
    timeSurfPointsFlat = surfPointsFlat2->header.stamp.toSec();
    surfPointsFlat->clear();
    pcl::fromROSMsg(*surfPointsFlat2, *surfPointsFlat);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsFlat,*surfPointsFlat, indices);
    newSurfPointsFlat = true;
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2::ConstPtr& surfPointsLessFlat2)
{
    timeSurfPointsLessFlat = surfPointsLessFlat2->header.stamp.toSec();
    surfPointsLessFlat->clear();
    pcl::fromROSMsg(*surfPointsLessFlat2, *surfPointsLessFlat);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsLessFlat,*surfPointsLessFlat, indices);
    newSurfPointsLessFlat = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloudFullRes2)
{
    timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
    laserCloudFullRes->clear();
    pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudFullRes,*laserCloudFullRes, indices);
    newLaserCloudFullRes = true;
}

void imuTransHandler(const sensor_msgs::PointCloud2::ConstPtr& imuTrans2)
{
    timeImuTrans = imuTrans2->header.stamp.toSec();

    imuTrans->clear();
    pcl::fromROSMsg(*imuTrans2, *imuTrans);

    //根据接收到的消息提取imu信息
    imuPitchStart = imuTrans->points[0].x;
    imuYawStart = imuTrans->points[0].y;
    imuRollStart = imuTrans->points[0].z;

    imuPitchLast = imuTrans->points[1].x;
    imuYawLast = imuTrans->points[1].y;
    imuRollLast = imuTrans->points[1].z;

    imuShiftFromStartX = imuTrans->points[2].x;
    imuShiftFromStartY = imuTrans->points[2].y;
    imuShiftFromStartZ = imuTrans->points[2].z;

    imuVeloFromStartX = imuTrans->points[3].x;
    imuVeloFromStartY = imuTrans->points[3].y;
    imuVeloFromStartZ = imuTrans->points[3].z;

    newImuTrans = true;
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"laser_odometry");
    ros::NodeHandle nh;

    //订阅scan_process发布的/laser_cloud_sharp话题,调用laserCloudSharpHandler函数
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp",2,laserCloudSharpHandler);
    //订阅scan_process发布的/laser_cloud_less_sharp话题,调用laserCloudLessSharpHandler函数
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp",2,laserCloudLessSharpHandler);
    //订阅scan_process发布的/laser_cloud_flat话题,调用laserCloudFlatHandler函数
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2, laserCloudFlatHandler);
    //订阅scan_process发布的/laser_cloud_less_flat话题,调用laserCloudLessFlatHandler函数
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2, laserCloudLessFlatHandler);
    //订阅scan_process发布的所有点云信息
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2> ("/laser_point_cloud", 2, laserCloudFullResHandler);
    //订阅scan_process发布的/imu_trans消息
    ros::Subscriber subImuTrans = nh.subscribe<sensor_msgs::PointCloud2> ("/imu_trans", 5, imuTransHandler);

    //发布消息
    //点云信息
    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2> ("/laser_point_cloud_odom", 2);
    //里程计信息
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);

    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "rslidar_init";
    laserOdometry.child_frame_id = "laser_odom";

    tf::TransformBroadcaster tfBroadcaster;//坐标变换的发布器
    tf::StampedTransform laserOdometryTrans;//记录里程计的姿态变换
    laserOdometryTrans.frame_id_ = "rslidar_init";
    laserOdometryTrans.child_frame_id_ = "laser_odom";

    std::vector<int> pointSearchInd;//搜索到的点序
    std::vector<float> pointSearchSqDis;//搜索到的点平方距离

    //pointOri:原始点; pointSel:选中的特征点;
    // tripod1, tripod2, tripod3:特征点的对应点
    PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

    //退化标志
    bool isDegenerate = false;

    int frameCount = skipFrameNum;

    ros::Rate rate(100);

    bool status = ros::ok();

    while(status)
    {
        //查看topic缓存区是否存在消息,有消息就传入回调函数,没有的话则继续向后执行
        ros::spinOnce();
        //fabs()函数:求绝对值
        if(newCornerPointsSharp && newCornerPointsLessSharp && newSurfPointsFlat && 
            newSurfPointsLessFlat && newLaserCloudFullRes && newImuTrans &&
            fabs(timeCornerPointsSharp - timeSurfPointsLessFlat) < 0.005 &&
            fabs(timeCornerPointsLessSharp - timeSurfPointsLessFlat) < 0.005 &&
            fabs(timeSurfPointsFlat - timeSurfPointsLessFlat) < 0.005 &&
            fabs(timeLaserCloudFullRes - timeSurfPointsLessFlat) < 0.005 &&
            fabs(timeImuTrans - timeSurfPointsLessFlat) < 0.005)
        {
            //同步作用，确保同时收到同一个点云的特征点以及IMU信息才进入
            newCornerPointsSharp = false;
            newCornerPointsLessSharp = false;
            newSurfPointsFlat = false;
            newSurfPointsLessFlat = false;
            newLaserCloudFullRes = false;
            newImuTrans = false;

            //将第一帧点云数据发送给laserMapping,从下一个点云数据开始处理
            if (!systemInited) 
            {
                //将cornerPointsLessSharp与laserCloudCornerLast交换,目的保存cornerPointsLessSharp的值下轮使用
                //需要交换,而不能简单地进行将cornerPointsLessSharp赋值给laserCloudCornerLast操作
                pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
                cornerPointsLessSharp = laserCloudCornerLast;
                laserCloudCornerLast = laserCloudTemp;
                //将surfPointLessFlat与laserCloudSurfLast交换，目的保存surfPointsLessFlat的值下轮使用
                laserCloudTemp = surfPointsLessFlat;
                surfPointsLessFlat = laserCloudSurfLast;
                laserCloudSurfLast = laserCloudTemp;
                //构建kd-tree
                kdtreeCornerLast->setInputCloud(laserCloudCornerLast);//角点
                kdtreeSurfLast->setInputCloud(laserCloudSurfLast);//面点
                //将角点和面点分别发送给laserMapping
                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast,laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "rslidar";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "rslidar";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
                
                //记住原点的横滚角和俯仰角,?????航向角呢
                transformSum[0] += imuPitchStart;
                transformSum[2] += imuRollStart;

                systemInited = true;
                continue;
            }
            
            //T平移量的初值赋值为加减速的位移量，为其梯度下降的方向（沿用上次转换的T（一个sweep匀速模型），同时在其基础上减去匀速运动位移，即只考虑加减速的位移量）
            //??????没看懂
            transform[3] -= imuVeloFromStartX * scanPeriod;
            transform[4] -= imuVeloFromStartY * scanPeriod;
            transform[5] -= imuVeloFromStartZ * scanPeriod;

            if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) 
            {
                //上一帧有足够多的角点和面点做点云配准
                std::vector<int> indices;
                //在收到消息时调用的回调函数不是去过一次Nan点了么????是否多余
                pcl::removeNaNFromPointCloud(*cornerPointsSharp,*cornerPointsSharp, indices);
                //当前帧中角点和面点的数量
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();
                //LM优化,最多迭代25次
                for (int iterCount = 0; iterCount < 25; iterCount++)
                {
                    laserCloudOri->clear();
                    coeffSel->clear();
                    //处理当前点云中的曲率最大的特征点,从上个点云中曲率比较大的特征点中找两个最近距离点
                    //一个点使用kd-tree查找，另一个根据找到的点在其相邻线找另外一个最近距离的点
                    for(int i = 0; i < cornerPointsSharpNum; i++)
                    {
                        TransformToStart(&cornerPointsSharp->points[i], &pointSel);


                    }

                }


            }


        }
    }



    ROS_INFO("Laser_odometry...");

    ros::spin();
    return 0;
}