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

//控制处理得到的点云map，每隔几次publich给rviz显示
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





int main()
{


    return 0;
}