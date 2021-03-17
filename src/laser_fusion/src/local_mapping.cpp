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
//答:LOAM维护点云地图的中心位置
int laserCloudCenWidth = 10;
int laserCloudCenHeight = 5;
int laserCloudCenDepth = 10;

//LOAM中维护的点云地图的大小,在空间中由很多个立方体构成
const int laserCloudWidth = 21;
const int laserCloudHeight = 11;
const int laserCloudDepth = 21;
//维护点云的立方体数量: 21*11*21=4851个
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

//lidar视域范围内(FOV)的点云集索引
int laserCloudValidInd[125];
//lidar周围的点云集索引
int laserCloudSurroundInd[125];

//odometry计算得到的到世界坐标系下的转移矩阵
float transformSum[6] = {0};
//转移增量，只使用了后三个平移增量
float transformIncre[6] = {0};

//以起始位置为原点的世界坐标系下的转换矩阵（猜测与调整的对象）
float transformTobeMapped[6] = {0};
//存放mapping之前的Odometry计算的世界坐标系的转换矩阵（注：低频量，不一定与transformSum一样）
float transformBefMapped[6] = {0};
//存放mapping之后的经过mapping微调之后的转换矩阵
float transformAftMapped[6] = {0};


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
//存放当前收到的下采样之后的角点(in the local frame)
pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
//存放当前收到的下采样之后的面点(in the local frame)
pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
//存放当前收到的角点，作为下采样的数据源
pcl::PointCloud<PointType>::Ptr laserCloudCornerStack2(new pcl::PointCloud<PointType>());
//存放当前收到的面点，作为下采样的数据源
pcl::PointCloud<PointType>::Ptr laserCloudSurfStack2(new pcl::PointCloud<PointType>());
//map中提取的匹配使用的角点
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
//map中提取的匹配使用的面点
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());

//array都是以50米为单位的立方体地图，运行过程中会一直保存
//存放角点的cube
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
//存放面点的cube
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];
//中间变量，存放下采样过的角点
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];
//中间变量，存放下采样过的面点
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];

//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());



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

//基于匀速运动模型
void transformAssociateToMap()
{
    float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
            - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
    float y1 = transformBefMapped[4] - transformSum[4];
    float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
            + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

    float x2 = x1;
    float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
    float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

    //平移增量
    transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
    transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
    transformIncre[5] = z2;

    float sbcx = sin(transformSum[0]);
    float cbcx = cos(transformSum[0]);
    float sbcy = sin(transformSum[1]);
    float cbcy = cos(transformSum[1]);
    float sbcz = sin(transformSum[2]);
    float cbcz = cos(transformSum[2]);

    float sblx = sin(transformBefMapped[0]);
    float cblx = cos(transformBefMapped[0]);
    float sbly = sin(transformBefMapped[1]);
    float cbly = cos(transformBefMapped[1]);
    float sblz = sin(transformBefMapped[2]);
    float cblz = cos(transformBefMapped[2]);

    float salx = sin(transformAftMapped[0]);
    float calx = cos(transformAftMapped[0]);
    float saly = sin(transformAftMapped[1]);
    float caly = cos(transformAftMapped[1]);
    float salz = sin(transformAftMapped[2]);
    float calz = cos(transformAftMapped[2]);

    float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
                - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
    transformTobeMapped[0] = -asin(srx);

    float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
    float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
    transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]), 
                                    crycrx / cos(transformTobeMapped[0]));
    
    float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
    float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
    transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]), 
                                    crzcrx / cos(transformTobeMapped[0]));

    x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
    y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
    z1 = transformIncre[5];

    x2 = x1;
    y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    transformTobeMapped[3] = transformAftMapped[3] 
                            - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
    transformTobeMapped[4] = transformAftMapped[4] - y2;
    transformTobeMapped[5] = transformAftMapped[5] 
                            - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);

}

//根据调整计算后的转移矩阵，将点投影到世界坐标系下
void pointAssociateToMap(PointType const* const pi, PointType * const po)
{
    //绕z轴旋转（transformTobeMapped[2]）
    float x1 = cos(transformTobeMapped[2]) * pi->x
            - sin(transformTobeMapped[2]) * pi->y;
    float y1 = sin(transformTobeMapped[2]) * pi->x
            + cos(transformTobeMapped[2]) * pi->y;
    float z1 = pi->z;

    //绕x轴旋转（transformTobeMapped[0]）
    float x2 = x1;
    float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    //绕y轴旋转（transformTobeMapped[1]），再平移
    po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
            + transformTobeMapped[3];
    po->y = y2 + transformTobeMapped[4];
    po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
            + transformTobeMapped[5];
    po->intensity = pi->intensity;

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
    ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround",1);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_full",2);

    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init",5);
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/rslidar_init";
    odomAftMapped.child_frame_id = "/aft_mapped";

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform aftMappedTrans;
    aftMappedTrans.frame_id_ = "/rslidar_init";
    aftMappedTrans.child_frame_id_ = "/aft_mapped";

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel, pointProj, coeff;

    cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));//这里为啥初始化成-1???
    cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

    //设置退化标志
    bool isDegenerate = false;
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

    //创建VoxelGrid滤波器（体素栅格滤波器）
    //体素栅格滤波器作用:PCL是实现的VoxelGrid类通过输入的点云数据创建一个三维体素栅格，
    //容纳后每个体素内用体素中所有点的重心来近似显示体素中其他点，这样该体素内所有点都用一个重心点最终表示
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    //设置体素大小
    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

    pcl::VoxelGrid<PointType> downSizeFilterMap;
    downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);
    
    //指针初始化
    for (int i = 0; i < laserCloudNum; i++) 
    {
        laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudCornerArray2[i].reset(new pcl::PointCloud<PointType>()); 
        laserCloudSurfArray2[i].reset(new pcl::PointCloud<PointType>());
    }

    int frameCount = stackFrameNum - 1;//0
    int mapFrameCount = mapFrameNum - 1; //4
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status)
    {
        ros::spinOnce();
        //如果接收到同一时刻的新消息
        if(newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry &&
            fabs(timeLaserCloudCornerLast - timeLaserOdometry)<0.005 &&
            fabs(timeLaserCloudSurfLast - timeLaserOdometry)<0.005 &&
            fabs(timeLaserCloudFullRes - timeLaserOdometry)<0.005)
        {
            newLaserCloudCornerLast = false;
            newLaserCloudSurfLast = false;
            newLaserCloudFullRes = false;
            newLaserOdometry = false;

            frameCount++;
            //控制跳帧数，>=这里实际并没有跳帧
            if(frameCount >= stackFrameNum)
            {
                //获取世界坐标系转换矩阵
                transformAssociateToMap();

                //将最新接收到的平面点和边沿点进行旋转平移转换到世界坐标系下(这里和后面的逆转换应无必要)
                int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
                for(int i = 0;i< laserCloudCornerLastNum;i++)
                {
                    pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
                    laserCloudCornerStack2->push_back(pointSel);
                }

                int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
                for (int i = 0; i < laserCloudSurfLastNum; i++)
                {
                    pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
                    laserCloudSurfStack2->push_back(pointSel);
                }
            }

            if(frameCount >= stackFrameNum)
            {
                frameCount = 0;
                PointType pointOnYAxis;
                pointOnYAxis.x = 0.0;
                pointOnYAxis.y = 10.0;
                pointOnYAxis.z = 0.0;
                //获取y方向上10米高位置的点在世界坐标系下的坐标
                pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);
                
                //立方体中点在世界坐标系下的（原点）位置, 以cube为单位
                //过半取一（以50米进行四舍五入的效果），由于数组下标只能为正数，而地图可能建立在原点前后，因此
                //每一维偏移一个laserCloudCenWidth（该值会动态调整，以使得数组利用最大化，初始值为该维数组长度1/2）的量
                int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;
                int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight;
                int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;
                
                //由于计算机求余是向零取整，为了保证求余结果统一向左偏移一个单位
                if (transformTobeMapped[3] + 25.0 < 0) centerCubeI--;
                if (transformTobeMapped[4] + 25.0 < 0) centerCubeJ--;
                if (transformTobeMapped[5] + 25.0 < 0) centerCubeK--;
                
                //调整之后取值范围:(长)3 < centerCubeI < 18， (高)3 < centerCubeJ < 8, (宽)3 < centerCubeK < 18
                //如果处于下边界，表明地图向负方向延伸的可能性比较大，则循环移位，将数组中心点向上边界调整一个单位
                while (centerCubeI < 3)
                {
                    for (int j = 0; j < laserCloudHeight; j++) 
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            //实现一次循环移位效果
                            int i = laserCloudWidth - 1;
                            //指针赋值，保存最后一个指针位置
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];//that's [i + 21 * j + 231 * k]
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            //循环移位，I维度上依次后移
                            for (; i >= 1; i--)
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCornerArray[i - 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            }
                            //将开始点赋值为最后一个点
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }

                    centerCubeI++;
                    laserCloudCenWidth++;
                }
                //如果处于上边界，表明地图向正方向延伸的可能性比较大，则循环移位，将数组中心点向下边界调整一个单位
                while (centerCubeI >= laserCloudWidth - 3)//18
                {
                    for (int j = 0; j < laserCloudHeight; j++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int i = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            //I维度上依次前移
                            for (; i < laserCloudWidth - 1; i++) 
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCornerArray[i + 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }
                    centerCubeI--;
                    laserCloudCenWidth--;
                }

                //J方向上调整, 3 < centerCubeJ < 8
                while (centerCubeJ < 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++)
                        {
                            int j = laserCloudHeight - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            //J维度上，依次后移
                            for (; j >= 1; j--) 
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCornerArray[i + laserCloudWidth*(j - 1) + laserCloudWidth * laserCloudHeight*k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }
                    centerCubeJ++;
                    laserCloudCenHeight++;
                }
                while (centerCubeJ >= laserCloudHeight - 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++)
                    {
                        for (int k = 0; k < laserCloudDepth; k++) 
                        {
                            int j = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            //J维度上一次前移
                            for (; j < laserCloudHeight - 1; j++) 
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCornerArray[i + laserCloudWidth*(j + 1) + laserCloudWidth * laserCloudHeight*k];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }
                    centerCubeJ--;
                    laserCloudCenHeight--;
                }

                //K方向调整,3 < K < 18
                while (centerCubeK < 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++) 
                    {
                        for (int j = 0; j < laserCloudHeight; j++) 
                        {
                            int k = laserCloudDepth - 1;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            //K维度上依次后移
                            for (; k >= 1; k--) 
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k - 1)];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }
                    centerCubeK++;
                    laserCloudCenDepth++;
                }
                while (centerCubeK >= laserCloudDepth - 3)
                {
                    for (int i = 0; i < laserCloudWidth; i++) 
                    {
                        for (int j = 0; j < laserCloudHeight; j++) 
                        {
                            int k = 0;
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                            //K维度上依次前移
                            for (; k < laserCloudDepth - 1; k++) 
                            {
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k + 1)];
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
                            }
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeCornerPointer;
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
                            laserCloudCubeSurfPointer;
                            laserCloudCubeCornerPointer->clear();
                            laserCloudCubeSurfPointer->clear();
                        }
                    }
                    centerCubeK--;
                    laserCloudCenDepth--;
                }

                int laserCloudValidNum = 0;
                int laserCloudSurroundNum = 0;

                //在每一维附近的5个cube(前后各两个,中间一个)中进行查找,一共是5*5*5=125个cube,即为submap
                //在submap里进一步筛选在视域范围内的cube
                //submap中的点是表示在世界坐标系下的
                for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
                {
                    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
                    {
                        for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
                        {
                            if(i >= 0 && i < laserCloudWidth &&  j >= 0 && j < laserCloudHeight && k >= 0 && k < laserCloudDepth)
                            {
                                //索引合理,换算成实际比例
                                //在世界坐标系下的坐标????
                                float centerX = 50.0 * (i - laserCloudCenWidth);
                                float centerY = 50.0 * (j - laserCloudCenHeight);
                                float centerZ = 50.0 * (k - laserCloudCenDepth);

                                bool isInLaserFOV = false;//判断是否在lidar视线范围的标志（Field of View）
                                //8个顶点坐标
                                for(int ii = -1;ii <= 1;ii += 2)
                                {
                                    for(int jj = -1; jj <= 1; jj += 2)
                                    {
                                        for(int kk = -1; kk <= 1; kk += 2)
                                        {
                                            float cornerX = centerX + 25.0 * ii;
                                            float cornerY = centerY + 25.0 * jj;
                                            float cornerZ = centerZ + 25.0 * kk;

                                            //原点到顶点距离的平方和
                                            float squaredSide1 = (transformTobeMapped[3] - cornerX) 
                                                                                    * (transformTobeMapped[3] - cornerX) 
                                                                                    + (transformTobeMapped[4] - cornerY) 
                                                                                    * (transformTobeMapped[4] - cornerY)
                                                                                    + (transformTobeMapped[5] - cornerZ) 
                                                                                    * (transformTobeMapped[5] - cornerZ);
                                                                                    
                                            //pointOnYAxis到顶点距离的平方和
                                            float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX) 
                                                                                    + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
                                                                                    + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);
                                            
                                            float check1 = 100.0 + squaredSide1 - squaredSide2
                                                                        - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                                            float check2 = 100.0 + squaredSide1 - squaredSide2
                                                                        + 10.0 * sqrt(3.0) * sqrt(squaredSide1);
                                            
                                            if(check1 < 0 && check2 > 0)
                                            {
                                                isInLaserFOV = true;
                                            }
                                        }
                                    }
                                }

                                //记住视域范围内的cube索引，匹配用
                                if(isInLaserFOV)
                                {
                                    laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j  + laserCloudWidth * laserCloudHeight * k;
                                    laserCloudValidNum++;
                                }
                                //记住附近所有cube的索引，显示用
                                laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j 
                                                                            + laserCloudWidth * laserCloudHeight * k;
                                laserCloudSurroundNum++;

                            }
                        }
                    }
                }

                laserCloudCornerFromMap->clear();
                laserCloudSurfFromMap->clear();
                //构建特征点地图，查找匹配使用
                for (int i = 0; i < laserCloudValidNum; i++)
                {
                    *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
                    *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
                }
                int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
                int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();
                !!!!!!!!!!!!





                


            }


        }



    }




    return 0;
}