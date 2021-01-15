/*
本代码是从LOAM中剥离出来,用于实现雷达点云数据前后帧融合,维护一个相对稠密的局部地图,
用于之后的障碍物识别和检测

融合方案: 1. 融合最新的固定帧数的雷达点云数据,剔除旧的数据
                    2. 融合固定距离的雷达点云数据

designed by Shubin
*/


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

//点云曲率, 40000为一帧点云中点的最大数量(单线)
float cloudCurvature[40000];
//曲率点对应的序号
int cloudSortInd[40000];
//点是否筛选过标志：0-未筛选过，1-筛选过
int cloudNeighborPicked[40000];
//点分类标号:2-代表曲率很大，1-代表曲率比较大,-1-代表曲率很小，0-曲率比较小(其中1包含了2,0包含了-1,0和1构成了点云全部的点)
int cloudLabel[40000];

//初始化控制变量
const int systemDelay = 20;//弃用前20帧初始数据
int systemInitCount = 0;
bool systemInited = false;

/*****IMU参数设置*****/
//imu时间戳 大于 当前点云时间戳的位置
int imuPointerFront = 0;
//imu最新收到的点在数组中的位置
int imuPointerLast = -1;
//imu循环队列长度
const int imuQueLength = 200;
//点云起始点对应IMU和当前时刻点云对应IMU的姿态,速度和位置
//姿态
float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;
//速度
float imuVeloXStart = 0, imuVeloYStart = 0, imuVeloZStart = 0;
float imuVeloXCur = 0, imuVeloYCur = 0, imuVeloZCur = 0;
//位置
float imuShiftXStart = 0, imuShiftYStart = 0, imuShiftZStart = 0;
float imuShiftXCur = 0, imuShiftYCur = 0, imuShiftZCur = 0;
//点云数据中,当前点云相对于起始点云imu位置和速度变化
float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;
//IMU信息存储
//imu数据时间
double imuTime[imuQueLength] = {0};
//姿态角
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};
//加速度信息
float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};
//速度信息
float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};
//位置
float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};







void ShiftToStartIMU(float pointTime)
{
    //计算相对于imuStart,由于加减速产生的畸变位移(计算结果表示在全局坐标系下)
    //x2 = x1 + v*t + 0.5*a*t^2, 这里计算的就是0.5*a*t^2
    imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
    imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
    imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;
    /********************************************************************************
    Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * delta_Tg
    transfrom from the global frame to the local frame
    *********************************************************************************/
    //将上面的计算结果从全局坐标系转换到imuStart局部坐标系
    //绕y轴旋转(-imuYawStart)，即Ry(yaw).inverse
    //????????旋转角为啥是负的
    //原代码:
    float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
    float y1 = imuShiftFromStartYCur;
    float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;
    //Debug
    // float x1 = cos(imuYawStart) * imuShiftFromStartXCur + sin(imuYawStart) * imuShiftFromStartZCur;
    // float y1 = imuShiftFromStartYCur;
    // float z1 = -sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

    //绕x轴旋转(-imuPitchStart)，即Rx(pitch).inverse
    float x2 = x1;
    float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
    float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

    //绕z轴旋转(-imuRollStart)，即Rz(pitch).inverse
    //计算结果转换到局部坐标系下
    imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
    imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
    imuShiftFromStartZCur = z2;

}

void VeloToStartIMU()
{
    //计算相对于imuStart,由于加减速产生的速度畸变(计算结果表示在全局坐标系下)
    imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
    imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
    imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;
    /********************************************************************************
    Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * delta_Vg
    transfrom from the global frame to the local frame
    *********************************************************************************/
    //绕y轴旋转(-imuYawStart)，即Ry(yaw).inverse
    //原代码
    float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
    float y1 = imuVeloFromStartYCur;
    float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;
    //Debug
    // float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
    // float y1 = imuVeloFromStartYCur;
    // float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

    //绕x轴旋转(-imuPitchStart)，即Rx(pitch).inverse
    float x2 = x1;
    float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
    float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

    //绕z轴旋转(-imuRollStart)，即Rz(pitch).inverse
    imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
    imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
    imuVeloFromStartZCur = z2;

}

//传入的点是在imuShiftCur/当前lidar系下的
void TransformToStartIMU(PointType *p)
{
    /*** 将点从当前坐标系->全局坐标系 ***/
    //绕z轴旋转(imuRollCur)
    float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
    float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
    float z1 = p->z;
    //Debug
    // float x1 = cos(imuRollCur) * p->x + sin(imuRollCur) * p->y;
    // float y1 = -sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
    // float z1 = p->z;

    //绕x轴旋转(imuPitchCur)
    float x2 = x1;
    float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
    float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;
    //Debug
    // float x2 = x1;
    // float y2 = cos(imuPitchCur) * y1 + sin(imuPitchCur) * z1;
    // float z2 = -sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

    //绕y轴旋转(imuYawCur)
    float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
    float y3 = y2;
    float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

    /*** 再将点从全局坐标系->初始坐标系(即完成了畸变矫正) ***/
    //绕y轴旋转(-imuYawStart)
    float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
    float y4 = y3;
    float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;
    //Debug
    // float x4 = cos(imuYawStart) * x3 + sin(imuYawStart) * z3;
    // float y4 = y3;
    // float z4 = -sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

    //绕x轴旋转(-imuPitchStart)
    float x5 = x4;
    float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
    float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

    //绕z轴旋转(-imuRollStart)，然后叠加平移量
    //当前帧点的坐标从当前imu系->初始imu系,完成畸变矫正
    p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
    p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
    p->z = z5 + imuShiftFromStartZCur;

    return;
}



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
        //计算点云中点的相对扫描时间
        float relTime = (ori - startOri)/(endOri - startOri);
        //点强度=线号+点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点的相对时间）,匀速扫描：根据当前扫描的角度和扫描周期计算相对扫描起始位置的时间
        point_.intensity = scanID + scanPeriod*relTime;
        if(imuPointerLast > 0)
        {
            //如果收到IMU数据,使用IMU矫正点云畸变
            //计算点在当前周期内的时间
            float pointTime = relTime * scanPeriod;
            //寻找是否有点云的时间戳 小于 IMU的时间戳
            while (imuPointerFront != imuPointerLast)
            {
                //点时间=点云时间+周期时间
                if(timeScanCur + pointTime < imuTime[imuPointerFront])
                {
                    //当有点云数据的时间 小于 imu的时间戳时,break(因为imu的时间戳范围要包含点云时间戳范围)
                    break;
                }
                //找到 小于 当前点时间的最大时间imu数据位置
                imuPointerFront = (imuPointerFront + 1) % imuQueLength;
            }

            if(timeScanCur + pointTime > imuTime[imuPointerFront])
            {
                //imuPointerFront是小于且最接近当前点时间的imu位置索引
                //把imuPointerFront位置的imu数据当做当前点的imu状态
                imuRollCur = imuRoll[imuPointerFront];
                imuPitchCur = imuPitch[imuPointerFront];
                imuYawCur = imuYaw[imuPointerFront];

                imuVeloXCur = imuVeloX[imuPointerFront];
                imuVeloYCur = imuVeloY[imuPointerFront];
                imuVeloZCur = imuVeloZ[imuPointerFront];

                imuShiftXCur = imuShiftX[imuPointerFront];
                imuShiftYCur = imuShiftY[imuPointerFront];
                imuShiftZCur = imuShiftZ[imuPointerFront];
            }
            else//当前点的时间 小于 imu时间,即当前点之前没有imu数据,imu数据在当前点之后出现
            {
                int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                //通过线性插值的方式计算出当前点对应的imu状态,假设imu状态在这段时间内是均匀变化的
                /*     线性插值的结论:直线上有两点a和b,对应的值分别是f(a)和f(b),函数值随自变量均匀变化,
                则直线上任意一点x对应的函数值为: f(x) =  f(a) * (b-x) / (b-a) + f(b) * (x-a) / (b-a)    */
                float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                //插值计算得到的当前点对应的imu状态
                //姿态角
                imuRollCur = imuRoll[imuPointerFront]*ratioFront + imuRoll[imuPointerBack]*ratioBack;
                imuPitchCur = imuPitch[imuPointerFront]*ratioFront + imuPitch[imuPointerBack]*ratioBack;
                //航向角由于存在奇异性,因此需要进一步判断处理
                //?????为什么要求 | imuYaw[imuPointerFront] - imuYaw[imuPointerBack] |  <= pi
                if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) 
                {
                    imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
                } 
                else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI)
                {
                    imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
                } 
                else
                {
                    imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
                }
                //速度
                imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
                imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
                imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;
                //位置
                imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
                imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
                imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;

            }

            //如果是第一个点,记录这个点对应的imu状态为imu初始状态
            if(i == 0)
            {
                imuRollStart = imuRollCur;
                imuPitchStart = imuPitchCur;
                imuYawStart = imuYawCur;

                imuVeloXStart = imuVeloXCur;
                imuVeloYStart = imuVeloYCur;
                imuVeloZStart = imuVeloZCur;

                imuShiftXStart = imuShiftXCur;
                imuShiftYStart = imuShiftYCur;
                imuShiftZStart = imuShiftZCur;
            }
            else//不是第一个点,计算当前点对应的imu相对第一个点对应imu的运动,对当前点进行运动补偿,消除运动畸变
            {
                ShiftToStartIMU(pointTime);
                VeloToStartIMU();
                //这里的&表示取地址
                TransformToStartIMU(&point_);
            }
         //收到imu消息时,利用imu信息矫正运动畸变

        laserCloudScans[scanID].push_back(point_);
        }

        //获得有效范围内的点的数量
        cloudSize = count;
        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
        //将所有的点按照线号从小到大放入一个容器
        for (int i = 0; i < N_SCANS; i++)
        {
            *laserCloud += laserCloudScans[i];
        }
        int scanCount = -1;//???
        //使用每个点的前后五个点计算曲率，因此前五个与最后五个点跳过
        for (int i = 5; i < cloudSize - 5; i++) 
        {
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x 
                        + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x 
                        + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x 
                        + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                        + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                        + laserCloud->points[i + 5].x;
            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y 
                        + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y 
                        + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y 
                        + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                        + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                        + laserCloud->points[i + 5].y;
            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z 
                        + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z 
                        + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z 
                        + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                        + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                        + laserCloud->points[i + 5].z;
            //曲率计算
            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            //记录曲率点的索引
            cloudSortInd[i] = i;
            //初始时，点全未筛选过(0-未筛选过，1-筛选过)
            cloudNeighborPicked[i] = 0;
            //初始化为less flat点(曲率比较小的点)
            cloudLabel[i] = 0;
            //每个scan，只有第一个符合的点会进来，因为每个scan的点都在一起存放
            if (int(laserCloud->points[i].intensity) != scanCount) 
            {
                //控制每个scan只进入第一个点
                scanCount = int(laserCloud->points[i].intensity);
                //曲率只取同一个scan计算出来的，跨scan计算的曲率非法，排除，也即排除每个scan的前后五个点
                if (scanCount > 0 && scanCount < N_SCANS)
                {
                    scanStartInd[scanCount] = i + 5;
                    scanEndInd[scanCount - 1] = i - 5;
                }
            }
        }//计算每个scan中点的曲率

        //第一个scan曲率点有效点序从第5个开始，最后一个激光线结束点序size-5
        scanStartInd[0] = 5;
        scanEndInd.back() = cloudSize - 5;

        //挑选点，排除容易被斜面挡住的点以及离群点，有些点容易被斜面挡住，而离群点可能出现带有偶然性，这些情况都可能导致前后两次扫描不能被同时看到
        for(int i = 5; i < cloudSize - 6; i++)
        {
            //相邻点的三维坐标差值
            float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
            float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
            float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
            //相邻点空间距离的平方
            float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

            //如果两个点之间的距离的平方大于0.1
            if(diff > 0.1)
            {
                //分别计算这两点的深度
                float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
                                        laserCloud->points[i].y * laserCloud->points[i].y +
                                        laserCloud->points[i].z * laserCloud->points[i].z);
                float depth2 = sqrt(laserCloud->points[i+1].x * laserCloud->points[i+1].x + 
                                        laserCloud->points[i+1].y * laserCloud->points[i+1].y +
                                        laserCloud->points[i+1].z * laserCloud->points[i+1].z);
                if(depth1 > depth2)
                {
                    //按照两点的深度的比例，将深度较大的点拉回后计算距离
                    diffX = laserCloud->points[i+1].x - laserCloud->points[i].x * depth2 / depth1;
                    diffY = laserCloud->points[i+1].y - laserCloud->points[i].y * depth2 / depth1;
                    diffZ = laserCloud->points[i+1].z - laserCloud->points[i].z * depth2 / depth1;
                    //计算两点之间形成的弧度值,如果弧度值比较小,说明斜面比较陡,深度变化剧烈,点处在近似与激光束平行的斜面上
                    if(sqrt(diffX * diffX + diffY * diffY + diffZ *diffZ) / depth2 < 0.1)
                    {
                        //该点及前面五个点（大致都在斜面上）全部置为筛选过
                        //筛选"过"意味着什么,意味着"弃用"
                        cloudNeighborPicked[i - 5] = 1;
                        cloudNeighborPicked[i - 4] = 1;
                        cloudNeighborPicked[i - 3] = 1;
                        cloudNeighborPicked[i - 2] = 1;
                        cloudNeighborPicked[i - 1] = 1;
                        cloudNeighborPicked[i] = 1;
                    }
                }
                else//depth1 < depth2, 与上面操作类似
                {
                    diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
                    diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
                    diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;
                    if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1)
                    {
                        cloudNeighborPicked[i + 1] = 1;
                        cloudNeighborPicked[i + 2] = 1;
                        cloudNeighborPicked[i + 3] = 1;
                        cloudNeighborPicked[i + 4] = 1;
                        cloudNeighborPicked[i + 5] = 1;
                        cloudNeighborPicked[i + 6] = 1;
                    }
                }
            }
            //与前一个点的距离
            float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
            float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
            float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
            //与前一个点的距离平方和
            float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
            //i点的深度平方和
            float dis = laserCloud->points[i].x * laserCloud->points[i].x
                                + laserCloud->points[i].y * laserCloud->points[i].y
                                + laserCloud->points[i].z * laserCloud->points[i].z;
            //与前后点的距离平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
            if (diff > 0.0002 * dis && diff2 > 0.0002 * dis)
            {
                cloudNeighborPicked[i] = 1;
            }

        }//挑选点,筛掉野值点和斜面上的点

    }//循环遍历当前帧中的点

    






    



    
    
    


 return ;
}



 

 




int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_process");
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points",2,laserCloudHandler);

    
    ros::spin();
    return 0;
}

