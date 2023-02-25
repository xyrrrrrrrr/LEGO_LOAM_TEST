// This file is an addtional module for LeGO-LOAM
// It is used to transform the point cloud to GPS coordinate system in real time
// author: Xiangyun Rao
// date: 2023.2.22

#include "utility.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <string.h>
#include "std_msgs/String.h"

// create a class to transform the point cloud to GPS coordinate system in real time

class TransformTogps
{
private:

    ros::NodeHandle nh;
    // create a publisher to publish the rotation matrix
    ros::Publisher pubRotationMatrix;
    // create a publisher to publish the transformed(gps) point cloud
    ros::Publisher pubLaserCloudGPS;
    // create a subscriber to subscribe the point cloud(origin)
    ros::Subscriber subLaserCloud;
    // create a subscriber to subscribe the gps data
    ros::Subscriber subGPS;
    // create a point cloud to store the origin point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_pc(new pcl::PointCloud<pcl::PointXYZ>());
    // create a point cloud to store the transformed point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr gps_pc(new pcl::PointCloud<pcl::PointXYZ>());
    // create a point cloud to store the transformed point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudGPS(new pcl::PointCloud<pcl::PointXYZI>());
    // create a point cloud to store the origin point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
    // gps数据队列互斥锁
    std::mutex gps_queue_mutex;
    // gps数据队列
    std::deque<sensor_msgs::JointStateConstPtr> gps_queue;

    // // create 2 arrays to store the gps data (for the first frame and the last frame)
    // double gps[6];
    // double last_gps[6];


public:
    TransformTogps(){
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pointclouddir,*origin_pc)==-1)//*打开点云文件
        {
            PCL_ERROR("Couldn't read file, check file or dir\n");
            return(-1);
        }
    // create a gps subscriber
    subGPS = nh.subscribe("/gpsdemo", 1000, &TransformTogps::gpsHandler, this);      
    // create a publisher to publish the transformed(gps) point cloud  
    pubLaserCloudGPS = nh.advertise<sensor_msgs::PointCloud2>("/gps_pointcloud", 2)           
    }

    // create a function to get the gps data
    void gpsHandler(const sensor_msgs::JointStateConstPtr &gpsMsg){
        //gps数据队列互斥锁
        lock(gps_queue_mutex);
        //gps数据队列
        gps_queue.push_back(gpsMsg);
        if (gps_queue.size() > 100)
            gps_queue.pop_front();
        if (gps_queue.size() < 100)
            return;
        unlock(gps_queue_mutex);
        // calculate the rotation matrix
        double R[3][3];
        string gps_first_string = gps_queue.front()
        string gps_last_string = gps_queue.back()
        // gps string的格式是：data: "0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000", 
        // 六个double型数据对应北方(x)，东方(y)，高度(z)，heading, pitch, roll, 用逗号分隔
        // 读取gps数据
        double gps_first[6];
        double gps_last[6];
        sscanf(gps_first_string, "%lf, %lf, %lf, %lf, %lf, %lf", &gps_first[0], &gps_first[1], &gps_first[2], &gps_first[3], &gps_first[4], &gps_first[5]);
        sscanf(gps_last_string, "%lf, %lf, %lf, %lf, %lf, %lf", &gps_last[0], &gps_last[1], &gps_last[2], &gps_last[3], &gps_last[4], &gps_last[5]);
        // 计算旋转矩阵
        double x_diff = gps_last[0] - gps_first[0];
        double y_diff = gps_last[1] - gps_first[1];
        double z_diff = gps_last[2] - gps_first[2]; 
        double heading_diff = gps_last[3] - gps_first[3];
        double pitch_diff = gps_last[4] - gps_first[4];
        double roll_diff = gps_last[5] - gps_first[5];
        // 在初始坐标系下，小车沿着x轴前进了（x_diff^2+y_diff^2）^0.5米
        double x_init = (x_diff*x_diff+y_diff*y_diff)^0.5;
        double y_init = 0;
        double z_init = z_diff;

        //创建两个向量
        Eigen::Vector3d Vector_init(x_init, y_init, z_init);
        Eigen::Vector3d Vector_gps(x_diff, y_diff, z_diff);

        //创建旋转矩阵、变换矩阵、位移向量
        Eigen::Matrix3d rotationMatrix;
        Eigen::Matrix4d transformMatrix;
        transformMatrix.setIdentity();
        Eigen::Vector3d t(0, 0, 0);

        //求两向量旋转矩阵
        rotationMatrix = Eigen::Quaterniond::FromTwoVectors(Vector_init, Vector_gps).toRotationMatrix();

        //旋转矩阵和平移向量凑成变换矩阵
        transformMatrix.block<3, 3>(0, 0) = rotationMatrix;
        transformMatrix.topRightCorner(3, 1) = t;

        // 将在初始坐标系下的点云转换到gps坐标系下
        pcl::transformPointCloud(*origin_pc, *gps_pc, transformMatrix);


    }

    // create a function to publish the transformed point cloud
    void publishGPSPointCloud(){
        // publish the transformed point cloud
        pcl::toROSMsg(*gps_pc, laserCloudGPS);
        laserCloudGPS->header.stamp = ros::Time().fromSec(timeLaserCloudGPS);
        laserCloudGPS->header.frame_id = "/gps_enu";
        pubLaserCloudGPS.publish(laserCloudGPS);
    }
    




}   