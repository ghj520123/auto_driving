#include<vector>

//ros用
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

//#include"lidar_localization/include/sensor_data/cloud_data.hpp"

//using namespace lidar_localization;

int N_SCANS = 16;//16线？

ros::Publisher pubLidarCloudUp;
ros::Publisher pubLidarCloudDown;

void callback(const sensor_msgs::PointCloud2 ros_cloud){
    //输出世界参考系的坐标

    //接收点云转换为PCL格式
    pcl::PointCloud<pcl::PointXYZ> lidarCloudIn;
    pcl::fromROSMsg(ros_cloud, lidarCloudIn);

    //计数用于循环
    int cloud_size = lidarCloudIn.points.size();
    int count = cloud_size;
    pcl::PointXYZI point;

    //判定各点的线数，按线数保存
    std::vector<pcl::PointCloud<pcl::PointXYZI>> lidarCloudScans(N_SCANS);
    for (int i = 0; i < cloud_size;i++){
        point.x = lidarCloudIn.points[i].x;
        point.y = lidarCloudIn.points[i].y;
        point.z = lidarCloudIn.points[i].z;

        //仰角
        //atan反正切函数
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;
        
        if(N_SCANS==16){
            scanID = int((angle + 15) / 2 + 0.5);
            if(scanID>(N_SCANS-1)||scanID<0){
                count--;
                continue;
            }
        }
        lidarCloudScans[scanID].push_back(point);
    }
    //按1-8，9-16线保存
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarCloudUp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarCloudDown(new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = 0; i < N_SCANS/2;i++){
        *lidarCloudUp += lidarCloudScans[i];
    }
    for (int i = N_SCANS/2; i < N_SCANS;i++){
        *lidarCloudDown += lidarCloudScans[i];
    }

    //pcl转ros输出格式，map是统一坐标系
    sensor_msgs::PointCloud2 lidarCloudUpOutMsg;
    pcl::toROSMsg(*lidarCloudUp, lidarCloudUpOutMsg);
    lidarCloudUpOutMsg.header.stamp = ros_cloud.header.stamp;
    lidarCloudUpOutMsg.header.frame_id = "map";
    pubLidarCloudUp.publish(lidarCloudUpOutMsg);

    sensor_msgs::PointCloud2 lidarCloudDownOutMsg;
    pcl::toROSMsg(*lidarCloudDown, lidarCloudDownOutMsg);
    lidarCloudDownOutMsg.header.stamp = ros_cloud.header.stamp;
    lidarCloudDownOutMsg.header.frame_id = "map";
    pubLidarCloudDown.publish(lidarCloudDownOutMsg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe("/kitti/velo/pointcloud", 100, callback);
    pubLidarCloudUp = nh.advertise<sensor_msgs::PointCloud2>("cloud_up", 100);
    pubLidarCloudDown = nh.advertise<sensor_msgs::PointCloud2>("cloud_down", 100);
    ros::spin();
    return 0;
}