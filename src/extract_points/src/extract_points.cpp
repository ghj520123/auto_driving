#include<time.h>
#include<vector>
//ros
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
//tf
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>
//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pubLidarCloudGround;
ros::Publisher pubLidarCloudEdge;
ros::Publisher pubLidarCloudPlane;
ros::Publisher pubLaserCloudall_01;

void callback(const sensor_msgs::PointCloud2 ros_cloud) {
    //时钟
    clock_t clock_start, clock_end;
    clock_start = clock();

    //输出世界参考系的坐标

    //创建一个tf发布对象
    static tf::TransformBroadcaster br;
    //创建一个tf对象
    tf::Transform transform;
    //创建一个四元数
    tf::Quaternion q;
    //设置map_child在map坐标系下的坐标原点
    transform.setOrigin(tf::Vector3(0, 0, 0));
    //设置四元数的值  （这里可以使用q.setRPY(0,1,msg->theta)）
    q.setW(1);
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    //设置map_child相对于map坐标系的旋转角度
    transform.setRotation(q);
    //发送变换信息
    br.sendTransform(tf::StampedTransform(transform, ros_cloud.header.stamp, "map", "map_child"));

    //输入的点云信息
    pcl::PointCloud<pcl::PointXYZ> lidarCloudIn;
    //从ros接收消息
    pcl::fromROSMsg(ros_cloud, lidarCloudIn);
    //保存点云的容器的大小
    int cloud_size = lidarCloudIn.points.size();
    int count = cloud_size;
    pcl::PointXYZRGB point;
    //创建一个包含16个元素的vector
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> lidarCloudScans(16);
    //指针类型
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidarCloudGroundPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidarCloudEdgePtr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidarCloudPlanePtr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidarCloudAllPtr(new pcl::PointCloud<pcl::PointXYZRGB>());

    //判定各点的线数
    for (int i = 0;i < cloud_size;i++) {
        point.x = lidarCloudIn.points[i].x;
        point.y = lidarCloudIn.points[i].y;
        point.z = lidarCloudIn.points[i].z;
        //r是是否为地面，1：非地面 2：地面
        //g是曲率，小于一定值（比如0.1）认为是plane point
        //b是scan的线号0-15
        point.r = 1;
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;
        scanID = int((angle + 15) / 2 + 0.5);
        if (scanID > 15 || scanID < 0) {
            count--;
            continue;
        }
        //将每一个point的线数scanID压入到lidarCloudScans的尾端
        point.b = scanID;
        lidarCloudScans[scanID].push_back(point);
    }

    //地面点获取,只对0-5号线进行处理
    for (int i = 0;i < 5;i++) {
        for (int j = 0;j<int(lidarCloudScans[i].size()) && j<int(lidarCloudScans[i + 1].size());j++) {
            float diffX = lidarCloudScans[i + 1].points[j].x - lidarCloudScans[i].points[j].x;
            float diffY = lidarCloudScans[i + 1].points[j].y - lidarCloudScans[i].points[j].y;
            float diffZ = lidarCloudScans[i + 1].points[j].z - lidarCloudScans[i].points[j].z;
            float angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;
            if (abs(angle) < 10) {
                lidarCloudGroundPtr->push_back(lidarCloudScans[i + 1].points[j]);
                lidarCloudGroundPtr->push_back(lidarCloudScans[i].points[j]);
                lidarCloudScans[i + 1].points[j].r = 2;
                lidarCloudScans[i].points[j].r = 2;
            }
        }
    }

    //计算曲率
    for (int i = 0;i < 16;i++) {
        for (int j = 3;j<int(lidarCloudScans[i].size()) - 3;j++) {
            float diffX = lidarCloudScans[i].points[j - 3].x +
                lidarCloudScans[i].points[j - 2].x +
                lidarCloudScans[i].points[j - 1].x -
                6 * lidarCloudScans[i].points[j].x +
                lidarCloudScans[i].points[j + 1].x +
                lidarCloudScans[i].points[j + 2].x +
                lidarCloudScans[i].points[j + 3].x;
            float diffY = lidarCloudScans[i].points[j - 3].y +
                lidarCloudScans[i].points[j - 2].y +
                lidarCloudScans[i].points[j - 1].y -
                6 * lidarCloudScans[i].points[j].y +
                lidarCloudScans[i].points[j + 1].y +
                lidarCloudScans[i].points[j + 2].y +
                lidarCloudScans[i].points[j + 3].y;
            float diffZ = lidarCloudScans[i].points[j - 3].z +
                lidarCloudScans[i].points[j - 2].z +
                lidarCloudScans[i].points[j - 1].z -
                6 * lidarCloudScans[i].points[j].z +
                lidarCloudScans[i].points[j + 1].z +
                lidarCloudScans[i].points[j + 2].z +
                lidarCloudScans[i].points[j + 3].z;
            lidarCloudScans[i].points[j].g = double(diffX * diffX + diffY * diffY + diffZ * diffZ);
        }
    }

    for (int i = 0;i < 16;i++) {
        *lidarCloudAllPtr += lidarCloudScans[i];
    }

    long all_points_num = lidarCloudAllPtr->points.size();
    for (int i = 0;i < 30;i++)
    {
        long start_num = i * all_points_num / 30;
        long end_num = (i + 1) * all_points_num / 30;
        for (long j = start_num;j < end_num;j++)
        {
            if (lidarCloudAllPtr->points[j].r == 1)
            {
                if (lidarCloudAllPtr->points[j].g > 0.2 && (int(lidarCloudEdgePtr->points.size()) < 4 * (i + 1)))
                {
                    lidarCloudEdgePtr->push_back(lidarCloudAllPtr->points[j]);
                    j = j + 5;
                }
                if (lidarCloudAllPtr->points[j].g < 0.05 && (int(lidarCloudPlanePtr->points.size()) < 20 * (i + 1)))
                {
                    lidarCloudPlanePtr->push_back(lidarCloudAllPtr->points[j]);
                    j = j + 5;
                }
            }
            if ((int(lidarCloudPlanePtr->points.size()) == 20 * (i + 1)) && (int(lidarCloudEdgePtr->points.size()) == 4 * (i + 1)))
            {
                j = end_num;
            }
        }
    }
    //根据曲率获取edge点和plane点
    for (int i = 0;i < 16;i++) {
        for (int j = 5;j<int(lidarCloudScans[i].size()) - 5;j++) {
            if (lidarCloudScans[i].points[j].r == 1) {
                if (lidarCloudScans[i].points[j].g > 0.2) {
                    lidarCloudEdgePtr->push_back(lidarCloudScans[i].points[j]);
                    j += 5;
                }
                if (lidarCloudScans[i].points[j].g < 0.1) {
                    lidarCloudPlanePtr->push_back(lidarCloudScans[i].points[j]);
                    j += 5;
                }
            }
        }
    }

    //地面，棱，平面，全部点云输出
    sensor_msgs::PointCloud2 lidarCloudGroundMsg;
    pcl::toROSMsg(*lidarCloudGroundPtr, lidarCloudGroundMsg);
    //时间戳对准
    lidarCloudGroundMsg.header.stamp = ros_cloud.header.stamp;
    //数据所在的坐标系的名称
    lidarCloudGroundMsg.header.frame_id = "map_child";
    //发布消息
    pubLidarCloudGround.publish(lidarCloudGroundMsg);

    sensor_msgs::PointCloud2 lidarCloudEdgeMsg;
    pcl::toROSMsg(*lidarCloudEdgePtr, lidarCloudEdgeMsg);
    lidarCloudEdgeMsg.header.stamp = ros_cloud.header.stamp;
    lidarCloudEdgeMsg.header.frame_id = "map_child";
    pubLidarCloudEdge.publish(lidarCloudEdgeMsg);

    sensor_msgs::PointCloud2 lidarCloudPlaneMsg;
    pcl::toROSMsg(*lidarCloudPlanePtr, lidarCloudPlaneMsg);
    lidarCloudPlaneMsg.header.stamp = ros_cloud.header.stamp;
    lidarCloudPlaneMsg.header.frame_id = "map_child";
    pubLidarCloudPlane.publish(lidarCloudPlaneMsg);

    sensor_msgs::PointCloud2 lidarCloudAllMsg;
    pcl::toROSMsg(*lidarCloudAllPtr, lidarCloudAllMsg);
    lidarCloudAllMsg.header.stamp = ros_cloud.header.stamp;
    lidarCloudAllMsg.header.frame_id = "map_child";
    pubLaserCloudall_01.publish(lidarCloudAllMsg);


    //计时结束
    clock_end = clock();
    std::cout << "time:" << double(clock_end - clock_start) / CLOCKS_PER_SEC << std::endl;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "extract_points");
    ros::NodeHandle nh;
    //订阅信息
    ros::Subscriber cloud_sub = nh.subscribe("/kitti/velo/pointcloud", 100, callback);
    //发布信息
    pubLidarCloudGround = nh.advertise<sensor_msgs::PointCloud2>("/cloud_ground", 100);
    pubLidarCloudEdge = nh.advertise<sensor_msgs::PointCloud2>("/cloud_edge", 100);
    pubLidarCloudPlane = nh.advertise<sensor_msgs::PointCloud2>("/cloud_plane", 100);
    pubLaserCloudall_01 = nh.advertise<sensor_msgs::PointCloud2>("/cloud_all_01", 100);
    ros::spin();

    return 0;
}