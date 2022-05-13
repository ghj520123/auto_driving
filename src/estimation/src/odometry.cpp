//c++
#include<iostream>
#include<vector>
#include<time.h>
//ros
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
//ceres
#include<ceres/ceres.h>
#include<ceres/rotation.h>
//eigen
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
//nav
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
//tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//当前帧到世界坐标系
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

//当前帧plane点（应该是）
pcl::PointCloud<pcl::PointXYZRGB> lidarCloudIn_plane;
//上一帧plane点
pcl::PointCloud<pcl::PointXYZRGB> lidarCloudIn_plane_last;
//寻找最近点的KD树
pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtreePlaneLast;
//所有帧地图点
pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidarCloud_map(new pcl::PointCloud<pcl::PointXYZRGB>());

//四元数Q，当前帧到上一帧
double para_q[4] = { 0,0,0,1 };
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
//当前帧到上一帧的位移量t，配合四元数累加在一起就是当前帧到最开始帧（即世界坐标系）
double para_t[3] = { 0,0,0 };
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

//后面要进行距离比较的参数,实际为两点相距3m为阈值
constexpr double DISTANCE_SQ_THRESHOLD = 9;
//找点进行匹配优化时的线数距离(13线-10线>2.5就break介样用)
constexpr double NEARBY_SCAN = 2.5;

//输出里程计，路径，当前全部点，地图点
ros::Publisher pubLidarOdometry;
ros::Publisher pubLidarPath;
ros::Publisher pubLidarCloudall_02;
ros::Publisher pubLidarCloud_map;
//定义路径，用于保存帧的位置，发布于pubLaserPath
nav_msgs::Path lidarPath;

//构建代价函数结构体，
//last_point_a_为这一帧中的点a，curr_point_b_为点a旋转后和上一帧里最近的点
//curr_point_c_为点b同线或上线号的点，curr_point_d_为点b下线号的点
//b，c，d与a点距离不超过3m
struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST(
        Eigen::Vector3d _curr_point_a_,
        Eigen::Vector3d _last_point_b_,
        Eigen::Vector3d _last_point_c_,
        Eigen::Vector3d _lsat_point_d_
    ) : curr_point_a_(_curr_point_a_),
        last_point_b_(_last_point_b_),
        last_point_c_(_last_point_c_),
        last_point_d_(_lsat_point_d_)
    {
        //plane_norm为根据向量bc和bd求出的法向量
        plane_norm = (last_point_d_ - last_point_b_).cross(last_point_c_ - last_point_b_);
        plane_norm.normalize();
    }
    //模板
    template <typename T>
    //重载（）运算符，括号中的const表示参数a对象不会被修改，最后的const表明调用函数对象不会被修改!
    //residual为残差
    bool operator()(const T* q, const T* t, T* residual)const {
        Eigen::Matrix<T, 3, 1> p_a_curr{
            T(curr_point_a_.x()),
            T(curr_point_a_.y()),
            T(curr_point_a_.z())
        };
        Eigen::Matrix<T, 3, 1> p_b_last{
            T(last_point_b_.x()),
            T(last_point_b_.y()),
            T(last_point_b_.z())
        };
        Eigen::Quaternion<T> rot_q{ q[3],q[0],q[1],q[2] };
        Eigen::Matrix<T, 3, 1> rot_t{ t[0],t[1],t[2] };
        Eigen::Matrix<T, 3, 1> p_a_last;
        p_a_last = rot_q * p_a_curr + rot_t;
        residual[0] = abs((p_a_last - p_b_last).dot(plane_norm));
        return true;
    }
    const Eigen::Vector3d curr_point_a_, last_point_b_, last_point_c_, last_point_d_;
    Eigen::Vector3d plane_norm;
    /* data */
};

//用于推算一个这一帧在上一帧lidar坐标系下的位置
//pi的值不能被修改，pi指向的地址也不能被修改
void TransformToStart(pcl::PointXYZRGB const* const pi, pcl::PointXYZRGB* const po) {
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point;
    un_point = q_last_curr * point + t_last_curr;
    //输出(输出什么我也不知道)
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->r = pi->r;
    po->g = pi->g;
    po->b = pi->b;
}

//用于推算一个这一帧的点在世界坐标系下的位置
void TransformToMap(pcl::PointXYZRGB const* const pi, pcl::PointXYZRGB* const po) {
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point;
    Eigen::Quaterniond q_w_(q_w_curr.w(), q_w_curr.x(), q_w_curr.y(), q_w_curr.z());
    Eigen::Vector3d t_w_(t_w_curr.x(), t_w_curr.y(), t_w_curr.z());
    un_point = q_w_ * point + t_w_;

    //输出
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->r = pi->r;
    po->g = pi->g;
    po->b = pi->b;
}

void cloud_plane_callback(const sensor_msgs::PointCloud2 ros_cloud_plane) {
    clock_t clock_start, clock_end;
    clock_start = clock();

    //点云转换
    pcl::fromROSMsg(ros_cloud_plane, lidarCloudIn_plane);
    //上一帧的点数
    int lidarCloudIn_plane_last_num = lidarCloudIn_plane_last.points.size();
    //当前帧的点数
    int lidarCloudIn_plane_num = lidarCloudIn_plane.points.size();
    if (lidarCloudIn_plane_last_num < 10) {
        lidarCloudIn_plane_last = lidarCloudIn_plane;
    }
    else {
        //??
        kdtreePlaneLast.setInputCloud(lidarCloudIn_plane_last.makeShared());
        //optize是什么
        for (int optize_num = 0;optize_num <= 1;optize_num++) {
            //优化问题构建
            ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
            ceres::Manifold* q_manifold = new ceres::EigenQuaternionManifold();
            //ceres::LocalParameterization* q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(para_q, 4, q_manifold);
            problem.AddParameterBlock(para_t, 3);

            //对于每个当前帧的点，也就是a点，进行寻找上一帧最近点b，并根据b点寻找c、d两点进行点面距离估计，引入优化函数中
            for (int i = 0;i < lidarCloudIn_plane_num;i++) {
                //构建一个KD树用的点kd_id,点距离；将本次的a点转换到本帧下（？），并输入本帧全部点来寻找最近点
                pcl::PointXYZRGB pointseed;
                //?
                std::vector<int> pointSearchInd;
                //?
                std::vector<float> pointSearchSqDis;
                TransformToStart(&lidarCloudIn_plane.points[i], &pointseed);
                kdtreePlaneLast.nearestKSearch(pointseed, 1, pointSearchInd, pointSearchSqDis);

                //b,c,d点的id，b点是kd树求出的；
                int closestPointInd = pointSearchInd[0];
                int minPointInd2 = -1, minPointInd3 = -1;
                //b的线号
                int closestPoint_scanID_b = lidarCloudIn_plane_last.points[pointSearchInd[0]].b;

                //如果ab的距离小于3，那么继续进行，否则不优化，直接下一帧
                if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
                    //向b点的线号往上寻找c点（可以同线号）
                    for (int j = closestPointInd + 2;(j < lidarCloudIn_plane_last_num) && (minPointInd2 == -1);j++) {
                        if (lidarCloudIn_plane_last.points[j].b > closestPoint_scanID_b + NEARBY_SCAN) {
                            continue;
                        }
                        else {
                            double distance_a_c =
                                (lidarCloudIn_plane_last.points[j].x - pointseed.x)
                                * (lidarCloudIn_plane_last.points[j].x - pointseed.x)
                                + (lidarCloudIn_plane_last.points[j].y - pointseed.y)
                                * (lidarCloudIn_plane_last.points[j].y - pointseed.y)
                                + (lidarCloudIn_plane_last.points[j].z - pointseed.z)
                                * (lidarCloudIn_plane_last.points[j].z - pointseed.z);
                            if (distance_a_c > DISTANCE_SQ_THRESHOLD) {
                                continue;
                            }
                            else {
                                minPointInd2 = j;
                            }
                        }
                    }

                    //从b点的线号往下寻找d点（不可同线号）（if取并集来实现）
                    for (int j = closestPointInd - 1;(j > 0) && (minPointInd3 == -1);j--) {
                        if ((lidarCloudIn_plane_last.points[j].b < closestPoint_scanID_b - NEARBY_SCAN) || (lidarCloudIn_plane_last.points[j].b == closestPoint_scanID_b)) {
                            continue;
                        }
                        else {
                            double distance_a_d = (lidarCloudIn_plane_last.points[j].x - pointseed.x)
                                * (lidarCloudIn_plane_last.points[j].x - pointseed.x)
                                + (lidarCloudIn_plane_last.points[j].y - pointseed.y)
                                * (lidarCloudIn_plane_last.points[j].y - pointseed.y)
                                + (lidarCloudIn_plane_last.points[j].z - pointseed.z)
                                * (lidarCloudIn_plane_last.points[j].z - pointseed.z);
                            if (distance_a_d > DISTANCE_SQ_THRESHOLD) {
                                continue;
                            }
                            else {
                                minPointInd3 = j;
                            }
                        }
                    }

                    //如果c，d都没找到，也就是id还是初始的-1，就过；否则记录abcd点放入残差快中
                    if (minPointInd2 == -1 || minPointInd3 == -1) {
                        continue;
                    }
                    else {
                        Eigen::Vector3d lastPoint_a(lidarCloudIn_plane.points[i].x,
                            lidarCloudIn_plane.points[i].y,
                            lidarCloudIn_plane.points[i].z);
                        Eigen::Vector3d currPoint_b(lidarCloudIn_plane_last.points[closestPointInd].x,
                            lidarCloudIn_plane_last.points[closestPointInd].y,
                            lidarCloudIn_plane_last.points[closestPointInd].z);
                        Eigen::Vector3d currPoint_c(lidarCloudIn_plane_last.points[minPointInd2].x,
                            lidarCloudIn_plane_last.points[minPointInd2].y,
                            lidarCloudIn_plane_last.points[minPointInd2].z);
                        Eigen::Vector3d currPoint_d(lidarCloudIn_plane_last.points[minPointInd3].x,
                            lidarCloudIn_plane_last.points[minPointInd3].y,
                            lidarCloudIn_plane_last.points[minPointInd3].z);
                        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4, 3>
                            (new CURVE_FITTING_COST(lastPoint_a, currPoint_b, currPoint_c, currPoint_d)),
                            loss_function, para_q, para_t);
                    }
                }
            }

            //所有前一帧里的点都当作a点遍历过后，进行优化
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            //迭代数
            options.max_num_iterations = 5;
            //进度是否发送到STDOUT
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
        }
    }

    //优化后，当前帧变为前一帧，位姿累积，并且根据位姿变换当前帧的所有点到世界坐标系，放入地图中
    lidarCloudIn_plane_last = lidarCloudIn_plane;
    t_w_curr = t_w_curr + q_w_curr * t_last_curr;
    q_w_curr = q_w_curr * q_last_curr;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidarCloud_map_curr(new pcl::PointCloud<pcl::PointXYZRGB>());;
    for (int i = 0;i < lidarCloudIn_plane_num;i = i + 10) {
        pcl::PointXYZRGB point_in_map;
        TransformToMap(&lidarCloudIn_plane.points[i], &point_in_map);
        lidarCloud_map_curr->push_back(point_in_map);
    }

    //降采样（？），输出地图点
    pcl::PointCloud<pcl::PointXYZRGB> lidarCloud_map_filter;
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(lidarCloud_map_curr);
    sor.setLeafSize(0.2, 0.2, 0.2);
    sor.filter(lidarCloud_map_filter);

    *lidarCloud_map += lidarCloud_map_filter;

    //输出地图点
    sensor_msgs::PointCloud2 lidarCloudMapMsg;
    pcl::toROSMsg(*lidarCloud_map, lidarCloudMapMsg);
    lidarCloudMapMsg.header.stamp = ros_cloud_plane.header.stamp;
    lidarCloudMapMsg.header.frame_id = "map";
    pubLidarCloud_map.publish(lidarCloudMapMsg);

    //输出位姿和path
    nav_msgs::Odometry lidarOdometry;
    lidarOdometry.header.frame_id = "map";
    lidarOdometry.child_frame_id = "map_child";
    lidarOdometry.header.stamp = ros_cloud_plane.header.stamp;
    lidarOdometry.pose.pose.orientation.x = q_w_curr.x();
    lidarOdometry.pose.pose.orientation.y = q_w_curr.y();
    lidarOdometry.pose.pose.orientation.z = q_w_curr.z();
    lidarOdometry.pose.pose.orientation.w = q_w_curr.w();
    lidarOdometry.pose.pose.position.x = t_w_curr.x();
    lidarOdometry.pose.pose.position.y = t_w_curr.y();
    lidarOdometry.pose.pose.position.z = t_w_curr.z();
    pubLidarOdometry.publish(lidarOdometry);

    geometry_msgs::PoseStamped lidarPose;
    lidarPose.header = lidarOdometry.header;
    lidarPose.pose = lidarOdometry.pose.pose;
    lidarPath.header.stamp = lidarOdometry.header.stamp;
    lidarPath.poses.push_back(lidarPose);
    lidarPath.header.frame_id = "map";
    pubLidarPath.publish(lidarPath);

    //输出世界参考系的坐标，transform为map_child的位姿，并在此位姿下显示当前所有点（通过 cloud_all_Callhandle函数）
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(t_w_curr.x(), t_w_curr.y(), t_w_curr.z()));
    q.setW(q_w_curr.w());
    q.setX(q_w_curr.x());
    q.setY(q_w_curr.y());
    q.setZ(q_w_curr.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros_cloud_plane.header.stamp, "map", "map_child"));

    //计时结束
    clock_end = clock();
    std::cout << "odo_time:" << double(clock_end - clock_start) / CLOCKS_PER_SEC << std::endl;
}

void cloud_all_callback(const sensor_msgs::PointCloud2 ros_cloud_all) {
    pcl::PointCloud<pcl::PointXYZRGB> lidarCloudIn_all;
    pcl::fromROSMsg(ros_cloud_all, lidarCloudIn_all);
    sensor_msgs::PointCloud2 lidarCloud_planeMsg;
    pcl::toROSMsg(lidarCloudIn_all, lidarCloud_planeMsg);
    lidarCloud_planeMsg.header.stamp = ros_cloud_all.header.stamp;
    lidarCloud_planeMsg.header.frame_id = "map_child";
    pubLidarCloudall_02.publish(lidarCloud_planeMsg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    //订阅信息
    ros::Subscriber cloud_plane = nh.subscribe("/kitti_cloud_plane", 100, cloud_plane_callback);
    ros::Subscriber claou_all = nh.subscribe("/kitti_cloud_all_01", 100, cloud_all_callback);
    ////pubLaserOdometry包括当前帧四元数Q和位置t,pubLaserPath包含当前帧的位置t
    pubLidarOdometry = nh.advertise<nav_msgs::Odometry>("/lidar_odometry_to_init", 100);
    pubLidarPath = nh.advertise<nav_msgs::Path>("/lidar_odometry_path", 100);
    pubLidarCloudall_02 = nh.advertise<sensor_msgs::PointCloud2>("/kitti_cloud_all_02", 100);
    pubLidarCloud_map = nh.advertise<sensor_msgs::PointCloud2>("/kitti_cloud_map", 100);
    ros::spin();
    return 0;
}