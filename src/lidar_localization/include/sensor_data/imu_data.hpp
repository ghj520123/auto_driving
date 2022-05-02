#ifndef _IMU_DATA_HPP_
#define _IMU_DATA_HPP_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace lidar_localization
{
    class IMUData
    {
    public:
        struct Orientation //方向
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 0.0;
            /* data */
        };
        struct AngularVelocity //角速度
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 0.0;
            /* data */
        };
        struct LinearAcceleration //线性加速度
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 0.0;
            /* data */
        };

        //进行实例化
        LinearAcceleration linear_acceleration;
        AngularVelocity angular_velocity;
        Orientation orientation;

    public:
        //将四元数转换成旋转矩阵送出去
        Eigen::Matrix3f GetOrientationMatrix()
        {
            //构造四元数，[x,y,z]是虚部，w是实部
            Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.y);
            //不知道什么意思
            Eigen::Matrix3f matrix = q.matrix().cast<float>();

            return matrix;
        };
    };
}

#endif