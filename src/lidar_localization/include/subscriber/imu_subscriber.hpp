#ifndef _IMU_SUBSCRIBER_HPP_
#define _IMU_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include "lidar_localization/include/sensor_data/imu_data.hpp"
#include "sensor_msgs/Imu.h"

namespace lidar_localization
{
    class IMUSubscriber
    {
    public:
    private:
        void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    };
}

#endif