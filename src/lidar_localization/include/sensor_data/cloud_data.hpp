#ifndef _CLOUD_DATA_HPP_
#define _CLOUD_DATA_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization
{
    class CloudData
    {
    public:
        using POINT = pcl::PointXYZ;
        using CLOUD = pcl::PointCloud<POINT>;
        using CLOUD_PTR = CLOUD::Ptr;

    public:
        CloudData()
            : cloud_ptr(new CLOUD()) {}

    public:
        double time = 0.0;
        CLOUD_PTR cloud_ptr;
    };
}

#endif