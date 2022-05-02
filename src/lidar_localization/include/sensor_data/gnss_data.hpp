#ifndef _GNSS_DATA_HPP_
#define _GNSS_DATA_HPP_

namespace lidar_localization
{
    class GNSSData
    {
    public:
        int status = 0;
        int service = 1;
        double latitude = 0.0;  //纬度
        double longitude = 0.0; //经度
        double altitude = 0.0;  //海拔
    };
}

#endif
