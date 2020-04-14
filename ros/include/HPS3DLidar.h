//
// Created by kolbe on 02.04.20.
//

#ifndef HPS_CAMERA_HPS3DLIDAR_H
#define HPS_CAMERA_HPS3DLIDAR_H

#include <cstring>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Temperature.h>
#include <api.h>
//#include <depth_image_converter.h>
//#include <point_cloud_converter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <hps_camera/HPSConfig.h>


class HPS3DLidar {
public:
    HPS3DLidar(ros::NodeHandle &n, std::string devicePath);
    ~HPS3DLidar();

    bool init();
    void release();

    static std::vector<std::string> getDevices(std::string dir, std::string dev_prefix);
    static std::string statusTypeStr(RET_StatusTypeDef status);

private:
    void* cbHandler(HPS3D_HandleTypeDef* handle, AsyncIObserver_t* event);
    static void* cbStaticHandler(HPS3D_HandleTypeDef* handle, AsyncIObserver_t* event);
    static void cbDebug(char* msg);

    void cbReconfigure(hps_camera::HPSConfig conf, uint32_t level);

    void publish(MeasureDataTypeDef& data);

    HPS3D_HandleTypeDef handle;
    AsyncIObserver_t eventObserver;

    ros::NodeHandle& node;

    // reconfigure
    dynamic_reconfigure::Server<hps_camera::HPSConfig> server;
    dynamic_reconfigure::Server<hps_camera::HPSConfig>::CallbackType f;
    hps_camera::HPSConfig reconf;

    ros::Publisher pub_pointcloud;
    ros::Publisher pub_depth;
    ros::Publisher pub_depth_color;
    ros::Publisher pub_temperature;
};


#endif //HPS_CAMERA_HPS3DLIDAR_H
