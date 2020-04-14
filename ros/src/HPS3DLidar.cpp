//
// Created by kolbe on 02.04.20.
//

#include "HPS3DLidar.h"
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>


// HACK, cannot set class member for callback so I use a static function that access this object
HPS3DLidar* current_object = nullptr;


HPS3DLidar::HPS3DLidar(ros::NodeHandle &n, std::string devicePath) : node(n) {
    pub_pointcloud = node.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    pub_depth = node.advertise<sensor_msgs::Image>("depth", 1);
    pub_depth_color = node.advertise<sensor_msgs::Image>("depth_color", 1);
    pub_temperature = node.advertise<sensor_msgs::Temperature>("temperature", 1);

    // set device path
    handle.DeviceName = const_cast<char *>(devicePath.c_str());
//    memcpy(handle.DeviceName, devicePath.c_str(), sizeof(uint8_t)*devicePath.size());

    // dynamic reconfigure
    f = boost::bind(&HPS3DLidar::cbReconfigure, this, _1, _2);
    server.setCallback(f);

    current_object = this;

    // print SDK version
    Version_t v = HPS3D_GetSDKVersion();
    ROS_INFO_STREAM("HPS3D SDK version: " << unsigned(v.year)
    << "." << unsigned(v.month) << "." << unsigned(v.day) << " v" << unsigned(v.major) << "." << unsigned(v.minor) << "." << unsigned(v.rev));
}

HPS3DLidar::~HPS3DLidar() {
    release();

    // shutdown ROS
    pub_pointcloud.shutdown();
    pub_depth.shutdown();
    pub_depth_color.shutdown();
    node.shutdown();
}

bool HPS3DLidar::init() {
    RET_StatusTypeDef ret = RET_OK;

    // connect
    ROS_INFO_STREAM("Open device " << handle.DeviceName);
    ret = HPS3D_Connect(&handle);
    if (ret != RET_OK)
    {
        ROS_ERROR_STREAM("Device open failed! " << HPS3DLidar::statusTypeStr(ret) << " (error code " << ret << ")");
        return false;
    }

    // set config
    ret = HPS3D_ConfigInit(&handle);
    if (ret != RET_OK)
    {
//        HPS3D_RemoveDevice(&handle);
        ROS_ERROR_STREAM("Initialization failed: " << HPS3DLidar::statusTypeStr(ret) << " (error code " << ret << ")");
        return false;
    }

    // enable pointcloud
    HPS3D_SetOpticalEnable(&handle, true);
    HPS3D_SetPointCloudEn(true);

    // add callback
    eventObserver.AsyncEvent = ISubject_Event_DataRecvd;
    eventObserver.NotifyEnable = true;
//    std::function<void*(HPS3D_HandleTypeDef *,AsyncIObserver_t *)> f;
//    HPS3D_AddObserver(std::bind(&HPS3DLidar::callbackHandler, this, std::placeholders::_1, std::placeholders::_2), &handle, &eventObserver);
//    HPS3D_AddObserver((void * (*fun)(HPS3D_HandleTypeDef *,AsyncIObserver_t *))f, &handle, &eventObserver);
    HPS3D_AddObserver(HPS3DLidar::cbStaticHandler, &handle, &eventObserver);

    // add debug callback
    HPS3D_SetDebugFunc(HPS3DLidar::cbDebug);

    // start capturing
    handle.RunMode = RUN_CONTINUOUS;
    HPS3D_SetRunMode(&handle);

    ROS_INFO_STREAM("Opened");
    return true;
}

void *HPS3DLidar::cbHandler(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event) {

    ROS_INFO_STREAM("Callback from observer " << event->ObserverID);

//    if (event->AsyncEvent != ISubject_Event_DataRecvd) {
//        return nullptr;
//    }
//
//    // handle packet
//    switch (event->RetPacketType) {
//        case SIMPLE_ROI_PACKET:
//        case SIMPLE_DEPTH_PACKET:
//        case OBSTACLE_PACKET:
//        case NULL_PACKET:
//        case FULL_ROI_PACKET:
//            break;
//        case FULL_DEPTH_PACKET:
//            publish(event->MeasureData);
//            break;
//        default:
//            ROS_ERROR("system error!\n");
//    }
//
//    return nullptr;
}

std::vector<std::string> HPS3DLidar::getDevices(std::string dir, std::string dev_prefix) {
    std::vector<std::string> devices;

    char fileName[DEV_NUM][DEV_NAME_SIZE];
    uint32_t dev_cnt = 0;

    dev_cnt = HPS3D_GetDeviceList("/dev/", "ttyACM", fileName);

    for(size_t i=0; i<dev_cnt; i++){
        ROS_INFO_STREAM("Found device: " << fileName[i]);
        devices.emplace_back((char*)fileName[i]);
    }

    return devices;
}

void *HPS3DLidar::cbStaticHandler(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event) {
//    ROS_INFO_STREAM("Callback from observer " << event->ObserverID);

    if (event->AsyncEvent != ISubject_Event_DataRecvd) {
        return nullptr;
    }

    if(!current_object){
        ROS_ERROR_STREAM("No current object!");
        return nullptr;
    }

    // handle packet
    switch (event->RetPacketType) {
        case SIMPLE_ROI_PACKET:
        case SIMPLE_DEPTH_PACKET:
        case OBSTACLE_PACKET:
        case NULL_PACKET:
        case FULL_ROI_PACKET:
            break;
        case FULL_DEPTH_PACKET:
            current_object->publish(event->MeasureData);
            break;
        case KEEP_ALIVE_PACKET:
            ROS_DEBUG("Got keepalive packet");
            break;
        default:
            ROS_ERROR("system error!\n");
    }
    return nullptr;
}

void HPS3DLidar::publish(MeasureDataTypeDef& data) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "hps_camera";
    header.seq = data.full_depth_data->frame_cnt;

    // publish data if there are subscribers
    if(pub_pointcloud.getNumSubscribers() and data.point_cloud_data){
        pcl::PointCloud<pcl::PointXYZ> cloud;
        sensor_msgs::PointCloud2 outputPointCloud;
        outputPointCloud.header.stamp = ros::Time::now();
        cloud.width = data.point_cloud_data->width;
        cloud.height = data.point_cloud_data->height;
        cloud.points.resize(cloud.width*cloud.height);
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            const auto& pointData = data.point_cloud_data->point_data[i];
            if (pointData.z < LOW_AMPLITUDE) {
                cloud.points[i].x = pointData.x / 1000.0;
                cloud.points[i].y = pointData.y / 1000.0;
                cloud.points[i].z = pointData.z / 1000.0;
            } else {
                cloud.points[i].x = 0;
                cloud.points[i].y = 0;
                cloud.points[i].z = 0;
            }
        }
        pcl::toROSMsg(cloud, outputPointCloud);
        outputPointCloud.header = header;
        pub_pointcloud.publish(outputPointCloud);
    }
    if(pub_depth.getNumSubscribers() or pub_depth_color.getNumSubscribers()){
        // set Mat pointer to distance data
        cv::Mat values(RES_HEIGHT, RES_WIDTH, CV_16U, data.full_depth_data->distance);
        cv::Mat mask = cv::Mat::ones(RES_HEIGHT, RES_WIDTH, CV_8U) * std::numeric_limits<uint8_t>::max();

        // get invalid data and set mask
        mask.setTo(0, values == LOW_AMPLITUDE);
        mask.setTo(0, values == INVALID_DATA);
        mask.setTo(0, values == ADC_OVERFLOW);
        mask.setTo(0, values == SATURATION);

        // apply mask
        cv::Mat valuesMasked;
        values.copyTo(valuesMasked, mask);

        // normalize / set min/max range and clip
        cv::Mat norm(RES_HEIGHT, RES_WIDTH, CV_16U);
        norm = cv::min(valuesMasked, reconf.depth_image_max);
        norm = cv::max(norm, reconf.depth_image_min);
        norm -= reconf.depth_image_min;
        norm *= (double)std::numeric_limits<uint16_t>::max() / (reconf.depth_image_max-reconf.depth_image_min);

        // setup message and cv bridge
        cv_bridge::CvImage out_msg;
        out_msg.header = header;

        // publish depth
        if(pub_depth.getNumSubscribers()){
            out_msg.encoding = sensor_msgs::image_encodings::MONO16;
            out_msg.image    = norm;
            pub_depth.publish(out_msg);
        }

        // color depth as HSV image
        // set mask to value channel so masked pixels are marked
        if(pub_depth_color.getNumSubscribers()){
            cv::Mat hsv, rgb;
            cv::Mat sat_val = cv::Mat::ones(norm.size(), CV_8U)*255;
            cv::Mat hue(norm);
            hue.convertTo(hue, CV_8U, 1/255.);
            std::vector<cv::Mat> channels;
            channels.emplace_back(hue);
            channels.emplace_back(sat_val);
            channels.emplace_back(mask);

            cv::merge(channels, hsv);
            cv::cvtColor(hsv, rgb, cv::COLOR_HSV2RGB);

            out_msg.encoding = sensor_msgs::image_encodings::RGB8;
            out_msg.image    = rgb;
            pub_depth_color.publish(out_msg);
        }
    }

    // device temperature
    if(pub_temperature.getNumSubscribers()){
        sensor_msgs::Temperature t;
        t.header = header;
        t.temperature = data.full_depth_data->temperature / 10.;
        pub_temperature.publish(t);
    }
}

void HPS3DLidar::release() {
    // handle shutdown
    if(handle.ConnectStatus){
        ROS_INFO_STREAM("Disconnect device " << handle.DeviceName);
        HPS3D_DisConnect(&handle);
    }
}

void HPS3DLidar::cbReconfigure(hps_camera::HPSConfig conf, uint32_t level) {
    ROS_INFO("Got reconfigure");
    // save latest config to member
    reconf = conf;

    if(!handle.ConnectStatus){
        ROS_INFO("Cannot set device config. Not connected.");
        return;
    }

    // pause device and update config
    ROS_INFO("Pause device for reconfigure...");
    RunModeTypeDef tmpRunMode = handle.RunMode;
    handle.RunMode = RUN_IDLE;
    HPS3D_SetRunMode(&handle);

    // debug
    HPS3D_SetDebugEnable(reconf.debugEnable);

    // HDR
    HDRConf hdrConf;
    hdrConf.hdr_mode = static_cast<HDRModeTypeDef>(reconf.hdrConfHdrMode);
    hdrConf.qualtity_overexposed = reconf.hdrConfQualityOverexposed;
    hdrConf.qualtity_overexposed_serious = reconf.hdrConfQualityOverexposedSerious;
    hdrConf.qualtity_weak = reconf.hdrConfQualityWeak;
    hdrConf.qualtity_weak_serious = reconf.hdrConfQualityWeakSerious;
    hdrConf.simple_hdr_max_integration = reconf.hdrConfSimpleHdrMaxIntegration;
    hdrConf.simple_hdr_min_integration = reconf.hdrConfSimpleHdrMinIntegration;
    hdrConf.super_hdr_frame_number = reconf.hdrConfSuperHdrFrameNumber;
    hdrConf.super_hdr_max_integration = reconf.hdrConfSuperHdrMaxIntegration;
    hdrConf.hdr_disable_integration_time = reconf.hdrConfHdrDisableIntegrationTime;
    HPS3D_SetHDRConfig(&handle, hdrConf);

    // distance filter
    DistanceFilterConfTypeDef distanceFilterConf;
    distanceFilterConf.filter_type = static_cast<DistanceFilterTypeDef>(reconf.distanceFilterConfFilterType);
    distanceFilterConf.kalman_K = reconf.distanceFilterConfKalmanK;
    distanceFilterConf.kalman_threshold = reconf.distanceFilterConfKalmanThreshold;
    distanceFilterConf.num_check = reconf.distanceFilterConfNumCheck;
    HPS3D_SetSimpleKalman(&handle, distanceFilterConf);

    // smoothing filter
    SmoothFilterConfTypeDef smoothFilterConf;
    smoothFilterConf.type = static_cast<SmoothFilterTypeDef>(reconf.smoothFilterConfType);
    smoothFilterConf.arg1 = reconf.smoothFilterConfArg1;
    HPS3D_SetSmoothFilter(&handle, smoothFilterConf);

    // mounting angle
    MountingAngleParamTypeDef mountingAngleParamConf;
    mountingAngleParamConf.enable = reconf.mountingAngleParamConfEnable;
    mountingAngleParamConf.angle_vertical = reconf.mountingAngleParamConfAngleVertical;
    mountingAngleParamConf.height = reconf.mountingAngleParamConfHeight;
    HPS3D_SetMountingAngleParamConf(&handle, mountingAngleParamConf);

    // distance offset
    HPS3D_SetDistanceOffset(&handle, reconf.distanceOffset);

    // edge detection
    HPS3D_SetEdgeDetectionEnable(reconf.edgeDetectionEnable);
    HPS3D_SetEdgeDetectionValue(reconf.edgeDetectionThresholdValue);

    // There is not setter for the opticalParamConf, yet.
    OpticalParamConfTypeDef opticalParamConf;

    // resume
    ROS_INFO("Configure done, resume...");
    handle.RunMode = tmpRunMode;
    HPS3D_SetRunMode(&handle);
}

std::string HPS3DLidar::statusTypeStr(RET_StatusTypeDef status) {
    switch (status) {
        case RET_OK:
            return std::string("OK");
        case RET_ERROR:
            return std::string("ERROR");
        case RET_BUSY:
            return std::string("DEVICE BUSY");
        case RET_CONNECT_FAILED:
            return std::string("CONNECTION FAILED");
        case RET_CREAT_PTHREAD_ERR:
            return std::string("THREAD CREATION FAILED");
        case RET_WRITE_ERR:
            return std::string("WRITE FAILURE");
        case RET_READ_ERR:
            return std::string("READ FAILURE");
        case RET_PACKET_HEAD_ERR:
            return std::string("HEADER ERROR");
        case RET_PACKET_ERR:
            return std::string("PACKET ERROR");
        case RET_BUFF_EMPTY:
            return std::string("BUFFER IS EMPTY");
        case RET_VER_MISMATCH:
            return std::string("VERSION DOES NOT MATCH");
    }

    return std::string("UNKNOWN ERROR CODE");
}

void HPS3DLidar::cbDebug(char *msg) {
    ROS_DEBUG_STREAM("HPS Debug: " << msg);
}
