//
// Created by kolbe on 02.04.20.
//
#include <ros/ros.h>
#include <HPSCamera.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "ros_hps_camera");
    ros::NodeHandle n("~");

    // read params
    std::string device = n.param<std::string>("device", "/dev/ttyACM0");

    HPSCamera cam(n, device);
    if(!cam.init()){
        return EXIT_FAILURE;
    }

    ros::spin();

    cam.release();

    return EXIT_SUCCESS;
}