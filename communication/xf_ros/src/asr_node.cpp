#include <ros/ros.h>

#include "xf_ros/asr.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "xf_ros");
    ros::NodeHandle n("~");
    xf_ros::asr_pub = n.advertise<std_msgs::String>("/xf/asr/result", 1);
    xf_ros::ASR asr(n);

    ros::spin();
    return 0;
}
