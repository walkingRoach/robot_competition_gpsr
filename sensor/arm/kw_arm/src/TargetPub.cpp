#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Int32.h> 
#include <std_msgs/Empty.h> 
#include <unistd.h>
#include "kw_arm/my_ik_solver.h"
#include "kw_arm/target.h"
#include <string>
#include <iostream>
using namespace::std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TargetPub"); 
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<kw_arm::target>("arm_target", 1);
    
    double x, y, z;
    while (x != -1)
    {
        cin >> x >> y >> z;
        kw_arm::target target_msg;
        target_msg.x = x;
        target_msg.y = y;
        target_msg.z = z;
        pub.publish(target_msg);
    }
    return 0;
}