#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "kw_arm/MoveToPoseAction.h"
#include <serial/serial.h> 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <string>

static const double L0_LENGTH = 100.0;
static const double L1_LENGTH = 250.0;
static const double L2_LENGTH = 250.0;
static const double L3_LENGTH = 75.0;

serial::Serial ser;

typedef actionlib::SimpleActionServer<kw_arm::MoveToPoseAction> Server;
 
// 收到action的goal后调用的回调函数
void execute(const kw_arm::MoveToPoseGoalConstPtr& goal, Server* as)
{
    kw_arm::MoveToPoseFeedback feedback;
 
    ROS_INFO("Arm target: %lf %lf %lf %lf %lf received.", goal->pose_x, goal->pose_y, goal->pose_z, goal->pose_pitch, goal->pose_gripper);
 
    ros::Rate timer(50);
    while(1) 
    { 
        if(ser.available()){ 
            std::string str = ser.read(ser.available()); 
            int length = str.length();
        } 
        timer.sleep(); 
    }
 
    // 当action完成后，向客户端返回结果
    ROS_INFO("Arm finish working.");
    as->setSucceeded();
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_to_pose_server");
    ros::NodeHandle nh;

    //设置串口属性，并打开串口 
    try 
    { 
        ser.setPort("/dev/kw_arm"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout(1, 3, 0, 3, 0);
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    }

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    // 定义一个服务器
    Server server(nh, "move_to_pose", boost::bind(&execute, _1, &server), false);
 
    // 服务器开始运行
    server.start();
    
    //消息接收循环
    ros::Rate loop_rate(50); 
    while(ros::ok()) 
    { 
        if(ser.available()){ 
            std::string str = ser.read(ser.available()); 
            int length = str.length();
        } 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }
    return 0;
}