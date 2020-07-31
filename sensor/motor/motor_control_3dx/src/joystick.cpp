#include <stack>
#include <vector>
#include <math.h>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <std_msgs/Float32.h>


using namespace std;

class TeleopJoy{
public:
  TeleopJoy();
  void callBack(const sensor_msgs::Joy::ConstPtr& joy);
  void publish_cmd();

private:
  ros::Publisher vel_pub;
  ros::Publisher stop_pub;
  ros::Subscriber sub;

  float scale_linear, scale_angular;
  geometry_msgs::Twist vel;
  std_msgs::Float32 stop;

  uint8_t speed_lock;  //速度锁止标志，为1时速度保持不变
  int button_state, button_state_last;  //记录按键状态
};

/* 构造函数 */
TeleopJoy::TeleopJoy()
{    
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    n_private.param<float>("scale_linear", scale_linear, 0.8);
    n_private.param<float>("scale_angular", scale_angular, 1.0);
    
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    stop_pub = n.advertise<std_msgs::Float32>("/stop",10);
    sub = n.subscribe<sensor_msgs::Joy>("/joy", 10, &TeleopJoy::callBack, this);

    speed_lock = 0;
}

/* 订阅话题的回调函数 */
void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{
    button_state_last = button_state;
    button_state = joy->buttons[4];

    if(button_state_last == 0 && button_state == 1)
    {
        if(speed_lock == 1)
          speed_lock = 0;
        else
          speed_lock = 1;
    }

    if( joy->axes[2] >= -0.1 && joy->axes[5] < 0.5 )
    {
        if( !speed_lock ) {
            vel.linear.x = scale_linear * joy->axes[1];   // 前进线速度
        }

        vel.angular.z = scale_angular * joy->axes[3];  // 旋转角速度
        stop.data = 0;
    }
    else
    {
        vel.angular.z = 0;
        vel.linear.x = 0;
        stop.data = 1;
    }

    publish_cmd();
}

void TeleopJoy::publish_cmd()
{
    vel_pub.publish(vel);    // 发布速度消息
    stop_pub.publish(stop);  // 发布急停消息
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleopJoy");

  TeleopJoy teleop_turtle;
	
  ros::Rate loop_rate(50);

  while (ros::ok())
  {
      //teleop_turtle.publish_cmd();

      loop_rate.sleep();
      ros::spinOnce();
  }

  return 0;
}
