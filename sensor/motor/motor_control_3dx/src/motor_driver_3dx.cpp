#include <iostream>
#include <sstream>
#include <ctime>
#include <cmath>
#include <stdlib.h>

#include <ros/ros.h>
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <motor_control_3dx/readDataAll.h>
#include <motor_control_3dx/multi_range.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

using namespace std;

#define BYTE0(dwTemp)  (*((char *)(&dwTemp)  ))
#define BYTE1(dwTemp)  (*((char *)(&dwTemp)+1))
#define BYTE2(dwTemp)  (*((char *)(&dwTemp)+2))
#define BYTE3(dwTemp)  (*((char *)(&dwTemp)+3))

serial::Serial ros_ser;  //声明串口对象
string usb2uart_port;    //端口
int baud;                //波特率

class MotorDriver
{
public:
    MotorDriver();

    void get_odom();
    void odom_publish();
    void range_publish();

    void receiveSpeed(geometry_msgs::Twist twist);
    void protect(std_msgs::Float32 stop);

    void setSpeed();

private:

    motor_control_3dx::readDataAll sensor_data;  //里程计数据

    ros::Publisher  odom_pub, range_pub, sensorData_pub;
    ros::Subscriber speed_sub, is_cmd_valid_sub;

    tf::TransformBroadcaster odom_broadcaster;

    uint8_t send_buff[100];  //发送缓存数组
    uint8_t receive[2];
    uint8_t valid_data[100];  //有效数据接收缓存数组

    short p1,p2,p3,p4;  // 四个电机的目标换向频率

    double width_robot;  // 轮宽
    double vth_scale;

    uint8_t is_cmd_valid;  //指令是否有效标志

    double pose_x;  //机器人位置坐标
    double pose_y;
    double pose_th;

    double vx;  //机器人的线速度
    double vy;
    double vth;

    double last_data[4];
    double last_time;
};

/* 构造函数 */
MotorDriver::MotorDriver()
{
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");  //定义为私有节点

    // 发布/订阅的ROS话题
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10, true);
    range_pub = n.advertise<motor_control_3dx::multi_range>("/range", 10, true);
    sensorData_pub = n.advertise<motor_control_3dx::readDataAll>("/sensor_data", 10, true);

    speed_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel",10, &MotorDriver::receiveSpeed, this);
    is_cmd_valid_sub = n.subscribe<std_msgs::Float32>("/stop",10, &MotorDriver::protect, this);

    // 从参数服务器获取参数
    n_private.param<string>("dev", usb2uart_port, "/dev/ttyUSB0");
    n_private.param<int>("baud", baud, +115200);
    n_private.param<double>("vth_scale", vth_scale, 1.00);

    ROS_INFO("MotorDriver USBPort: %s", usb2uart_port.c_str());

    width_robot = 0.355;

    is_cmd_valid = 1;

    pose_x = 0.0;
    pose_y = 0.0;
    pose_th = 0.0;

    vx = 0.0;
    vy = 0.0;
    vth = 0.0;
}

int time_stt;
/**
 * @brief 读取里程计以及电流数据
 */
void MotorDriver::get_odom()
{
    if(ros_ser.available())
    {
        memset(receive, 0, 2);  //帧头数组清零

        ros_ser.read(receive,1);  //从串口读取一个字节

        if(receive[0] == 0xAA)
        {
            ros_ser.read(receive,1);

            //连续两次读到帧头0XAA
            if(receive[0] == 0xAA)
            {
                ros_ser.read(receive,2);  //再读两个字节

                uint8_t data_len = receive[1];  //获取此数据帧的长度

                ros_ser.read(valid_data, data_len+1);  //从串口读取此数据帧剩余数据

                uint8_t check_sum = (0xAA + 0xAA + receive[0] + receive[1]);  //最后一个字节是校验和

                // 计算校验和
                for(int i=0; i<data_len; i++)
                {
                    check_sum += valid_data[i];
                }
                //ROS_INFO("Len:%d,sum:%d\n", (int)data_len,check_sum);
                if( check_sum == valid_data[data_len] )
                {
                    uint8_t cnt = 0;
                    int data_tmp;

                    sensor_data.time = ros::Time::now();

                    // 获取里程计数据
                    data_tmp = (valid_data[cnt++]<<24) | (valid_data[cnt++]<<16) | (valid_data[cnt++]<<8) | valid_data[cnt++];
                    sensor_data.odom1 = -data_tmp / 1000.0 * 1.3772;  // 轮式里程计,单位m
                    //ROS_INFO("Get odom1 data:%f", sensor_data.odom1);

                    data_tmp = (valid_data[cnt++]<<24) | (valid_data[cnt++]<<16) | (valid_data[cnt++]<<8) | valid_data[cnt++];
                    sensor_data.odom2 = data_tmp / 1000.0 * 1.3772;
                    //ROS_INFO("Get odom2 data:%f", sensor_data.odom1);

                    // 获取速度数据
                    data_tmp = (short)((valid_data[cnt++]<<8) | valid_data[cnt++]);
                    sensor_data.vel1 = -data_tmp / 1000.0 * 1.3772;  // 车轮速度,m/s

                    data_tmp = (short)((valid_data[cnt++]<<8) | valid_data[cnt++]);
                    sensor_data.vel2 = data_tmp / 1000.0 * 1.3772;

                    // 获取超声传感器距离数据
                    data_tmp = (valid_data[cnt++]<<24) | (valid_data[cnt++]<<16) | (valid_data[cnt++]<<8) | valid_data[cnt++];
                    sensor_data.sonar_distance1 = *((float*)&data_tmp);

                    data_tmp = (valid_data[cnt++]<<24) | (valid_data[cnt++]<<16) | (valid_data[cnt++]<<8) | valid_data[cnt++];
                    sensor_data.sonar_distance2 = *((float*)&data_tmp);

                    data_tmp = (valid_data[cnt++]<<24) | (valid_data[cnt++]<<16) | (valid_data[cnt++]<<8) | valid_data[cnt++];
                    sensor_data.sonar_distance3 = *((float*)&data_tmp);

                    // 获取红外传感器距离数据
                    data_tmp = (valid_data[cnt++]<<24) | (valid_data[cnt++]<<16) | (valid_data[cnt++]<<8) | valid_data[cnt++];
                    sensor_data.ir_distance1 = *((float*)&data_tmp);

                    data_tmp = (valid_data[cnt++]<<24) | (valid_data[cnt++]<<16) | (valid_data[cnt++]<<8) | valid_data[cnt++];
                    sensor_data.ir_distance2 = *((float*)&data_tmp);

                    // 获取光电开关传感器数据
                    data_tmp = valid_data[cnt++];
                    sensor_data.photoelectric_switch1 = *((uint8_t*)&data_tmp);

                    data_tmp = valid_data[cnt++];
                    sensor_data.photoelectric_switch2 = *((uint8_t*)&data_tmp);

                    odom_publish();
                    range_publish();
                    sensorData_pub.publish(sensor_data);
                    //cout<<"time:"<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
                    //time_stt = clock();
                }
            }
        }
    }
}

/**
 * @brief 发布里程计数据
 *        两轮差分驱动
 */
void MotorDriver::odom_publish()
{
    static int init = 0;

    if(++init < 6)
    {
        last_data[0] = sensor_data.odom1;
        last_data[1] = sensor_data.odom2;

        //更新时间
        last_time = ros::Time::now().toSec();
        //init=true;
        return;
    }

    ros::Time time_now = ros::Time::now();  // 当前时间
    double dt = time_now.toSec() - last_time;  // 单位为秒
    //double dt = 0.33;
    last_time = time_now.toSec();
    //cout << "dt:" << dt*1000 << endl;

    //注意电源关闭之后，驱动器并没有马上掉电S
    //    double vel1 = (double)((double)(sensor_data.odom1 - last_data[0]) / dt);
    //    double vel2 = (double)((double)(sensor_data.odom2 - last_data[1]) / dt);
    double vel1 = (double)sensor_data.vel1 * 0.67;
    double vel2 = (double)sensor_data.vel2 * 0.67;

    //cout << "vel1:" << vel1 << " ";
    //cout << "vel2:" << vel2 << endl;

    last_data[0] = (double)sensor_data.odom1;
    last_data[1] = (double)sensor_data.odom2;

    //    cout << " sensor_data.odom1" << sensor_data.odom1;
    //    cout << " sensor_data.odom2" << sensor_data.odom2;

    // 计算前进速度
    vx = (vel1 + vel2) / 2.0;
    vy = 0.0;
    // 计算旋转角速度
    vth = (vel2 - vel1) / width_robot * vth_scale;

    //ROS_INFO("vel1:%f  vel2:%f  vth:%f", vel1, vel2, vth);

    double delta_x = (vx * cos(pose_th) - vy * sin(pose_th)) * dt;
    double delta_y = (vx * sin(pose_th) + vy * cos(pose_th)) * dt;
    double delta_th = vth * dt;
    //cout << "delta_th:" << delta_th << endl;

    // 积分得出位置坐标
    pose_x += (double)delta_x;
    pose_y += (double)delta_y;
    pose_th += (double)delta_th;
    //cout << "th:" << pose_th << endl;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time_now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pose_x;
    odom_trans.transform.translation.y = pose_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    // 发布base_link->odom的tf变换
    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = time_now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = pose_x;
    odom.pose.pose.position.y = pose_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    // 里程计数据发布
    odom_pub.publish(odom);
}

void MotorDriver::range_publish()
{
    motor_control_3dx::multi_range ultrasound_msg;

    ultrasound_msg.radiation_type = 0;
    ultrasound_msg.field_of_view = 60.0/180.0*3.14;
    ultrasound_msg.min_range = 0.10;
    ultrasound_msg.max_range = 5.00;

    ultrasound_msg.range.resize(3);
    ultrasound_msg.range[0] = sensor_data.sonar_distance1;
    ultrasound_msg.range[1] = sensor_data.sonar_distance2;
    ultrasound_msg.range[2] = sensor_data.sonar_distance3;

    ultrasound_msg.header.frame_id = "ultrasound";
    ultrasound_msg.header.stamp = ros::Time::now();  //时间

    range_pub.publish(ultrasound_msg);
}

/**
 * @brief 手柄消息的回调函数，接收上层传来的控制速度
 * @param twist 目标速度
 */
void MotorDriver::receiveSpeed(geometry_msgs::Twist twist)
{
    double vel_x, vel_th;  // 线速度,角速度
    double left_vel, right_vel;

    // 42减速电机,1:19速比,轮径100mm,1000Hz换向频率对应1.3772m/s(1000/4*20rpm/19/60*3.14*0.1)
    vel_x  = twist.linear.x;  //线速度 m/s
    vel_th = twist.angular.zj;  //角速度 rad/s 转化为弧度每秒

    left_vel  = vel_x - vel_th * width_robot / 2.0;  // 左轮速度
    right_vel = vel_x + vel_th * width_robot / 2.0;  // 右轮速度

    p1 = left_vel * (1000 / 1.3772) * 10;  //线速度，m/s->换向频率（换向频率除以电机极个数再乘以20为电机转速RPM）
    //p2 = p1;
    p2 = right_vel * (1000 / 1.3772) * 10;  //数值乘以0.1为目标换向频率
    //p4 = p3;

    //ROS_INFO("p1:%d  p2:%d", (int)p1, (int)p2);
    setSpeed();
}

/**
 * @brief 与下层控制板通信，设置电机速度，
 */
void MotorDriver::setSpeed()
{
    uint32_t _cnt = 0;
    uint8_t check_sum = 0;

    // 先发帧头
    send_buff[_cnt++] = 0xAA;
    send_buff[_cnt++] = 0xAA;
    // 功能码
    send_buff[_cnt++] = 0xF2;
    // 第四个字节为内容长度
    send_buff[_cnt++] = 0;

    send_buff[_cnt++] = BYTE0(is_cmd_valid);

    send_buff[_cnt++] = BYTE1(p1);
    send_buff[_cnt++] = BYTE0(p1);
    send_buff[_cnt++] = BYTE1(p2);
    send_buff[_cnt++] = BYTE0(p2);
    send_buff[_cnt++] = BYTE1(p3);
    send_buff[_cnt++] = BYTE0(p3);
    send_buff[_cnt++] = BYTE1(p4);
    send_buff[_cnt++] = BYTE0(p4);

    send_buff[3] = _cnt -4;  // 有效数据长度
    for(int i=0; i<_cnt; i++)
    {
        check_sum += send_buff[i];
    }

    send_buff[_cnt++] = check_sum;

    // 发送数据
    ros_ser.write(send_buff, _cnt);
    //ROS_INFO("Send Data OK");
}

/**
 * @brief 手柄消息的回调函数
 * @param stop 紧急停车保护指令
 */
void MotorDriver::protect(std_msgs::Float32 stop)
{
    if (stop.data == 1)
        is_cmd_valid = 0;  //指令有效标志置0，车轮锁止
    else
        is_cmd_valid = 1;  //指令有效标志置１
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MotorDriver");

    MotorDriver motorDriver;

    try
    {
        //设置串口属性，并打开串口
        ros_ser.setPort(usb2uart_port);  //端口
        ros_ser.setBaudrate((uint32_t)baud);  //波特率
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ros_ser.setTimeout(to);
        ros_ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ros_ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(10000);

    while (ros::ok())
    {
        //接收里程计数据
        motorDriver.get_odom();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
