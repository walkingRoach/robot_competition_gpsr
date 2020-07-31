#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Int32.h> 
#include <std_msgs/Empty.h> 
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "kw_arm/my_ik_solver.h"
#include "kw_arm/target.h"
#include <math.h>
#include <string>
using namespace::std;

static const double L0_LENGTH = 100.0;
static const double L1_LENGTH = 250.0;
static const double L2_LENGTH = 250.0;
static const double L3_LENGTH = 110.0;

//local params
string SerialPort = "/dev/ttyACM0";
int InitX = 0;
int InitY = 0;
int InitZ = 150;
int InitPitch = 0;
int ForwardDistance = 20;
int GripDistance = 30;

serial::Serial Ser; //声明串口对象 


//Inverse Kinematics
my_ik_solver MySolver(L0_LENGTH, L1_LENGTH, L2_LENGTH, L3_LENGTH);

//Global State
bool TargetReceived = false;
bool HeightReceived = false;
double TargetX, TargetY, TargetZ;
int TargetHeight;
int CurrentJoint1 = 0, CurrentJoint2 = 40, CurrentJoint3 = 140, CurrentJoint4 = -90;
int CurrentX, CurrentY, CurrentZ;


int GetDistance()  
{
    Ser.flushInput();
    Ser.write("zsed");
    string str;
    ros::Rate read_loop(30);
    //返回的状态标识符
    int distance = -1;
    while (distance == -1)
    {
        if(Ser.available()){  
            str = Ser.read(Ser.available()); 
            //ROS_INFO("content : %s", str.c_str());
            int length = str.length();
            ROS_INFO("length: %d", length);
            if (str[length - 1] == '\n' && str[length - 2] == 'm' && str[length - 3] == 'm')
            {
                distance = atoi(str.substr(0, length - 3).c_str());
                ROS_INFO("distance : %d", distance);
                return distance;
            }
            return 0;
        }
        read_loop.sleep();
    }
    return 0;
}

void Arm_Reset()
{
    string str;
     //清空缓冲区
    Ser.flushInput();
    Ser.write("zoed");

    //等待结束运行
    ros::Rate read_loop(30);

    //返回的状态标识符
    char state = 'a';
    while (state != 'b')
    {
        if(Ser.available()){  
            str = Ser.read(Ser.available()); 
            ROS_INFO("Reseting...Serial feedback: %c", str[0]);
            int length = str.length();
            if (length == 2 && str[1] == '\n')
            {
                state = str[0];
            }
            else
            {
                Ser.flushInput();
            }
        }
        read_loop.sleep();
    }
}

bool Arm_SendAngle(int theta1, int theta2, int theta3, int theta4)
{
    char cmd[100];
    //sprintf(cmd, "%d", theta1); 
    sprintf(cmd, "zt%d,%d,%d,%ded", theta1, theta2, theta3, theta4); 
    ROS_INFO("%s", cmd);
    // string str(cmd);
    // ROS_INFO("%s", str.c_str());
    // int length = str.length();
    // ROS_INFO("%d", length);
    Ser.write(cmd);
    
    string str;
    ros::Rate read_loop(30);
    char state = 'a';
    while (state != 'b' && state != 'e')
    {
        if(Ser.available()){  
            str = Ser.read(Ser.available()); 
            ROS_INFO("Moving...Serial feedback: %s", str.c_str());
            int length = str.length();
            if (length == 2 && str[1] == '\n')
            {
                state = str[0];
            }
            else
            {
                Ser.flushInput();
            }
        }
        read_loop.sleep();
    }
    if (state == 'b')
    {
        CurrentJoint1 = theta1;
        CurrentJoint2 = theta2;
        CurrentJoint3 = theta3;
        CurrentJoint4 = theta4;
        return true;
    }
    else if (state == 'e')
    {
        return false;
    }
}

void Arm_Grip()
{
    Ser.write("zg1ed");
    ROS_INFO("Grip...");
    sleep(1);
    ROS_INFO("Grip...finished");
}

void Arm_Loose()
{
    Ser.write("zg0ed");
    ROS_INFO("Loose...");
    sleep(1);
    ROS_INFO("Loose...finished");
}

bool Arm_MoveToPose(double x, double y, double z, double pitch)
{
    int n = MySolver.Solve(x, y, z, 0);
    if (n == 0)
    {
        ROS_INFO("No solution!!!");
        return false;
    }
    double d_angles[4];
    int i_angles[4];
    d_angles[0] = MySolver.GetJoint0Angles(0);
    d_angles[1] = MySolver.GetJoint1Angles(0);
    d_angles[2] = MySolver.GetJoint2Angles(0);
    d_angles[3] = MySolver.GetJoint3Angles(0);
    for (int i = 0; i < 4; i++)
    {
        i_angles[i] = (d_angles[i] * 180 / 3.141593);
        ROS_INFO("Joint%d : %d", i, i_angles[i]);
    }
    if (Arm_SendAngle(i_angles[0],i_angles[1], i_angles[2], i_angles[3]) == true)
    {
        CurrentX = x;
        CurrentY = y;
        CurrentZ = z;
        return true;
    }
    else
    {
        return false;
    }
}

//回调函数 
void target_callback(const kw_arm::targetConstPtr& msg) 
{ 
    TargetX = msg->x;
    TargetY = msg->y;
    TargetZ = msg->z;
    ROS_INFO("Target received: %.2lf %.2lf %.2lf", msg->x, msg->y, msg->z); 
    TargetReceived = true;
    //Ser.write(msg->data);   //发送串口数据 
} 

void height_callback(const std_msgs::Int32ConstPtr& msg) 
{ 
    TargetHeight = msg->data;
    ROS_INFO("Height received: %d", TargetHeight); 
    HeightReceived = true;
    //Ser.write(msg->data);   //发送串口数据 
} 

int Arm_scan()
{
    const int samples[5] = {-4, 4, 4, 0, 0};
    int angles[5] = {0};
    int min = 256, min_angle;
    for (int i = 0; i < 3; i++)
    {
        int sum = 0;
        int once = 0;
        Arm_SendAngle(CurrentJoint1 + samples[i], CurrentJoint2, CurrentJoint3, CurrentJoint4);
        //sleep(1);
        int count = 0;
        while (count < 4)
        {
            once = GetDistance();
            if (once != 0)
            {
                count++;
                sum += once;
            }
        }
        angles[i] = sum / 4;
        if (angles[i] < min)
        {
            min = angles[i];
            min_angle = CurrentJoint1;
        }
    }
    ROS_INFO("min_angle: %d", min_angle);
    ROS_INFO("min_distance: %d", min);
    Arm_SendAngle(min_angle, CurrentJoint2, CurrentJoint3, CurrentJoint4);
    return min;
}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "FixedMotionController"); 
    ros::NodeHandle nh;
    ros::NodeHandle pn("~"); //私有参数

    pn.param<std::string>("SerialPort", SerialPort, "/dev/kw_arm");
    pn.param<int>("InitX", InitX, 150);
    pn.param<int>("InitY", InitY, 0);
    pn.param<int>("InitZ", InitZ, 150);
    pn.param<int>("InitPitch", InitPitch, 0);
    pn.param<int>("ForwardDistance", ForwardDistance, 200);
    pn.param<int>("GripDistance", GripDistance, 30);
    // pn.param("L", L, 0.33);
    // pn.param("Lrv", Lrv, 10.0);
    // pn.param("lfw", lfw, 0.165); 
  
    ros::Subscriber target_sub = nh.subscribe("arm_target", 10, target_callback); 
    ros::Subscriber height_sub = nh.subscribe("arm_height", 10, height_callback);
    
    ros::Publisher result_pub = nh.advertise<std_msgs::Int32>("arm_result", 10); 

    try 
    { 
    //设置串口属性，并打开串口 
        Ser.setPort(SerialPort); 
        Ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout(1, 5, 0, 5, 0);
        Ser.setTimeout(to); 
        Ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
    //检测串口是否已经打开，并给出提示信息 
    if(Ser.isOpen()) 
    {   
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    Arm_Reset();
    if (Arm_MoveToPose(InitX, InitY, InitZ, InitPitch) == false)
    {
        ROS_INFO("Init failed...Pose out of range");
        return 0;
    }
    //Arm_scan();
        // while(1)
        // {
        //     GetDistance();
        // }

    ros::Rate loop_rate(30); 
    while(ros::ok()) 
    { 
        if (TargetReceived == true)
        {
            std_msgs::Int32 result_msg;
            result_msg.data = 1;
            if ((int)(TargetX) == -2) {
                ROS_INFO("start close");
                Arm_Grip();
            }
            else if ((int)(TargetX) == -3) Arm_Loose();
            else if ((int)(TargetX) == -4) Arm_Reset();
            else 
            {
              if (!Arm_MoveToPose(TargetX, TargetY, TargetZ, 0))
              {
                  result_msg.data = 0;
              }
            }
            TargetReceived = false;
            result_pub.publish(result_msg);
        }
        if (HeightReceived == true)
        {
            Arm_Loose();
            Arm_MoveToPose(ForwardDistance, 0, TargetHeight, 0);
            ROS_INFO("forward distance: %d", ForwardDistance);
            int distance = Arm_scan();
            std_msgs::Int32 result_msg;
            if (distance < 255)
            {
                double length = distance + ForwardDistance - GripDistance;
                double RadianAngle = CurrentJoint1 / 180.0 * 3.141593;
                int new_x = length * cos(RadianAngle);
                int new_y = - length * sin(RadianAngle);
                bool result = Arm_MoveToPose(new_x, new_y, TargetHeight, 0);
                if (result == true)
                {
                    result_msg.data = 1;
                    Arm_Grip();
                }
                else
                {
                    ROS_INFO("Failed to move to pose. ");
                    result_msg.data = 0;
                }     
            }
            else
            {
                ROS_INFO("Object not detected.");
                result_msg.data = -1;
            }
            result_pub.publish(result_msg);
            HeightReceived = false;
        }
        if(Ser.available()){ 
            string str = Ser.read(Ser.available()); 
            int length = str.length(); 
            //ROS_INFO("Serial feedback: %c", str[0]);
        } 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
} 