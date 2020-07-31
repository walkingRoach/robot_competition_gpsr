#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "kw_arm/MoveToPoseAction.h"
 
typedef actionlib::SimpleActionClient<kw_arm::MoveToPoseAction> Client;
 
// 当action完成后会调用次回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const kw_arm::MoveToPoseResultConstPtr& result)
{
    ROS_INFO("Yay! The dishes are now clean");
    //ros::shutdown();
}
 
// 当action激活后会调用次回调函数一次
void activeCb()
{
    ROS_INFO("Goal just went active");
}
 
// 收到feedback后调用的回调函数
void feedbackCb(const kw_arm::MoveToPoseFeedbackConstPtr& feedback)
{
    ROS_INFO(" percent_complete : %f ", feedback->percent_complete);
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "do_dishes_client");
 
    // 定义一个客户端
    Client client("move_to_pose", true);
 
    // 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");
 
    // 创建一个action的goal
    kw_arm::MoveToPoseGoal goal;
    goal.pose_x = 1.0;
 
    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);
 
    ros::spin();
 
    return 0;
}