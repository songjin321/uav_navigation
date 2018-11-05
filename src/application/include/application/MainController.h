//
// Created by songjin on 18-6-4.
//

#ifndef IYDC_TASKS_MAINCONTROLLER_H
#define IYDC_TASKS_MAINCONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "uav_controller/FlyToGoalAction.h"
#include <thread>
#include <vector>

class MainController
{
public:
    MainController(std::string uav_controller_server_name);

    void init();

    /*
     * 返回原点
     */
    void returnToOrigin();

    /*
     * 飞到固定的高度,维持目标位置不变
     */
    void flyFixedHeight(double z, double step_length = 0.1, double precision = 0.1);

    /*
     * 让飞机在平面进行平移,维持目标高度不变
     */
    void flyInPlane(double x, double y, double step_length=0.1, double precision = 0.1);

    /*
     * 关闭飞机
     */
    void shutDownUav();

    /*
    * let uav exploration in unknow enviroment 
    */
    void exploration()
    {
    }

    void ros_message_callback(int callback_rate);
    
    void uav_control_loop(int loop_rate);
    /*
    * Use rviz to set the target point
    */
    void rviz_target_pose_callback(const geometry_msgs::PoseStamped &msg);

    void uav_pose_callback(const geometry_msgs::PoseStamped &msg);
private:
    ros::NodeHandle nh_;
    uav_controller::FlyToGoalGoal goal;
    // 通过修改goal_pose来控制飞机,可以实现相对运动
    actionlib::SimpleActionClient<uav_controller::FlyToGoalAction> ac;
    ros::ServiceClient arming_client;
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped uav_pose;
    std::thread t_message_callback;
    std::thread t_uav_control_loop;

    double origin_pose_x;
    double origin_pose_y;
    double origin_pose_z;
};


#endif //IYDC_TASKS_MAINCONTROLLER_H
