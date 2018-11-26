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
    bool flyFixedHeight(double z, double step_length = 0.1);

    /*
     * 让飞机在平面进行平移,维持目标高度不变
     */
    bool flyInPlane(double x, double y, double step_length , bool *is_plan_failed=nullptr);

    /*
     * 关闭飞机
     */
    void shutDownUav();

    /*
    * Stage1: let uav exploration in unknow enviroment 
    */
    void exploration();

    /*
    * Stage3: use optimized map to localiztion 
    */  
    void localization();

    void ros_message_callback(int callback_rate);

    void uav_control_loop(int loop_rate);
    /*
    * Use rviz to set the target point
    */
    void rviz_target_pose_callback(const geometry_msgs::PoseStamped &msg)
    {
        rviz_target_pose = msg;
        std::cout << " rviz_target_pose x = " << rviz_target_pose.pose.position.x 
                  << " rviz_target_pose y = " << rviz_target_pose.pose.position.y << std::endl;
    }

    void uav_pose_callback(const geometry_msgs::PoseStamped &msg)
    {
        uav_pose = msg;
    }

    void exploration_pose_callback(const geometry_msgs::PoseStamped &msg)
    {
        exploration_goal_pose = msg;
    }

  private:
    ros::NodeHandle nh_;
    uav_controller::FlyToGoalGoal goal;
    // 通过修改goal_pose来控制飞机,可以实现相对运动
    actionlib::SimpleActionClient<uav_controller::FlyToGoalAction> ac;
    ros::ServiceClient arming_client;
    ros::ServiceClient frontiers_client;
    ros::Publisher exploration_pose_pub;
    geometry_msgs::PoseStamped rviz_target_pose;
    geometry_msgs::PoseStamped uav_pose;
    geometry_msgs::PoseStamped exploration_goal_pose;
    std::thread t_message_callback;

    double origin_pose_x;
    double origin_pose_y;
    double origin_pose_z;

    // the return value of uav_controller is used to indicate when the target point can be reached.
    bool is_goal_reachable;
};

#endif //IYDC_TASKS_MAINCONTROLLER_H
