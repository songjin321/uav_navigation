//
// Created by songjin on 18-5-25.
//

#ifndef UAV_CONTROLLER_ROSWRAPPERUAV_H
#define UAV_CONTROLLER_ROSWRAPPERUAV_H

#include "nav_msgs/GetPlan.h"
#include "uav_controller/RosWrapperUAV.h"
#include <ros/ros.h>
/*
 * 保存无人机的状态,利用mavros底层控制无人机运动
 * 利用glog,将飞行状态记录下来
 */
class RosWrapperUAV
{
public:
    RosWrapperUAV(std::string vision_pose_name);
    void vision_pose_callback(const geometry_msgs::PoseStamped &vision_pose);
    void uav_local_pose_callback(const geometry_msgs::PoseStamped &uav_local_pose);
    void fly_to_goal(const geometry_msgs::PoseStamped &goal_pose, double fly_vel);
    geometry_msgs::PoseStamped getCurrentPoseStamped();
private:
    ros::NodeHandle n_;
    ros::Subscriber vision_pose_sub_;
    ros::Subscriber uav_local_pose_sub_;
    ros::Publisher mavros_position_pub_;
    ros::Publisher mavros_vision_pose_pub_;
    ros::Publisher mavros_attitute_pub_;
    ros::Publisher mavros_velocity_pub_;
    /*
     * 时刻发布出去的视觉估计的姿态
     */
    geometry_msgs::PoseStamped uav_pose_pub_;

    /*
     * 从mavros返回回来的无人机的位姿
     */
    geometry_msgs::PoseStamped uav_pose_;
    /*
     * true表示视觉的位姿可用, false表示不可用
     */
    bool vision_pose_ok_flag;
};

#endif
