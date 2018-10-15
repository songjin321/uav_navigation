//
// Created by songjin on 18-5-25.
//

#ifndef UAV_CONTROLLER_FLYTOGOALACTION_H
#define UAV_CONTROLLER_FLYTOGOALACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "uav_controller/FlyToGoalAction.h"
#include "RosWrapperUAV.h"
#include "nav_msgs/Path.h"

/*
 *  provide a action server, let uav fly to a goal point.
 */
class FlyToGoalActionServer {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<uav_controller::FlyToGoalAction> as_;
    std::string action_name_;
    uav_controller::FlyToGoalFeedback feedback_;
    uav_controller::FlyToGoalResult result_;
public:
    FlyToGoalActionServer(std::string name, RosWrapperUAV *ros_uav);

    void executeCB(const uav_controller::FlyToGoalGoalConstPtr &goal);

private:
    /*
     * you can add other action class object below
     * fly circle
     * fly follow
     */
    bool generatePath(const std::string &path_planner_name,
                      double step_length,
                      const geometry_msgs::PoseStamped &start_pose,
                      const geometry_msgs::PoseStamped &goal_pose,
                      nav_msgs::Path &path);

    RosWrapperUAV *p_ros_uav_;
};


#endif //UAV_CONTROLLER_FLYTOGOALACTION_H
