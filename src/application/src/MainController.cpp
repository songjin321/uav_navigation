//
// Changed by songjin on 18-11-5.
//
#include <opencv2/opencv.hpp>
#include "application/MainController.h"
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include "ros_common/RosMath.h"
#include <mavros_msgs/CommandBool.h>
#include "frontier_exploration/GetFrontiers.h"
#include "uav_controller/FlyToGoalAction.h"
#include <algorithm>
MainController::MainController(std::string uav_controller_server_name) :
        ac(uav_controller_server_name, true) {
    ROS_INFO("Waiting for uav_controller_server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("uav_controller_server start!");

    // init goal pose and goal type
    goal.goal_pose.pose.orientation.w = 1.0;
    goal.fly_type = "rrt_planner_server";
    goal.step_length = 0.1;

    // uav arming command
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    // frontiers client
    frontiers_client = nh_.serviceClient<frontier_exploration::GetFrontiers>("/frontiers_server");

    // exploration goal pose
    exploration_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/exploration_pose", 1);

    // ros message callback, 60HZ
    t_message_callback = std::thread(&MainController::ros_message_callback, this, 60);
}

void MainController::ros_message_callback(int callback_rate) {
    // localization goal pose
    ros::Subscriber object_pose_sub = nh_.subscribe("/move_base_simple/goal", 1, &MainController::rviz_target_pose_callback, this);

    // uav pose subscribe
    ros::Subscriber uav_pose_sub = nh_.subscribe("/mavros/local_position/pose", 1, &MainController::uav_pose_callback, this);

    ros::Rate loop_rate(callback_rate);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MainController::init() {
    origin_pose_x = uav_pose.pose.position.x;
    origin_pose_y = uav_pose.pose.position.y;
    origin_pose_z = uav_pose.pose.position.z;

    goal.goal_pose.pose.position.x = origin_pose_x;
    goal.goal_pose.pose.position.y = origin_pose_y;
}

void MainController::returnToOrigin() {
    //　返回到原点上方
    flyInPlane(origin_pose_x, origin_pose_y, 0.3);

    // 降落
    flyFixedHeight(origin_pose_z, 0.3);

    // close uav
    shutDownUav();
}

void MainController::shutDownUav() {
    //　TODO::关闭飞机
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle locked");
        ROS_INFO("shut down uav!");
    } else {
        ROS_INFO("shut down uav faled!");
    }
}

bool MainController::flyFixedHeight(double z, double step_length) {
    //　起飞到一定的高度, x和y不变
    goal.goal_pose.pose.position.z = z;
    goal.step_length = step_length;
    goal.fly_type = "linear_planner_server";
    ac.sendGoal(goal);
    ROS_INFO("try to arrive at height %.3f meters, step_length = %.3f", z, step_length);
    ac.waitForResult();
    const uav_controller::FlyToGoalResultConstPtr result = ac.getResult();
    if (!result->is_reachable)
    {
        return false;
    }
    ROS_INFO("arrive at goal height, the actual height of uav is %.3f meters", uav_pose.pose.position.z);
    return true;
}

bool MainController::flyInPlane(double x, double y, double step_length) {
    // 高度和姿态不变,做平面运动
    goal.goal_pose.pose.position.x = x;
    goal.goal_pose.pose.position.y = y;
    goal.step_length = step_length;
    goal.fly_type = "rrt_planner_server";
    ac.sendGoal(goal);
    ROS_INFO("try to arrive at plane point x = %.3f, y = %.3f, z = %.3f, step_length = %.3f", 
    x, y, goal.goal_pose.pose.position.z,step_length);
    ac.waitForResult();
    const uav_controller::FlyToGoalResultConstPtr result = ac.getResult();
    if (!result->is_reachable)
    {
        return false;
    }
    ROS_INFO("arrive at goal plane point, the position of uav:x = %.3f, y = %.3f, z = %.3f", uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z);
    return true;
}

void MainController::exploration()
{
    flyFixedHeight(0.8, 0.3);

    bool is_exploration_finished = false;
    while(!is_exploration_finished)
    {
        frontier_exploration::GetFrontiers srv;
        srv.request.min_area_size = 15;     
        if (frontiers_client.call(srv))
        {
            if (srv.response.frontier_points.size() == 0)
            {
                is_exploration_finished = true;
                ROS_INFO("exploration finished!");
            }else
            {
                // sort by distance from uav
                // fly to closest region center
                // If all region center are unreachable, the exploration is finished
                std::vector<geometry_msgs::Point> frontier_points;
                frontier_points = srv.response.frontier_points;
                std::cout << "the number of qualified frontiers is " << frontier_points.size() << std::endl;     
                sort(frontier_points.begin(), frontier_points.end(), [this](const geometry_msgs::Point& a, const geometry_msgs::Point& b)
                {
                    double distance_a = std::sqrt((a.x - uav_pose.pose.position.x) * (a.x - uav_pose.pose.position.x)
                                      + (a.y - uav_pose.pose.position.y) * (a.y - uav_pose.pose.position.y));
                    double distance_b = std::sqrt((b.x - uav_pose.pose.position.x) * (b.x - uav_pose.pose.position.x)
                                      + (b.y - uav_pose.pose.position.y) * (b.y - uav_pose.pose.position.y));
                    return distance_a < distance_b;
                });
                is_exploration_finished = true;
                for (auto center : frontier_points)
                {
                    geometry_msgs::PoseStamped exploration_goal_pose;
                    exploration_goal_pose.header.frame_id = "map";
                    exploration_goal_pose.pose.position.x = center.x;
                    exploration_goal_pose.pose.position.y = center.y;
                    exploration_pose_pub.publish(exploration_goal_pose);
                    if (flyInPlane(center.x, center.y, 0.3))
                    {
                        is_exploration_finished = false;
                        break;
                    } 
                }
            }
        }else
        {
            ROS_ERROR("Failed to call frontier service!");
        }
    }
    ROS_INFO("exploration finished!");  
}

void MainController::localization()
{
    flyFixedHeight(0.8, 0.3);

    while(!flyInPlane(rviz_target_pose.pose.position.x, rviz_target_pose.pose.position.y, 0.3))
    {
        ROS_ERROR("The goal point is unreachable, please specify a suitable goal point!");
    }
}
