//
// Changed by songjin on 18-11-5.
//
#include <opencv2/opencv.hpp>
#include "application/MainController.h"
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include "ros_common/RosMath.h"
#include <mavros_msgs/CommandBool.h>

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

    // ros message callback, 60HZ
    t_message_callback = std::thread(&MainController::ros_message_callback, this, 60);

    // control uav thread
    t_uav_control_loop = std::thread(&MainController::uav_control_loop, this, 1);
}

void MainController::ros_message_callback(int callback_rate) {
    // localization goal pose
    ros::Subscriber object_pose_sub = nh_.subscribe("/move_base_simple/goal", 1, &MainController::rviz_target_pose_callback, this);

    // uav pose subscribe
    ros::Subscriber uav_pose_sub = nh_.subscribe("/mavros/local_position/pose", 1, &MainController::uav_pose_callback, this);

    // exploration goal pose
    ros::Subscriber exploration_pose_sub = nh_.subscribe("/exploration_goalpose", 1, &MainController::exploration_pose_callback, this); 

    ros::Rate loop_rate(callback_rate);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MainController::uav_control_loop(int loop_rate) {
    ros::Rate rate(loop_rate);
    while (ros::ok()) {
        // std::cout << "step length = " << goal.step_length << std::endl;
        // std::cout << "z =  " << goal_pose.pose.position.z << std::endl;
        ac.sendGoal(goal);
        rate.sleep();
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
    flyInPlane(origin_pose_x, origin_pose_y, 0.3, 0.3);

    // 降落
    flyFixedHeight(origin_pose_z, 0.3, 0.3);

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

void MainController::flyFixedHeight(double z, double step_length, double precision) {
    //　起飞到一定的高度, x和y不变
    goal.goal_pose.pose.position.z = z;
    goal.step_length = step_length;
    goal.fly_type = "linear_planner_server";

    ROS_INFO("try to arrive at height %.3f meters, step_length = %.3f, "
             "precision = %.3f", z, step_length, precision);

    ros::Rate rate(10);
    int stable_count = 0;
    while (stable_count < 10) {
        if (fabs(z - uav_pose.pose.position.z) < precision) stable_count++;
        else stable_count = 0;
        rate.sleep();
        // std::cout << "stable count = " << stable_count << std::endl;
    }
    ROS_INFO("arrive at goal height, the actual height of uav is %.3f meters", uav_pose.pose.position.z);
}

void MainController::flyInPlane(double x, double y, double step_length, double precision) {
    // 高度和姿态不变,做平面运动
    goal.goal_pose.pose.position.x = x;
    goal.goal_pose.pose.position.y = y;
    goal.step_length = step_length;
    goal.fly_type = "rrt_planner_server";

    ROS_INFO("try to arrive at plane point x = %.3f, y = %.3f, z = %.3f, "
             "precision = %.3f, step_length = %.3f", x, y, goal.goal_pose.pose.position.z,
             precision, step_length);

    ros::Rate rate(10);
    int stable_count = 0;
    while (stable_count < 10) {
        if (RosMath::calDistance(x, uav_pose.pose.position.x, y, uav_pose.pose.position.y) < precision) stable_count++;
        else stable_count = 0;
        rate.sleep();
        // std::cout << "stable count = " << stable_count << std::endl;
    }
    ROS_INFO("arrive at goal plane point, the position of uav:x = %.3f, y = %.3f, z = %.3f", uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z);
}

void MainController::exploration()
{
    flyFixedHeight(0.8);

    while(exploration_goal_pose.pose.position.z > 0)
        flyInPlane(exploration_goal_pose.pose.position.x, exploration_goal_pose.pose.position.y, 0.3);  
}

void MainController::localization()
{
    flyFixedHeight(0.8);

    flyInPlane(rviz_target_pose.pose.position.x, rviz_target_pose.pose.position.y, 0.3);
}
