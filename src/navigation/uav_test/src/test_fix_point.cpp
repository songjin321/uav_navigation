#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped set_pose;
geometry_msgs::PoseStamped origin_pose;
geometry_msgs::PoseStamped local_pose;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
void set_uav_local_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    local_pose = *msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_fix_point");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber uav_local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, set_uav_local_pose);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // set point
    double set_point_x = 0;
    double set_point_y = 0;
    double set_point_z = 0;
    nh.param<double>("/uav_test/set_point_x", set_point_x, 0);
    nh.param<double>("/uav_test/set_point_y", set_point_y, 0);
    nh.param<double>("/uav_test/set_point_z", set_point_z, 0);
    set_pose.pose = local_pose.pose;
    set_pose.pose.position.x = set_point_x;
    set_pose.pose.position.y = set_point_y;
    set_pose.pose.position.z = set_point_z;
    set_pose.pose.orientation.w = 1;

    // origin pose
    origin_pose.pose = local_pose.pose;
    origin_pose.pose.position.x = 0;
    origin_pose.pose.position.y = 0;
    origin_pose.pose.position.z = 0;
    origin_pose.pose.orientation.w = 1;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(set_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    while (ros::ok())
    {
        if (current_state.mode == "OFFBOARD")
            ROS_INFO("set OFFBOARD OK");
        local_pos_pub.publish(set_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
