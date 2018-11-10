#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/RCIn.h>

mavros_msgs::State current_state;
geometry_msgs::Twist uav_twist;
geometry_msgs::PoseStamped set_pose;
geometry_msgs::PoseStamped local_pose;
int channel_6;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}
void set_uav_local_pose(const geometry_msgs::PoseStampedConstPtr& msg)
{ 
    local_pose = *msg;
}
void read_rc_input(const mavros_msgs::RCIn::ConstPtr &rc_in)
{
    channel_6 = rc_in->channels[5];
   // std::cout << "channel_6 = " << channel_6 << std::endl;
}
// up 0.3m/s 3s; x 0.3m/s 3s; down 0.3m/s 3s;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_uav");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>
            ("/mavros/rc/in", 1, read_rc_input);
    ros::Subscriber uav_local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose",1, set_uav_local_pose);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 1);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // set point 
    double set_point_x = 0;
    double set_point_y = 0;
    double set_point_z = 0;
    nh.param<double>("/test_uav/set_point_x",set_point_x,0);
    nh.param<double>("/test_uav/set_point_y",set_point_y,0);
    nh.param<double>("/test_uav/set_point_z",set_point_z,0);

    uav_twist.linear.x = 0;
    uav_twist.linear.y = 0;
    uav_twist.linear.z = set_point_z;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(uav_twist);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
    /*
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
    */
        if( current_state.mode == "OFFBOARD")
            ROS_INFO("set OFFBOARD OK");
        if (fabs(1.0 - local_pose.pose.position.z) < 0.1 )
        {    
            uav_twist.linear.z = 0;
            //uav_twist.linear.y = -set_point_x;
        }
        if (channel_6 > 1500)
        {
            if(local_pose.pose.position.y < -0.5)
            {
                 uav_twist.linear.y = set_point_x;
            }
            if(local_pose.pose.position.y > 1.3)
            {
                 uav_twist.linear.y = -set_point_x; 
            } 
        }else
        {
            uav_twist.linear.y = 0; 
        }
        //local_pos_pub.publish(set_pose);
        local_vel_pub.publish(uav_twist);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
