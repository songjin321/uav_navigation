//
// Created by songjin on 18-5-25.
//
#include "uav_controller/FlyToGoalActionServer.h"
#include "uav_controller/RosWrapperUAV.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_controller_server");
    RosWrapperUAV ros_uav("/vision_pose_world/pose");
    FlyToGoalActionServer flyToGoalActionServer("uav_controller_server",&ros_uav);
    ros::spin();

    return 0;
}
