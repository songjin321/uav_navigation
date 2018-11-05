#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include <tf/transform_datatypes.h>
#include "ros_common/RosMath.h"

bool calLinePath(nav_msgs::GetPlan::Request  &req,
                 nav_msgs::GetPlan::Response &res)
{
    double dx = req.goal.pose.position.x - req.start.pose.position.x;
    double dy = req.goal.pose.position.y - req.start.pose.position.y;
    double dz = req.goal.pose.position.z - req.start.pose.position.z;
    double path_length = sqrt(dx*dx + dy*dy + dz*dz);
    res.plan.header = req.start.header;

    // Guaranteed posture is unchanged
    geometry_msgs::PoseStamped planned_pose = req.goal;
    planned_pose.header.frame_id = "local";

    // let tolerance as step length
    double step_length = req.tolerance;
    double steps = path_length/step_length;
    for (int i = 1; i < steps; i++)
    {
        //　规划的点的时间怎么确定
        planned_pose.pose.position.x = req.start.pose.position.x + i*dx/steps;
        planned_pose.pose.position.y = req.start.pose.position.y + i*dy/steps;
        planned_pose.pose.position.z = req.start.pose.position.z + i*dz/steps;
        //planned_pose.pose.position.z = req.goal.pose.position.z;
        res.plan.poses.push_back(planned_pose);
    }
    planned_pose.pose.position = req.goal.pose.position;
    planned_pose.pose.orientation = req.goal.pose.orientation;
    res.plan.poses.push_back(planned_pose);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "linear_planner_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("linear_planner_server", calLinePath);
    ROS_INFO("Ready to calculate position_line path.");
    ros::spin();

    return 0;
}
