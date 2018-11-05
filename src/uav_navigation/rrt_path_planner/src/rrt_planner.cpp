#include "ros/ros.h"
#include "Planner.h"
#include "nav_msgs/GetPlan.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include <eigen3/Eigen/Dense>
void octomapCallback(const octomap_msgs::OctomapConstPtr msg, Planner* planner_ptr)
{
	// convert octree to collision object
    auto tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));

	if (tree_oct)
    {
        // Update the octree used for collision checking
        std::shared_ptr<const octomap::OcTree> smart_tree_oct(tree_oct);
        planner_ptr->updateMap(smart_tree_oct);
        ROS_INFO("map updated!");
    }else {
        std::cout << "from base convert to derived failed!" << std::endl;
        delete tree_oct;
    }
}
bool planCallback(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res, Planner* planner_ptr)
{
    Vec3 start, goal;
    std::vector<Vec3> path;
    start.x = req.start.pose.position.x;
    start.y = req.start.pose.position.y;
    start.z = req.start.pose.position.z;
    goal.x = req.goal.pose.position.x;
    goal.y = req.goal.pose.position.y;
    goal.z = req.goal.pose.position.z;

    ROS_INFO("try to plan a path, \n"
             "start.x = %.3f start.y = %.3f start.z = %.3f\n"
             "goal.x = %.3f goal.y = %.3f goal.z = %.3f",
             start.x, start.y, start.z, goal.x, goal.y, goal.z);
    if(planner_ptr->planPath(start, goal, path))
    {
        res.plan.header.stamp = ros::Time::now();
        for (auto p : path)
        {
            geometry_msgs::PoseStamped planned_pose = req.start;
            planned_pose.pose.position.x = p.x;
            planned_pose.pose.position.y = p.y;
            planned_pose.pose.position.z = p.z;
            res.plan.poses.push_back(planned_pose);
        }
        return true;
    } else
        return false;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "rrt_planner_node");
	ros::NodeHandle n;
    Planner path_planner;
	ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>
	        ("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &path_planner));
	ros::ServiceServer service = n.advertiseService<nav_msgs::GetPlan::Request, nav_msgs::GetPlan::Response>
            ("/planner_server", boost::bind(&planCallback, _1, _2, &path_planner));
	ros::spin();
	return 0;
}
