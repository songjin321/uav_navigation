#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <limits>

geometry_msgs::Pose uav_current_pose;
geometry_msgs::PoseStamped uav_goal_pose;
ros::Publisher exploration_pose_pub;

void map_callback(const nav_msgs::OccupancyGrid &map_occupancy)
{
    // std::cout << "begin callback!" << std::endl;
    // store the map in cv::Mat, assign 255 to the frontier
    // occupancy map x->down, y->right
    // cv::Mat x->right, y->down
    cv::Mat map(map_occupancy.info.width, map_occupancy.info.height, CV_8U, cv::Scalar::all(0));
    if (map.rows < 3 || map.cols < 3)
    {
        std::cout << "projected map is too small!" << std::endl;
        return;
    }
    for (int j = 1; j < map.rows - 1; j++)
    {
        uchar *data = map.ptr<uchar>(j);
        for (int i = 1; i < map.cols - 1; i++)
        {
            if (map_occupancy.data[j + i * map.rows] == 0 && 
            (map_occupancy.data[j - 1 + i * map.rows] == -1 
            || map_occupancy.data[j + 1 + i * map.rows] == -1 
            || map_occupancy.data[j + (i - 1) * map.rows] == -1 
            || map_occupancy.data[j + (i + 1) * map.rows] == -1))
                data[i] = 255;
        }
    }
    std::cerr << "convert projected map to cv::Mat Ok!" << std::endl;

    // Get the center of the closest connectable area
    cv::Mat frontiers, stats, centroids;
    int num_frontiers = cv::connectedComponentsWithStats(map, frontiers, stats, centroids);
    if (num_frontiers == 1)
    {
        std::cout << "Map exploration completed, uav will return to the origin " << std::endl;
        
        // let goal position.z < 0, means that the environment has been explored
        uav_goal_pose.pose.position.z = -1.0;
        return;
    }
    // 0 is the label of background
    // assume the orientation of map is [1 0 0 0 ]
    std::vector<geometry_msgs::Point> frontiers_center;
    double min_distance = std::numeric_limits<double>::max();
    for (int i = 1; i < num_frontiers; i++)
    {
        int area_size = stats.at<int>(i, cv::CC_STAT_AREA);
        std::cout << "the area size of components " << i << " = " << area_size << std::endl;
        if ( area_size < 15)
        {
            continue;
        }
        geometry_msgs::Point center;
        // the representation of the x and y axes is the opposite
        center.x = centroids.at<double>(i, 1) * map_occupancy.info.resolution + map_occupancy.info.origin.position.x;

        center.y = centroids.at<double>(i, 0) * map_occupancy.info.resolution + map_occupancy.info.origin.position.y;

        center.z = 0.0;

        double distance = std::sqrt((center.x - uav_current_pose.position.x) * (center.x - uav_current_pose.position.x)
         + (center.y - uav_current_pose.position.y) * (center.y - uav_current_pose.position.y));

        if (distance < min_distance)
        {
            min_distance = distance;
            uav_goal_pose.header.frame_id = "map";
            uav_goal_pose.pose.position.x = center.x;
            uav_goal_pose.pose.position.y = center.y;
            uav_goal_pose.pose.position.z = uav_current_pose.position.z;
        }
        frontiers_center.push_back(center);
    }
    std::cout << "the number of qualified frontiers is " << frontiers_center.size() << std::endl;

    // publish the exploration result
    exploration_pose_pub.publish(uav_goal_pose);
    
    // cv::imshow("map", map);
    // cv::waitKey(1);
    return;
}
void uav_pose_callback(const geometry_msgs::PoseStamped &msg)
{
    uav_current_pose = msg.pose;
}
// let robot explore enviorment with frontier-based method
int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_exploration_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/projected_map", 1, map_callback);
    // uav pose subscribe
    ros::Subscriber uav_pose_sub = nh.subscribe("/mavros/local_position/pose", 1, uav_pose_callback);
    exploration_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/exploration_pose", 1);
    ros::spin();
    return 0;
}