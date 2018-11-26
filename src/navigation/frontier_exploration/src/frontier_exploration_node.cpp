#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <limits>
#include "frontier_exploration/GetFrontiers.h"
nav_msgs::OccupancyGrid map_occupancy;

void map_callback(const nav_msgs::OccupancyGrid &map_occupancy_)
{
    map_occupancy = map_occupancy_;
    return;
}
bool generate_frotiers(frontier_exploration::GetFrontiers::Request &req,
                       frontier_exploration::GetFrontiers::Response &res)
{
    // store the map in cv::Mat, assign 255 to the frontier
    // occupancy map x->down, y->right
    // cv::Mat x->right, y->down
    cv::Mat map(map_occupancy.info.width, map_occupancy.info.height, CV_8U, cv::Scalar::all(0));
    if (map.rows < 3 || map.cols < 3)
    {
        std::cout << "projected map is too small!" << std::endl;
        return false;
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
    std::cout << "convert projected map to cv::Mat Ok!" << std::endl;
    cv::imshow("map", map);
    cv::waitKey(1);

    // Get the center of the closest connectable area
    cv::Mat frontiers, stats, centroids;
    int num_frontiers = cv::connectedComponentsWithStats(map, frontiers, stats, centroids);
    if (num_frontiers == 1)
    {
        // std::cout << "Map exploration completed" << std::endl;
        return true;
    }
    // 0 is the label of background
    // assume the orientation of map is [1 0 0 0 ]

    for (int i = 1; i < num_frontiers; i++)
    {
        int area_size = stats.at<int>(i, cv::CC_STAT_AREA);
        // std::cout << "the area size of components " << i << " = " << area_size << std::endl;
        if ( area_size < req.min_area_size)
        {
            continue;
        }
        geometry_msgs::Point center;
        // the representation of the x and y axes is the opposite
        center.x = centroids.at<double>(i, 1) * map_occupancy.info.resolution + map_occupancy.info.origin.position.x;

        center.y = centroids.at<double>(i, 0) * map_occupancy.info.resolution + map_occupancy.info.origin.position.y;

        center.z = 0.0;

        res.frontier_points.push_back(center);
    }
    return true;
}

// let robot explore enviorment with frontier-based method
int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_exploration_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/projected_map", 1, map_callback);
    ros::ServiceServer service = nh.advertiseService("/frontiers_server", generate_frotiers);
    ros::spin();
    return 0;
}