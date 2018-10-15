//
// Created by songjin on 18-5-30.
//

#include "ros_common/RosMath.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

double RosMath::calDistance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    double dz = p1.pose.position.z - p2.pose.position.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}
double RosMath::calDistance(double x1, double x2, double y1, double y2, double z1, double z2)
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    double dz = z1 - z2;
    return sqrt(dx*dx + dy*dy + dz*dz);
}
double RosMath::getYawFromPoseStamp(const geometry_msgs::PoseStamped &p)
{
    tf::Quaternion q(
            p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
            p.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return  yaw;
}

void RosMath::getRPYFromPoseStamp(const geometry_msgs::PoseStamped &p,
                                  double &roll, double &pitch, double &yaw)
{
    tf::Quaternion q(
            p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
            p.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}
