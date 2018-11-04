#include "ros/ros.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

using namespace std;

ros::Publisher body_puber;

// camera relative to body coordinate
double x = 0;
double y = 0;
double z = 0;

// from body rotate to camera coordinate frame, z->yaw, y->pitch, x->roll
double yaw = 0;
double pitch = 0;
double roll = 0;

void PoseStamped_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    tf::Transform T_M_C;
    tf::poseMsgToTF(msg->pose, T_M_C);

    tf::Quaternion q;
    q.setEulerZYX(yaw, pitch, roll);
    // std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
	tf::Transform T_B_C = tf::Transform(q, tf::Vector3(x, y, z));

	tf::Transform T_W_B = T_B_C * T_M_C * T_B_C.inverse();

        //q.setEulerZYX(world_yaw, 0, 0);
	//tf::Transform T_P_W = tf::Transform(q, tf::Vector3(0, 0, 0));
	//tf::Transform T_P_B = T_P_W * T_W_B;

	geometry_msgs::PoseStamped current_pose;
	current_pose.header.stamp = msg->header.stamp;
	current_pose.header.frame_id = msg->header.frame_id;
	tf::poseTFToMsg(T_W_B, current_pose.pose);

	body_puber.publish(current_pose);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "map_world_convert");
  	ros::NodeHandle n;

	std::string vision_pose_map_name;
	std::string vision_pose_world_name;
    n.param<std::string>("/map_world_convert/vision_pose_map_name",vision_pose_map_name,"/vision_pose_map/pose");
	n.param<std::string>("/map_world_convert/vision_pose_world_name",vision_pose_world_name,"/vision_pose_world/pose");
	n.param<double>("/map_world_convert/yaw",yaw,0);
	n.param<double>("/map_world_convert/pitch",pitch,0);
	n.param<double>("/map_world_convert/roll",roll,0);
	n.param<double>("/map_world_convert/x",x,0);
	n.param<double>("/map_world_convert/y",y,0);
	n.param<double>("/map_world_convert/z",z,0);
	yaw  = yaw/180.0*M_PI;
	roll  = roll/180.0*M_PI;
	pitch  = pitch/180.0*M_PI;
  	ros::Subscriber pose_feedback=n.subscribe(vision_pose_map_name, 1, PoseStamped_Callback);
  	body_puber = n.advertise<geometry_msgs::PoseStamped>(vision_pose_world_name, 1);
  	
  	ros::spin();

	return 0;

}
