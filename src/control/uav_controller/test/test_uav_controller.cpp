// 控制无人机升高0.3ｍ

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <uav_controller/FlyToGoalAction.h>

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const uav_controller::FlyToGoalResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("FinalDistance: %f", result->final_distance);
    ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const uav_controller::FlyToGoalFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback of distance %f", feedback->distance);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_uav_controller");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<uav_controller::FlyToGoalAction> ac("uav_controller_server", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    uav_controller::FlyToGoalGoal goal;
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose.position.z = 0.2;
    goal.goal_pose = goal_pose;
    goal.fly_type = "line_planner_server";
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();
    //exit
    return 0;
}