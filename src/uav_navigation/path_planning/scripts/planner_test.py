#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped

start = PoseStamped()
goal = PoseStamped()
start_point = PointStamped()
goal_point = PointStamped()

start.pose.position.x = 0
start.pose.position.y = 0
goal.pose.position.x = 0
goal.pose.position.y = 0

def callback_start(data):
    global start
    start.pose = data.pose.pose
    start.pose.position.z = 0.5
    start_point.point.x =  start.pose.position.x
    start_point.point.y =  start.pose.position.y
    start_point.point.z =  start.pose.position.z
    start_pub.publish(start_point)
    print "set start point"

def callback_goal(data):
    global goal
    goal = data
    goal.pose.position.z = 0.5
    goal_point.point.x =  goal.pose.position.x
    goal_point.point.y =  goal.pose.position.y
    goal_point.point.z =  goal.pose.position.z
    goal_pub.publish(goal_point)
    print "set goal point"
    path = get_path()

def get_path():
    rospy.wait_for_service('planner_server')
    try:
        print "try to plan a path"
        plan_path = rospy.ServiceProxy('planner_server', GetPlan)
        resp = plan_path(start, goal, 1)
        resp.plan.header.frame_id = "map"
        path_pub.publish(resp.plan)
        return resp.plan
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("initialpose", PoseWithCovarianceStamped, callback_start)

    rospy.Subscriber("move_base_simple/goal", PoseStamped, callback_goal)

    start_pub = rospy.Publisher('start/clicked_point', PointStamped, queue_size=1)

    goal_pub = rospy.Publisher("goal/clicked_point", PointStamped, queue_size=1)

    path_pub = rospy.Publisher('/path', Path, queue_size=1)
    rospy.spin()
'''
    rate = rospy.Rate(0.2)

    while not rospy.is_shutdown():
        path = get_path()
        rate.sleep()
'''
