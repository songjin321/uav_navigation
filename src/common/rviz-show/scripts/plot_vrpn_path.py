#!/usr/bin/env python

# publish path and tf transformation
import rospy
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

path = Path()
def callback_ts(ts):
    path.header.stamp = rospy.get_rostime()
    path.header.frame_id = "map"
    pose_stamped = PoseStamped()
    pose_stamped.header = path.header
    pose_stamped.pose = ts.pose
    path.poses.append(pose_stamped)
    path_pub.publish(path)
    print "publish a path and transform"

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/vrpn_pose", PoseStamped, callback_ts)

    path_pub = rospy.Publisher('/vrpn_path', Path, queue_size=1)

    rospy.spin()
