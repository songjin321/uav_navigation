#!/usr/bin/env python

# publish path and tf transformation
import rospy
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

path = Path()
def callback_ts(ts):
    path.header = ts.header
    path.header.frame_id = "map"
    pose_stamped = PoseStamped()
    pose_stamped.header = path.header
    pose_stamped.pose.position.x = ts.transform.translation.x
    pose_stamped.pose.position.y = ts.transform.translation.y
    pose_stamped.pose.position.z = ts.transform.translation.z
    pose_stamped.pose.orientation = ts.transform.rotation
    path.poses.append(pose_stamped)
    path_pub.publish(path)

    # publish tf
    br = tf.TransformBroadcaster()
    br.sendTransformMessage(ts)

    # publish static tf 
    static_transformStamped = TransformStamped()
  
    static_transformStamped.header.stamp = ts.header.stamp
    static_transformStamped.header.frame_id = "imu"
    static_transformStamped.child_frame_id = "camera"
  
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0
  
    quat = tf.transformations.quaternion_from_euler(-1.57, 0, -1.57)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    br.sendTransformMessage(static_transformStamped)

    print "publish a path and transform"

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensor_pose", TransformStamped, callback_ts)

    path_pub = rospy.Publisher('/sensor_path', Path, queue_size=1)

    rospy.spin()
