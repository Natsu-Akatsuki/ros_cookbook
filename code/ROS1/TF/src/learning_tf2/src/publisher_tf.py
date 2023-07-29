#!/usr/bin/env python
import rospy

import tf_conversions
import numpy as np
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import math
publish_tf_ = True

odom_pub_ = rospy.Publisher('/odom', Odometry, queue_size=10)


def publish_global_tf():
    br = tf2_ros.TransformBroadcaster()
    odom_trans = geometry_msgs.msg.TransformStamped()

    frame_id = "map"
    child_frame_id = "base_link"

    if publish_tf_:
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = frame_id
        odom_trans.child_frame_id = child_frame_id
        odom_trans.transform.translation.x = 0.0
        odom_trans.transform.translation.y = 0.0
        odom_trans.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        odom_trans.transform.rotation.x = q[0]
        odom_trans.transform.rotation.y = q[1]
        odom_trans.transform.rotation.z = q[2]
        odom_trans.transform.rotation.w = q[3]
        br.sendTransform(odom_trans)

    odom = Odometry()
    odom.header.frame_id = frame_id
    odom.header.stamp = child_frame_id

    odom.pose.pose.position.x = odom.pose.pose.position.x
    odom.pose.pose.position.y = odom.pose.pose.position.y
    odom.pose.pose.position.z = odom.pose.pose.position.z
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]

    odom.twist.twist.linear.x = 0.0
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = 0.0

    odom_pub_.publish(odom)


if __name__ == '__main__':
    rospy.init_node('fake_global_tf_publisher')
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        publish_global_tf()
    rospy.spin()
