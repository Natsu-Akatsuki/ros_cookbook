#!/usr/bin/env python

import argparse
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class OdometryToPath:
    def __init__(self, input_topic, output_topic, min_distance):
        rospy.init_node('odom_to_path_converter', anonymous=True)

        self.odom_subscriber = rospy.Subscriber(input_topic, Odometry, self.odom_callback)
        self.path_publisher = rospy.Publisher(output_topic, Path, queue_size=10, latch=True)

        self.path = Path()
        self.last_pose = None

        self.min_distance = min_distance

    def odom_callback(self, odom_msg):
        current_pose = odom_msg.pose.pose

        if self.last_pose is not None:
            dx = current_pose.position.x - self.last_pose.position.x
            dy = current_pose.position.y - self.last_pose.position.y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance < self.min_distance:
                return

        pose_stamped = PoseStamped()
        pose_stamped.header = odom_msg.header
        pose_stamped.pose = current_pose

        self.path.header = odom_msg.header
        self.path.poses.append(pose_stamped)

        self.path_publisher.publish(self.path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert Odometry data to Path and publish it.')
    parser.add_argument('--input_topic', type=str, default='/robot/dlio/odom_node/odom',
                        help='Input topic for Odometry messages')
    parser.add_argument('--output_topic', type=str, default='/debug/path',
                        help='Output topic for Path messages')
    parser.add_argument('--min_distance', type=float, default=0.1,
                        help='Minimum distance threshold for path update')

    args = parser.parse_args()

    try:
        converter = OdometryToPath(args.input_topic, args.output_topic, args.min_distance)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
