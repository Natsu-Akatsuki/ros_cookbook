import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
rospy.init_node('distance_marker', anonymous=False)

distance_marker_pub = rospy.Publisher('/marker/text/distance', MarkerArray, queue_size=10)

marker_arr = MarkerArray()
for r in range(10):
    marker = Marker()
    marker.header.frame_id = 'base_link'
    marker.header.stamp = rospy.Time.now()

    marker.ns = "text"
    marker.id = r
    marker.action = Marker.ADD
    marker.type = Marker.TEXT_VIEW_FACING
    marker.lifetime = rospy.Duration(0)

    theta = -45 * math.pi / 180
    marker.pose.position.x = (r * 10) * math.cos(theta)
    marker.pose.position.y = (r * 10) * math.sin(theta)
    marker.pose.position.z = 0

    # font size
    marker.scale.z = 2.0

    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 0.9 # Required, otherwise rviz can not be displayed

    marker.text = f"{r * 10}m"
    marker_arr.markers.append(marker)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    distance_marker_pub.publish(marker_arr)
    r.sleep()
