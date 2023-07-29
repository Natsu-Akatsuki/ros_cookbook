import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

rospy.init_node('kitti_dataset', anonymous=False)

shape_marker_pub = rospy.Publisher('/marker/shape', Marker, queue_size=10)
text_marker_pub = rospy.Publisher('/marker/text', Marker, queue_size=10)
compose_marker_pub = rospy.Publisher('/marker/compose', MarkerArray, queue_size=10)


def shape_marker():
    marker = Marker()
    marker.header.frame_id = 'livox_frame'
    marker.header.stamp = rospy.Time.now()

    marker.ns = "shape"
    marker.id = 1
    marker.action = Marker.ADD
    marker.type = Marker.CUBE
    marker.lifetime = rospy.Duration(0)

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1
    marker.scale.x = 0.2
    shape_marker_pub.publish(marker)


def text_marker():
    marker = Marker()
    marker.header.frame_id = 'livox_frame'
    marker.header.stamp = rospy.Time.now()

    marker.ns = "text"
    marker.id = 1
    marker.action = Marker.ADD
    marker.type = Marker.TEXT_VIEW_FACING
    marker.lifetime = rospy.Duration(0)

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1
    marker.scale.x = 0.2
    marker.text = "Hello World"
    text_marker_pub.publish(marker)


def compose_marker():
    marker = MarkerArray()
    marker1 = Marker()
    marker1.header.frame_id = 'livox_frame'
    marker1.header.stamp = rospy.Time.now()

    marker1.ns = "shape"
    marker1.id = 1
    marker1.action = Marker.ADD
    marker1.type = Marker.CUBE
    marker1.lifetime = rospy.Duration(0)

    # shape
    marker1.scale.x = 1.0
    marker1.scale.y = 1.0
    marker1.scale.z = 1.0
    # position
    marker1.pose.position.x = 0
    marker1.pose.position.y = 0
    marker1.pose.position.z = 0
    marker1.pose.orientation.x = 0.0
    marker1.pose.orientation.y = 0.0
    marker1.pose.orientation.z = 0.0
    marker1.pose.orientation.w = 1.0
    # color
    marker1.color.r = 0
    marker1.color.g = 0
    marker1.color.b = 0
    marker1.color.a = 0.5
    marker.markers.append(marker1)

    marker2 = Marker()
    marker2.header.frame_id = 'livox_frame'
    marker2.header.stamp = rospy.Time.now()

    marker2.ns = "text"
    marker2.id = 1
    marker2.action = Marker.ADD
    marker2.type = Marker.TEXT_VIEW_FACING
    marker2.lifetime = rospy.Duration(0)

    marker2.scale.x = 1.0
    marker2.scale.y = 1.0
    marker2.scale.z = 1.0

    marker2.pose.position.x = 0
    marker2.pose.position.y = 0
    marker2.pose.position.z = 0
    marker2.pose.orientation.x = 0.0
    marker2.pose.orientation.y = 0.0
    marker2.pose.orientation.z = 0.0
    marker2.pose.orientation.w = 1.0

    marker2.color.r = 1
    marker2.color.g = 1
    marker2.color.b = 1
    marker2.color.a = 1
    marker2.scale.x = 0.2
    marker2.text = "Hello World"
    marker.markers.append(marker2)

    compose_marker_pub.publish(marker)



r = rospy.Rate(10)
while not rospy.is_shutdown():
    # shape_marker()
    # text_marker()
    compose_marker()
    r.sleep()
