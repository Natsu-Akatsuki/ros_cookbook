import rospy
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # target_frame, source_frame
            trans = tfBuffer.lookup_transform("lidar", 'camera', rospy.Time())
            pass
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
            

        rate.sleep()