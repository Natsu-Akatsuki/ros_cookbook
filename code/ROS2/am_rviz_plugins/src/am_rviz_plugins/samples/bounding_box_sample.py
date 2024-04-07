import rclpy
from am_rviz_plugins_msgs.msg import BoundingBox, BoundingBoxArray
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


class MyNode(Node):
    def __init__(self):
        super().__init__("bounding_box_sample")
        timer_period = 0.2  # 单位：秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(BoundingBoxArray, "bounding_box_array", qos_profile=latching_qos)

    def timer_callback(self):
        box_a = BoundingBox()
        box_b = BoundingBox()
        box_a.label = 2
        box_b.label = 5
        box_arr = BoundingBoxArray()
        now = self.get_clock().now().to_msg()
        box_a.header.stamp = now
        box_b.header.stamp = now
        box_arr.header.stamp = now
        box_a.header.frame_id = "map"
        box_b.header.frame_id = "map"
        box_arr.header.frame_id = "map"
        q = [0.0, 0.0, 0.0, 1.0]
        box_a.pose.orientation.x = q[0]
        box_a.pose.orientation.y = q[1]
        box_a.pose.orientation.z = q[2]
        box_a.pose.orientation.w = q[3]
        box_b.pose.orientation.w = 1.0
        box_b.pose.position.y = 2.0
        box_b.dimensions.x = (self.counter % 10 + 1) * 0.1
        box_b.dimensions.y = ((self.counter + 1) % 10 + 1) * 0.1
        box_b.dimensions.z = ((self.counter + 2) % 10 + 1) * 0.1
        box_a.dimensions.x = 1.0
        box_a.dimensions.y = 1.0
        box_a.dimensions.z = 1.0
        box_a.value = (self.counter % 100) / 100.0
        box_b.value = 1 - (self.counter % 100) / 100.0
        box_arr.boxes.append(box_a)
        box_arr.boxes.append(box_b)
        self.pub.publish(box_arr)
        self.counter = self.counter + 1


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
