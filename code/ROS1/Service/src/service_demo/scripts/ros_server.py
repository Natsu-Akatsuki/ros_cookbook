#!/usr/bin/python3
"""
需要指定python3，否则或有如下问题：
import-im6.q16: attempt to perform an operation not allowed by the security policy
"""
import rospy
from service_demo.srv import AddTwoInts, AddTwoIntsResponse


def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b)))

    return AddTwoIntsResponse(req.a + req.b)


def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    # create a service
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()
