#!/usr/bin/python3
import sys
import rospy
from service_demo.srv import AddTwoInts


def add_two_ints_client(x, y):
    # 1. The client code for calling services is also simple. For clients you don't have to call init_node()
    # 2. This is a convenience method that blocks until the service named add_two_ints is available.
    rospy.wait_for_service('add_two_ints')
    try:
        # create a handle for calling the service / used to trigger service
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        # call serivce
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "Usage: rosrun <client_node_name> [x y]"


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print(f"Requesting {x}+{y}")
    print(f"{x} + {y} = {add_two_ints_client(x, y)}")
