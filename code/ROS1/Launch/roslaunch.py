import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
cli_args = ["./rviz.launch", 'vel:=2.19']
roslaunch_args = cli_args[1:]
roslaunch_file = [("./rviz.launch")]

parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

parent.start()