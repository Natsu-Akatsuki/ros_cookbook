^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_animated_view_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2021-08-19)
------------------
* add license file
* fix typo in README image link
* clip gif a bit shorter
* add README and demo gif
* Add option to pause the current animation
* Publish camera view images
* accidentally deleted cv_bridge and image_transport
* fix dependencies in package.xml
* Add publisher for the current view camera pose
* Add properties showing view window size in panel
* Add support for camera trajectories messages
* switch ci to use noetic-devel branch
* add bool property allowing the user to activate publishing of the view images in the view controller
  Is automatically activated when a trajectory is requested to be rendered frame by frame.
* add image publisher for rviz camera's view
  Publish what the user sees in the rviz visualization window.
  Publishing is only active when the render_frame_by_frame parameter is set to true in the CameraTrajectory message requesting the trajectory.
  Otherwise there is a lag on slower computers when the resolution of the view image is large.
* add publisher for a message to indicate the animation is finished
* add functionality to render the trajectory frame by frame with a specified number of frames per second
* add support for view_controller_msgs::CameraTrajectory messages
  First, the movements requested using the CameraPlacement or CameraTrajectory messages are stored in a movement buffer.
  Then, animation is enabled, causing the update method to use the movements from the buffer to perform camera movements.
* convert transformCameraPlacementToAttachedFrame method's parameter from whole CameraPlacement to only the relevant eye, focus and up
  this way the method is more general
* add gh actions and gitignore
* compile on kinetic Qt5
* add myself as maintainer (`#8 <https://github.com/ros-visualization/rviz_animated_view_controller/issues/8>`_)
* Contributors: Evan Flynn, Gene Merewether, Razlaw, razlaw

0.1.1 (2014-05-20)
------------------
* force package version
* add demo launch file
* update urls in package.xml
* minor style fix
* update plugin macro
* apply catkin_lint
* catkinized
* minor change
* separating...
* separating packages
* preparing for first release
* removes CameraPlacementTrajectory from everything
* removes CameraPlacementTrajectory
* fixes singularity; simplifies up-vector
* splits up msgs and plugin
* moving from visualization trunk
* Contributors: Adam Leeper, Sachin Chitta, aleeper
