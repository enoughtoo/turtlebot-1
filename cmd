roslaunch demo_turtlebot_localization.launch

roslaunch demo_turtlebot_mapping.launch

roslaunch demo_turtlebot_rviz.launch

rosrun tf tf_echo map camera_link







rostopic list

rosrun image_view image_view image:=/camera/rgb/image_raw