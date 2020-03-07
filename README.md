# turtlebot_start


To launch the robot with laser transformation, amcl and move_base

$ roslaunch turtlebot_start robot_complete.launch


============

To launch the arm controller

$ roslaunch turtlebot_start arm.launch


============

To install: 

sudo apt-get install ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers


To launch the map saver

roslaunch turtlebot_start simple_map_creator.launch


To save the map:

rosrun map_server map_saver -f /tmp/my_map

