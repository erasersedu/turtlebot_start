# turtlebot_start


To launch the robot with laser transformation, amcl and move_base

$ roslaunch turtlebot_start robot_complete.launch


============

To launch the arm controller

$ roslaunch turtlebot_start arm.launch


Motor ID change:

Download it: 

http://www.robotis.com/service/download.php?no=1671


2. Enter this command:

cd ~/Downloads

sudo chmod 775 DynamixelWizard2Setup_x64


3. Enter this command:

./DynamixelWizard2Setup_x64 


============

To launch the map saver

roslaunch turtlebot_start simple_map_creator.launch


To install: 

sudo apt-get install ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers


To save the map:

rosrun map_server map_saver -f /tmp/my_map

