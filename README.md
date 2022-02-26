# tobesim
Python implementation of Gazebo simulation of TOBE robot

I made a step-by-step video for the initial commit, illustrating how to clone this repo onto a PC and use it to execute walking for TOBE in simulation. 
The video is available at: https://www.youtube.com/watch?v=yBU5GnSuJKs

Here is the summary:

1.  In your existing .bashrc file, change (or add, if necessary) your ROS_IP to the IP address your internet is connected to (which can be found using 'ifconfig').
2.  Close the terminal, and open a new one. The changes to the .bashrc will not take effect for the old terminal.
3.  Clone the 'tobesim' repo.
4.  Go to the catkin_ws folder, and use the command 'catkin_make' to create the ROS packages 'tobe_control', 'tobe_description', and 'tobe_gazebo'.
5.  Use the command 'source devel/setup.bash'.
6.  Enter 'roslaunch tobe_gazebo tobe_gazebo.launch'. This should open Gazebo automatically, and TOBE should appear in the center of the area.
7.  After seeing that the controllers are loaded, click the 'Play' button at the lower left of the Gazebo screen. If the robot falls down, the controllers have not loaded properly.
8.  Open another terminal, and enter the command "rostopic pub /tobe/cmd_vel geometry_msgs/Vector3 '0.5' '0' '0'", which will execute a forward walking motion. 
(NOTE: the first value is a parameter to control forward gait speed, the second is for lateral gait, and the third is for turning, but I have not tested the lateral or turning gait
parameters yet, so entering values other than zeros for the second and third values probably won't work well. Also, I wouldn't advise using a forward gait parameter higher than 0.8, 
or the robot will probably fall immediately.)
9.  To stop the simulation at any time, click the 'Pause' button at the lower left of the Gazebo screen.
10. To close Gazebo, press Ctrl-C in the first terminal, and the window should close after a few seconds.

To experiment with your own controller, go to the 'tobe_gazebo/scripts' folder, and either

1.  Modify the 'tobe_demo.py' node (more info about the commands used there can be found by looking at the 'tobe.py' file in the 'tobe_gazebo/src/tobe_gazebo' folder)

2.  Or, modify the main walking loop (called 'do_walk', near the end of the file) in the 'tobe_walker.py' node, and then publish to the /tobe/cmd_vel topic as described in step #8.

A third option is to dig deeper (i.e., examine or change what is used to execute those high-level actions). For that, take a look at the entire 'tobe_walker.py' file.
