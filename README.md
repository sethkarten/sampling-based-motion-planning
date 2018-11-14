# Instructions

## Initial Steps:

1. Download comprobfall2018-hw2.zip 

2. Unzip the folder inside $HOME/catkin_ws/src


## Visualization of the world with piano inside a room in Gazebo:

1. export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/comprobfall2018-hw2/models:$GAZEBO_MODEL_PATH

2. cd $HOME/catkin_ws/src/comprobfall2018-hw2/piano/piano_gazebo/world

3. gazebo environment_piano.world



## PQP description and using PQP for collision check:

### CPP compatible version:

1. Package for PQP is located in "pqp_comprob" folder. 

2. There are 2 folders inside "pqp_comprob": 

   2.1 "pqp" includes an already compiled PQP library

   2.2 "pqp_client_comprob" includes a PQP client demo program.

3. Inside "pqp_client_comprob" folder, there is a PQP client demo program named "pqp_client.cpp". 
This demo program reads triangle meshes of piano and room from "piano.txt" and "room.txt" files, 
then creates PQP models base on these meshes. 
The demo program also gives an example for collision checking based on these models with translation vectors and 
rotation matrices.

4. You will be using this demo program for generating PRM. Whenever you change the demo program, 
remember to "make" it before running.
   
   $ cd $HOME/catkin_ws/src/comprobfall2018-hw2/pqp_comprob/pqp_client_comprob; make clean; make

### Python and ROS compatible version

1. "pqp_server" is a ROS package, put this folder in your catkin_ws/src/ directory.

2. Compile the package in ROS: run "catkin_make" in catkin_ws.

3. Bring up the server: First, run roscore in a separate terminal. Second, run "roscd pqp_server", then run "rosrun pqp_server pqp_ros_server" in the same terminal. (pqp_ros_server reads mesh correctly only by following these steps)

4. Collision check using Python node: We have a example python code in "pyscript" folder. Run this example using "rosrun pqp_server pqp_ros_client.py"

5. Write your own Python node: In the node you have to give the service 2 parameters, T is a float list with size 3, R is a float list with size 9. T is the translation vector of the piano, R is the rotation matrix flatten in row major order of the piano.


## Ackermann - Compilation, Visualization, Demo Client:

1. cd $HOME/catkin_ws/; catkin_make

2. source $HOME/catkin_ws/devel/setup.bash

3. export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/comprobfall2018-hw2/models:$GAZEBO_MODEL_PATH

4. roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch &

5. To visualize the ackermann vehicle navigating, run:

   ./devel/lib/ackermann_client/navigate

### Publishing to ackermann_cmd:

/ackermann_cmd is a ROS topic that is subscribed to by the node ackermann_controller, which is provided by us. You can view the message description by running the command:

rosmsg show ackermann_msgs/AckermannDrive

The result:

float32 steering_angle

float32 steering_angle_velocity

float32 speed

float32 acceleration

float32 jerk

You will have to write a ROS publisher (in C++ or Python) that will
publish 'steering_angle' and 'speed' to this topic. Please refer to the ackermann_client/navigate.cc for basic understanding.

