# Microstrain Examples

A example listener node is provided in to demonstrate a very basic C++ node that subscribes to and displays some of the data published by the ros_mscl driver.

Over time we will provide more robust and varied examples in both C++ and Python, but in the meantime we hope this helps in quickly testing to ensure everything is installed and working properly!

Prerequisite: completed setup and build steps found [here](../).

#### Create the example package
1. If the entire microstrain_inertial package, including the `microstrain_inertial_examples` directory, is not already there move the `microstrain_inertial_examples` package to the `your_workspace/src` folder.

2. Locate and register the package to the workspace: `rospack find microstrain_inertial_examples`

3. Build your workspace:
        
        cd ~/your_workspace
        catkin_make
        source ~/your_workspace/devel/setup.bash
   The source command may need to be run in each terminal prior to launching a ROS node.
   You may need to change the permissions on the listener.py file if it is failing to run.


#### Launch the listener node
Launch the inertial device node:
            
    roslaunch microstrain_inertial_node microstrain.launch

In a separate terminal, launch the example listener node:

    roslaunch microstrain_inertial_examples listener_cpp.launch

Optional launch parameters:
- name: the namespace of the listener node, default: listener_cpp
- device: the namespace of the inertial device node to subscribe to, default: gx5


**Example**: Launch two listener subscribed to different namespaces

In two different terminals:
    
    roslaunch microstrain_inertial_examples listener_cpp.launch name:=listener1234 device:=sensor1234

    roslaunch microstrain_inertial_examples listener_cpp.launch name:=bestListener device:=bestSensor
This will launch two nodes that listen to IMU data in different namespaces:
- listener1234, subscribed to: /sensor1234/imu/data
- bestListener, subscribed to: /bestSensor/imu/data

Test this by changing the namespace of the inertial device node:
1. Launch sensor1234:

        roslaunch microstrain_inertial_driver microstrain.launch name:=sensor1234
    Data will begin to stream in only the listener1234 terminal.
2. Stop sensor1234
3. Launch bestSensor:

        roslaunch microstrain_inertial_driver microstrain.launch name:=bestSensor
    Data will begin to stream in only the bestListener terminal.
