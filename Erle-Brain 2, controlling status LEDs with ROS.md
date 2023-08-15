# Erle-Brain 2, controlling status LEDs with ROS
This tutorial explains how to control the status LEDs from Erle-Brain 2 with the Robot Operating System (ROS).

## Things used in this project
**Hardware components**
1. Erle Robotics Erle-Brain 2 ×	1	
2. Erle Robotics compatible WiFi dongle ×	1	

**Software apps and online services**
1. ROS Robot Operating System

**Story**
This tutorial explains how to control the status LEDs from Erle-Brain 2 with the Robot Operating System (ROS).

**Connecting to Erle-Brain 2**
There're several ways to access Erle-Brain 2 but let's assume that you've purchased the WiFi dongle adapter and the brain itself is creating a WiFi network. Connect to it by using the password: _holaerle._

Once connected, type:
```ssh erle@10.0.0.1```

and use again the same password: _holaerle._

**Creating the catkin workspace**
We'll user a new ROS catkin workspace for this tutorial:
```
cd ~/
mkdir -p hackster_catkin_ws/src
cd hackster_catkin_ws/src
catkin_init_workspace
```
That's it, a new ROS workspace has been created.

**Cloning the code and compiling it**
```
cd ~/hackster_catkin_ws/src
git clone https://github.com/erlerobot/ros_erle_statusled 
cd cd ~/hackster_catkin_ws
catkin_make
```
which should give you something like:
```
erle@erle-brain-2 ~/hackster_catkin_ws $ catkin_make
Base path: /home/erle/hackster_catkin_ws
Source space: /home/erle/hackster_catkin_ws/src
Build space: /home/erle/hackster_catkin_ws/build
Devel space: /home/erle/hackster_catkin_ws/devel
Install space: /home/erle/hackster_catkin_ws/install
####
#### Running command: "cmake /home/erle/hackster_catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/erle/hackster_catkin_ws/devel
-DCMAKE_INSTALL_PREFIX=/home/erle/hackster_catkin_ws/install -G Unix Makefiles" in "/home/erle/hackster_catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/erle/hackster_catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/erle/ros_catkin_ws/devel_isolated/rosbag;/home/erle/ros_catkin_ws/install_isolated
-- This workspace overlays: /home/erle/ros_catkin_ws/devel_isolated/rosbag;/home/erle/ros_catkin_ws/install_isolated
-- Using PYTHON_EXECUTABLE: /usr/bin/python
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/erle/hackster_catkin_ws/build/hackster_results
-- gtest not found, C++ tests can not be built. Please install the gtest headers globally in your system or 
   checkout gtest (by running 'svn checkout http://googletest.googlecode.com/svn/tags/release-1.6.0 gtest' in the 
   source space '/home/erle/hackster_catkin_ws/src' of your workspace) to enable gtests
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.6.15
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - ros_erle_statusled
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'ros_erle_statusled'
-- ==> add_subdirectory(ros_erle_statusled)
-- Using these message generators: gencpp;genlisp;genpy
-- Configuring done
-- Generating done
-- Build files have been written to: /home/erle/hackster_catkin_ws/build
####
#### Running command: "make -j4 -l4" in "/home/erle/hackster_catkin_ws/build"
####
```

**Trying it out**
Let's launch our ROS package and give it a try:

```sudo -s```  // you'll need privileges since we're accessing /dev/mem
```cd ~/hackster_catkin_ws```
```source devel/setup.bash```
```rosrun ros_erle_statusled statusleds.py```

All right, ROS package launched, now from another terminal, type:
```rostopic pub /statusleds std_msgs/String "blue"```

**References:**
[Hackster](https://www.hackster.io/)
[<Packt>](https://www.packtpub.com/product/ros-robotics-projects)
[Skyfi labs](https://www.skyfilabs.com/blog/10-simple-ros-projects-for-beginners)
[Robocademy](https://robocademy.com/2020/10/15/open-source-ros-projects-from-ros-developer-learning-path/)
