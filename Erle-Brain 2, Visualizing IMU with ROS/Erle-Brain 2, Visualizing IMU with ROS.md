# Erle-Brain 2, Visualizing IMU with ROS
This tutorial explains how to control and visualize the IMU Erle-Brain 2 with the Robot Operating System (ROS).

## Things used in this project
**Hardware components**
Erle Robotics Erle-Brain 2 ×1

**Software apps and online services**
ROS Robot Operating System

### Story
**Connecting to Erle-Brain 2**
There're several ways to access Erle-Brain 2 but let's assume that you've purchased the WiFi dongle adapter and the brain itself is creating a WiFi network. Connect to it by using the password: _holaerle._

Once connected, type:
```ssh erle@10.0.0.1```

And use again the same password: _holaerle._

**Creating the Catkin Workspace**
We'll user a new ROS catkin workspace for this tutorial:
```
cd ~/
mkdir -p hackster_catkin_ws/src
cd hackster_catkin_ws/src
catkin_init_workspace
```
That's it, a new ROS workspace has been created.

**Cloning the Code and Compiling It**
```
cd ~/erle_catkin_ws/src
git clone https://github.com/erlerobot/ros_erle_imu 
cd ~/erle_catkin_ws
catkin_make_isolated
```

Which should give you something like:
```
Base path: /home/erle/ros_brain
Source space: /home/erle/ros_brain/src
Build space: /home/erle/ros_brain/build_isolated
Devel space: /home/erle/ros_brain/devel_isolated
Install space: /home/erle/ros_brain/install_isolated
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
~~  traversing 1 packages in topological order:
~~  - ros_erle_imu
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The packages or cmake arguments have changed, forcing cmake invocation
==> Processing catkin package: 'ros_erle_imu'
==> Creating build directory: 'build_isolated/ros_erle_imu'
==> cmake /home/erle/ros_brain/src/ros_erle_imu -DCATKIN_DEVEL_PREFIX=/home/erle/ros_brain/devel_isolated/ros_erle_imu -DCMAKE_INSTALL_PREFIX=/home/erle/ros_brain/install_isolated -G Unix Makefiles in '/home/erle/ros_brain/build_isolated/ros_erle_imu'
-- The C compiler identification is GNU 4.9.2
-- The CXX compiler identification is GNU 4.9.2
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Using CATKIN_DEVEL_PREFIX: /home/erle/ros_brain/devel_isolated/ros_erle_imu
-- Using CMAKE_PREFIX_PATH: /opt/ros/indigo
-- This workspace overlays: /opt/ros/indigo
-- Found PythonInterp: /usr/bin/python (found version "2.7.9") 
-- Using PYTHON_EXECUTABLE: /usr/bin/python
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/erle/ros_brain/build_isolated/ros_erle_imu/test_results
-- gtest not found, C++ tests can not be built. Please install the gtest headers globally in your system to enable gtests
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.6.15
-- Using these message generators: gencpp;genlisp;genpy
CMake Warning at /home/erle/ros_brain/build_isolated/ros_erle_imu/cmake/ros_erle_imu-genmsg.cmake:3 (message):
  Invoking generate_messages() without having added any message or service
  file before.
  You should either add add_message_files() and/or add_service_files() calls
  or remove the invocation of generate_messages().
Call Stack (most recent call first):
  /opt/ros/indigo/share/genmsg/cmake/genmsg-extras.cmake:304 (include)
  CMakeLists.txt:13 (generate_messages)
-- ros_erle_imu: 0 messages, 0 services
-- Configuring done
-- Generating done
-- Build files have been written to: /home/erle/ros_brain/build_isolated/ros_erle_imu
==> make -j4 -l4 in '/home/erle/ros_brain/build_isolated/ros_erle_imu'
Scanning dependencies of target imu_listener
Scanning dependencies of target std_msgs_generate_messages_py
Scanning dependencies of target std_msgs_generate_messages_lisp
Scanning dependencies of target imu_talker
[  0%] [  0%] Built target std_msgs_generate_messages_lisp
Built target std_msgs_generate_messages_py
Scanning dependencies of target std_msgs_generate_messages_cpp
Scanning dependencies of target ros_erle_imu_generate_messages_lisp
[  0%] [  0%] Built target std_msgs_generate_messages_cpp
Built target ros_erle_imu_generate_messages_lisp
[ 33%] [ 66%] Building CXX object CMakeFiles/imu_talker.dir/src/imu_talker.cpp.o
Scanning dependencies of target ros_erle_imu_generate_messages_py
Scanning dependencies of target ros_erle_imu_generate_messages_cpp
Building CXX object CMakeFiles/imu_listener.dir/src/imu_listener.cpp.o
[ 66%] [ 66%] Built target ros_erle_imu_generate_messages_py
Built target ros_erle_imu_generate_messages_cpp
[100%] Scanning dependencies of target ros_erle_imu_generate_messages
Building CXX object CMakeFiles/imu_talker.dir/src/MPU9250.cpp.o
[100%] Built target ros_erle_imu_generate_messages
Linking CXX executable /home/erle/ros_brain/devel_isolated/ros_erle_imu/lib/ros_erle_imu/imu_talker
Linking CXX executable /home/erle/ros_brain/devel_isolated/ros_erle_imu/lib/ros_erle_imu/imu_listener
[100%] Built target imu_talker
[100%] Built target imu_listener
<== Finished processing package [1 of 1]: 'ros_erle_imu'
``` 

**Configure your Network**
If you are working with two different machines, for example: Erle-Brain and your laptop remember to configure **ROS_MASTER_URI** and **ROS_IP.** Maybe could be interesting to put this lines in your ~/.bashrc.

**Erle-Brain:**
```export ROS_MASTER_URI=http://10.0.0.1:11311export ROS_IP=10.0.0.1```

**Laptop or PC:**
```export ROS_MASTER_URI=http://10.0.0.1:11311```
```export ROS_IP=10.0.0.2```

**Trying it Out**
Let's launch our ROS package and give it a try:
```
sudo -s # you'll need privileges since we're accessing spi
cd ~/erle_catkin_ws
source devel/setup.bash 
rosrun ros_erle_imu imu_talker.py
```

All right, ROS package launched, now in your computer, type:

```export ROS_MASTER_URI=http://<erle_brain_ip>:11311```
```rosrun ros_erle_imu visualization.py```
![Alt text](1_YqYOmchJE5S9nEgroVoPbg-1.webp)

**References:**
[Hackster](https://www.hackster.io/)
[<Packt>](https://www.packtpub.com/product/ros-robotics-projects)
[Skyfi labs](https://www.skyfilabs.com/blog/10-simple-ros-projects-for-beginners)
[Robocademy](https://robocademy.com/2020/10/15/open-source-ros-projects-from-ros-developer-learning-path/)