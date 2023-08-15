# Getting Started with ROS
In this project you will know about ROS installation, Creating a workspace and making packages.

## Things used in this project
**Software apps and online services**
1. Snappy Ubuntu Core
   _Ubuntu 18.04_

2. ROS Robot Operating System
   _This project will teach you how to install this in your system._

### Story
**Description:**

**Installation Instruction**
This Document assumes that the reader has installed Ubuntu 18.04. However, if you haven't installed [Ubuntu 18.04](https://old-releases.ubuntu.com/releases/18.04.5/)
yet make sure to install it before proceeding. There are tons of resources available on the Internet to get this done.

_Note: You can download Ubuntu 18.04 ISO file from [here](https://old-releases.ubuntu.com/releases/18.04.5/)._

**ROS Melodic Installation**
ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS is licensed under an open-source, BSD license.

Here the [distribution](http://wiki.ros.org/Distributions) compatible with Ubuntu 18.04 is the [ROS Melodic Morenia](http://wiki.ros.org/melodic). Follow the steps below to install ROS Melodic

**Installation Steps**
* Set up your computer to accept software from packages.ros.org.
```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'```

* Set up your keys.
```sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654```

* Make sure your Debian package index is up-to-date.
```sudo apt-get update```

* Upgrade your system
```sudo apt-get upgrade```
```sudo apt-get dist-upgrade```

* Install the ROS-recommended configuration.
```sudo apt-get install ros-melodic-desktop-full```

**Configuration Steps**
* Adding environment variables: To Automatically add ROS environment variables to your bash session every time a new shell [(terminal)](https://www.gnu.org/software/bash/manual/html_node/What-is-Bash_003f.html) is launched, enter the following commands (this step is similar as adding an environmental variable in windows):
```echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc```
```source ~/.bashrc```

* Initialize rosdep: Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.
```sudo apt-get install python-rosdep```
```sudo rosdep init```
```rosdep update```

**Additional packages to install**
1.Catkin Tools
```sudo apt-get install ros-melodic-catkin python-catkin-tools```

2.std_msg package
```sudo apt-get install ros-melodic-std-msgs```

3.turtlesim
```sudo apt-get install ros-melodic-ros-tutorials```

4.ROS-Controllers
```
#Install Ros controllers by the this command
sudo apt-get install ros controllers**
>> #Or this command 
sudo apt-get install ros*controller*
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers

#If the above method doesn't work then try this. Replace the version with your ROS Version
sudo apt-get install ros-VERSION-ros-control ros-<VERSION>-ros-controllers
```
5.Moveit installation
```sudo apt-get install ros-melodic-moveit```


**Creating Catkin Workspace**
**Step 1:** Open up the terminal. Create the root workspace directory. You can name your directory anything but by ROS convention we will use _catkin_ws_ as the name.
```cd ~/```
```mkdir --parents catkin_ws/src```
```cd catkin_ws```

Step 2: Initialize the catkin workspace.
```catkin init```

Look for the statement **“Workspace configuration appears valid”**, showing that your catkin workspace was created successfully. If you forgot to create the _src_ directory, or did not run _catkin init_ from the workspace root (both common mistakes), you’ll get an error message like **“WARNING: Source space does not yet exist”.**

Step 3: Build the workspace.
```cd ~/catkin_ws```
```catkin build```

Now your catkin workspace will have additional directories _build, devel, logs._
```ls```

Step 4: Now to make your workspace visible to ROS. Source the setup file in the devel directory.
```source ~/catkin_ws/devel/setup.bash```

By doing this, all the packages that you create inside the _src_ folder will be visible to ROS.

_Note:This setup.bash file of your workspace must be source everytime when you want to use ROS packages created inside this workspace._

Step 5: To save typing, add this to your _.bashrc_,
```gedit ~/.bashrc```

Add to the end:
```source ~/catkin_ws/devel/setup.bash```
Save and close the editor.

**Create a ROS Package**
This section will demonstrate how to use the _catkin_create_pkg__ script to create a new catkin package, and what you can do with it after it has been created.

First, navigate to the source space directory of the catkin workspace you've created.
```cd ~/catkin_ws/src```

Now, use the _catkin_create_pkg script_ to create a new package called _pkg_ros_basics_ which depends on std_msgs, roscpp, and rospy:
```catkin_create_pkg pkg_getting_started std_msgs rospy roscpp```

This will create a beginner_tutorials folder which contains a _package.xml_ and a _CMakeLists.txt_, which have been partially filled out with the information you gave _catkin_create_pkg._

_Note : catkin_create_pkg requires that you give it a package_name and optionally a list of dependencies on which that package depends:_
```catkin_create_pkg <package_name> [depend1] [depend2] [depend3]```

Now, use the catkin_create_pkg script to create a new package called pkg_get_started which depends on std_msgs, roscpp, and rospy:
```catkin_create_pkg pkg_get_started std_msgs rospy roscpp```

This will create a beginner_tutorials folder which contains a package.xml and a CMakeLists.txt, which have been partially filled out with the information you gave catkin_create_pkg.

Now, you need to build the packages in the catkin workspace:
```cd ~/catkin_ws```
```catkin build```
Inside the package, there are _src_ folder, _package.xml_, _CMakeLists.txt_, and the _include_ folders.

* CMakeLists.txt: This file has all the commands to build the ROS source code inside the package and create the executable.   For more information about CMakeLists visit [here](http://wiki.ros.org/catkin/CMakeLists.txt).
* package.xml: This is an XML file. It mainly contains the package dependencies, information, and so forth.
* src: The source code of ROS packages are kept in this folder.

**ROS Master**
* ROS Nodes are building blocks of any ROS Application. A single ROS Application may have multiple ROS Nodes which            communicate with each other.
* The role of the ROS Master is to enable individual ROS nodes to locate one another.
* Once these nodes have located each other they communicate with each other peer-to-peer.
* The ROS Master provides naming and registration services to the rest of the nodes in the ROS system.
* You can say, communication is established between nodes by the ROS Master. So, without ROS Master running ROS Nodes can     not communicate with each other.

**Start ROS Master**
To start ROS Master you just have to enter the following command in the terminal.
```roscore```

So roscore will start the following:
* ROS Master
* ROS Parameter Server
* _rosout_ Logging Node

In the preceding output, you can see information about the computer, parameter which list the name (Melodic) and version number of ROS distribution, and some other information.

**ROS Nodes**
* A ROS Node is a piece of software/executable that uses ROS to communicate with other ROS Nodes.
ROS Nodes are building block of any ROS Application.
* For example, if you have a line-following robot then one ROS Node could get distance sensor values and another node can control the motors of the robot. So, these two nodes will communicate with each other in order to move the robot.
* You can write your entire ROS Application in a single node but having multiple nodes ensures that if a node crashes it won't crash your entire ROS application.
* A ROS package can have multiple ROS Nodes.
* _Python_ and _C++_ are majorly used to write ROS Nodes.


Given below is example of how to create ROS-Node.
```
#!/usr/bin/env python 
'''
Creating a ROS-Node
'''
import rospy
def main():    
    
    # 1. Make the script a ROS Node.
    rospy.init_node('hello_ros', anonymous=True)

    # 2. Print info on console.
    rospy.loginfo("Hello World!")
    
    # 3. Keep the node alive till it is killed by the user.
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```
To make this file executable type the following command in the terminal:
```sudo chmod +x hello_ros.py```

Then run the rosmaster by this command:
```roscore```

* While roscore is running open another terminal and run the node you created by this command:
```rosrun <package_name> <python_file_name>```

**ROS Launch Files**
* In the previous sections you must have noticed that we need to use roscore command to start ROS Master and Parameter        Server, rosrun command to run a ROS Node, rosparam load command to load parameters etc.
* This is a tedious process to manually run nodes and load parameters.
* Launch files provides the capability to do all these stuff using a single command.
* The idea is to mention all the nodes that you want to run, all the config file that you want to load etc in a single file   which you can run using roslaunch command.
>>Steps to create a launch file

After creating a package, create a folder in the package names as a launch folder to store all the launch files in that folder.
```cd ~/catkin_ws/src/<package>```
```mkdir launch```

Here we can create launch files by running this command by going into the launch directory, we can keep any name for the launch file,
```cd launch```
```touch filename.launch```
Now you can edit your launch file by adding different nodes that you have to run simultaneously.

>>Steps to add a ROS node in the launch file

Launch files always starts with
```<launch>```
and end with
```</launch>```
Now to add any executable file which we have seen in the rosrun_command section, add this line in the file.
```
<node pkg="name_of_package" type="name_of_executable.py" name="name_of_executable" 
output="screen"/>
```
* pkg is the package name which you have created
* type is the name of executable file
* name is the name of the node which is created in that executable
* output means it will print the data given to the roslog command

Given below is an example of a launch file in ROS containing two nodes:
```
<launch>
  <node name="node_a" pkg="<package_name>" type="node_1.py" output="screen"/>
  <node name="node_b" pkg="<package_name>" type="node_2.py" output="screen"/>
</launch>
```
To run a launch file in ros type this command:
```roslaunch pkg_ros_basics filename.launch```

**Common Errors Faced**
1.Catkin build makes laptop/pc hang: This happens when this command utilizes complete memory of pc while building heavy packages. This can be resolved by modifying the catkin build command as below:
```
#Try Running this command
catkin build --mem-limit 50% -j 1
```
2.GPG Pubkey not found: This happens when the system is not able to find GPG pubkey in your system. This can be resolved by adding the key through the terminal by the command shown below:
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys <PUBKEY>
```
**Summary**
Congrats! You learned all the basics of ros and are ready to get started with practical programming with ROS. Here is what you have learned so far:

**ROS-Installation**
* Creating ROS-Workspace
* ROS-packages
* ROS-Nodes
* ROS-Master
* ROS-Launch files
* ROS-Communication

**References:**
[Hackster](https://www.hackster.io/)
[<Packt>](https://www.packtpub.com/product/ros-robotics-projects)
[Skyfi labs](https://www.skyfilabs.com/blog/10-simple-ros-projects-for-beginners)
[Robocademy](https://robocademy.com/2020/10/15/open-source-ros-projects-from-ros-developer-learning-path/)
