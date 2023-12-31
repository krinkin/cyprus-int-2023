# Docker File

**Following are the instructions and in-depth details on how to build a docker container from scratch**
- Download docker for your respective OS from Docker Desktop
- Install WSL2 package for to run Linux file systems

Run Windows PowerShell in administrator mode.

## Command to install WSL2 
```wsl``` 
``` wsl --update #To get latest packages for your machine. ``` 
``` wsl --list --online #To check which WSL is supported. ``` 
``` wsl –install #To install ubuntu.``` 


### Setup Ubuntu

``` Sudo apt update #To fetch all the updates. ``` 
``` Sudo apt upgrade #To install the updates. ``` 
``` Wsl.exe --shutdown #To shutdown ubuntu.``` 

**If you use VScode download Remote development extension to change to Wsl window by using ``` Ctrl+Shift+P```**
 

**Download Xlaunch to use GUI tools in Docker _Optional_**


#### Command to build it 

1) Search for “Ros docker images osrf.”
2) Copy Docker Pull Command and paste it on cmd.

**Use ``` “docker pull osrf/ros:foxy-desktop”```  for ROS2 and 
    ``` “docker pull osrf/ros:noetic-desktop"```  for ROS1.**


**``` docker images``` to view all the installed images**
**``` docker run -it osrf/ros:noetic-desktop```  To change the cmd format from Windows to a Linux based system**
**``` lsb_release -a``` To check current version of ubuntu** 

``` 
docker built -t <image name >.
FROM osrf/ros:foxy-desktop
RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src/

RUN git clone https://github.com/krinkin/cyprus-int-2023 && \
    cd ~/catkin_ws
RUN echo "CY2023!"
``` 