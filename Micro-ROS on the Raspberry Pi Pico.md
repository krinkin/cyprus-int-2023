# Using micro-ROS on the Raspberry Pi Pico
Raspberry Pi Pico can communicate with a ROS2 graph natively via micro-ROS... with ROS2 development using VSCode

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

## Things used in this project
### Hardware components
SB Components Pico Board With Soldered Header ×	1	
#### Software apps and online services
ROS Robot Operating System
Microsoft VS Code

- Overview
- Running ROS2 on RP2040 microcontrollers
- micro-ROS
- Dependencies are being installed
- Obtaining the Information
- Installing VSCode
- Uploading to a Raspberry Pi Pico
- Micro ros-agent installation
- Code


**Overview**
In this post, we'll look at how the Raspberry Pi Pico can interface with a ROS2 graph natively via micro-ROS. In VSCode, we'll create a project, build it, and upload it to the microcontroller. As a result, we'll assume you're familiar with ROS2 development with VSCode.

**Running ROS2 on RP2040 microcontrollers.**
The Raspberry Pi Pico with RP2040 processor is completely open-hardware, designed by the Pi's creators. It is quite affordable, as is usual of the Pi foundation, costing only $4.
The specifications of the board, the differences between a microprocessor and a microcontroller, the 101 on how to get started, and what the Pi Pico can do are all outside the scope of this essay. But, whether you're familiar with microcontrollers or not, I strongly recommend that you look for yourself.


**Micro-ROS**
In the ROS (1) world, microcontrollers have long been viewed as second-class citizens. Because they cannot connect directly with the ROS graph, developers must rely on libraries such as rosserial. However, ROS2 is an entirely new cosmos in which things are constantly developing.

> micro-ROS installs ROS 2 on microcontrollers, making them first-class citizens in the ROS 2 ecosystem.

The micro-ROS project is led by industry titans such as Bosch, eProsima, the Fiware Foundation, and a plethora of partners and collaborators such as Amazon and Canonical, as well as the OFERA H2020 project.

So, what is it exactly? It's just a thin wrapper (see its design paper) on top of DDS for eXtremely Resource Constrained Environments (DDS-XRCE), which runs on a real-time OS and allows microcontrollers to connect with a ROS2 graph (the standard talker/listener) using an optimized subset of the DDS protocol. It's quite a mouthful. It employs a 'bridged' communication architecture, with the'micro-ROS Agent' acting as a 'broker.' The agent is in charge of connecting the ROS2 graph to one or more micro-ROS devices.

So, now that we've cleared up some terms, let's get started with the official micro-ROS on Raspberry Pi Pico example, which is accessible on github. For this course, I'm running Ubuntu 20.04 with the VSCode snap.

If you haven't yet upgraded to Ubuntu 20.04, you should think about using an LXD container. Refer to our earlier article 'ROS Development with LXC' to get started setting up the container.

**Dependencies are being installed.**
Let's begin by installing the required prerequisites.

```sudo apt install build-essential cmake gcc-arm-none-eabi libnewlib-arm-none-eabi doxygen git python3```

**Obtaining the Information**
We'll now establish a workspace and gather all of the necessary information.

```
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws/src
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git
git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git
```
The Pi Pico SDK, offered by the Pi Foundation, is the initial repository. The second includes a precompiled micro-ROS stack as well as a hello-world-style example.

**Installing VS Code**
Let's customize the example by opening it in VSCode. To follow along, you'll need two upgrades to VSCode that are both common in C++ development. These extensions for VSCode include the C++ extension and the CMake tools. We'll set up a CMake tools configuration file and specify a variable to instruct our project where to look for the Pi Pico SDK when we've installed them.

__Use Code:__
```
cd ~/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk
mkdir .vscode
touch .vscode/settings.json
```
__Use your preferred editor to open the newly produced file.__
```vi .vscode/settings.json```
**plus the following,**
```
{
"cmake.configureEnvironment": {
"PICO_SDK_PATH": "/home/$USER/micro_ros_ws/src/pico-sdk",
},
}
```
This is an environment variable that is only supplied to CMake during the configuration process. For further information, see the CMake-Tools manual.

Let's get started on the project, shall we?
```code .```
Before running the CMake configuration and producing it (perhaps VSCode has already requested that you do so), we must select the appropriate "kit." In the palette, type CMake: (ctrl+shift+p) to find it. Checking for Kits, then CMake: Make that the compiler GCC for arm-non-eabi, which we previously installed, is selected when choosing a Kit.

Now that we're all ready, let's lead by example! Select CMake: Build when you are back on the palette.

Let's now quickly review what the example performs. A repeating timer, an executor, a publisher that publishes a message with the title "pico publisher," and a node called "pico node" are all created in order to coordinate everything. Every 0.1 seconds, the executor rotates. However, the timer will only cause the publisher to post a message and add one bit of data to the message per second.

**Uploading to a Raspberry Pi Pico**

If all goes properly during compilation, you should see a new build folder in your project view. The file with the name pico micro ros example.uf2 that we must now upload to the Pi Pico is located in this folder.

To upload it, merely use a USB cable to connect the board to a computer while pressing the tiny white BOOTSEL button. After that, the Pi Pico will mount as a flash drive, making it simple for us to copy and paste the.uf2 file. Go to a computer and enter the following:
```cp build/pico_micro_ros_example.uf2 /media/$USER/RPI-RP2```

The board will reboot when the file has been copied, at which point the example will execute.

**Installation of the Micro-ros-agent**
As was said in the introduction, the communication architecture of Micro-ROS is bridged. Therefore, we must build that bridge. Fortunately, it has already been made by the development team and is accessible as a Snap or a Docker image. The former will be used in this instance.

If you are using Ubuntu 16.04 or later, Snap is already pre-installed and available for usage. Use the Docker image if you're running a different operating system, or install Snap. Type "install micro-ros-agent snap" to install it.
```sudo snap install micro-ros-agent```

We'll need to tweak a few things after installing it because we're utilising a serial connection. We must first enable the hotplug functionality.
```sudo snap set core experimental.hotplug=true```

and then restart the snap demon for it to take effect
```sudo systemctl restart snapd```

Execute after ensuring that the Pi Pico is plugged in.
```
$ snap interface serial-port
name:    serial-port
summary: allows accessing a specific serial port
plugs:
- micro-ros-agent
slots:
- snapd:pico (allows accessing a specific serial port)
```
The micro-ros-agent snap features a serial connector, while a pico slot appears out of nowhere. We should probably link them together based on semantics. To do so, sprint.
```snap connect micro-ros-agent:serial-port snapd:pico```

We're now ready to put our example into action.

We'll start the micro-ros-agent with the Pi Pico connected via USB as follows:
```micro-ros-agent serial --dev /dev/ttyACM0 baudrate=115200```

As soon as the Pi Pico's LED illuminates, the main loop is operational; give it a few seconds to do so. You might need to unplug and then replug the board if it still does not light up after several long seconds (up to 10 mississippi). A few error checks should be included in the example's initialization process. Possibly making that better could be one of your first endeavors.

The LED ought to now be an intense shade of green. I understand. What do you believe to be more cool? It is operating after being installed on your host PC.
```
$ source /opt/ros/dashing/setup.bash
$ ros2 topic echo /pico_publisher
data: 41
---
data: 42
---
```

Then hit
```
$ ros2 node list
/pico_node
```
demonstrates that the Raspberry Pi Pico's micro-ROS node is visible to ROS2 on the host system. Now Enjoy!

**Code Snippet**
```
$ snap interface serial-port
name:    serial-port
summary: allows accessing a specific serial port
plugs:
  - micro-ros-agent
slots:
  - snapd:pico (allows accessing a specific serial port)
```

**References:**
https://www.hackster.io/ros/projects
