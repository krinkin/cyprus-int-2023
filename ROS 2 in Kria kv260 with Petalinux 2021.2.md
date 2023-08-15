# ROS 2 in Kria kv260 with Petalinux 2021.2

**This tutorial is on how to install ROS 2 in a Petalinux 2021.2 image and launch minimal publisher/subscriber examples.**

Adding ROS 2 to Kria kv260 PetaLinux 2021.2 build
1. Create the project from BSP
2. Adding meta-ros layer to PetaLinux
3. Create a custom image containing ROS 2 packages
4. Build PetaLinux project
5. Create a bootable image for Kria kv26
6. Run ROS 2 minimal publisher/subscriber examples


_Things used in this project_
1. Hardware components
- AMD Kria KV260 Vision AI Starter Kit ×	1	

2. Software apps and online services
-  AMD PetaLinux
-  ROS Robot Operating System

## Adding ROS 2 to Kria kv260 PetaLinux 2021.2 build

This issue demonstrates how to integrate ROS 2 into a Petalinux project. Since the PetaLinux 2021.2 on which this article is based doesn't yet have a Kria KV260 Starter Kit BSP, we created one from scratch using the older Kria K26 Starter Kit 2021 as a starting point.1. This procedure for creating the 2021.2 Starter Kit BSP is not yet documented; it will be made accessible shortly, but in the meantime, if you need further details, get in touch with me or read the README file contained in the 2021.1 Starter Kit BSP.
We'll skip the step of installing PetaLinux on your Linux system because there are plenty of resources online, and instead move right on to the procedures for starting a minimal publish/subscriber example utilizing ROS 2 in the Kria board:


**1. Create the project from BSP**
Run the following commands to create the PetaLinux template from the custom BSP.

```$ petalinux-create -t project -s xilinx-k26-starterkit-v2021.2-custom.bsp -n petalinux-ros2```
```$ cd petalinux-ros2```
```$ petalinux-config --silentconfig```

**2. Adding meta-ros layer to PetaLinux**
If you are interested in what is included inside meta-ros or to know more about the installation process you can read the official guide.
- First, we need to install the needed dependencies:
```sudo apt-get install gawk wget git-core diffstat unzip texinfo gcc-multilib \```
```build-essential chrpath socat cpio python python3-pip python3-pexpect \```
```xz-utils debianutils iputils-ping \```
```python3-git python3-jinja2 libegl1-mesa libsdl1.2-dev xterm \```
```g++-multilib locales lsb-release python3-distutils time```
```sudo locale-gen en_US.utf8```

- Second, inside the project-spec folder, you need to download meta-ros layer.
```git clone https://github.com/ros/meta-ros.git -b gatesgarth``` 
```cd meta-ros```
```git checkout 56cf4de00280451bf5a36f55acbec32b9f22f841```
 
Here we are using gatesgarth branch because PetaLinux is based on this release as we can see inside project-spec/meta-user/conf/layer.conf.
```LAYERSERIES_COMPAT_meta-user = "gatesgarth"```

Then we move to the 56cf4de commit point in history. But why to this point?

We move here because in the next commit to this, 0e373be, they did a change of all the ''_'' with '':'' that is not supported in PetaLinux 2021.2 openembedded core layer and throws an error when adding the layers to the Petalinux project because it doesn't know how to parse this ":".

- Third, before using the PetaLinux config interface we need to add in build/conf/bblayers.conf the next two variables:
//define the ROS 2 Yocto target release
```ROS_OE_RELEASE_SERIES = "gatesgarth"```
//define ROS 2 distro
```ROS_DISTRO = "rolling"```

We will be adding rolling ROS 2 distro to our images but if you want you can add here foxy or galactic. Depending on which one you use you might need to change the imported layers below.
This is a little bit ugly because if you do petalinux-build -x mrproper this file will be erased and you will need to set these variables again but we haven't found how to do it in another way.

- Forth, for configuring the layer path in the PetaLinux build system you need to move to your PetaLinux project directory,  and add the layer path in the below config option.
```petalinux-config --> Yocto Settings --> User Layers```
```--> (${PROOT}/project-spec/meta-ros/*) user layer 0```
```--> ()    user layer 1```

You need to exactly add the next 4 layers:
```${PROOT}/project-spec/meta-ros/meta-ros-backports-hardknott```
```${PROOT}/project-spec/meta-ros/meta-ros-common```
```${PROOT}/project-spec/meta-ros/meta-ros2```
```${PROOT}/project-spec/meta-ros/meta-ros2-rolling```

And the end you will have something like this:


After saving you should see now how are they added to build/conf/bblayers.conf

**3. Create a custom image containing ROS 2 packages**
For this, you need to define a petalinux-image-minimal.bbappend in your PetaLinux project:
```cd project-spec/meta-user```
```mkdir -p recipes-images/images;```
```cd recipes-images/images```
```touch petalinux-image-minimal.bbappend```

And add this content:
```
require ${COREBASE}/../meta-petalinux/recipes-core/images/petalinux-image-minimal.bb

SUMMARY = "A image including a bare-minimum installation of ROS 2 and including some basic pub/sub examples. It includes two DDS middleware implementations, FastDDS and Cyclone DDS"
DESCRIPTION = "${SUMMARY}"

inherit ros_distro_${ROS_DISTRO}
inherit ${ROS_DISTRO_TYPE}_image

ROS_SYSROOT_BUILD_DEPENDENCIES = " \
    ament-lint-auto \
    ament-cmake-auto \
    ament-cmake-core \
    ament-cmake-cppcheck \
    ament-cmake-cpplint \
    ament-cmake-export-definitions \
    ament-cmake-export-dependencies \
    ament-cmake-export-include-directories \
    ament-cmake-export-interfaces \
    ament-cmake-export-libraries \
    ament-cmake-export-link-flags \
    ament-cmake-export-targets \
    ament-cmake-gmock \
    ament-cmake-gtest \
    ament-cmake-include-directories \
    ament-cmake-libraries \
    ament-cmake \
    ament-cmake-pytest \
    ament-cmake-python \
    ament-cmake-ros \
    ament-cmake-target-dependencies \
    ament-cmake-test \
    ament-cmake-version \
    ament-cmake-uncrustify \
    ament-cmake-flake8 \
    ament-cmake-pep257 \
    ament-copyright \
    ament-cpplint \
    ament-flake8 \
    ament-index-python \
    ament-lint-cmake \
    ament-mypy \
    ament-package \
    ament-pclint \
    ament-pep257 \
    ament-pycodestyle \
    ament-pyflakes \
    ament-uncrustify \
    ament-xmllint \
    cmake \
    eigen3-cmake-module \
    fastcdr \
    fastrtps-cmake-module \
    fastrtps \
    git \
    gmock-vendor \
    gtest-vendor \
    pkgconfig \
    python-cmake-module \
    python3-catkin-pkg \
    python3-empy \
    python3 \
    python3-nose \
    python3-pytest \
    rcutils \
    rmw-implementation-cmake \
    rosidl-cmake \
    rosidl-default-generators \
    rosidl-generator-c \
    rosidl-generator-cpp \
    rosidl-generator-dds-idl \
    rosidl-generator-py \
    rosidl-parser \
    rosidl-runtime-c \
    rosidl-runtime-cpp \
    rosidl-typesupport-c \
    rosidl-typesupport-cpp \
    rosidl-typesupport-fastrtps-cpp \
    rosidl-typesupport-interface \
    rosidl-typesupport-introspection-c \
    rosidl-typesupport-introspection-cpp \
    foonathan-memory-vendor \
    libyaml-vendor \
"

IMAGE_INSTALL_append = " \
    ros-base \
    examples-rclcpp-minimal-action-client \
    examples-rclcpp-minimal-action-server \
    examples-rclcpp-minimal-client \
    examples-rclcpp-minimal-composition \
    examples-rclcpp-minimal-publisher \
    examples-rclcpp-minimal-service \
    examples-rclcpp-minimal-subscriber \
    examples-rclcpp-minimal-timer \
    examples-rclcpp-multithreaded-executor \
    examples-rclpy-executors \
    examples-rclpy-minimal-action-client \
    examples-rclpy-minimal-action-server \
    examples-rclpy-minimal-client \
    examples-rclpy-minimal-publisher \
    examples-rclpy-minimal-service \
    examples-rclpy-minimal-subscriber \
    demo-nodes-cpp \
    demo-nodes-cpp-rosnative \
    demo-nodes-py \
    cyclonedds \
    rmw-cyclonedds-cpp \
    tmux \
    python3-argcomplete \
    glibc-utils \
    localedef \
    rt-tests \
    stress \
    xrt-dev \
    xrt \
    zocl \
    opencl-headers-dev \
    opencl-clhpp-dev \
    ${ROS_SYSROOT_BUILD_DEPENDENCIES} \
"

IMAGE_LINGUAS = "en-us"
GLIBC_GENERATE_LOCALES = "en_US.UTF-8"
```
**4. Build PetaLinux project**

Before building the last thing you need to do is to add in project-spec/meta-user/conf/petalinuxbsp.conf the next line:
```SIGGEN_UNLOCKED_RECIPES += "gcc-cross-aarch64"```

This solves some problems with building the image.
Now, you are finally ready to build the image with:
```petalinux-build```

**5. Create a bootable image for Kria kv26**

Once the image is created you can go to images/linux folder and create your sd-card image:
```petalinux-package --wic --bootfiles "ramdisk.cpio.gz.u-boot boot.scr Image system.dtb"```

This will create a _petalinux-sdimage.wic_ that you can flash using _balenaEtcher_.

**6. Run ROS 2 minimal publisher/subscriber examples**

If you connect to your board using ssh and use the next commands:

- Terminal 1
```$ source /usr/bin/ros_setup.bash```
```$ ros2 run examples_rclcpp_minimal_publisher publisher_lambda```

- Terminal 2
```
$ ros2 node list
/minimal_publisher
$ ros2 node info /minimal_publisher
/minimal_publisher
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /topic: std_msgs/msg/String
  Service Servers:
    /minimal_publisher/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /minimal_publisher/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /minimal_publisher/get_parameters: rcl_interfaces/srv/GetParameters
    /minimal_publisher/list_parameters: rcl_interfaces/srv/ListParameters
    /minimal_publisher/set_parameters: rcl_interfaces/srv/SetParameters
    /minimal_publisher/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
$ ros2 topic list
/parameter_events
/rosout
/topic
```

And finally,

```$ source /usr/bin/ros_setup.bash```
```$ ros2 run examples_rclcpp_minimal_subscriber subscriber_lambda```


Thank you for reading! I hope you find this tutorial useful and that you can start using ROS 2 with this new version of PetaLinux. Cheers!

- This project also works for Ultra96v2 board. You can skip step 1 and start from step 2 . You only need an Ultra96v2    PetaLinux project.

- To create one you can follow Mario Bergeron _Appendix 2- Rebuilding the Design guide_ and then change the PetaLinux      project.

- If you have problems fetching rolling recipes, please use galactic or foxy.

**References:**
[Hackster](https://www.hackster.io/)
[<Packt>](https://www.packtpub.com/product/ros-robotics-projects)
[Skyfi labs](https://www.skyfilabs.com/blog/10-simple-ros-projects-for-beginners)
[Robocademy](https://robocademy.com/2020/10/15/open-source-ros-projects-from-ros-developer-learning-path/)







