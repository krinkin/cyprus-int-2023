# ROS 2 Humble Hawksbill with Yocto and PetaLinux
**Yocto and ROS 2 allow you to build a production-grade custom operating system for robots. How to get ROS 2 Humble into Yocto and PetaLinux?**

![Alt text](HumbleHawksbill_TransparentBG-NoROS.png)

### Things used in this project
**Hardware components**
* AMD Kria KV260 Vision AI Starter Kit ×1	

**Software apps and online services**
* AMD PetaLinux
* ROS Robot Operating System

### Story
First published at https://news.accelerationrobotics.com/ros2-humble-yocto-petalinux/.

Industrial-grade embedded systems often rely on strong multidisciplinary engineering teams that build custom Linux distributions for production by leveraging **Yocto**, a project to build embedded Linux. Rationale is that instead of relying on common development-oriented Linux distros (such as **Debian** or **Ubuntu**), Yocto allows you to build a customized Linux system for your use case. This of course requires you to know what you're doing, but allows you an unmatched granularity wherein you can customize from the bootloader, going through the Linux kernel and all the way into userspace libraries, such as those required to enable ROS 2 support.

_Yocto allows you to build a customized Linux system for your use case, providing unmatched granularity at the cost of complexity._

If you're looking to take ownership of your industrial Linux embedded systems and ROS 2 Humble is what you're looking for, the following might turn helpful.

**Why Yocto/PetaLinux? Why complicate yourself so much?**

Yocto/PetaLinux is hard and the learning curve steep, still, the effort is worth it if you're serious about quality and security in your embedded systems. Let's start with some terminology:

* **Yocto:** an open source collaboration project that helps developers create custom Linux-based systems for embedded products, regardless of the hardware architecture. Technically, it's defined as an umbrella open source project that builds and maintains validated open source tools and components associated with embedded Linux.

* **Poky:** _Yocto_ project open source reference embedded distribution.
* **OpenEmbedded:** Open source build engine for the _Yocto_ project.
* **BitBake:** The task executor and scheduler used by the _OpenEmbedded_ build system to build images.
* **Recipe:** A set of instructions for building packages. A recipe describes where you get source code, which patches to apply, how to configure the source, how to compile it and so on. Recipes also describe dependencies for libraries or for other recipes.
* **Layer:** A collection of related recipes.
* **OpenEmbedded-Core _(OE-core)_:** meta-data comprised of foundational _BitBake_ recipes, classes, and associated files that are meant to be common among many different _OpenEmbedded_-derived systems, including the _Yocto_ project.
* **Honister:** Codename of _Yocto's_ project version _3.4._ Refer to [Yocto releases](https://wiki.yoctoproject.org/wiki/Releases) for more details.
* **PetaLinux:** AMD/Xilinx's proprietary extension of _Yocto_ to support their silicon. As a derivative of _Yocto_, it inherits most aspects and includes additional capabilities provided by AMD/Xilinx.
* **meta-ros:** _OpenEmbedded_ recipes for ROS 1 and ROS 2.
For more definitions, refer to [Yocto Project Terms.](https://docs.yoctoproject.org/3.4.4/ref-manual/terms.html)

Sounds pretty complicated, right? **Well, it is complicated.** Even getting a good grasp of the concepts above and the links between components may take a few months/years[1] :). But the good news is that the resulting knowhow is pretty useful for production environments and industrial-grade robots.

Many articles have been written comparing Yocto and Ubuntu, most seem to agree that the former is meant to build production-grade OSs for embedded systems while the latter for fast prototyping, PoCs and development.

In our experience this is fairly accurate. While developing with widely known Linux distros is definitely recommended in robotics, the resulting firmware when using Yocto delivers not only a smaller (size-wise) footprint, but also a more lean and unloaded execution environment where you have absolute control of what runs. This is crucial for real-time systems _(all robots are network of networks exchanging real-time information)_ wherein besides determinism and performance, quality and security also benefit significantly from using _Yocto._ A security-centric approach requires to keep an inventory of all dependencies used within your system. This comes naturally when using _Yocto_ given its _Recipe_-oriented nature, including the tracking specific versions of each package at use. This allows you to build proper security models to define appropriate security policies matching your use case. _Yocto_ **in combination with OTA capabilities is a hard to beat combination for production environments.**

**Why Yocto/PetaLinux is a great fit for hardware acceleration in robotics?**
Hardware acceleration involves creating custom compute architectures to improve the computing performance. In a nutshell, by designing specialized acceleration kernels, one can build custom brains for robots to hasten their response time. This becomes specially feasible when using adaptive computing and FPGAs which according to previous benchmarks, deliver best results in robotics with ROS [2] [3].

Creating such custom compute architectures involves both hardware and software customization, thereby: _Yocto._ Though complicated, _Yocto_ in combination with hardware acceleration helps deliver high performance production-grade robotic systems.
_Yocto_ in combination with hardware acceleration helps deliver high performance production-grade robotic systems.
All right, so how do I get ROS 2 Humble in Yocto/PetaLinux for my robots' production embedded systems?

**meta-ros layers for ROS 2 Humble Hawksbill**
[Acceleration Robotics](https://accelerationrobotics.com/) is a firm focused on designing customized brains for robots to hasten their response time. The company creates custom compute architectures for high performance robots through hardware acceleration solutions while remaining accelerator-agnostic (FPGAs or GPUs) and as such, building custom high-performing Linux distributions becomes second nature. Adding ROS to such distributions while using _Yocto_ or derivatives (such as _PetaLinux_) can be done with the [meta-ros](https://github.com/ros/meta-ros)layers.

As an active member of the ROS and ROS 2 communities, we're among the early contributors of -meta-ros_ (back in the old ROS 1 days) and first ported meta-ros to ROS 2 a few years back. This time, while serving customers, we have participated in the [ROS 2 Humble beta testing](https://discourse.ros.org/t/branched-humble-call-for-beta-testing/25284/8) with various contributions, including a [Pull Request](https://github.com/ros/meta-ros/pull/1003) with _BitBake_ recipes that update -meta-ros_ and add initial support for ROS 2 Humble in the _Honister_ Yocto release version[4].

Following after some additional support requests, here's a writeup on how you can use these ROS meta layers (and thereby _meta-ros_) to include ROS 2 Humble support in your Yocto/PetaLinux projects. Here it goes in a _development machine using Ubuntu 22.04 and featuring an AMD Ryzen 5 PRO 4650G processor:_

**Step 0. Install Xilinx's PetaLinux**
Get the _2022.1_ that ship with Honister from [AMD XILINX](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/embedded-design-tools.htm) and source it:
```
xilinx@xilinx:~/Downloads$ source /media/xilinx/hd3/tools/Xilinx/PetaLinux2022.1/settings.sh
WARNING: /bin/sh is not bash!
bash is PetaLinux recommended shell. Please set your default shell to bash.
WARNING: This is not a supported OS
INFO: Checking free disk space
INFO: Checking installed tools
INFO: Checking installed development libraries
INFO: Checking network and other services
```

**Step 1. Download Honister's Yocto/PetaLinux BSP**

Get it from Xilinx and set up a project with it. E.g. I'll be using Kria K26 SOM BSP:
```
xilinx@xilinx:~/Downloads$ petalinux-create -t project -s xilinx-k26-som-v2022.1-04191534.bsp
INFO: Create project:
INFO: Projects:
INFO: 	* xilinx-k26-som-2022.1
INFO: Has been successfully installed to /home/xilinx/Downloads/
INFO: New project successfully created in /home/xilinx/Downloads/

xilinx@xilinx:~/Downloads$ cd /home/xilinx/Downloads/xilinx-k26-som-2022.1
xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$
```

**Step 2. Configure the Yocto/PetaLinux project:**
```
xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$ petalinux-config --silentconfig
[INFO] Sourcing buildtools
[INFO] Generating Kconfig for project
[INFO] Silentconfig project
[INFO] Extracting yocto SDK to components/yocto. This may take time!
[INFO] Sourcing build environment
[INFO] Generating kconfig for Rootfs
[INFO] Silentconfig rootfs
[INFO] Generating plnxtool conf
[INFO] Adding user layers
[INFO] Generating workspace directory
[INFO] Successfully configured project
```

**Step 3. Add meta-layers for ROS 2 Humble and configure them in Yocto/PetaLinux:**

**_IMPORTANT:_** keep in mind Xilinx's PetaLinux 2022.1 aligns with Yocto. Honister, so we need to fetch recipes that will build against that
``` xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$ git clone https://github.com/vmayoral/meta-ros -b honister-humble project-spec/meta-ros ```

After cloning the meta-layer (layer of layers) into a the project, the layer should be configured to be built by editing _build/conf/bblayers.conf_ (in my case, _/home/xilinx/Downloads/xilinx-k26-som-2022.1/build/conf/bblayers.conf_) and adding the following at the end:
```
${SDKBASEMETAPATH}/../../project-spec/meta-ros/meta-ros2-humble \
  ${SDKBASEMETAPATH}/../../project-spec/meta-ros/meta-ros2 \
  ${SDKBASEMETAPATH}/../../project-spec/meta-ros/meta-ros-common \
  ```
  
In addition, add the following (e.g. at the beginning of the file) as well to -build/conf/bblayers.conf_ (in my case, _/home/xilinx/Downloads/xilinx-k26-som-2022.1/build/conf/bblayers.conf_), which defines some variables to be used by the _meta-ros_ recipes:
```
#define the ROS 2 Yocto target release
ROS_OE_RELEASE_SERIES = "honister"

# define ROS 2 distro
ROS_DISTRO = "humble"
Altogether, it ends up as follows:
```

Resulting "bblayers.conf" file after adding meta-ros with Humble

**Step 4. Extend Yocto's minimal image with ROS 2 desired content**
There're various ways to build the -meta-ros_ recipes. One simple one (which simplifies the development and debug of recipes) is to extend default PetaLinux image recipe _(petalinux-image-minimal.bb)_ by adding the ROS 2 Humble's content. In particular, I'm adding bare minimum packages required to execute pub/sub examples while including two open source DDS implementations as follows:
```
xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$ mkdir -p project-spec/meta-user/recipes-images/images
xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$ cat << 'EOF' > project-spec/meta-user/recipes-images/images/petalinux-image-minimal.bbappend
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

IMAGE_INSTALL:append = " \
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
    byobu \
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

EOF
```

**Step 5. Build the image**
What's left is to build the project and generate the embedded artifacts desired (kernel, bootloader firmware, rootfs, sysroot, etc.):
``` xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$ petalinux-build ```

This will take a while and in time, you should expect to start seeing something like this until completion:
```
xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$ petalinux-build
[INFO] Sourcing buildtools
[INFO] Building project
[INFO] Sourcing build environment
[INFO] Generating workspace directory
INFO: bitbake petalinux-image-minimal
NOTE: Started PRServer with DBfile: /home/xilinx/Downloads/xilinx-k26-som-2022.1/build/cache/prserv.sqlite3, Address: 127.0.0.1:42673, PID: 891435
WARNING: Host distribution "ubuntu-22.04" has not been validated with this version of the build system; you may possibly experience unexpected failures. It is recommended that you use a tested distribution.
Loading cache: 100% |##########################################################################################################################################################| Time: 0:00:03
Loaded 6463 entries from dependency cache.
Parsing recipes: 100% |########################################################################################################################################################| Time: 0:00:02
Parsing of 4435 .bb files complete (4427 cached, 8 parsed). 6471 targets, 566 skipped, 1 masked, 0 errors.
NOTE: Resolving any missing task queue dependencies
NOTE: Fetching uninative binary shim file:///home/xilinx/Downloads/xilinx-k26-som-2022.1/components/yocto/downloads/uninative/126f4f7f6f21084ee140dac3eb4c536b963837826b7c38599db0b512c3377ba2/x86_64-nativesdk-libc-3.4.tar.xz;sha256sum=126f4f7f6f21084ee140dac3eb4c536b963837826b7c38599db0b512c3377ba2 (will check PREMIRRORS first)
WARNING: Your host glibc version (2.35) is newer than that in uninative (2.34). Disabling uninative so that sstate is not corrupted.
Initialising tasks: 100% |#####################################################################################################################################################| Time: 0:00:29
Checking sstate mirror object availability: 100% |#############################################################################################################################| Time: 0:03:50
Sstate summary: Wanted 4084 Local 0 Network 9 Missed 4075 Current 1688 (0% match, 29% complete)
WARNING: The gcc-cross-aarch64:do_configure sig is computed to be 3d3f3cf0ddf8425c9a9066c82597dd712a38f9fae41bb15e13f48f90d1f78c60, but the sig is locked to 06506f9b8be7a94ee794a1a7979ec5380d298c4c00eb62885f4cc11fb21b78ba in SIGGEN_LOCKEDSIGS_t-x86-64-aarch64
The gcc-cross-aarch64:do_install sig is computed to be a3caa47e1d6280594dd7b32d795b48fad06ea227ccfd39cbdc7deaff3ebabd04, but the sig is locked to fca519891ac4bc8a37f96e3c3fcc43e1cfb5a51b7aaf4089f9e92d5b0081e12c in SIGGEN_LOCKEDSIGS_t-x86-64-aarch64
NOTE: Executing Tasks
WARNING: orocos-kdl-3.3.3-3-r0 do_populate_lic: orocos-kdl: No generic license file exists for: LGPL in any provider
WARNING: cyclonedds-0.9.0-2-r0 do_package: cyclonedds: No generic license file exists for: Eclipse-Distribution-License-1.0 in any provider
WARNING: orocos-kdl-3.3.3-3-r0 do_package: orocos-kdl: No generic license file exists for: LGPL in any provider
Currently 12 running tasks (5773 of 5773/8511 of 14358)  59% |###########################################################################                                                    |
0: boost-1.77.0-r0 do_compile - 4m8s (pid 2313055)
1: python3-numpy-1.21.2-r0 do_compile - 3m37s (pid 2330922)
2: gnutls-3.7.2-r0 do_compile - 3m10s (pid 2342676)
3: perl-5.34.0-r0 do_package - 1m24s (pid 2361158)
4: mpfr-4.1.0-r0 do_configure - 41s (pid 2370206)
5: protobuf-3.18.0-r0 do_configure - 40s (pid 2370851)
6: m4-1.4.19-r0 do_package - 28s (pid 2372860)
7: libxml2-2.9.12-r0 do_package - 27s (pid 2372936)
8: std-msgs-4.2.1-2-r0 do_package - 11s (pid 2377607)
9: orocos-kdl-vendor-0.2.2-2-r0 do_compile (pid 2377860)  62% |##############################################################################                                                |
10: wayland-1.19.0-r0 do_compile (pid 2379087)  96% |###################################################################################################################################     |
11: geometry-msgs-4.2.1-2-r0 do_configure - 2s (pid 2380157)
```
Once it finalizes, you'll have your rootfs under images/linux with ROS 2 Humble.

See the [⚠️Troubleshooting ](https://news.accelerationrobotics.com/ros2-humble-yocto-petalinux/#troubleshooting) section below if you encounter issues. If yours is not reported there, please do report it at [Github](https://github.com/ros/meta-ros/pull/1003) so that it gets considered and added to the list below.

**⚠️ Troubleshooting
cyclonedds-native-0.9.0-2-r0 do_compile: ExecutionError**
**_Edit:_** This issue should've been addressed by  [Github Commits](https://github.com/ros/meta-ros/pull/1003/commits/46ebc6666309320cd3d9346c42e076a2b1a9057e). Fetch the latest version of the branch to get the commit.
```
ERROR: cyclonedds-native-0.9.0-2-r0 do_compile: ExecutionError('/home/xilinx/Downloads/xilinx-k26-som-2022.1/build/tmp/work/x86_64-linux/cyclonedds-native/0.9.0-2-r0/temp/run.do_compile.792086', 1, None, None)
ERROR: Logfile of failure stored in: /home/xilinx/Downloads/xilinx-k26-som-2022.1/build/tmp/work/x86_64-linux/cyclonedds-native/0.9.0-2-r0/temp/log.do_compile.792086
Log data follows:
| DEBUG: Executing shell function do_compile
| NOTE: VERBOSE=1 cmake --build /home/xilinx/Downloads/xilinx-k26-som-2022.1/build/tmp/work/x86_64-linux/cyclonedds-native/0.9.0-2-r0/build --target all --
| ninja: error: 'src/tools/ddsperf/CycloneDDS::idlc', needed by 'src/tools/ddsperf/ddsperf_types.c', missing and no known rule to make it
| WARNING: exit code 1 from a shell command.
ERROR: Task (virtual:native:/home/xilinx/Downloads/xilinx-k26-som-2022.1/components/yocto/../../project-spec/meta-ros/meta-ros2-humble/generated-recipes/cyclonedds/cyclonedds_0.9.0-2.bb:do_compile) failed with exit code '1'
```

what fails ❌ is the -native version of the recipe. Looking into the logs, it appears a misconfiguration with ninja, which doesn't happen if done manually.

⚠️As a quick fix ⚠️, removing the Yocto/PetaLinux misconfiguration (in my case living under living under /home/xilinx/Downloads/xilinx-k26-som-2022.1/build/tmp/work/x86_64-linux/cyclonedds-native/0.9.0-2-r0/build) and manually configuring things properly against the sources (through cmake -G Ninja ../git), recipe led things to build just fine. More specifically:
```
xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$ cd build/tmp/work/x86_64-linux/cyclonedds-native/0.9.0-2-r0/build
xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$ rm -r * && cmake -G Ninja ../git
xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$ cd ~/Downloads/xilinx-k26-som-2022.1
xilinx@xilinx:~/Downloads/xilinx-k26-som-2022.1$ petalinux-build -c cyclonedds-native  # re-build only cyclonedds
[INFO] Sourcing buildtools
[INFO] Building cyclonedds-native
[INFO] Sourcing build environment
[INFO] Generating workspace directory
INFO: bitbake cyclonedds-native
NOTE: Started PRServer with DBfile: /home/xilinx/Downloads/xilinx-k26-som-2022.1/build/cache/prserv.sqlite3, Address: 127.0.0.1:39399, PID: 889351
WARNING: Host distribution "ubuntu-22.04" has not been validated with this version of the build system; you may possibly experience unexpected failures. It is recommended that you use a tested distribution.
Loading cache: 100% |##########################################################################################################################################################| Time: 0:00:03
Loaded 6463 entries from dependency cache.
Parsing recipes: 100% |########################################################################################################################################################| Time: 0:00:02
Parsing of 4435 .bb files complete (4427 cached, 8 parsed). 6471 targets, 566 skipped, 1 masked, 0 errors.
NOTE: Resolving any missing task queue dependencies
NOTE: Fetching uninative binary shim file:///home/xilinx/Downloads/xilinx-k26-som-2022.1/components/yocto/downloads/uninative/126f4f7f6f21084ee140dac3eb4c536b963837826b7c38599db0b512c3377ba2/x86_64-nativesdk-libc-3.4.tar.xz;sha256sum=126f4f7f6f21084ee140dac3eb4c536b963837826b7c38599db0b512c3377ba2 (will check PREMIRRORS first)
WARNING: Your host glibc version (2.35) is newer than that in uninative (2.34). Disabling uninative so that sstate is not corrupted.
Initialising tasks: 100% |#####################################################################################################################################################| Time: 0:00:05
Sstate summary: Wanted 1 Local 0 Network 0 Missed 1 Current 74 (0% match, 98% complete)
NOTE: Executing Tasks
NOTE: Tasks Summary: Attempted 281 tasks of which 275 didn't need to be rerun and all succeeded.

Summary: There were 2 WARNING messages shown.
INFO: Failed to copy built images to tftp dir: /tftpboot
[INFO] Successfully built cyclonedds-native
```

**graphviz dpkg-architecture: command not found**
```
| /usr/lib//tcl8.6/tclConfig.sh: line 2: dpkg-architecture: command not found
| /usr/lib//tcl8.6/tclConfig.sh: line 2: dpkg-architecture: command not found
| Segmentation fault (core dumped)
| NOTE: The following config.log files may provide further information.
| NOTE: /home/xilinx/Downloads/xilinx-k26-som-2022.1/build/tmp/work/x86_64-linux/graphviz-native/2.44.1-r0/graphviz-2.44.1/config.log
| ERROR: configure failed
| WARNING: exit code 1 from a shell command.
ERROR: Task (virtual:native:/home/xilinx/Downloads/xilinx-k26-som-2022.1/components/yocto/layers/meta-openembedded/meta-oe/recipes-graphics/graphviz/graphviz_2.44.1.bb:do_configure) failed with exit code '1'
```

**The following did for me:**
``` sudo apt-get remove libtcl8.6 ```
``` sudo apt-get update ```
``` sudo apt-get install blt libopencv-contrib-dev libopencv-dev libopencv-viz-dev libopencv-viz4.5d libtcl8.6 ```

**References**
Given the already very complex system integration effort, let's simplify :
* [Hardware Accelerating ROS 2 Nodes for Perception](https://news.accelerationrobotics.com/hardware-accelerating-ros-2-nodes/)
* [Hardware accelerated ROS 2 pipelines and towards the Robotic Processing Unit (RPU)](https://news.accelerationrobotics.com/hardware-accelerated-ros2-pipelines/)
* [ROS 2 Humble Hawksbill release timeline](https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html#release-timeline)
