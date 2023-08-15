# SLAM-based autonomous localization and navigation for robots

## Things used in this project
**Hardware components**
* Elephant Robotics myAGV ×1		
* Raspberry Pi 4 Model B ×1	

**Software apps and online services**
* ROS Robot Operating System


### Story

**Program**：Map creation and automatic navigation with myAGV

**Equipment** ：
1. myAGV

myAGV is an automous navigation smart vehicle from Elephant Robotics. It uses the competition-level Mecanum wheels and a full wrap design with a metal frame. There are two built-in SLAM algorithms to meet the learning of mapping and navigation directions.

2. A normal computer.
The autonomous robot positioning and navigation technology includes two parts: position&map creation (SLAM), path planning&motion control. SLAM only completes the robot's positioning and map creation.

The main solution for implementing localization and navigation technology is, SLAM, path planning, andmotion control.

The process that a robot descirbe the environment and recognize the environment mainly depends on the map. It uses environmental maps to describe its current environmental information and adopts different forms of map descriptions with the algorithm and sensor differences used.

We use the gmapping algorithm in the SLAM algorithm. The gmapping algorithm builds the map based on the raster map.

**Gmapping**
Gmapping is a SLAM algorithm based on 2D LiDAR using RBPF (Rao-Blackwellized Particle Filters) algorithm to complete 2D raster map construction.

**Advantages:** gmapping can build indoor environment maps in real-time, with less computation in small scenes and higher map accuracy, and requires less LIDAR scanning frequency.

**Disadvantages:** As the environment grows larger, the memory and computation required to build the map become huge, so gmapping is unsuitable for large scene composition. An intuitive feeling is that for a range of 200x200 meters, if the raster resolution is 5cm and each raster occupies one byte of memory, then each particle carries 16M of RAM for the map, and if it is 100 particles, it is 1.6G of RAM.

**Raster Map**
The most common way for robots to describe the environment is Grid Map or Occupancy Map, which divides the environment into a series of grids, where each grid is given a possible value indicating the probability that the grid will be occupied.

#### Start the project
Because myAGV has a built-in Raspberry Pi computer, controlling myAGV requires leaving the keyboard and mouse and monitor, so a computer is needed to control myAGV's computer. We use VNC remote control it.

**VNC**
VNC (Virtual Network Console) stands for Virtual Network Console. It is an excellent remote control tool software developed by the famous AT&T's European research labs.VNC is a free open source software based on UNIX and Linux operating systems with powerful remote control capabilities, efficient and practical, and its performance is comparable to any remote control software in Windows and MAC.

Put the myAGV on a horizontal place.

The launch file will start the odometer and IMU sensor of myAGV.

_Enter the command in the terminal:_
```roslaunch myagv_odometry myagv_active.launch```

After turning on the odometer and IMU sensors, we then turn on the radar and gmapping algorithms to start the map creation.

_Enter the command in the terminal:_
```roslaunch myagv_navigation myagv_slam_laser.launch```

This is the page we just started, then we move myAGV and we can draw the map out. To control myAGV movement, Elephant Robotics launched the keyboard control.

Save the map.

_Enter the command in the terminal:_
```rosrun map_server map_saver```

The next step is to make myAGV able to navigate automatically on the map, where myAGV can avoid obstacles to its destination (navigation) automatically by clicking on it.

First, we load the map and modify the path into our startup file.

**Path planning + motion control**
Movement planning is a big concept. For example, the movement of robotic arms, the flight of vehicles, and the path planning of myAGVs we are talking about here, all are in motion planning.

Let's talk about motion planning for these types of wheeled robots. The basic capability required here is path planning, that is, the ability to perform what is generally called target point navigation after completing SLAM. In short, it means planning a path from point A to point B, and then make the robot move over it.

1. Global Planning
To achieve this process, motion planning has to implement at least two levels of modules, and one is called global planning, which is a bit like our car navigator. It needs to pre-plan a route on the map and also the current robot's position. This is provided by our SLAM system. The industry typically uses an algorithm called A* to implement this process, which is a heuristic search algorithm that is excellent. It is mostly used in games, such as real-time strategy games like Starcraft and Warcraft, which use this algorithm to calculate the movement trajectory of units.

2. Partial Planning
Of course, just planning the path is not enough. There are many unexpected situations in reality. For example, a small child is in the way, so the original path needs to be adjusted. Of course, sometimes, this adjustment does not require a recalculation of the global path, and the robot may be able to make a slight detour. In this case, we need another level of planning called local planning. It may not know where the robot will end up, but it is particularly good at getting around the obstacles in front of it.

Next we start the program and open the saved maps and the autopilot function.

_Enter the command in the terminal:_
```roslaunch myagv_navigation navigation_active.launch```

Use keyboard control to make myAGV rotate in place for positioning. After the positioning is completed and the point cloud converges, proceed to the next navigation step.

Click "2D Nav Goal" on the top, click the point on the map you want to reach, and myAGV will set off towards the target point. You can also see a planned path of myAGV between the starting point and the target point in rviz, and myAGV will move along the route to the target point.

_This is the end of the project_

**Summary**
The navigation demonstrated at present is only a relatively basic situation. There are many mobile robots on the market, such as sweeping robots. It needs to plan paths according to different environments, which is more complicated. For the problem of path planning in different environments, there is a unique called space coverage, which also proposes a lot of algorithms and theories.

There is still a lot of work to be done to navigate the SLAM algorithm in the future.

**References:**
[Hackster](https://www.hackster.io/)
[<Packt>](https://www.packtpub.com/product/ros-robotics-projects)
[Skyfi labs](https://www.skyfilabs.com/blog/10-simple-ros-projects-for-beginners)
[Robocademy](https://robocademy.com/2020/10/15/open-source-ros-projects-from-ros-developer-learning-path/)
