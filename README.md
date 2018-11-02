# Laser agregator

Laser agregator is a ROS node that agregator lightweight point clouds from pre-registered from organized point clouds (eg. velodyne laser scanner).

### Features
 - Mesh viewer through Libigl
 - Realtime point cloud agregation and interactive manipulation

### Build

Laser agregator uses a number of open source projects to work properly:
* [Libigl] - Geometry processing library
* [Imgui] - Immediate Mode Graphical User interface for C++
* [Loguru] - Lightweight logging library

To download the dependencies and build:

```sh
$ git clone --recursive https://github.com/RaduAlexandru/laser_agregator.git
$ cd laser_agregator
$ catkin build --this
```

Also it uses organized point clouds processed using the modified [velodyne drivers] which need to be built as a ROS package inside your workspace.

### Usage 

The program is started using ROS launch files. An examples of such launch file can be seen in launch/lbh_long_full.launch 
It starts running a certain ROS bag which contains the necessary topics (specifically velodyne_points). If the bag does not have velodyne_points published as an organized cloud you can use an additional node which subscribes to the velodyne_packets and publishes velodyne_points as organized cloud. This can be done using the modified [velodyne drivers]. An example of a small launch file that uses the velodyne_drivers can be seen in launch/agregate_lbg_long.launch.


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [Libigl]: <https://github.com/libigl/libigl>
   [Imgui]: <https://github.com/ocornut/imgui>
   [Loguru]: <https://github.com/emilk/loguru>
   [velodyne drivers]: <https://github.com/RaduAlexandru/velodyne_drivers>
