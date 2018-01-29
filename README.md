# Laser mesher

Laser mesher is a ROS node that builds textured meshes from organized point clouds (eg. velodyne laser scanner).

### Features
 - Mesh viewer through Libigl
 - Realtime meshing and interactive manipulation

### Build

Laser mesher uses a number of open source projects to work properly:
* [Libigl] - Geometry processing library
* [Imgui] - Immediate Mode Graphical User interface for C++
* [Loguru] - Lightweight logging library

To download the dependencies and build:

```sh
$ git clone --recursive https://github.com/RaduAlexandru/laser_mesher.git
$ cd laser_mesher
$ catkin build --this
```

Also it uses organized point clouds processed using the modified [velodyne drivers] which need to be built as a ROS package inside your workspace.

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [Libigl]: <https://github.com/libigl/libigl>
   [Imgui]: <https://github.com/ocornut/imgui>
   [Loguru]: <https://github.com/emilk/loguru>
   [velodyne drivers]: <https://github.com/RaduAlexandru/velodyne_drivers>
# laser_mesher_rbf
# laser_texturer
# laser_agregator
