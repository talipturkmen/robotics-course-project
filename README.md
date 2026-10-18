# Robotics Course Project

> **Archived.** This is a 2020 university course project (robotics, ROS) and is no longer maintained.

Project 1 of a robotics course: a ROS node (`lla2enu`) that converts GPS fixes (latitude, longitude, altitude) into a local East-North-Up (ENU) Cartesian frame around a fixed reference point. For each subscribed GPS topic it:

- converts the fix to ECEF and then to ENU relative to a reference origin (set by ROS parameters, with built-in defaults),
- broadcasts a TF transform from the `world` frame to the topic name,
- publishes a `nav_msgs/Odometry` message with the position and a finite-difference linear velocity,
- treats an all-zero fix as signal loss and publishes NaN positions.

## Tech stack

- ROS 1 (catkin), C++11
- Packages: roscpp, geodesy, nav_msgs, sensor_msgs, std_msgs, tf

## Repository structure

- `Project-1/lla2enu/` - the catkin package
  - `src/sub.cpp` - the node (`lla2enu`)
  - `launch/lla2enu.launch` - starts two node instances (`front` and `obs`) under the `/lla2enu` namespace with reference-point parameters
  - `CMakeLists.txt`, `package.xml` - build and package definitions

## How to run

Assumes a ROS 1 installation with catkin. From a catkin workspace:

```
cp -r /path/to/robotics-course-project/Project-1/lla2enu ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch lla2enu lla2enu.launch
```

The node subscribes to `/swiftnav/<name>/gps_pose` (`sensor_msgs/NavSatFix`) for the names `front` and `obs`. Those topics are not included in this repository; a GPS source or a bag file publishing them is needed to see output.

Known gaps: `package.xml` still contains template placeholders (maintainer address, license `TODO`), and it does not declare the `tf`, `nav_msgs` and `sensor_msgs` dependencies that the source uses.
