## laser_scan_matcher
1. laser_scan_matcher can be used to simulate odometry, laser_scan_matcher is inside scan_tools but we 
only need it.
2. listen from [scan_tools](https://github.com/ccny-ros-pkg/scan_tools), laser_scan_matcher, one processing tool among scan_tools.

### Dependencies
laser_scan_matcher depends on csm, while csm depends on pcl(point cloud library)'s 3 packages: pcl_conversions, pcl_msgs and pcl_ros.

### Based on [wstool](http://wiki.ros.org/wstool)
We are more familiar with catkin build process, so, here mention something about wstool using follow.
1>. Create a catkin Workspace with wstool
   First, create a catkin workspace:
   ```sh
   $ mkdir -p ~/ros_catkin_ws
   $ cd ~/ros_catkin_ws
   ```
2>. Initialize the Workspace Without a rosinstall file
   This will initialize an empty workspace. If you have a rosinstall file that you want to base your workspace on, skip to Initialize the Workspace from a rosinstall File below.
   ```sh
   $ wstool init src
   ```
    If you have rosinstall files to add to the workspace, proceed to Merge in Additional rosinstall Files below.
3>. Initialize the Workspace from a rosinstall File
   If you have already initialized your workspace, skip this step. If you have rosinstall files to add to the workspace, proceed to Merge in Additional rosinstall Files below.
   ```sh
   $ wstool init src PATH_TO_ROSINSTALL_FILE.rosinstall
   ```
   If you have rosinstall files to add to the workspace, proceed to Merge in Additional rosinstall Files below.
4>. Merge in Additional rosinstall Files
   For each rosinstall file you want to add to your workspace, run this command
   ```sh
   $ wstool merge -t src PATH_TO_ROSINSTALL_FILE.rosinstall
   ```
5>. Updating the Workspace
   After you've created your workspace and added repositories, you should update it to download the latest versions.
   ```sh
   $ wstool update -t src
   ```

### How to use on Ubuntu?

1>. Installing PCL packages: pcl_conversions, pcl_msgs and pcl_ros

   ```sh
   cd <your-ros-catkin-ws>
   ```

2>. ​



### How to use on Raspberrypi?
1>. Installing PCL

> - Reference: [Errors at catkin_make for laser_scan_matcher](http://answers.ros.org/question/197658/errors-at-catkin_make-for-laser_scan_matcher/)
```shell
$ cd ~/ros_catkin_ws/
$ rosinstall_generator pcl_conversions pcl_msgs pcl_ros --rosdistro <your-ros-version> --deps --wet-only --exclude roslisp --tar > ros_pcl.rosinstall
$ wstool init src indigo-ros_pcl.rosinstall
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie
# >j1 may fail with memory errors and given the swap space usage uses up too much bandwidth
$ sudo ./src/catkin/bin/catkin_make_isolated --pkg pcl_conversions pcl_msgs pcl_ros --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo -j1
```
> In China, you may need to change the source，
```shell
$ sudo nano /etc/apt/sources.list
```
> and use the following source:
```shell
$ deb http://mirrors.ustc.edu.cn/raspbian/raspbian/ jessie main contrib non-free rpi
```
> c++: internal compiler error: Killed (program cc1plus)
> - Default swap space for raspberrypi is about 100Mb, and the Raspberrypi2 has 1Gb RAM. 1Gb is not large enough, so it's better to set the swap space twice as the RAM, 2Gb. Howerer, if your raspberrypi's SD card is just 8Gb, 512Mb is better.
> - The following commands set the swap space to 512Mb, invalid after restarting
```shell
$ free -m
$ sudo dd if=/dev/zero of=/var/swap.img bs=1024k count=512
$ sudo mkswap /var/swap.img
$ sudo swapon /var/swap.img
$ free -m
```
2>. Installing csm
> Reference: [how to: Building PCL & laser_scan_matcher on Raspberry pi 2](http://answers.ros.org/question/229788/how-to-building-pcl-laser_scan_matcher-on-raspberry-pi-2/)
```shell
$ cd ~/my_catkin_ws/src
# clone the dependency csm into catkin workspace's src
$ git clone https://github.com/AndreaCensi/csm.git
$ cd csm/
$ ./install_quickstart.sh
```
3>. laser_scan_matcher, clone and catkin_make
```shell
$ cd ~/my_catkin_ws/src
# clone laser_scan_matcher into catkin workspace's src
$ git clone https://github.com/Durant35/laser_scan_matcher.git
# add csm path
$ export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:<your-catkin-workspace>/src/csm/sm/pkg-config
# make targets
$ cd ..
catkin_make laser_scan_matcher
```
