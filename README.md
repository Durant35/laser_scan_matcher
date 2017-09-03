# laser_scan_matcher
1. laser_scan_matcher can be used to fake odometry data from LiDAR data.
2. listen from [scan_tools](https://github.com/ccny-ros-pkg/scan_tools), laser_scan_matcher is inside scan_tools but we only need it.

### Dependencies
+ laser_scan_matcher depends on [csm](https://github.com/AndreaCensi/csm), while csm depends on pcl(point cloud library)'s 3 packages: pcl_conversions, pcl_msgs and pcl_ros.

### Based on [wstool](http://wiki.ros.org/wstool)
+ We are more familiar with catkin build process, so, here mention something about wstool using follow.

1. Create a catkin Workspace with wstool
   First, create a catkin workspace:
   ```sh
   $ mkdir -p ~/ros_catkin_ws
   $ cd ~/ros_catkin_ws
   ```

2. Initialize the Workspace Without a rosinstall file
   This will initialize an empty workspace. If you have a rosinstall file that you want to base your workspace on, skip to Initialize the Workspace from a rosinstall File below.
   ```sh
   $ wstool init src
   ```

    If you have rosinstall files to add to the workspace, proceed to Merge in Additional rosinstall Files below.

3. Initialize the Workspace from a rosinstall File
   If you have already initialized your workspace, skip this step. If you have rosinstall files to add to the workspace, proceed to Merge in Additional rosinstall Files below.
   ```sh
   $ wstool init src PATH_TO_ROSINSTALL_FILE.rosinstall
   ```

   If you have rosinstall files to add to the workspace, proceed to Merge in Additional rosinstall Files below.

4. Merge in Additional rosinstall Files
   For each rosinstall file you want to add to your workspace, run this command
   ```sh
   $ wstool merge -t src PATH_TO_ROSINSTALL_FILE.rosinstall
   ```

5. Updating the Workspace
   After you've created your workspace and added repositories, you should update it to download the latest versions.
   ```sh
   $ wstool update -t src
   ```

### How to use on Ubuntu?

> Maybe you need to install following tools when necessary.
>
> ```sh
> $ sudo apt-get install python-wstool
> $ sudo apt-get install python-rosinstall-generator
> ```

1. Installing PCL packages: [pcl_conversions](https://github.com/ros-perception/pcl_conversions), [pcl_msgs](https://github.com/ros-perception/pcl_msgs) and [pcl_ros](https://github.com/ros-perception/perception_pcl).
   ```sh
   $ cd <your-catkin-workspace-path>
   $ rosinstall_generator pcl_conversions pcl_msgs pcl_ros ‐‐rosdistro <your-ros-version> ‐‐deps ‐‐wet‐only ‐‐exclude roslisp ‐‐tar > ros_pcl.rosinstall
   $ wstool init src ros_pcl.rosinstall
   # This rosdep command installs all the missing system dependency
   # (must be described in package.xml) in all the packages in your src directory.
   $ rosdep install ‐‐from‐paths src ‐‐ignore‐src ‐‐rosdistro <your-ros-version> ‐y ‐r
   $ catkin_make
   ```

2. Installing csm.

   ```sh
   $ sudo apt-get install gsl-bin libgsl0-dev
   $ cd <your-catkin-workspace-path>/src
   $ git clone https://github.com/AndreaCensi/csm.git
   $ cd csm/
   $ ./install_quickstart.sh
   ```

3. Installing laser_scan_matcher.

   ```sh
   $ cd <your-catkin-workspace-path>/src
   # clone laser_scan_matcher into catkin workspace's src
   $ git clone https://github.com/Durant35/laser_scan_matcher.git
   # add csm path into your cmake's PKG_CONFIG_PATH
   $ export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:<your-catkin-workspace-path>/src/csm/sm/pkg‐config
   # make targets
   $ cd <your-catkin-workspace-path>
   $ catkin_make laser_scan_matcher
   ```

### How to use on Raspberrypi?

1. Installing PCL.

   > Reference: [Errors at catkin_make for laser_scan_matcher](http://answers.ros.org/question/197658/errors-at-catkin_make-for-laser_scan_matcher/)

   ```sh
   $ cd <your-catkin-workspace-path>
   $ rosinstall_generator pcl_conversions pcl_msgs pcl_ros --rosdistro <your-ros-version> --deps --wet-only --exclude roslisp --tar > ros_pcl.rosinstall
   $ wstool init src ros_pcl.rosinstall
   $ rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie
   # >j1 may fail with memory errors and given the swap space usage uses up too much bandwidth
   $ sudo ./src/catkin/bin/catkin_make_isolated --pkg pcl_conversions pcl_msgs pcl_ros --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/<your-ros-version> -j1
   ```

   > Problem 1: In China, you may need to **change the source**，
   >
   > ```sh
   > $ sudo nano /etc/apt/sources.list
   > ```
   >
   > and use the following source:
   >
   > ```sh
   > $ deb http://mirrors.ustc.edu.cn/raspbian/raspbian/ jessie main contrib non-free rpi
   > ```

   >Problem 2: c++: **internal compiler error: Killed** (program cc1plus)
   >
   >+ Default swap space for raspberrypi is about 100Mb, and the Raspberrypi2 has 1Gb RAM. 1Gb is not large enough, so it's better to set the swap space twice as the RAM, 2Gb. Howerer, if your raspberrypi's SD card is just 8Gb, 512Mb is better.
   >+ The following commands set the swap space to 512Mb, invalid after restarting.
   >
   >```sh
   >$ free -m
   >$ sudo dd if=/dev/zero of=/var/swap.img bs=1024k count=512
   >$ sudo mkswap /var/swap.img
   >$ sudo swapon /var/swap.img
   >$ free -m
   >```

2. Installing csm.

   > Reference: [how to: Building PCL & laser_scan_matcher on Raspberry pi 2](http://answers.ros.org/question/229788/how-to-building-pcl-laser_scan_matcher-on-raspberry-pi-2/)

   ```sh
   $ cd <your-catkin-workspace-path>/src
   # clone the dependency csm into catkin workspace's src
   $ git clone https://github.com/AndreaCensi/csm.git
   $ cd csm/
   $ ./install_quickstart.sh
   ```

3. laser_scan_matcher, clone and catkin_make.

   ```sh
   $ cd <your-catkin-workspace-path>/src
   # clone laser_scan_matcher into catkin workspace's src
   $ git clone https://github.com/Durant35/laser_scan_matcher.git
   # add csm path into your cmake's PKG_CONFIG_PATH
   $ export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:<your-catkin-workspace>/src/csm/sm/pkg-config
   # make targets
   $ cd ..
   $ catkin_make laser_scan_matcher
   ```

### Thanks
+ [csm](http://wiki.ros.org/csm)
> The C(anonical) Scan Matcher (CSM) is a pure C implementation of a very fast variation of ICP using a point-to-line metric optimized for range-finder scan matching.
> + Censi A. [An ICP variant using a point-to-line metric[C]](http://citeseerx.ist.psu.edu/viewdoc/download;jsessionid=18BFB87BCB86DF72E31CFBD702384421?doi=10.1.1.329.6781&rep=rep1&type=pdf)//Robotics and Automation, 2008. ICRA 2008. IEEE International Conference on. IEEE, 2008: 19-25.
> + ppt: https://censi.science/pub/research/2008-icra-plicp-slides.pdf
