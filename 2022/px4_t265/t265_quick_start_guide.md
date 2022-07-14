<font color=#33ccff size=3> Release </font>

<font color=#FF0 size=6> 请控制组成员先阅读[Reference Links](t265_refer_links.md) 自行学习</font> 包括official_doc中内容

该文章的中文与部分英文为个人经验总结与提示，部分为资料原文摘抄引用，遇到无法解决的问题时方便快速定位。


SLAM: Simultaneous Localization and Mapping

VIO: Visual Inertial Odometry


---

In this tutorial

px4 firmware version:

px4_fmu-v5_default.px4 (1.12.3 stable)


---
# [QGC](http://qgroundcontrol.com/) Paramete Setting

请务必熟悉并掌握QGC地面站的使用，清楚下面各参数的意义与作用

CBRK_IO_SAFETY|22027
|-|-|
MAV_1_CONFIG | TELEM 2
MAV_1_MODE | Onboard
SER_TEL2_BAUD | 921600 8N1
EKF2_AID_MASK | 24
EKF2_HGT_MODE | Vision
MAV_ODOM_LP| 1
MPC_XY_VEL_MAX | 2.24mph
MPC_Z_VEL_MAX_DN | 2.24mph
MPC_Z_VEL_MAX_UP | 2.24mph

通过地面站参数设置配置Pixhawk上的Telem2作为MAVLINK端口时，如果一开始找不到MAV_1_MODE和SER_TEL2_BAUD，就先设置MAV_1_CONFIG再reboot飞控。

SER_TEL2_BAUD波特率设置会受到usb转ttl芯片的性能和接线长度影响，导致mavros与飞控传输失败，这个时候可以先把921600降低验证配置是否正确，后续再与电路沟通。

>PID Setting：
>-
>- in Paramete Setting :
>Multicopter Attitude Control
Multicopter Position Control
Multicopter Rate Control
>
>OR you can use
>-
>PID Tuning

---

# PX4 Firmware

在飞控的sd卡的根目录下创建/etc/extras.txt，写入
```c
mavlink stream -d /dev/ttyUSB0 -s ATTITUDE_QUATERNION -r 200
mavlink stream -d /dev/ttyUSB0 -s HIGHRES_IMU -r 200
```
可以提高飞控的响应速度，之后记得映射为物理地址




---
机载电脑Ubuntu20.04 server的安装

    分区设置：
        EFI系统分区（主分区）512M
        交换空间（逻辑分区）8192M（根据实际内存大小的两倍即可）
        挂载点/（主分区）剩余所有容量


机载电脑安装 [Ros](https://blog.csdn.net/qq_46037020/article/details/122608460) Noetic base

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# [Librealsense](https://github.com/IntelRealSense/librealsense)
[Realsense SDK2.0 Linux Distribution](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
```bash
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

//sudo apt-get install librealsense2-dkms && sudo apt-get install librealsense2-utils && sudo apt-get install librealsense2-dev && sudo apt-get install librealsense2-dbg

//test
realsense-viewer	//using USB 3.0
```

---

# [Realsense-Ros](https://github.com/IntelRealSense/realsense-ros)
ROS Wrapper for Intel® RealSense™ Devices
These are packages for using Intel RealSense cameras (D400 series SR300 camera and T265 Tracking Module) with ROS.

## 2 sources to install realsense2_camera

<font color=#bb5 size=3> ***If you have installed Realsense SDK, Method 1 is OK.***</font>

### Method 1: The ROS distribution:

realsense2_camera is available as a debian package of ROS distribution. It can be installed by typing:
```bash
sudo apt-get install ros-noetic-realsense2-camera
```

This will install both realsense2_camera and its dependents, including librealsense2 library and matching udev-rules.

Notice:

>- The version of librealsense2 is almost always behind the one availeable in RealSense™ official repository.
>- librealsense2 is not built to use native v4l2 driver but the less stable RS-USB protocol. That is because the last is more general and operational on a larger variety of platforms.
>- realsense2_description is available as a separate debian package of ROS distribution. It includes the 3D-models of the devices and is necessary for running launch files that include these models (i.e. rs_d435_camera_with_model.launch). It can be installed by typing:
>   -     sudo apt-get install ros-$ROS_DISTRO-realsense2-description



### Method 2: The RealSense™ distribution:
>This option is demonstrated in the .travis.yml file. It basically summerize the elaborate instructions in the following 2 steps:

#### Step 1: Install the latest Intel® RealSense™ SDK 2.0
Install librealsense2 debian package:

>- Jetson users - use the [Jetson Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
>
>- Otherwise, install from [Linux Debian Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
>   - In that case treat yourself as a developer. Make sure you follow the instructions to also install librealsense2-dev and librealsense2-dkms packages.

OR
Build [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0) from [sources](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)


#### Step 2. Install Intel® RealSense™ ROS from Source
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
```
Clone the latest Intel® RealSense™ ROS from [here](https://github.com/intel-ros/realsense/releases) into 'catkin_ws/src/'
```bash
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
```

>- Make sure all dependent packages are installed. You can check .travis.yml file for reference.
>- Specifically, make sure that the ros package ddynamic_reconfigure is installed. If ddynamic_reconfigure cannot be installed using APT or if you are using Windows you may clone it into your workspace 'catkin_ws/src/' from [here](https://github.com/pal-robotics/ddynamic_reconfigure/tree/kinetic-devel)

```bash
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
```

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Using T265
### Start the camera node

To start the camera node in ROS:
```bash
roslaunch realsense2_camera rs_t265.launch
```
This will stream all camera sensors and publish on the appropriate ROS topics.

The T265 sets its usb unique ID during initialization and without this parameter it wont be found. Once running it will publish, among others, the following topics:

    /camera/odom/sample
    /camera/accel/sample
    /camera/gyro/sample
    /camera/fisheye1/image_raw
    /camera/fisheye2/image_raw

To visualize the pose output and frames in RViz, start:
```bash
roslaunch realsense2_camera demo_t265.launch
```
About Frame ID

The wrapper publishes static transformations(TFs). The Frame Ids are divided into 3 groups:

> - ROS convention frames: follow the format of <tf_prefix>_<_stream>"_frame" for example: camera_depth_frame, camera_infra1_frame, etc.
> - Original frame coordinate system: with the suffix of <_optical_frame>. For example: camera_infra1_optical_frame. Check the device documentation for specific coordinate system for each stream.
> - base_link: For example: camera_link. A reference frame for the device. In D400 series and SR300 it is the depth frame. In T265, the pose frame.

### realsense2_description package:

For viewing included models, a separate package is included. For example:
```bash
roslaunch realsense2_description view_d415_model.launch
```
### Unit tests:

Unit-tests are based on bag files saved on S3 server. These can be downloaded using the following commands:
```bash
cd catkin_ws
wget "https://librealsense.intel.com/rs-tests/TestData/outdoors.bag" -P "records/"
wget "https://librealsense.intel.com/rs-tests/D435i_Depth_and_IMU_Stands_still.bag" -P "records/"
```
Then, unit-tests can be run using the following command (use either python or python3):
```bash
python src/realsense/realsense2_camera/scripts/rs2_test.py --all
```

 ---
# [MAVROS](https://github.com/mavlink/mavros/)
>mavros is a ROS (1) package that enables MAVLink extendable communication between computers running ROS (1) for any MAVLink enabled autopilot, ground station, or peripheral. MAVROS is the "official" supported bridge between ROS (1) and the MAVLink protocol.

[mavros (noetic) - 1.13.0-1](https://github.com/mavlink/mavros-release)

[official installation guide](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)

```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
```

[.](https://zhuanlan.zhihu.com/p/364390798)[.](https://github.com/mavlink/mavros/tree/master/mavros#installation)[.](https://docs.px4.io/master/en/ros/external_position_estimation.html)[.](https://dev.px4.io/v1.11_noredirect/en/companion_computer/pixhawk_companion.html)[.](https://docs.px4.io/master/en/peripherals/mavlink_peripherals.html)
[ROS 2](https://docs.px4.io/master/en/ros/ros2.html)


```bash
sudo chmod 777 /dev/ttyUSB3
sudo nano /opt/ros/noetic/share/mavros/launch/px4.launch

//*****************
/CHANGE/

<arg name="fcu_url" default="/dev/ttyACM0:57600" />

/TO/

<arg name="fcu_url" default="/dev/ttyUSB3:921600" />
*****************//
```

---
# [VIO](https://github.com/Auterion/VIO)或[Vision_to_mavros](https://github.com/thien94/vision_to_mavros)
用于坐标转换发送节点信息，分别是 PX4官方 / APM官方，任选
[](https://zhuanlan.zhihu.com/p/367357019)

采用VIO包比较稳定
### [VIO](https://docs.px4.io/master/en/computer_vision/visual_inertial_odometry.html)

[catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
```bash
sudo apt install python3-catkin-tools
```

```bash
//编译VIO时会提示缺失这个ROS包，安装后会导致出现桌面，记得确认系统的休眠是否关闭
sudo apt install ros-noetic-pcl-ros
```


remember use rm_ws
```bash
mkdir -p ~/rm_ws/src
cd ~/rm_ws/src
git clone https://github.com/Auterion/VIO.git
cd ~/rm_ws
catkin build px4_realsense_bridge
source ~/rm_ws/devel/setup.bash
roslaunch px4_realsense_bridge bridge_mavros.launch
```

```bash
nano ~/rm_ws/src/VIO/launch/bridge_mavros.launch

/*****************Change default
  <arg name="fcu_url" default="udp://:14540@localhost:14557"/>
******************/

/**To
/dev/ttyUSB0:921600
**/
```


### [Vision_to_mavros](https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html)
在此仅做拓展
```bash
cd ~/catkin_ws/src
git clone https://github.com/hoangthien94/vision_to_mavros.git
/*************local to server
scp -r ./vision_to_mavros/ dynamicx@10.42.0.151:~/catkin_ws/src/vision_to_mavros
*************/
cd ..
catkin build vision_to_mavros
source ~/.bashrc
```
```bash
nano ~/catkin_ws/src/vision_to_mavros/launch/t265_all_nodes.launch

/********change apm to px4
<include file="$(find mavros)/launch/px4.launch"/>
*********/


//cam_pose
nano ~/catkin_ws/src/vision_to_mavros/launch/t265_tf_to_mavros.launch
/**这个文件里面关于坐标转换已经很清晰了，在此不赘述**/

```


### After all
```bash
echo "source ~/rm_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



---

# Camera Pose

![](../img/T265_sensor_extrinsics.png)

**Here's for VIO.**

Configure the camera orientation if needed:

The VIO bridge doesn't require any configuration if the camera is mounted with the lenses facing down (the default).
For any other orientation modify [bridge.launch](https://github.com/Auterion/VIO/blob/master/launch/bridge.launch) in the section below:
```c
<node pkg="tf" type="static_transform_publisher" name="tf_baseLink_cameraPose"
	args="0 0 0 0 1.5708 0 base_link camera_pose_frame 1000"/>
```
This is a static transform that links the camera ROS frame camera_pose_frame to the mavros drone frame base_link.

>the first three args specify translation x,y,z in metres from the center of flight controller to camera. For example, if the camera is 10cm in front of the controller and 4cm up, the first three numbers would be : [0.1, 0, 0.04,...]
>the next three args specify rotation in radians (yaw, pitch, roll). So [... 0, 1.5708, 0] means pitch down by 90deg (facing the ground). Facing straight forward would be [... 0 0 0].


```c
nano ~/catkin_ws/src/VIO/launch/bridge.launch

0 0 0 0 0 0			//Facing straight forward
0 0 0 0 0 3.1416	//
0.1 0 -0.12 0 0.7853981 0	//45 degree down

0.370 0 -0.030 0 0.7853981 0	//DX
```

```bash
rostopic echo /tf

/***********vision_to_mavros
rostopic echo /mavros/vison_pose/pose
***********/
```
check odometer pose
[3D Rotation Converter](https://www.andre-gaschler.com/rotationconverter/)

>注意：
>
>camera_pose_frame 1000
>camera_odom_frame 1000
>
>即1000ms发布一次，建议改为500

---

# Permanent USB authority

直接给用户所有usb的权限
```bash
whoami	//check username
sudo usermod -aG dialout <username> 

//dialout用户组: Full and direct access to serial ports. Members of this group can reconfigure the modem, dial anywhere, etc. 

//Logout and Login
```

[UDEV](https://docs.px4.io/master/en/companion_computer/pixhawk_companion.html)<font color=#FF0 size=4><--------MUST DO IT </font>




---

# Check Everything
在roslaunch之前先把相应的包单独跑一下，终端显示与下图无明显出入，则配置没问题
```bash
//经典小海龟
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
```

```bash
//t265启动是否正常
roslaunch realsense2_camera demo_t265.launch
```

```bash
//机载电脑与飞控通信
roslaunch mavros px4.launch
```
![](img/mavros.jpg)
在飞控参数设置正确的情况下，此时已经可以将模式切换至position，断开此节点地面站会提示
>Connection to mission computer lost


# Now Enjoy
```bash
roslaunch px4_realsense_bridge bridge_mavros.launch
```
![](img/roslaunch.jpg)
