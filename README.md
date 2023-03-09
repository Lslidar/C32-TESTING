### LSLIDAR_CX_V4.1.0_221227_ROS2驱动说明

## 1.工程介绍

​		LSLIDAR_CX_V4.1.0_221227_ROS2为linux环境下雷达ros2驱动，适用于镭神c1/c8/c16/c32(32°，70°，90°)    v4.0版本的雷达 ，程序在ubuntu18.04 ros dashing , ubuntu18.04 ros eloquent ,ubuntu 20.04 ros foxy,ubuntu 20.04 ros galactic以及ubuntu22.04 ros humble下测试通过。

## 2.依赖

1.ubuntu18.04 ros dashing/ubuntu18.04 ros eloquent/ubuntu 20.04 ros foxy/ubuntu 20.04 ros galactic/ubuntu22.04 ros humble

2.ros依赖

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.其他依赖

pcap,boost

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #选择适合的版本
~~~

## 3.编译与运行：

~~~shell
mkdir -p ~/lslidar_ws/src
cd ~/lslidar_ws/src
把驱动压缩包拷贝到src目录下，并解压
cd ~/lslidar_ws
colcon build
source install/setup.bash
#启动单个c1雷达
ros2 launch lslidar_driver lslidar_c1_launch.py
#启动两个c1雷达
ros2 launch lslidar_driver lslidar_c1_double_launch.py

#启动单个c8雷达
ros2 launch lslidar_driver lslidar_c8_launch.py
#启动两个c8雷达
ros2 launch lslidar_driver lslidar_c8_double_launch.py

#启动单个c16雷达
ros2 launch lslidar_driver lslidar_c16_launch.py
#启动两个c18雷达
ros2 launch lslidar_driver lslidar_c16_double_launch.py

#启动单个c32雷达
ros2 launch lslidar_driver lslidar_c32_launch.py
#启动两个c32雷达
ros2 launch lslidar_driver lslidar_c32_double_launch.py
~~~



## 4.launch 文件参数说明：

~~~shell
/c32/lslidar_driver_node:
  ros__parameters:
    packet_rate: 1695.0             #播放pcap时，每秒钟播放的包数
    device_ip: 192.168.1.200        #雷达ip
    msop_port: 2368                 #数据包目的端口
    difop_port: 2369                #设备包目的端口
    pcl_type: false                 #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
    lidar_type: c32                 # 雷达类型  c1/c8/c16/c32
    c32_type: c32_32                #c32_32: 垂直角度是的30度c32   c32_70: 垂直角度是的70度c32(c32w)  c32_90: 垂直角度是的90度c32(ch32r)
    add_multicast: false            #是否开启组播模式
    group_ip: 224.1.1.2             #组播ip地址
    use_gps_ts: true                #雷达是否使用gps或ptp授时，使用改为true
    min_range: 0.3                  #单位，米。雷达盲区最小值，小于此值的点被过滤
    max_range: 200.0                #单位，米。雷达盲区最大值 ，大于此值的点被过滤
    frame_id: laser_link            #坐标系id
    distance_unit: 0.4              #雷达距离分辨率
    angle_disable_min: 0            #雷达裁剪角度开始值 ，单位0.01°
    angle_disable_max: 0            #雷达裁剪角度结束值，单位0.01°
    horizontal_angle_resolution: 0.2     #10Hz:0.2  20Hz:0.4 5Hz: 0.1
    scan_num: 16                            #laserscan线号
    topic_name: lslidar_point_cloud         #点云话题名称，可修改
    publish_scan: false                     #是否发布scan
    pcl_type: false                         #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
    coordinate_opt: false                   #默认false  雷达零度角对应点云方向
    #pcap: /home/ls/work/fuhong/data/V4.0/c32w.pcap                        #pcap包路径，加载pcap包时打开此注释
~~~

### 组播模式：

- 上位机设置雷达开启组播模式

- 修改launch文件以下参数

  ~~~shell
  add_multicast: true                    #是否开启组播模式。
  group_ip: 224.1.1.2                     #组播ip地址
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### 离线pcap模式：

- 把录制好的pcap文件，拷贝到cx_4.0_ws/src/lslidar_ros/lslidar_driver/pcap文件夹下。（cx_4.0_ws是ros工作空间,根据实际工作空间修改）

- 修改launch文件以下参数

  ~~~shell
  #取消注释
  pcap: /home/chris/Documents/leishen/1212bytes_c32/gps.pcap                        #pcap包路径，加载pcap包时打开此注释
  ~~~



###  pcl点云类型：

- 修改launch文件以下参数

  ~~~shell
  pcl_type: false                         #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
  ~~~

  

- 默认false为自定义点云类型，定义参考lslidar_driver/include/lslidar_driver.h头文件

- 改为true,为pcl自带类型 :

  ~~~shell
  pcl::PointCloud<pcl::PointXYZI>
  ~~~




### 修改雷达授时方式：

source install/setup.bash

GPS授时：

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'gps',ntp_ip: ''}"   #说明：xx为命名空间，例如c1/c8/c16/c32
~~~

PTP授时：

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'ptp',ntp_ip: ''}"  #说明：xx为命名空间，例如c1/c8/c16/c32   
~~~

NTP授时：

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'ntp',ntp_ip: '192.168.1.102'}"   #说明：xx为命名空间，例如c1/c8/c16/c32
~~~



### 雷达上下电(雷达依然转动，只发设备包，不发送数据包)：

source install/setup.bash

上电：

~~~bash
ros2 service call /xx/lslidar_control lslidar_msgs/srv/LslidarControl "{laser_control: 1}"   #说明：xx为命名空间，例如c1/c8/c16/c32
~~~

下电：

~~~bash
ros2 service call /xx/lslidar_control lslidar_msgs/srv/LslidarControl "{laser_control: 0}"   #说明：xx为命名空间，例如c1/c8/c16/c32
~~~



### 雷达转动/停止转动(电机停转)：

source install/setup.bash

转动：

~~~bash
ros2 service call /xx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: 1}"   #说明：xx为命名空间，例如c1/c8/c16/c32  
~~~

停止转动：

~~~bash
ros2 service call /xx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: 0}"   #说明：xx为命名空间，例如c1/c8/c16/c32
~~~



### 设置雷达转速：

source install/setup.bash

可选频率  5Hz/10Hz/20Hz

~~~bash
ros2 service call /xx/set_motor_speed lslidar_msgs/srv/MotorSpeed "{motor_speed: 20}"   #说明：xx为命名空间，例如c1/c8/c16/c32
~~~



### 设置雷达数据包端口

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_port lslidar_msgs/srv/DataPort "{data_port: 2368}"  #范围[1025,65535]   #说明：xx为命名空间，例如c1/c8/c16/c32
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



### 设置雷达设备包端口

source install/setup.bash

~~~bash
ros2 service call /xx/set_dev_port lslidar_msgs/srv/DevPort "{dev_port: 2369}"   #范围[1025,65535]   #说明：xx为命名空间，例如c1/c8/c16/c32
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



### 设置雷达ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_ip lslidar_msgs/srv/DataIp "{data_ip: "192.168.1.200"}"   #说明：xx为命名空间，例如c1/c8/c16/c32
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



### 设置雷达目的ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_destination_ip lslidar_msgs/srv/DestinationIp "{destination_ip: "192.168.1.102"}"   #说明：xx为命名空间，例如c1/c8/c16/c32
~~~

**备注：设置完以后，需要修改launch文件参数，然后重启驱动程序。**



## FAQ

Bug Report

Original version : LSLIDAR_CX_V4.0.0_221031_ROS2

Modify:  original version

Date    : 2022-10-31

-----------------------

Original version : LSLIDAR_CX_V4.1.0_221227_ROS2

Modify:  1.scan话题新增强度信息

2.fpga升级，C32 90度修改计算公式

3.ROS驱动新增修改授时方式的功能

4.新增雷达上下电,修改雷达ip，端口，转速等功能。

5.新增对ros2 dashing、ros2 eloquent、ros2 humble的支持

6.修复ntp授时解析问题。

Date    : 2022-12-27

