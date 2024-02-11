# 4 How to coexist with rslidar_sdk_node v1.3.x?

## 4.1 Problem Description

The configurations of `rslidar_sdk_node` v1.3.x and v1.5.x are different. Except for the two potential interaction scenarios described below, they can run independently without any interference.

1. `rslidar_sdk_node` publishes point clouds on the `/rslidar_points` topic, and the data is recorded to a cloud rosbag file using rosbag. Later, this rosbag file is played back and published to `/rslidar_points`, where `rslidar_sdk_node` subscribes and plays it.

2. `rslidar_sdk_node` publishes raw `MSOP/DIFOP Packets` on the `/rslidar_packets` topic, and the data is recorded to a packet rosbag file using rosbag. Later, this rosbag file is played back to `/rslidar_packets`, where `rslidar_sdk_node` subscribes and plays it.

In the first scenario, the point cloud format published by v1.3.x and v1.5.x is the same, so playing the rosbag recorded by v1.3.x on v1.5.x works seamlessly.

In the second scenario, v1.3.x publishes MSOP/DIFOP Packets separately on two topics `/rslidar_packets` and `/rslidar_packets_difop`, while v1.5.x publishes them on a single topic `/rslidar_packets`. Furthermore, the message definitions in v1.3.x and v1.5.x are different, so playing the packet rosbag recorded by v1.3.x on v1.5.x is not possible. ROS detects the mismatched MD5 Checksum for these two message formats and reports an error.

This document explains how to configure `rslidar_sdk` v1.5.x to allow it to simultaneously play packet rosbags recorded by both v1.3.x and v1.5.x in the second scenario.

## 4.2 Scenario Description

The scenario is as follows:
+ Two Lidars, `Lidar1` running `v1.3.x` and `Lidar2` running `v1.5.x`.
+ One host for analyzing data from `Lidar1` and `Lidar2`.

![Packet Rosbag](./img/04_01_packet_rosbag.png)

## 4.3 Steps

### 4.3.1 Configure v1.3.x Lidar

Record a packet rosbag using `v1.3.x rslidar_sdk_node`.

As per the default `config.yaml` configuration, messages are published to the `/rslidar_packets` and `/rslidar_packets_difop` topics.

```yaml
common:
  msg_source: 1
  send_packet_ros: true
  send_point_cloud_ros: true
lidar:
  - driver:
      lidar_type: RSM1
      msop_port: 6699
      difop_port: 7788
    ros:
      ros_send_packet_topic: /rslidar_packets
      ros_send_point_cloud_topic: /rslidar_points
```

### 4.3.2 Configure v1.5.x Lidar

Use `v1.5.6 rslidar_sdk_node` to record packet rosbag.

To differentiate from `v1.3.2` messages, output the messages to the `/rslidar_packets_v2` topic.

```yaml
common:
  msg_source: 1 
  send_packet_ros: true
  send_point_cloud_ros: true
lidar:
  - driver:
      lidar_type: RSM1
      msop_port: 6699
      difop_port: 7788
    ros:
      ros_send_packet_topic: /rslidar_packets_v2
      ros_send_point_cloud_topic: /rslidar_points
```


### 4.3.3 Configure v1.5.x Host

+ Open CMake compilation option `ENABLE_SOURCE_PACKET_LEGACY=ON` and compile `rslidar_sdk`.

```cmake
# CMakeLists.txt

option(ENABLE_SOURCE_PACKET_LEGACY "Enable ROS Source of MSOP/DIFOP Packet v1.3.x" ON)

```

+ In `config.yaml`, add a configuration `ros_recv_packet_legacy_topic: /rslidar_packets`. This way, `rslidar_sdk_node` will simultaneously subscribe to two topics:
  + Subscribe to `/rslidar_packets` and `/rslidar_packets_difop`, reading `v1.3.x` messages.
  + Subscribe to `/rslidar_packets_v2`, reading `v1.5.x` messages.

```yaml
common:
  msg_source: 1                                         #0: not use Lidar
                                                        #1: packet message comes from online Lidar
                                                        #2: packet message comes from ROS or ROS2
                                                        #3: packet message comes from Pcap file
  send_packet_ros: false                                #true: Send packets through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true                            #true: Send point cloud through ROS or ROS2
lidar:
  - driver:
      lidar_type: RSM1                                  #LiDAR type - RS16, RS32, RSBP, RSHELIOS, RS128, RS80, RS48, RSM1
      msop_port: 6699                                   #Msop port of lidar
      difop_port: 7788                                  #Difop port of lidar
    ros:
      ros_recv_packet_legacy_topic: /rslidar_packets    #Topic used to receive lidar packets from ROS
      ros_recv_packet_topic: /rslidar_packets_v2        #Topic used to receive lidar packets from ROS
      ros_send_point_cloud_topic: /rslidar_points       #Topic used to send point cloud through ROS
```