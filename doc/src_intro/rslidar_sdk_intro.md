# Analysis of rslidar_sdk v1.5.9 Source Code


## 1 Introduction

rslidar_sdk is a ROS/ROS2-based radar driver that relies on rs_driver to receive and parse MSOP/DIFOP Packets.


The basic functionalities of rslidar_sdk include:

+ Obtaining point clouds from LiDAR or PCAP files and publishing them through the ROS topic `/rslidar_points`. Users can subscribe to this topic to visualize point clouds in rviz.
+ Obtaining raw MSOP/DIFOP Packets from online radar and publishing them through the ROS topic `/rslidar_packets`. Users can subscribe to this topic to record Packets in a rosbag file.
+ Parsing MSOP/DIFOP Packets from the ROS topic `/rslidar_packets` to obtain point clouds and publishing them to the topic `/rslidar_points`.
  + The topic `/rslidar_packets` is published by users through rosbag file playback.



## 2 Source and Destination

As mentioned earlier, rslidar_sdk obtains MSOP/DIFOP Packets from three sources: online radar, PCAP files, and ROS topics. It publishes Packets to the ROS topic `/rslidar_packets` and point clouds to the target ROS topic /`rslidar_points`.

+ Source defines the source interface.
+ DestinationPointCloud defines the target interface for sending point clouds.
+ DestinationPacket defines the target interface for sending MSOP/DIFOP Packets.

![source](./img/class_source_destination.png)

### 2.1 DestinationPointCloud

DestinationPointCloud defines the interface for sending point clouds.
+ Virtual member function `init()` initializes the DestinationPointCloud instance.
+ irtual member function `start()` starts the instance.
+ Virtual member function `sendPointCloud()` sends PointCloud messages.

### 2.2 DestinationPacket

DestinationPacket defines the interface for sending MSOP/DIFOP Packets.
+ Virtual member function `init()` initializes the DestinationPacket instance.
+ Virtual member function `start()` starts the instance.
+ Virtual member function `sendPacket()` sends Packet messages.

### 2.3 Source

Source is the interface definition for the source.

+ Member `src_type_` is the type of the source.

  ```
  enum SourceType
  {
    MSG_FROM_LIDAR = 1,
    MSG_FROM_ROS_PACKET = 2,
    MSG_FROM_PCAP = 3,
  };
  ```

+ Member `pc_cb_vec_[]` is an array of DestinationPointCloud instances.
  + Member function `sendPointCloud()` calls instances in `point_cb_vec_[]` to send point cloud messages.

+ Member `pkt_cb_vec_[]` is an array of DestinationPacket instances.
  + Member function `sendPacket()` sends Packet messages to instances in `pkt_cb_vec_[]`.

+ Virtual member function `init()` initializes the Source instance.
+ Virtual member function `start()` starts the instance.
+ Virtual member function `regPointCloudCallback()` registers a PointCloudDestination instance in `point_cb_vec_[]`.
+ Virtual member function `regPacketCallback()` registers a PacketDestination instance in `packet_cb_vec_[]`.


### 2.4 DestinationPointCloudRos

DestinationPointCloudRos publishes point clouds on the ROS topic `/rslidar_points`.
+ Member `pkt_pub_` is the ROS topic publisher.
+ Member `frame_id_` stores the `frame_id`. The `frame_id` is the name of the coordinate system.

![destination pointcloud ros](./img/class_destination_pointcloud.png)

#### 2.4.1 DestinationPointCloudRos::init()

`init()` initializes the DestinationPointCloudRos instance.

+ Reads user configuration parameters from the YAML file.
  + Reads `frame_id` and stores it in the member `frame_id_` (default value is `rslidar`).
  + Reads ROS topic and stores it in the local variable `ros_send_topic_` (default value is `/rslidar_points`).
+ reates a ROS topic publisher, which is stored in the member `pkt_sub_`.

#### 2.4.2 DestinationPointCloudRos::sendPointCloud()

`sendPacket()` publishes point clouds on the ROS topic /rslidar_points.

+ Calls `Publisher::publish()` to publish ROS-formatted point cloud messages.

### 2.5 DestinationPacketRos

DestinationPacketRos publishes MSOP/DIFOP Packets on the ROS topic /`rslidar_packets`.

+ Member `pkt_sub_` is the ROS topic publisher.
+ Member `frame_id_` stores the `frame_id`. The `frame_id` is the name of the coordinate system.


![destination packet ros](./img/class_destination_packet.png)

#### 2.5.1 DestinationPacketRos::init()

`init()` initializes the DestinationPacketRos instance.

+ Reads user configuration parameters from the YAML file.
  + Reads `frame_id` and stores it in the member `frame_id_` (default value is `rslidar`).
  + Reads ROS topic and stores it in the local variable `ros_send_topic_` (default value is `/rslidar_packets`).
+ Creates a ROS topic publisher, which is stored in the member `pkt_sub_`.

#### 2.5.2 DestinationPacketRos::sendPacket()

`sendPacket()` publishes MOSP/DIFOP packets on the ROS topic `/rslidar_packets`.

+ Calls `Publisher::publish()` to publish ROS-formatted Packet messages.

### 2.6 SourceDriver

SourceDriver obtains MSOP/DIFOP Packets from online radar and PCAP files, and then parses them to obtain point clouds.

+ Member `driver_ptr_` is an instance of the rs_driver, i.e., LidarDriver.
+ Members `free_point_cloud_queue_` and `point_cloud_queue_` are queues for free and pending point clouds, respectively.
+ Member `point_cloud_handle_thread_` is the point cloud processing thread.

![source driver](./img/class_source_driver.png)

#### 2.6.1 SourceDriver::init()

`init()` initializes the SourceDriver instance.


+ Reads the user configuration parameters for radar from the YAML file.
+ reates an instance of the LidarDriver, i.e., `driver_ptr_`, based on the source type, which is specified in the constructor of SourceDriver.
  + `src_type_` is set in the SourceDriver constructor.
+ Calls `LidarDriver::regPointCloudCallback()` to register callback functions. Here, `getPointCloud()` and `putPointCloud()` are registered. The former provides free point clouds to `driver_ptr_`, and the latter retrieves filled point clouds from `driver_ptr_`.
  + Note: MSOP/DIFOP Packet callback is not registered here since Packets are obtained on-demand to avoid unnecessary CPU resource consumption.
+ Calls `LidarDriver::init()` to initialize `driver_ptr_`.
+ Creates and starts the point cloud processing thread `point_cloud_handle_thread_`, and the thread function is `processPointCloud()`.

#### 2.6.2 SourceDriver::getPointCloud()

`getPointCloud()` provides free point clouds to `driver_ptr_`.
+ Tries to get a point cloud from the `free_point_cloud_queue_` first.
+ If not available, allocates a new point cloud.

#### 2.6.3 SourceDriver::putPointCloud()

putPointCloud() provides filled point clouds obtained from `driver_ptr_`.

+ Pushes the obtained point cloud to the

#### 2.6.4 SourceDriver::processPointCloud()

`processPointCloud()` processes point clouds in a while loop:
+ Retrieves point clouds from the `point_cloud_queue_` (pending point clouds).
+ Calls `sendPointCloud()`, which invokes instances in `pc_cb_vec_[]` of DestinationPointCloud to send the point clouds.
+ Recycles the processed point clouds by placing them in the queue `free_cloud_queue_` for future use.


#### 2.6.5 SourceDriver::regPacketCallback()

`regPacketCallback()` is used to register DestinationPacket instances.

+ Calls `Source::regPacketCallback()` to add the DestinationPacket instance to the member `pkt_cb_vec_[]`.
+ If this is the first request for Packets (the first instance in `pkt_cb_vec_[]`), calls LidarDriver::regPacketCallback() to register the Packet callback function in `driver_ptr_` and start receiving Packets. The callback function is putPacket().
  
#### 2.6.6 SourceDriver::putPacket()

putPacket() calls sendPacket(), which invokes all instances in `pkt_cb_vec_[]` to send MSOP/DIFOP Packets.

### 2.7 SourcePacketRos

SourcePacketRos obtains MSOP/DIFOP Packets from the ROS topic `/rslidar_packets` and parses them to obtain point clouds.
+ SourcePacketRos is derived from SourceDriver instead of directly from Source because it uses SourceDriver to parse Packets into point clouds.
+ Member `pkt_sub_` is the ROS topic subscriber.

![source](./img/class_source_packet_ros.png)

#### 2.7.1 SourcePacketRos::init()

`init()` initializes the SourcePacketRos instance.

+ Calls `SourceDriver::init()` to initialize the member `driver_ptr_`.
  + In the constructor of SourcePacketRos, the source type is set to `SourceType::MSG_FROM_ROS_PACKET`. This way, in `SourceDriver::init()`, the `input_type` of `driver_ptr_` is set to `InputType::RAW_PACKET`, which receives Packets via `LidarDriver::feedPacket`.
+ Parses user configuration parameters for the radar from the YAML file.
  + Retrieves the topic for receiving Packets, with a default value of `/rslidar_packets`.
+ Creates a subscriber for the Packet topic, which is the member `pkt_sub_`, with the receiving function set to `putPacket()`.

#### 2.7.2 SourcePacketRos::putPacket()

`putPacket()` receives Packets and feeds them to `driver_ptr_` for parsing.
+ Calls `LidarDriver::decodePacket()` to feed the Packet to `driver_ptr_`.
+ The reception of point clouds uses the existing implementation of SourceDriver.



## 3 NodeManager

NodeManager manages Source instances, including creating, initializing, starting, and stopping Sources. It supports multiple sources, but these sources must have the same type.

+ Member `sources_[]` is an array of Source instances.

![node_manager](./img/class_node_manager.png)

### 3.1 NodeManager::init()

`init()` initializes the NodeManager instance.
+ Retrieves user configuration parameters from the `config.yaml` file.
  + Local variable `msg_source` is the data source type.
  + Local variable `send_point_cloud_ros` indicates whether to send point clouds on ROS topics.
  + Local variable `send_packet_ros` indicates whether to send MSOP/DIFOP Packets on ROS topics.

+ Iterates over data sources in the YAML file. In the loop:
  + Creates a Source instance based on `msg_source`.
    + If it is an online radar (`SourceType::MSG_FROM_LIDAR`), creates a SourceDriver instance and initializes it with a source type of `MSG_FROM_LIDAR`.
    + If it is a PCAP file (`SourceType::MSG_FROM_PCAP`), creates a SourceDriver instance and initializes it with a source type of `MSG_FROM_PCAP`.
    + If it is a ROS topic (`SourceType::MSG_FROM_ROS_PACKET`), creates a SourcePacketRos instance and initializes it. The constructor of SourcePacketRos has already set the source type to `MSG_FROM_ROS_PACKET`.
  + If sending point clouds on ROS topics (`send_point_cloud_ros` is true), creates a DestinationPointCloudRos instance, initializes it, and calls `Source::regPointCloudCallback()` to add it to the pc_cb_vec_[] of Source.
  + If sending Packets on ROS topics (`send_packet_ros` is `true`), creates a DestinationPacketRos instance, initializes it, and calls 
  + Adds the Source instance to the member `sources_[]`.
  
### 3.2 NodeManager::start()

`start()` starts all instances in the member `sources_[]`.



  



  



