# Ros 2 JETTY handler
The main role of **the ROS 2 Jettty handler** is to:
1. Receive sensor data from the Arduino Mega via the JETTY transport layer on top of the serial link and public this data to the ROS network in form of topics
2. Receive motor commands from the ROS network and send to Arduino viq JETTY for low level control

## Work flow

The handler is implemented as a single RO2 node called **jetty**, its work flow is demonstrated in the following schema:

[[@book:image:/c_2/s_1/Jarvis_ros_base.png]]

Since the sensor data is continuously streamed from Arduino to ROS over JETTY, a single threading node is not fast enough to capture sensor data and to communicate with the ROS network at the same time. We need, therefore, a multi-threading solution.

The node consists of two principle threads. One thread - **the main thread**- handles the communication with the ROS network while the other one - **the reception thread** - is dedicated to sensor data reading from JETTY. 

The **reception thread** takes care of all tasks related to JETTY such as frame reading, decoding, processing and logging. There is no solution for queuing incoming sensor data, the thread keeps only the latest read data. Note that the raw IMU sensor data will processed by an IMU filter before being published. The detail of this filter is presented in sub section.

[[@book:image:/c_2/s_3/jarvis_imu-view.gif]]

The decoded sensor data from the **reception thread** will be published to the ROS network by the **main thread**, this thread publishes four topics:

* `/imu`  topic: publishes IMU sensor data processed by the IMU filter
* `/battery` topic: publishes the battery voltage
* `/left_tick` topic: publishes left odometry data
* `/right_tick` topic: publishes right odometry data

[[@book:image:/c_2/s_1/javis_base_node.png]]

In additional to the published topics, in order to receive commands from the ROS network, the node also subscribes to the two topics: `left_motor_pwm` and `right_motor_pwm`, as shown in the figure above. These topics allow to control the two motors speed and direction from other nodes. From each message received from these topics, the main thread creates the command frame, encodes it and sends to the Arduino via JETTY for low level control.

## Implementation

The implementation of JETTY handler on ROS 2 is a part of the `jarvis_core` package that can be found in `~/workspace/ros2/src/jarvis_core/jarvis_core/jettty.py`

Build the `jarvis_core` package with `colcon`:

```sh
cd ~/workspace/ros2
colcon build --packages-select jarvis_core
```

The node can be run via the following command:

```
. install/setup.bash
ros2 run jarvis_core jetty
```

TODO display data and control motor via the command line