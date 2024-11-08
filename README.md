# Raw Can Driver  [![Build](https://github.com/HUMDA-Lab/raw_can_driver/actions/workflows/build.yml/badge.svg)](https://github.com/HUMDA-Lab/humda_stack_workspace/actions/workflows/build.yml)

A ROS2 driver for receiving and sending raw CAN messages on any number of CAN interfaces. Provides a more robust and flexible alternative to the `socketcan_bridge` package with less CPU overhead and better error frame handling.

## Usage

The driver can be started with the following command:

```bash
ros2 run raw_can_driver raw_can_driver --params --ros-args -p interfaces:="[can0, can1]"
```

The `interfaces` parameter is a list of CAN interfaces that the driver should listen to. The driver will listen to all interfaces specified in the list and publish any received messages on the `/raw_can_driver/<interface_name>/received_messages` topic. The driver will also listen to messages on the `/raw_can_driver/<interface_name>/sent_messages` topic and send any received messages on the specified interface.

The topics are of type `can_msgs/msg/Frame`  and have the [following structure](https://github.com/ros-industrial/ros_canopen/blob/melodic-devel/can_msgs/msg/Frame.msg):

```json
{
  "header": {
    "stamp": {
      "sec": 0,
      "nanosec": 0
    },
    "frame_id": "",
  },
  "id": 0,              // CAN message id (11/29 bit)
  "is_rtr": false,      // Remote transmission request flag (1 = rtr frame)
  "is_extended": false, // Frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
  "is_error": false,    // Error flag bit (0 = data frame, 1 = error message)
  "dlc": 0,             // CAN data size in bytes
  "data": []            // CAN data
}
```

## Acknowledgements

This project makes use of the [async_port](https://github.com/westonrobot/async_port) library developed by [WestonRobot](https://www.westonrobot.com/). The library is licensed under the Apache License 2.0. All credit goes to the original authors. Any modifications made to the library are licensed under the same license.

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for more information.

Copyright © 2024 [Humda Lab Nonprofit Ltd.](https://humdalab.hu/en/)
