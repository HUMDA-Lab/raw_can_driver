/*
 * File: raw_can_driver.hh
 * Description: RawCanDriver class declaration
 *
 * Copyright 2024 Humda Lab Nonprofit Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CAN_DRIVER_CAN_DRIVER_HH
#define CAN_DRIVER_CAN_DRIVER_HH

#include <algorithm>
#include <string>
#include <vector>

#include "async_port/async_can.hpp"
#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

class RawCanDriver : public rclcpp::Node
{
 public:
  RawCanDriver(std::vector<std::string> interfaces_ = {});
  ~RawCanDriver();

  inline std::string replace_special_chars(std::string str);
  inline unsigned create_mask(unsigned a, unsigned b);
  can_msgs::msg::Frame create_ros_frame(const can_frame* frame);
  can_frame create_can_frame(const can_msgs::msg::Frame::SharedPtr msg);

 private:
  std::vector<std::string> interfaces;
  std::vector<rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr> received_pubs;
  std::vector<rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr> sent_subs;
  std::vector<std::shared_ptr<AsyncCAN>> asyncs;
};

#endif  // CAN_DRIVER_CAN_DRIVER_HH