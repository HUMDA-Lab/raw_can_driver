/*
 * File: raw_can_driver.cc
 * Description: RawCanDriver class implementation
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

#include "raw_can_driver.hh"

RawCanDriver::RawCanDriver(std::vector<std::string> interfaces_)
    : Node("raw_can_driver"), interfaces(interfaces_)
{
  // declare interfaces parameter
  this->declare_parameter<std::vector<std::string>>("interfaces", std::vector<std::string>{});
  this->get_parameter("interfaces", interfaces);

  // remove duplicates from interfaces
  std::sort(interfaces.begin(), interfaces.end());
  interfaces.erase(std::unique(interfaces.begin(), interfaces.end()), interfaces.end());

  for (size_t index = 0; index < interfaces.size(); ++index)
  {
    received_pubs.push_back(this->create_publisher<can_msgs::msg::Frame>(
        "/raw_can_driver/" + replace_special_chars(interfaces[index]) + "/received_messages", 10));
    sent_subs.push_back(this->create_subscription<can_msgs::msg::Frame>(
        "/raw_can_driver/" + replace_special_chars(interfaces[index]) + "/sent_messages", 10,
        [this, index](const can_msgs::msg::Frame::SharedPtr msg)
        {
          if (!asyncs[index]->IsOpened())
          {
            RCLCPP_ERROR(this->get_logger(), "Interface: %s is not open!",
                         interfaces[index].c_str());
            return;
          }

          can_frame can_frame = create_can_frame(msg);
          asyncs[index]->SendFrame(can_frame);
        }));

    asyncs.push_back(std::make_shared<AsyncCAN>(interfaces[index]));
    asyncs[index]->SetReceiveCallback(
        [this, index](can_frame* frame)
        {
          if (!asyncs[index]->IsOpened())
          {
            RCLCPP_ERROR(this->get_logger(), "Interface: %s is not open!",
                         interfaces[index].c_str());
            return;
          }

          can_msgs::msg::Frame ros_frame = create_ros_frame(frame);
          received_pubs[index]->publish(ros_frame);
        });

    try
    {
      asyncs[index]->StartListening();
      RCLCPP_INFO(this->get_logger(), "Started listening on %s", interfaces[index].c_str());
    }
    catch (const boost::system::system_error& e)
    {
      if (e.code())
      {
        RCLCPP_ERROR(this->get_logger(), "Error occurred on %s! Error code = %d. Message: %s",
                     interfaces[index].c_str(), e.code().value(), e.what());
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "node started.");
}

RawCanDriver::~RawCanDriver()
{
  for (std::shared_ptr<AsyncCAN> async : asyncs)
  {
    async->StopService();
  }
  RCLCPP_INFO(this->get_logger(), "node stopped.");
}

std::string RawCanDriver::replace_special_chars(std::string str)
{
  // must not contain characters other than alphanumerics, '_', '~', '{', or '}':
  // if there are any, replace them with '_'
  std::replace_if(
      str.begin(), str.end(),
      [](char c) { return !std::isalnum(c) && c != '_' && c != '~' && c != '{' && c != '}'; }, '_');
  return str;
}

unsigned RawCanDriver::create_mask(unsigned a, unsigned b)
{
  if (a > b)
  {
    throw std::invalid_argument("a must be less than or equal to b");
  }
  if (b - a + 1 > 32)
  {
    throw std::invalid_argument("b - a + 1 must be less than or equal to 32");
  }
  return ((1u << (b - a + 1)) - 1) << a;
}

can_msgs::msg::Frame RawCanDriver::create_ros_frame(const can_frame* can_frame)
{
  /*
   * Controller Area Network Identifier structure
   *
   * bit 0-28	: CAN identifier (11/29 bit)
   * bit 29	: error message frame flag (0 = data frame, 1 = error message)
   * bit 30	: remote transmission request flag (1 = rtr frame)
   * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
   */
  can_msgs::msg::Frame ros_frame{};

  ros_frame.header.stamp = this->get_clock()->now();
  ros_frame.id = can_frame->can_id & create_mask(0, 28);
  ros_frame.is_error = (can_frame->can_id & create_mask(29, 29)) >> 29;
  ros_frame.is_rtr = (can_frame->can_id & create_mask(30, 30)) >> 30;
  ros_frame.is_extended = (can_frame->can_id & create_mask(31, 31)) >> 31;
  ros_frame.dlc = can_frame->len;
  std::copy_n(std::begin(can_frame->data), 8, std::begin(ros_frame.data));
  return ros_frame;
}

can_frame RawCanDriver::create_can_frame(const can_msgs::msg::Frame::SharedPtr ros_frame)
{
  can_frame can_frame{};

  uint32_t id = ros_frame->id;
  uint32_t is_error = ros_frame->is_error << 29;
  uint32_t is_rtr = ros_frame->is_rtr << 30;
  uint32_t is_extended = ros_frame->is_extended << 31;
  can_frame.can_id = id | is_rtr | is_extended | is_error;
  can_frame.len = ros_frame->dlc;
  std::copy_n(std::begin(ros_frame->data), 8, std::begin(can_frame.data));
  return can_frame;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RawCanDriver>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
