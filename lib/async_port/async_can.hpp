/*
 * File: async_can.hpp
 * Description: AsyncCAN class declaration
 * Note: CAN TX is not buffered and only the latest frame will be transmitted.
 *  Buffered transmission will need to be added if a message has to be divided
 *  into multiple frames and in applications where no frame should be dropped.
 *
 * Copyright 2020 Weston Robot Pte. Ltd.
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

#ifndef ASYNC_CAN_HPP
#define ASYNC_CAN_HPP

#include <linux/can.h>

#include <boost/asio/posix/basic_stream_descriptor.hpp>
#include <boost/system/error_code.hpp>
#include <memory>

#include "async_port/async_listener.hpp"

class AsyncCAN : public AsyncListener, public std::enable_shared_from_this<AsyncCAN>
{
 public:
  using ReceiveCallback = std::function<void(can_frame *rx_frame)>;

 public:
  AsyncCAN(std::string can_port = "can0");

  void StopService() override;

  void SetReceiveCallback(ReceiveCallback cb) { rcv_cb_ = cb; }
  void SendFrame(const struct can_frame &frame);

 private:
  int can_fd_;
  boost::asio::posix::basic_stream_descriptor<> socketcan_stream_;

  struct can_frame rcv_frame_;
  ReceiveCallback rcv_cb_ = nullptr;

  bool SetupPort();
  void DefaultReceiveCallback(can_frame *rx_frame);
  void ReadFromPort(struct can_frame &rec_frame,
                    boost::asio::posix::basic_stream_descriptor<> &stream);
};

#endif /* ASYNC_CAN_HPP */
