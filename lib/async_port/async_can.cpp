/*
 * File: async_can.cpp
 * Description: AsyncCAN class implementation
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

#include "async_port/async_can.hpp"

#include <linux/can.h>
#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>

AsyncCAN::AsyncCAN(std::string can_port) : AsyncListener(can_port), socketcan_stream_(io_context_)
{
}

bool AsyncCAN::SetupPort()
{
  try
  {
    const size_t iface_name_size = strlen(port_.c_str()) + 1;
    if (iface_name_size > IFNAMSIZ) return false;

    can_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_fd_ < 0) return false;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    memcpy(ifr.ifr_name, port_.c_str(), iface_name_size);

    const int ioctl_result = ioctl(can_fd_, SIOCGIFINDEX, &ifr);
    if (ioctl_result < 0) StopService();

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    const int bind_result = bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr));
    if (bind_result < 0) StopService();

    port_opened_ = true;
  }
  catch (std::system_error &e)
  {
    std::cout << e.what();
    return false;
  }

  // give some work to io_service to start async io chain
  socketcan_stream_.assign(can_fd_);
  boost::asio::post(io_context_, std::bind(&AsyncCAN::ReadFromPort, this, std::ref(rcv_frame_),
                                           std::ref(socketcan_stream_)));
  return true;
}

void AsyncCAN::StopService()
{
  // release port fd
  [[maybe_unused]] const int close_result = ::close(can_fd_);
  can_fd_ = -1;

  // stop io thread
  io_context_.stop();
  if (io_thread_.joinable()) io_thread_.join();
  io_context_.reset();

  port_opened_ = false;
}

void AsyncCAN::DefaultReceiveCallback(can_frame *rx_frame)
{
  std::cout << std::hex << rx_frame->can_id << "  ";
  for (int i = 0; i < rx_frame->can_dlc; i++)
    std::cout << std::hex << int(rx_frame->data[i]) << " ";
  std::cout << std::dec << std::endl;
}

void AsyncCAN::ReadFromPort(struct can_frame &rec_frame,
                            boost::asio::posix::basic_stream_descriptor<> &stream)
{
  auto sthis = shared_from_this();
  stream.async_read_some(
      boost::asio::buffer(&rec_frame, sizeof(rec_frame)),
      [sthis](boost::system::error_code error, [[maybe_unused]] size_t bytes_transferred)
      {
        if (error)
        {
          sthis->StopService();
          return;
        }

        if (sthis->rcv_cb_ != nullptr)
          sthis->rcv_cb_(&sthis->rcv_frame_);
        else
          sthis->DefaultReceiveCallback(&sthis->rcv_frame_);

        sthis->ReadFromPort(std::ref(sthis->rcv_frame_), std::ref(sthis->socketcan_stream_));
      });
}

void AsyncCAN::SendFrame(const struct can_frame &frame)
{
  socketcan_stream_.async_write_some(
      boost::asio::buffer(&frame, sizeof(frame)),
      [](boost::system::error_code error, [[maybe_unused]] size_t bytes_transferred)
      {
        if (error)
        {
          std::cerr << "Failed to send CAN frame" << std::endl;
        }
        // std::cout << "frame sent" << std::endl;
      });
}