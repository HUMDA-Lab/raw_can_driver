/*
 * File: async_listener.hpp
 * Description: AsyncListener class declaration
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

#ifndef ASYNC_LISTENER_HPP
#define ASYNC_LISTENER_HPP

#include <boost/asio.hpp>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

class AsyncListener
{
 public:
  AsyncListener(std::string port) : port_(port) {}
  virtual ~AsyncListener() {};

  // do not allow copy
  AsyncListener() = delete;
  AsyncListener(const AsyncListener& other) = delete;

  virtual bool IsOpened() const { return port_opened_; }

  bool StartListening()
  {
    if (SetupPort())
    {
      io_thread_ = std::thread([this]() { io_context_.run(); });
      return true;
    }
    std::cerr << "Failed to setup port, please check if specified port exits "
                 "or if you have proper permissions to access it"
              << std::endl;
    return false;
  };

  virtual void StopService() {}

 protected:
  std::string port_;
  bool port_opened_ = false;

  boost::asio::io_context io_context_;
  std::thread io_thread_;

  virtual bool SetupPort() = 0;
};

#endif /* ASYNC_LISTENER_HPP */
