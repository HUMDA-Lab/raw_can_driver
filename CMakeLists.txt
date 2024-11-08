cmake_minimum_required(VERSION 3.8)
project(raw_can_driver)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Disable compiler-specific extensions

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(lib)

add_library(can_lib lib/async_port/async_can.cpp)
target_link_libraries(can_lib ${Boost_LIBRARIES})
target_include_directories(can_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})

add_executable(raw_can_driver src/raw_can_driver.cc)
ament_target_dependencies(raw_can_driver 
  "rclcpp"
  "can_msgs"
)
target_include_directories(raw_can_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(raw_can_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(raw_can_driver can_lib)

install(TARGETS raw_can_driver
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclcpp can_msgs)

ament_package()
