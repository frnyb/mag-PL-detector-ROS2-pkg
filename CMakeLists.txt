cmake_minimum_required(VERSION 3.5)
project(mag_pl_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include/MagSampleFetcher_cpp/src)
link_directories(include/MagSampleFetcher_cpp/src)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mag_sample_fetcher
  src/mag_sample_fetcher.cpp
  include/MagSampleFetcher_cpp/src/mag_sample*.cpp 
  include/MagSampleFetcher_cpp/include/BRAM-uio-driver/src/bram_uio.cpp 
  include/MagSampleFetcher_cpp/include/PL-Mag-Sensor/ip/MagSampleFetcher/drivers/MagSampleFetcher_v1_0/src/*.c
)
ament_target_dependencies(mag_sample_fetcher
  rclcpp
)
install(TARGETS mag_sample_fetcher DESTINATION lib/${PROJECT_NAME})

ament_package()
