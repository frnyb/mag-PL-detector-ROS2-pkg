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

include_directories(include/SlidingWindowMagSampleFetcher_cpp/src)
link_directories(include/SlidingWindowMagSampleFetcher_cpp/src)

include_directories(include/cppopt/include)

include_directories(include/BRAM-uio-driver/src)
link_directories(include/BRAM-uio-driver/src)

include_directories(include/PL-Mag-Sensor/ip/CPU_GainController/drivers/ControlGain_v1_0/src)
link_directories(include/PL-Mag-Sensor/ip/CPU_GainController/drivers/ControlGain_v1_0/src)

include_directories(include/PL-Mag-Sensor/ip/SampleCntTargetController/drivers/SampleCntTargetController_v1_0/src)
link_directories(include/PL-Mag-Sensor/ip/SampleCntTargetController/drivers/SampleCntTargetController_v1_0/src)

include_directories("/usr/local/include")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MagMeasurement.msg"
  "msg/MagMeasurements.msg"
  "msg/SineReconstruction.msg"
  "msg/MagneticPhasor3D.msg"
  "msg/MagneticPhasors3D.msg"
  "msg/PowerlinePosesComputationResult.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(mag_sample_publisher
  src/mag_sample_publisher.cpp
  include/SlidingWindowMagSampleFetcher_cpp/src/sliding_window_mag_sample_fetcher.cpp 
  include/SlidingWindowMagSampleFetcher_cpp/src/mag_sample.cpp 
  include/SlidingWindowMagSampleFetcher_cpp/include/BRAM-uio-driver/src/bram_uio.cpp 
  include/SlidingWindowMagSampleFetcher_cpp/include/PL-Mag-Sensor/ip/SlidingWindowMagSampleFetcher/drivers/SlidingWindowMagSampleFetcher_v1_0/src/xslidingwindowmagsamplefetcher.c
  include/SlidingWindowMagSampleFetcher_cpp/include/PL-Mag-Sensor/ip/SlidingWindowMagSampleFetcher/drivers/SlidingWindowMagSampleFetcher_v1_0/src/xslidingwindowmagsamplefetcher_linux.c
)
ament_target_dependencies(mag_sample_publisher
  rclcpp
)
rosidl_target_interfaces(mag_sample_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS mag_sample_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(mag_sample_file_publisher
  src/mag_sample_file_publisher.cpp
)
ament_target_dependencies(mag_sample_file_publisher
  rclcpp
)
rosidl_target_interfaces(mag_sample_file_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS mag_sample_file_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(sine_reconstruction_publisher
  src/sine_reconstruction_publisher.cpp
  include/BRAM-uio-driver/src/bram_uio.cpp 
  src/geometry.cpp
)
ament_target_dependencies(sine_reconstruction_publisher
  rclcpp
  Eigen3
  geometry_msgs
  tf2
  tf2_ros
)
rosidl_target_interfaces(sine_reconstruction_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS sine_reconstruction_publisher DESTINATION lib/${PROJECT_NAME})

#add_executable(sample_frequency_controller
#  src/sample_frequency_controller.cpp
#  include/PL-Mag-Sensor/ip/SampleCntTargetController/drivers/SampleCntTargetController_v1_0/src/xsamplecnttargetcontroller.c
#  include/PL-Mag-Sensor/ip/SampleCntTargetController/drivers/SampleCntTargetController_v1_0/src/xsamplecnttargetcontroller_linux.c
#)
#ament_target_dependencies(sample_frequency_controller
#  rclcpp
#  std_msgs
#)
#rosidl_target_interfaces(sample_frequency_controller
#  ${PROJECT_NAME} "rosidl_typesupport_cpp"
#)
#install(TARGETS sample_frequency_controller DESTINATION lib/${PROJECT_NAME})

#add_executable(gain_controller
#  src/gain_controller.cpp
#  include/PL-Mag-Sensor/ip/CPU_GainController/drivers/ControlGain_v1_0/src/xcontrolgain.c
#  include/PL-Mag-Sensor/ip/CPU_GainController/drivers/ControlGain_v1_0/src/xcontrolgain_linux.c
#)
#ament_target_dependencies(gain_controller
#  rclcpp
#  std_msgs
#)
#rosidl_target_interfaces(gain_controller
#  ${PROJECT_NAME} "rosidl_typesupport_cpp"
#)
#install(TARGETS gain_controller DESTINATION lib/${PROJECT_NAME})

#add_executable(gain_computer
#  src/gain_computer.cpp
#)
#ament_target_dependencies(gain_computer
#  rclcpp
#  std_msgs
#)
#rosidl_target_interfaces(gain_computer
#  ${PROJECT_NAME} "rosidl_typesupport_cpp"
#)
#install(TARGETS gain_computer DESTINATION lib/${PROJECT_NAME})

add_executable(sine_reconstructor
  src/sine_reconstructor.cpp
  src/mag_measurements_class.cpp
  src/geometry.cpp
)
ament_target_dependencies(sine_reconstructor
  rclcpp
  Eigen3
  geometry_msgs
  tf2
  tf2_ros
)
rosidl_target_interfaces(sine_reconstructor
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS sine_reconstructor DESTINATION lib/${PROJECT_NAME})

add_executable(pl_dir_computer
  src/pl_dir_computer.cpp
  src/geometry.cpp
)
ament_target_dependencies(pl_dir_computer
  rclcpp
  Eigen3
  geometry_msgs
  tf2
  tf2_ros
)
rosidl_target_interfaces(pl_dir_computer
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS pl_dir_computer DESTINATION lib/${PROJECT_NAME})

add_executable(pl_dir_estimator
  src/pl_dir_estimator.cpp
  src/geometry.cpp
)
ament_target_dependencies(pl_dir_estimator
  rclcpp
  Eigen3
  geometry_msgs
  tf2
  tf2_ros
)
rosidl_target_interfaces(pl_dir_estimator
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS pl_dir_estimator DESTINATION lib/${PROJECT_NAME})

add_executable(pl_positions_computer
  src/pl_positions_computer.cpp
  src/geometry.cpp
)
ament_target_dependencies(pl_positions_computer
  rclcpp
  Eigen3
  geometry_msgs
  tf2
  tf2_ros
)
#target_link_libraries(pl_positions_computer nlopt)
rosidl_target_interfaces(pl_positions_computer
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS pl_positions_computer DESTINATION lib/${PROJECT_NAME})

add_executable(pl_positions_estimator
  src/pl_positions_estimator.cpp
  src/geometry.cpp
)
ament_target_dependencies(pl_positions_estimator
  rclcpp
  Eigen3
  geometry_msgs
  tf2
  tf2_ros
)
rosidl_target_interfaces(pl_positions_estimator
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
install(TARGETS pl_positions_estimator DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/save_mag_measurements.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
