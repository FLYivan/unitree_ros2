cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

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


include_directories(include include/common)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  unitree_ros2
  rclcpp
  std_msgs
  rosbag2_cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(h1_2_low_level_ctrl src/h1-2/lowlevel/low_level_ctrl_hg.cpp src/common/motor_crc_hg.cpp)
add_executable(g1_low_level_example src/g1/lowlevel/g1_low_level_example.cpp src/common/motor_crc_hg.cpp)
add_executable(g1_arm7_control src/g1/arm7_control/g1_arm7_control.cpp)
add_executable(g1_ankle_swing src/g1/ankle_swing/g1_ankle_swing.cpp src/common/motor_crc_hg.cpp)
add_executable(g1_loco_client_example src/g1/lococlient/g1_loco_client_example.cpp)
add_executable(g1_audio_client_example src/g1/audioclient/g1_audio_client_example.cpp)
add_executable(g1_read_low_state src/g1/lowstate/read_low_state_hg.cpp)
add_executable(go2_low_level_ctrl src/go2/low_level_ctrl.cpp src/common/motor_crc.cpp)
add_executable(go2_read_low_state src/go2/read_low_state.cpp)
add_executable(go2_read_motion_state src/go2/read_motion_state.cpp)
add_executable(go2_read_wireless_controller src/go2/read_wireless_controller.cpp)
add_executable(go2_sport_client_example src/go2/sport_client_example.cpp)
#add_executable(record_bag src/record_bag.cpp)

ament_target_dependencies(h1_2_low_level_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(g1_low_level_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_arm7_control ${DEPENDENCY_LIST})
ament_target_dependencies(g1_ankle_swing ${DEPENDENCY_LIST})
ament_target_dependencies(g1_loco_client_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_audio_client_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(go2_low_level_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(go2_read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(go2_read_motion_state ${DEPENDENCY_LIST})
ament_target_dependencies(go2_read_wireless_controller ${DEPENDENCY_LIST})
ament_target_dependencies(go2_sport_client_example ${DEPENDENCY_LIST})
#ament_target_dependencies(record_bag ${DEPENDENCY_LIST})

install(TARGETS 
        h1_2_low_level_ctrl
        g1_low_level_example
        g1_arm7_control
        g1_ankle_swing
        g1_loco_client_example
        g1_audio_client_example
        g1_read_low_state
        go2_low_level_ctrl
        go2_read_low_state
        go2_read_motion_state
        go2_read_wireless_controller
        go2_sport_client_example
        DESTINATION)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
