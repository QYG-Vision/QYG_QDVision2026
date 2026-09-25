cmake_minimum_required(VERSION 3.10)
project(rm_serial_driver)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN qd::serial_driver::SerialDriverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN qd::serial_driver::VirtualSerialNode
  EXECUTABLE virtual_serial_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN qd::serial_driver::ZeroOrderGimbalTestNode
  EXECUTABLE zero_order_gimbal_test_node
)

ament_auto_add_executable(set_gimbal_target
  tools/set_gimbal_target.cpp
)

target_link_libraries(set_gimbal_target ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}_node PUBLIC include)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)

  ament_add_gtest(test_fixed_packet_tool test/test_fixed_packet_tool.cpp)
  target_link_libraries(test_fixed_packet_tool ${PROJECT_NAME})

  ament_add_gtest(test_qyg_protocol test/test_qyg_protocol.cpp)
  # 协议解析仅依赖标准库；直接编译实现，避免单测启动时加载 Hik 相机动态库。
  target_sources(test_qyg_protocol PRIVATE src/protocol/qyg_protocol.cpp)
  target_include_directories(test_qyg_protocol PRIVATE include)

  ament_add_gtest(test_zero_order_gimbal_controller test/test_zero_order_gimbal_controller.cpp)
  target_link_libraries(test_zero_order_gimbal_controller ${PROJECT_NAME})

  ament_add_gtest(test_gimbal_target_cli test/test_gimbal_target_cli.cpp)
  target_link_libraries(test_gimbal_target_cli ${PROJECT_NAME})

  ament_add_gtest(test_mode_sync_config test/test_mode_sync_config.cpp)
  target_link_libraries(test_mode_sync_config ${PROJECT_NAME})
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
)
