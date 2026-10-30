cmake_minimum_required(VERSION 3.10)
project(armor_solver)

## Use C++14
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

add_library(armor_tinympc STATIC
  third_party/tinympc/admm.cpp
  third_party/tinympc/tiny_api.cpp
  third_party/tinympc/rho_benchmark.cpp
)
set_target_properties(armor_tinympc PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(armor_tinympc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinympc>)
target_link_libraries(armor_tinympc PUBLIC Eigen3::Eigen)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)



target_link_libraries(${PROJECT_NAME} $<BUILD_INTERFACE:armor_tinympc>)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN qd::auto_aim::ArmorSolverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_axis_mpc test/test_axis_mpc.cpp src/axis_mpc.cpp)
  target_include_directories(test_axis_mpc PRIVATE include)
  target_link_libraries(test_axis_mpc Eigen3::Eigen armor_tinympc)
  ament_add_gtest(test_bullet_speed_state test/test_bullet_speed_state.cpp)
  target_include_directories(test_bullet_speed_state PRIVATE include)

  ament_add_gtest(test_tracker_prediction test/test_tracker_prediction.cpp)
  target_link_libraries(test_tracker_prediction ${PROJECT_NAME})

  ament_add_gtest(test_solver_motion_switches test/test_solver_motion_switches.cpp)
  target_link_libraries(test_solver_motion_switches ${PROJECT_NAME})

endif()

#############
## Install ##
#############

ament_auto_package()
