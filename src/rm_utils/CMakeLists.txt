cmake_minimum_required(VERSION 3.5)
project(rm_utils)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(rcpputils REQUIRED)
find_package(fmt REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

# include
include_directories(include)

add_library(qdlogger SHARED
  src/logger/logger_impl.cpp
  src/logger/writer.cpp
  src/logger/logger_pool.cpp
)
target_link_libraries(qdlogger fmt::fmt)
ament_target_dependencies(qdlogger fmt)

# create rm_utils lib
add_library(${PROJECT_NAME} SHARED
  src/math/utils.cpp
  src/math/pnp_solver.cpp
  src/math/trajectory_compensator.cpp
  src/math/manual_compensator.cpp
  src/math/extended_kalman_filter.cpp
  src/url_resolver.cpp
  src/heartbeat.cpp
  src/vision_state_machine.cpp
  src/driver/HikCamera.cpp
)

set(dependencies
  rclcpp
  rcpputils
  Eigen3
  eigen3_cmake_module
  geometry_msgs
  rm_interfaces
  Ceres
  OpenCV
  camera_info_manager
  cv_bridge
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

# 相机SDK路径（可选，如果hik-camera包存在则使用其SDK）
find_path(HIK_SDK_INCLUDE_DIR MvCameraControl.h
  PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include
    /usr/local/include
    /usr/include
  NO_DEFAULT_PATH
)

if(HIK_SDK_INCLUDE_DIR)
  message(STATUS "Found Hik SDK at: ${HIK_SDK_INCLUDE_DIR}")
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${HIK_SDK_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
  )

  # 根据架构设置库路径
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    find_library(MVCAMERACONTROL_LIB MvCameraControl
      PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64
        /usr/local/lib
        /usr/lib
      NO_DEFAULT_PATH
    )
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    find_library(MVCAMERACONTROL_LIB MvCameraControl
      PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64
        /usr/local/lib
        /usr/lib
      NO_DEFAULT_PATH
    )
  endif()

  if(MVCAMERACONTROL_LIB)
    target_link_libraries(${PROJECT_NAME} ${MVCAMERACONTROL_LIB})
    message(STATUS "Linked MvCameraControl library")
  else()
    message(WARNING "MvCameraControl library not found, vision_state_machine will not support camera")
  endif()
else()
  message(WARNING "Hik SDK not found, vision_state_machine will not support camera")
endif()

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install hikSDK include directories for dependent packages
install(DIRECTORY hikSDK/include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS qdlogger
  EXPORT qdlogger
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# export lib lib
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_targets(qdlogger HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} fmt)
ament_export_include_directories(include)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
