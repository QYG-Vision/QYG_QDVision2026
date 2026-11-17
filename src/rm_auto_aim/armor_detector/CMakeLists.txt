cmake_minimum_required(VERSION 3.18)
project(armor_detector)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)
add_definitions(-O3)


## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
# set(EXTERNAL_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
# set(EXTERNAL_LIBS ${G2O_LIBRARIES})

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
find_package(fmt REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
option(ENABLE_TENSORRT "Build the TensorRT 10 inference backend" OFF)
if(ENABLE_TENSORRT)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu)
  find_library(TENSORRT_LIBRARY nvinfer)
  find_package(CUDAToolkit REQUIRED)
  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
    message(FATAL_ERROR "TensorRT development files missing: install libnvinfer-dev in the container")
  endif()
endif()
# find_package(G2O REQUIRED)
# find_package(Sophus REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

if(ENABLE_TENSORRT)
  target_compile_definitions(${PROJECT_NAME} PRIVATE QD_HAS_TENSORRT)
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${TENSORRT_LIBRARY} CUDA::cudart)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
# ${G2O_INCLUDE_DIRS}
# ${Sophus_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
openvino::runtime
fmt::fmt
TBB::tbb
# g2o_core
# g2o_stuff
# g2o_solver_csparse
# g2o_types_sba
# g2o_types_slam3d
# g2o_solver_dense

)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN qd::auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
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
    ament_cmake_lint_cmake
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_detector test/test_detector.cpp)
  target_link_libraries(test_detector ${PROJECT_NAME})
  ament_add_gtest(test_yolo_backends test/test_yolo_backends.cpp)
  target_link_libraries(test_yolo_backends ${PROJECT_NAME})

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  docs
  model
)
