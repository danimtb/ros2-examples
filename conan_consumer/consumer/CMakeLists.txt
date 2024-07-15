cmake_minimum_required(VERSION 3.8)
project(consumer)

message("[consumer] AMENT_PREFIX_PATH env: $ENV{AMENT_PREFIX_PATH}")
message("[consumer] COLCON_PREFIX_PATH env: $ENV{COLCON_PREFIX_PATH}")
message("[consumer] COLCON_CURRENT_PREFIX env: $ENV{COLCON_CURRENT_PREFIX}")
message("[consumer] CMAKE_PREFIX_PATH env: $ENV{CMAKE_PREFIX_PATH}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(box2d REQUIRED)

message("[consumer] box2d found?: ${BOX2D_VERSION}")

add_library(consumer src/hello.cpp)

target_include_directories(consumer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/consumer>
  $<INSTALL_INTERFACE:include>)

target_compile_features(consumer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(consumer box2d::box2d)

ament_export_targets(consumerTargets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS consumer
  EXPORT consumerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
