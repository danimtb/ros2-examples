cmake_minimum_required(VERSION 3.8)
project(meet)

message("[meet] AMENT_PREFIX_PATH env: $ENV{AMENT_PREFIX_PATH}")
message("[meet] COLCON_PREFIX_PATH env: $ENV{COLCON_PREFIX_PATH}")
message("[meet] COLCON_CURRENT_PREFIX env: $ENV{COLCON_CURRENT_PREFIX}")
message("[meet] CMAKE_PREFIX_PATH env: $ENV{CMAKE_PREFIX_PATH}")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(greet REQUIRED)

add_library(meet src/meet.cpp)
target_include_directories(meet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/meet>
  $<INSTALL_INTERFACE:include>)

target_compile_features(meet PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(meet greet)

ament_export_targets(meetTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(greet)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS meet
  EXPORT meetTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()