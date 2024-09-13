cmake_minimum_required(VERSION 3.8)
project(greet)

message("[greet] AMENT_PREFIX_PATH env: $ENV{AMENT_PREFIX_PATH}")
message("[greet] COLCON_PREFIX_PATH env: $ENV{COLCON_PREFIX_PATH}")
message("[greet] COLCON_CURRENT_PREFIX env: $ENV{COLCON_CURRENT_PREFIX}")
message("[greet] CMAKE_PREFIX_PATH env: $ENV{CMAKE_PREFIX_PATH}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hello REQUIRED)

add_library(greet src/greet.cpp)

target_include_directories(greet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/greet>
  $<INSTALL_INTERFACE:include>)

target_compile_features(greet PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(greet hello)

ament_export_targets(greetTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(hello)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS greet
  EXPORT greetTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
