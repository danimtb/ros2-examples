cmake_minimum_required(VERSION 3.8)
project(hello)

message("[hello] AMENT_PREFIX_PATH env: $ENV{AMENT_PREFIX_PATH}")
message("[hello] COLCON_PREFIX_PATH env: $ENV{COLCON_PREFIX_PATH}")
message("[hello] COLCON_CURRENT_PREFIX env: $ENV{COLCON_CURRENT_PREFIX}")
message("[hello] CMAKE_PREFIX_PATH env: $ENV{CMAKE_PREFIX_PATH}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(hello src/hello.cpp)

target_include_directories(hello PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/hello>
  $<INSTALL_INTERFACE:include>)

target_compile_features(hello PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_export_targets(helloTargets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS hello
  EXPORT helloTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
