cmake_minimum_required(VERSION 3.8)
project(app)

message("[app] AMENT_PREFIX_PATH env: $ENV{AMENT_PREFIX_PATH}")
message("[app] COLCON_PREFIX_PATH env: $ENV{COLCON_PREFIX_PATH}")
message("[app] COLCON_CURRENT_PREFIX env: $ENV{COLCON_CURRENT_PREFIX}")
message("[app] CMAKE_PREFIX_PATH env: $ENV{CMAKE_PREFIX_PATH}")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(meet REQUIRED)

add_executable(app src/app.cpp)
target_include_directories(app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(app PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(app meet)

install(TARGETS app
  DESTINATION lib/${PROJECT_NAME})

ament_package()
