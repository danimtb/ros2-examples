cmake_minimum_required(VERSION 3.8)
project(package_dep)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(package_dep src/hello.cpp)

target_include_directories(package_dep PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/package_dep>
  $<INSTALL_INTERFACE:include>)

target_compile_features(package_dep PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_export_targets(package_depTargets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS package_dep
  EXPORT package_depTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
