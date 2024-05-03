cmake_minimum_required(VERSION 3.8)

set(CMAKE_TOOLCHAIN_FILE ../conan/conan_toolchain.cmake)

project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(package_dep REQUIRED)

find_package(Poco REQUIRED)

get_property(importTargetsAfter DIRECTORY "${CMAKE_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
message("${importTargetsAfter}")

get_target_property(LIBB_INCLUDES package_dep::package_dep INCLUDE_DIRECTORIES)
message("${LIBB_INCLUDES}")

add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(my_node package_dep)
#target_include_directories(my_node ${package_dep_INCLUDE_DIRS})

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
