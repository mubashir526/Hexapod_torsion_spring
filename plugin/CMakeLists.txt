cmake_minimum_required(VERSION 3.16)
project(gz_joint_torsional_spring)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# --- Gazebo version selection -------------------------------------------
# Harmonic (gz-sim8) ships with ROS 2 Jazzy; Ionic (gz-sim9) with Kilted/
# Rolling. Try newest first and fall back.
find_package(gz-sim9 QUIET)
if(gz-sim9_FOUND)
  set(GZ_SIM_VER 9)
  set(GZ_PLUGIN_VER 3)
else()
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER 8)
  set(GZ_PLUGIN_VER 2)
endif()

find_package(gz-plugin${GZ_PLUGIN_VER} REQUIRED COMPONENTS register)
find_package(gz-common5 QUIET)
find_package(sdformat14 QUIET)

message(STATUS "Building against gz-sim${GZ_SIM_VER}")

add_library(${PROJECT_NAME} SHARED
  src/torsional_spring_system.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Make the .so discoverable by gz-sim without manual export.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
