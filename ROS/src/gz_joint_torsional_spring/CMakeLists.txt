cmake_minimum_required(VERSION 3.16)
project(gz_joint_torsional_spring)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

# --- Gazebo generation auto-detect ---------------------------------------
# Harmonic (gz-sim8) ships with the gz-harmonic apt metapackage used here.
# Ionic (gz-sim9) is tried first and falls through to Harmonic. The matching
# gz-plugin / gz-transport / gz-msgs generations are selected alongside.
find_package(gz-sim9 QUIET)
if(gz-sim9_FOUND)
  set(GZ_SIM_VER 9)
  set(GZ_PLUGIN_VER 3)
  set(GZ_TRANSPORT_VER 14)
  set(GZ_MSGS_VER 11)
else()
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER 8)
  set(GZ_PLUGIN_VER 2)
  set(GZ_TRANSPORT_VER 13)
  set(GZ_MSGS_VER 10)
endif()

find_package(gz-plugin${GZ_PLUGIN_VER} REQUIRED COMPONENTS register)
find_package(gz-transport${GZ_TRANSPORT_VER} REQUIRED)
find_package(gz-msgs${GZ_MSGS_VER} REQUIRED)

message(STATUS "Building gz_joint_torsional_spring against gz-sim${GZ_SIM_VER}")

# ---- Plugin 1: the parallel torsion spring ------------------------------
# Library target named after the project so the documented SDF usage
# (filename="gz_joint_torsional_spring") resolves to libgz_joint_torsional_spring.so.
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

# ---- Plugin 2: commanded-effort publisher (measures motor/PID torque) ----
add_library(commanded_effort_publisher SHARED
  src/commanded_effort_publisher.cc
)
target_include_directories(commanded_effort_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(commanded_effort_publisher
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

install(TARGETS ${PROJECT_NAME} commanded_effort_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Make the .so files discoverable by gz-sim without manual export: sourcing
# the workspace prepends install/lib to GZ_SIM_SYSTEM_PLUGIN_PATH.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
