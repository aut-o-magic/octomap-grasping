cmake_minimum_required(VERSION 3.5)
project(octomap_grasping VERSION 1.0)

# Set VERSION
set(OCTOMAP_GRASPING_VERSION 1.0.0)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-copy)
endif()

# Octomap imports
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

# CMake variables
list(APPEND ${PROJECT_NAME}_LIBRARIES ${OCTOMAP_LIBRARIES})
list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${OCTOMAP_INCLUDE_DIRS})

# Create library 1
set(LIB_GQ OcTreeGraspQuality)

add_library(${LIB_GQ} src/OcTreeGraspQuality.cpp)
target_include_directories(${LIB_GQ} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set_property(TARGET ${LIB_GQ} PROPERTY VERSION ${OCTOMAP_GRASPING_VERSION})

# Create library 2
set(LIB_GRIPPER OcTreeGripper)

add_library(${LIB_GRIPPER} src/OcTreeGripper.cpp)
target_include_directories(${LIB_GRIPPER} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set_property(TARGET ${LIB_GRIPPER} PROPERTY VERSION ${OCTOMAP_GRASPING_VERSION})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
#target_compile_definitions(${LIB_NAME} PRIVATE "OCTOMAP_GRASPING_BUILDING_LIBRARY")

install(
  TARGETS ${LIB_GQ} ${LIB_GRIPPER}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
  #RUNTIME DESTINATION bin
  #INCLUDES DESTINATION include
)

# add self cmake variables
list(APPEND ${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME}::${LIB_GQ} ${PROJECT_NAME}::${LIB_GRIPPER}) # Would probably be better to use absolute path, not namespaced variables...
list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

set(ConfigPackageLocation lib/cmake/${PROJECT_NAME})
# Explicitly install the auto-generated .cmake file from the targets install command
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${ConfigPackageLocation}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "Config.cmake.in"
  "${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${ConfigPackageLocation}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${OCTOMAP_GRASPING_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# Install Config and ConfigVersion files
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${ConfigPackageLocation}
)