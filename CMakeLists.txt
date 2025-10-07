cmake_minimum_required(VERSION 3.16)
project(camdrive_arduino LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs

)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


find_path(LibSerial_INCLUDE_DIRS "libserial/SerialPort.h")
find_library(LibSerial_LIBRARIES NAMES serial)
include_directories(${LibSerial_INCLUDE_DIRS})


## COMPILE
add_library(
  camdrive_arduino
  SHARED
  src/camdrive_system.cpp
)

target_compile_features(camdrive_arduino PUBLIC cxx_std_17)

target_include_directories(camdrive_arduino PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/>
)

ament_target_dependencies(camdrive_arduino 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#target_link_libraries(camdrive_arduino PUBLIC serial)
target_link_libraries(${PROJECT_NAME} ${LibSerial_LIBRARIES})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "CAMDRIVE_ARDUINO_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface camdrive_arduino.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/camdrive_arduino
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/camdrive_arduino
)
install(TARGETS camdrive_arduino
  EXPORT export_camdrive_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_camdrive_arduino HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
