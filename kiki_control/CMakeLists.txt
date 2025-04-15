cmake_minimum_required(VERSION 3.8)
project(kiki_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(JetsonGPIO REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/hw_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  JetsonGPIO
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# target_link_libraries(${PROJECT_NAME} PRIVATE JetsonGPIO)

# target_include_directories(${PROJECT_NAME} PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include/kiki_control>
# )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PUBLIC "KIKI_DIFFDRIVE_BUILDING_DLL")

# do not let plugninlib use boost (galactic thing)
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface kiki_diffdrive.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# INSTALL
# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}/
# )

# install(
#   DIRECTORY description/
#   DESTINATION share/${PROJECT_NAME}/
# )

# install(TARGETS ${PROJECT_NAME}
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(DIRECTORY config param plugin launch include src DESTINATION share/${PROJECT_NAME}/)

# install(FILES
#   kiki_diffdrive.xml
#   DESTINATION share/${PROJECT_NAME}
# )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  JetsonGPIO
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()

