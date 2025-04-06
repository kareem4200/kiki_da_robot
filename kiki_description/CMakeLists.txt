cmake_minimum_required(VERSION 3.8)
project(kiki_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  install(
  DIRECTORY launch urdf config
  DESTINATION share/kiki_description
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
