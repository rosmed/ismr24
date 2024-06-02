cmake_minimum_required(VERSION 3.8)
project(ignition_gazebo_us_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(PlusLib REQUIRED )

ign_find_package( ignition-plugin1 REQUIRED COMPONENTS register )
ign_find_package( ignition-gazebo6 REQUIRED )
ign_find_package( ignition-sensors6 REQUIRED )

include_directories( include ${PlusLib_INCLUDE_DIRS} )

add_library( us_sensor SHARED src/us_sensor.cpp )
add_library( us_system SHARED src/us_system.cpp )

set_property(TARGET us_sensor PROPERTY CXX_STANDARD 17)
set_property(TARGET us_system PROPERTY CXX_STANDARD 17)

target_link_libraries( us_sensor ignition-gazebo6 ignition-sensors6)
target_link_libraries( us_system us_sensor ignition-plugin1 ignition-gazebo6 ${VTK_LIBRARIES} ${PlusLib_LIBRARIES} )

# target_link_libraries( rsp_plugin
#  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
#  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER} )

install( DIRECTORY include DESTINATION share/${PROJECT_NAME} )
install( DIRECTORY launch DESTINATION share/${PROJECT_NAME} )
install( DIRECTORY worlds DESTINATION share/${PROJECT_NAME} )
install( DIRECTORY config DESTINATION share/${PROJECT_NAME} )
install( TARGETS us_system us_sensor LIBRARY DESTINATION lib )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
