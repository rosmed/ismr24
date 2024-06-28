cmake_minimum_required(VERSION 3.8)
project(ignition_gazebo_us_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(IGSIO_DIR /home/sleonard/ros/pluslib/build/igsio/)

find_package(ament_cmake REQUIRED)
find_package(ignition-cmake2 REQUIRED)

ign_find_package( ignition-plugin1 REQUIRED COMPONENTS register )
ign_find_package( ignition-gazebo6 REQUIRED )
ign_find_package( ignition-sensors6 REQUIRED )

# this needs to come last because of a bug in JsonCpp::JsonCpp
# https://github.com/traversaro/vtk-gazebo-cross-talking/issues/1
find_package(PlusLib REQUIRED)
find_package(VTK REQUIRED)

include_directories( include /home/sleonard/ros/pluslib/src/PlusLib/src/PlusCommon /home/sleonard/ros/pluslib/src/igsio/IGSIOCommon /home/sleonard/ros/pluslib/build/igsio /home/sleonard/ros/pluslib/build/igsio/IGSIOCommon /home/sleonard/ros/pluslib/src/vtkAddon /home/sleonard/ros/pluslib/build/vtkAddon /home/sleonard/ros/pluslib/src/PlusLib/src/PlusCommon/IO /home/sleonard/ros/pluslib/src/PlusLib/src/PlusUsSimulator )

add_library( us_sensor SHARED src/us_sensor.cpp )
add_library( us_system SHARED src/us_system.cpp )

set_property(TARGET us_sensor PROPERTY CXX_STANDARD 17)
set_property(TARGET us_system PROPERTY CXX_STANDARD 17)
ament_target_dependencies( us_system PlusLib )

target_link_libraries( us_sensor ignition-gazebo6 ignition-sensors6 )
target_link_libraries( us_system us_sensor ignition-plugin1 ignition-gazebo6 ${PlusLib_LIBRARIES} )

# target_link_libraries( rsp_plugin
#  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
#  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER} )

install( DIRECTORY include DESTINATION share/${PROJECT_NAME} )
install( DIRECTORY launch DESTINATION share/${PROJECT_NAME} )
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
