cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(waypoint_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# set(action_files
#   "action/WaypointAction.action"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${action_files}
#   DEPENDENCIES geometry_msgs  # Add packages that above messages depend on
# )


# # rosidl_target_interfaces(tortoisebot_waypoints
# #   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ament_export_dependencies(rosidl_default_runtime)


# Add Library
include_directories(include)

add_library(tortoisebot_action_server_lib src/tortoisebot_action_server.cpp include/tortoisebot_waypoints/tortoisebot_action_server.hpp)
ament_target_dependencies(tortoisebot_action_server_lib 
  rclcpp 
  rclcpp_action 
  std_msgs 
  tf2_geometry_msgs 
  geometry_msgs 
  nav_msgs 
  waypoint_interfaces
  )
ament_export_targets(tortoisebot_action_server_lib
					HAS_LIBRARY_TARGET)

add_library(tortoisebot_action_client_lib src/tortoisebot_action_client.cpp include/tortoisebot_waypoints/tortoisebot_action_client.hpp)
ament_target_dependencies(tortoisebot_action_client_lib 
  rclcpp 
  rclcpp_action 
  geometry_msgs 
  waypoint_interfaces
  )

ament_export_targets(tortoisebot_action_client_lib
					HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/tortoisebot_waypoints
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
      test/main.cpp
      test/tortoisebot_action_server_test.cpp
  )

  # Add gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test tortoisebot_action_client_lib tortoisebot_action_server_lib)

  install(TARGETS
        ${PROJECT_NAME}_test
        # Install the generated test executable file to the path after DESTINATION
        DESTINATION lib/${PROJECT_NAME})
#   ament_lint_auto_find_test_dependencies()
endif()

add_executable(tortoisebot_action_server src/main.cpp)
target_link_libraries(tortoisebot_action_server PUBLIC tortoisebot_action_server_lib)


add_executable(tortoisebot_action_client src/ac_main.cpp)
target_link_libraries(tortoisebot_action_client PUBLIC tortoisebot_action_client_lib)

install(TARGETS tortoisebot_action_server_lib 
  DESTINATION lib/tortoisebot_action_server_lib)

install(TARGETS tortoisebot_action_client_lib 
  DESTINATION lib/tortoisebot_action_client_lib)

install(TARGETS
   tortoisebot_action_server
   tortoisebot_action_client
   DESTINATION lib/${PROJECT_NAME}/
 )

install(
  TARGETS tortoisebot_action_server_lib
  EXPORT tortoisebot_action_server_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS tortoisebot_action_client_lib
  EXPORT  tortoisebot_action_client_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


ament_package()
