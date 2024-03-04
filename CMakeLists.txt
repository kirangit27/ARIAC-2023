cmake_minimum_required(VERSION 3.8)
project(ARIAC-2023)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ariac_msgs REQUIRED)	#UPDATED
find_package(std_srvs REQUIRED)	#UPDATED
find_package(shape_msgs REQUIRED)	#UPDATED
find_package(moveit_ros_planning_interface REQUIRED)	#UPDATED

include_directories(include/group4)

set(SENSOR_CAMERA_DEPENDENCIES
  rclcpp
  std_srvs
  ariac_msgs
  geometry_msgs
  tf2  
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
)

foreach(Dependency IN ITEMS ${SENSOR_CAMERA_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(rwa2 src/rwa2.cpp src/rwa2_main.cpp)		#UPDATED	     
ament_target_dependencies(rwa2 rclcpp ariac_msgs std_srvs)	#UPDATED

install(TARGETS				#UPDATED
	rwa2			#UPDATED
	DESTINATION lib/${PROJECT_NAME}		#UPDATED
	)

install(DIRECTORY include
	launch
	config
	rviz
	DESTINATION share/${PROJECT_NAME})	#UPDATED

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

ament_package()
