cmake_minimum_required(VERSION 2.8)
project(vehicle_simulator)
add_definitions(-Wall -Wunused -std=c++11 -pipe -Wunused -Wextra)
set(VEHICLE_SIMULATOR_DEPENDENCIES
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf
  message_generation
  message_runtime
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${VEHICLE_SIMULATOR_DEPENDENCIES})
find_package(Boost REQUIRED)
find_package(Qt5Widgets REQUIRED)

add_message_files(FILES SocialVehicles.msg)
generate_messages(
  DEPENDENCIES
  std_msgs nav_msgs
)

# dynamic reconfigure parameters
# generate_dynamic_reconfigure_options(config/PedsimSimulator.cfg)

catkin_package(
  CATKIN_DEPENDS ${VEHICLE_SIMULATOR_DEPENDENCIES}
  INCLUDE_DIRS include
)

include_directories(include)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDES})
add_definitions(${Qt5Widgets_DEFINITIONS})


add_executable(simulate_environment src/simulate_environment.cpp src/scenarioreader.cpp)
add_dependencies(simulate_environment ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulate_environment ${BOOST_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(simulate_environment
  ${Qt5Widgets_LIBRARIES} ${BOOST_LIBRARIES} ${catkin_LIBRARIES}
)
add_executable(simulate_automated_car src/simulate_automated_car.cpp)
add_dependencies(simulate_automated_car ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulate_automated_car ${BOOST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(simulate_social_car src/simulate_social_car.cpp)
add_dependencies(simulate_social_car ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulate_social_car ${BOOST_LIBRARIES} ${catkin_LIBRARIES})


## Unit Tests
