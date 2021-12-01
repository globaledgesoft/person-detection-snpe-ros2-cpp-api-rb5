cmake_minimum_required(VERSION 3.5)
project(pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msg REQUIRED)                         # CHANGE


find_package( OpenCV REQUIRED )

link_directories(/root/rb5/)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${PROJECT_SOURCE_DIR}/include/ )
include_directories( ${PROJECT_SOURCE_DIR}/include/zdl/ )


#add_executable( HumanTracker src/main.cpp src/qcsnpe.cpp src/utils.cpp )



add_executable(talker src/main.cpp src/qcsnpe.cpp src/utils.cpp)
target_link_libraries( talker ${OpenCV_LIBS} )
target_link_libraries( talker SNPE)
ament_target_dependencies(talker rclcpp custom_msg)         # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp custom_msg)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
