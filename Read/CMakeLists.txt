cmake_minimum_required(VERSION 2.8.3)
project(Read)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(USE_Ros "Use ROS to catkin_make" ON)

configure_file("${PROJECT_SOURCE_DIR}/src/ReadConfig.h.in" "${PROJECT_SOURCE_DIR}/src/ReadConfig.h")

if(USE_Ros)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs novatel_gps_msgs cv_bridge sensor_msgs pcl_ros message_generation)

generate_messages(DEPENDENCIES sensor_msgs novatel_gps_msgs std_msgs)#我们的message需要依赖的ros中的其他数据类型
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

endif()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(include 
                    ${PROJECT_SOURCE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    )

add_subdirectory(src)








