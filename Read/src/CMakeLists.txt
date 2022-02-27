
add_library(libread read.cpp)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(read read.cpp)
if(USE_Ros)
target_link_libraries(read ${catkin_LIBRARIES})
endif()
target_link_libraries(read ${OpenCV_LIBS} ${PCL_LIBRARIES})
