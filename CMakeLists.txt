#cmake_minimum_required(VERSION 2.8)
cmake_minimum_required(VERSION 3.5)
project(rgbd_align)
message("start make cmake_template")

# 一定要加-march=native， 不然使用pcl的一些函数会报错
#add_definitions(-DCPP11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -march=native")

find_package(catkin REQUIRED COMPONENTS
        rospy
        rosbag
        roscpp
        cv_bridge
        message_filters
        image_transport
        tf
        )

set(ALL_TARGET_LIBRARIES "")
include(cmake/global_defination.cmake)
include(cmake/boost.cmake)
include(cmake/OpenCV.cmake)

catkin_package()

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE ALL_SRCS "src/*.cpp")


# lidar slam and localize
add_executable(rgbd_node node/rgbd.cpp ${ALL_SRCS})
target_link_libraries(rgbd_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

