cmake_minimum_required(VERSION 2.8)
project(MIE443_Lab3)

find_package(catkin REQUIRED cv_bridge genmsg image_transport sensor_msgs)

# add the resized image message
add_message_files(DIRECTORY msg
   FILES ResizedImage.msg
)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

find_package(OpenCV REQUIRED)
find_package(OpenCV REQUIRED)
find_library(NONFREE opencv_nonfree2_4
						 PATHS /usr/lib
									 /usr/local/lib)

MESSAGE( STATUS "OPENCV_NONFREE_PATH: " ${NONFREE} )

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the publisher example
add_executable(publisher src/my_publisher_picture.cpp)
#add_executable(publisher src/my_publisher_video.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber example
#add_executable(subscriber src/follower.cpp)
add_executable(subscriber src/follower_restructure.cpp)
#add_executable(subscriber src/my_subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  ${NONFREE})

# add the plugin examples
add_library(resize_publisher src/manifest.cpp src/resized_publisher.cpp src/resized_subscriber.cpp)
target_link_libraries(resize_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
