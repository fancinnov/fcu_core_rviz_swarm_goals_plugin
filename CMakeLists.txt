cmake_minimum_required(VERSION 3.0.2)
project(navi_multi_goals_pub_rviz_plugin)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  tf
)
catkin_package(CATKIN_DEPENDS roscpp rviz geometry_msgs nav_msgs visualization_msgs std_msgs tf)
set(CMAKE_AUTOMOC ON)
add_compile_options(-std=c++11)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
add_definitions(-DQT_NO_KEYWORDS)

set(SOURCE_FILES
  src/multi_navi_goal_panel.cpp
)
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${catkin_LIBRARIES})


## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

