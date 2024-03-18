cmake_minimum_required(VERSION 3.0.2)
project(px4_gcs_rqt)


find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  rqt_gui
  rqt_gui_cpp
  std_msgs
  ctrller_msgs
)

find_package(Qt5Widgets REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(px4_gcs_rqt_SRCS
  src/px4_gcs_rqt/gcs_plugin.cpp
  # src/px4_gcs_rqt/service_caller.cpp
)

set(px4_gcs_rqt_HDRS
  include/px4_gcs_rqt/gcs_plugin.h
  # include/px4_gcs_rqt/service_caller.h
)

set(px4_gcs_rqt_UIS
  resources/gcs_plugin_6dofAM.ui
  # resources/ServiceCaller.ui
)

set(px4_gcs_rqt_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()


qt5_wrap_cpp(px4_gcs_rqt_MOCS 
  ${px4_gcs_rqt_HDRS}
)

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

qt5_wrap_ui(px4_gcs_rqt_UIS_H 
  ${px4_gcs_rqt_UIS}
)

message(${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")


include_directories(${px4_gcs_rqt_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${px4_gcs_rqt_SRCS} ${px4_gcs_rqt_MOCS} ${px4_gcs_rqt_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)


# TODO use?
find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)