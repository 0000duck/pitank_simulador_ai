cmake_minimum_required(VERSION 2.8.0)
project(game_engine)

find_package(catkin REQUIRED COMPONENTS
                    qt_build
                    roscpp
                    cv_bridge
                    roscpp
                    sensor_msgs
                    std_msgs
                    geometry_msgs
                    image_transport
                    apriltags_ros
		    message_generation)

find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

link_directories(/usr/include/)
add_compile_options(-std=c++11)

add_message_files(
  FILES
  RobotDescription.msg
  RobotDescriptionArray.msg
  UIState.msg
  ShootAndTurbo.msg
  WallInfo.msg
  WallInfoArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

rosbuild_prepare_qt4(QtCore QtGui)

SET(game_engine_HEADERS include/objects/controller.hpp include/objects/racemap.hpp include/objects/game_clock.hpp include/objects/bullet.hpp include/objects/person.hpp include/objects/pallet.hpp include/objects/machine.hpp include/objects/circle.hpp include/objects/explosion.hpp include/objects/wall.hpp include/objects/tag.hpp include/objects/map.hpp include/utils/rosConnection.hpp include/graphics/main_window.hpp include/graphics/dialog.hpp include/objects/grid.hpp)
SET(game_engine_SOURCES src/objects/controller.cpp src/objects/racemap.cpp src/objects/game_clock.cpp src/objects/circle.cpp src/objects/pallet.cpp src/objects/person.cpp src/objects/machine.cpp src/objects/bullet.cpp src/objects/explosion.cpp src/objects/tag.cpp src/objects/wall.cpp src/objects/tag.cpp src/objects/map.cpp src/objects/grid.cpp src/graphics/main_window.cpp src/graphics/map_drawer.cpp src/graphics/dialog.cpp src/utils/rosConnection.cpp src/main.cpp)
SET(game_engine_FORMS   ui/main_window.ui ui/dialog.ui)
QT4_WRAP_CPP(game_engine_HEADERS_MOC ${game_engine_HEADERS})
QT4_WRAP_UI(game_engine_FORMS_HEADERS ${game_engine_FORMS})

add_executable(game_engine ${game_engine_SOURCES} ${game_engine_HEADERS_MOC} ${game_engine_FORMS_HEADERS})
target_link_libraries(game_engine ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} -lserial)
install(TARGETS game_engine RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


