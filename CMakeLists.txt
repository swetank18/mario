cmake_minimum_required(VERSION 3.31.6)

project(
  Mario
  VERSION 0.1
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

find_package(yasmin REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(
  Boost 1.87.0
  COMPONENTS program_options
  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(GLEW REQUIRED)
find_package(stella_vslam REQUIRED)
find_package(cppzmq 4.11.0 REQUIRED)
find_package(spdlog REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ompl 1.7.0 REQUIRED)
# pkg_check_modules is only defined once FindPkgConfig has been included; it
# happened to work when some other find_package pulled it in first.
find_package(PkgConfig REQUIRED)
pkg_check_modules(Asio asio)
pkg_check_modules(Cobs cobs)

# utils
add_library(utils src/utils.cpp)
target_include_directories(utils PUBLIC include ${realsense2_INCLUDE_DIRS}
                                        ${OpenCV_INCLUDE_DIRS} ${spdlog_DIR})
target_link_libraries(utils PUBLIC realsense2 ${OpenCV_LIBS} cppzmq
                                   Eigen3::Eigen spdlog::spdlog slam)

# tarzan
add_library(serial src/serial.cpp)
target_include_directories(serial PUBLIC include ${Cobs_INCLUDE_DIRS}
                                         ${Asio_INCLUDE_DIRS})
target_link_libraries(serial PUBLIC ${Cobs_LDFLAGS})

# slam
add_library(slam src/slam.cpp)
target_include_directories(
  slam PUBLIC include ${rerun_sdk_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS}
              ${Pangolin_INCLUDE_DIRS})
target_link_libraries(
  slam PUBLIC utils rerun_sdk Eigen3::Eigen stella_vslam::stella_vslam
              ${GLEW_LIBS} ${Pangolin_LIBS})

# nav: parameter loading
add_library(nav_params src/nav/params.cpp)
target_include_directories(nav_params PUBLIC include)
target_link_libraries(nav_params PUBLIC yaml-cpp)

# nav: occupancy mapping — owns grid_map, PCL, rerun
add_library(nav_map src/nav/occupancy_map.cpp)
target_include_directories(nav_map PUBLIC include ${PCL_INCLUDE_DIRS}
                                          ${rerun_sdk_INCLUDE_DIRS})
target_link_libraries(nav_map PUBLIC nav_params ${PCL_LIBRARIES} Eigen3::Eigen
                                     grid_map_core rerun_sdk spdlog::spdlog)

# nav: planning — owns OMPL. Deliberately does NOT link nav_map: the planners
# reach the map through include/nav/map_query.hpp, which has no headers of its
# own, so nothing on the mapping side can leak across.
add_library(nav_planner src/nav/planner_ompl.cpp src/nav/planner_astar.cpp)
target_include_directories(nav_planner PUBLIC include)
target_link_libraries(nav_planner PUBLIC nav_params ompl::ompl spdlog::spdlog)

# pid
add_library(pid src/pid.cpp)
target_include_directories(pid PUBLIC include)

option(BUILD_TESTS "Build Test Scipts" ON)
if(BUILD_TESTS)
  enable_testing()
  find_package(Threads REQUIRED)

  # tarzan test
  add_executable(serial_test src/serial.cpp)
  target_compile_definitions(serial_test PRIVATE SERIAL_TEST_CPP=1)
  target_link_libraries(serial_test PRIVATE serial)

  # slam test
  add_executable(slam_test src/slam.cpp)
  target_compile_definitions(slam_test PRIVATE SLAM_TEST_CPP=1)
  target_include_directories(slam_test PRIVATE slam)
  target_link_libraries(slam_test PRIVATE slam)

  # utils test
  add_executable(utils_test src/utils.cpp)
  target_compile_definitions(utils_test PRIVATE UTILS_TEST_CPP=1)
  target_include_directories(utils_test PRIVATE utils)
  target_link_libraries(utils_test PRIVATE utils)

  # pid test
  add_executable(pid_test src/pid.cpp)
  target_compile_definitions(pid_test PRIVATE PID_TEST_CPP=1)
  target_include_directories(pid_test PRIVATE pid slam serial)
  target_link_libraries(pid_test PRIVATE pid slam serial)

  # fsm test -- drives include/fsm.hpp, the same transition graph mario.cpp
  # runs, with scripted action results. Deliberately depends on nothing but
  # taskflow, so the state machine can be tested without stella_vslam, OMPL,
  # grid_map, rerun or a RealSense attached.
  add_executable(fsm_test test/fsm_test.cpp)
  target_include_directories(fsm_test PRIVATE include)
  target_link_libraries(fsm_test PRIVATE Threads::Threads)
  add_test(NAME fsm COMMAND fsm_test)

  # nav test -- drives OccupancyMap and AStarPlanner together over a simulated
  # 14 m leg. Needs no SLAM, no RealSense and no rerun viewer, so it covers the
  # recentring/clearance behaviour without the rest of the stack being up.
  add_executable(nav_test test/nav_test.cpp)
  target_link_libraries(nav_test PRIVATE nav_map nav_planner)
  add_test(NAME nav COMMAND nav_test)

  # planner test -- AStarPlanner against a synthetic MapQuery. Links
  # nav_planner and nothing else, so it doubles as the structural check that
  # the mapping dependency tree has not leaked back through map_query.hpp: if
  # this target ever needs PCL, grid_map or rerun to link, the split is gone.
  add_executable(planner_test test/planner_test.cpp)
  target_link_libraries(planner_test PRIVATE nav_planner)
  add_test(NAME planner COMMAND planner_test)

  # aruco test -- src/aruco.cpp is not in the tree, so only wire this up if it
  # reappears. Referencing it unconditionally fails the configure step.
  if(EXISTS ${CMAKE_SOURCE_DIR}/src/aruco.cpp)
    add_executable(aruco_test src/aruco.cpp)
    target_include_directories(aruco_test PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(aruco_test PUBLIC ${OpenCV_LIBS})
  endif()

endif()

add_executable(mario ${CMAKE_SOURCE_DIR}/src/mario.cpp)

target_include_directories(mario PRIVATE yasmin slam serial utils nav_map
                                         nav_planner pid)

target_link_libraries(
  mario
  PRIVATE yasmin
          Boost::program_options
          slam
          utils
          serial
          nav_map
          nav_planner
          pid)
