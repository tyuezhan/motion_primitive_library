cmake_minimum_required(VERSION 2.8)
project(motion_primitive_library)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

set(BUILD_SHARED_LIBS ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS 
 roscpp
)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)

if(YAMLCPP_FOUND)
  message(STATUS "Found yaml-cpp.")
else()
  message(FATAL_ERROR "Could not find yaml-cpp.")
endif()

add_library(poly_solver src/mpl_traj_solver/poly_solver.cpp
  src/mpl_traj_solver/poly_traj.cpp)

add_library(map_planner src/mpl_planner/map_planner.cpp ${catkin_LIBRARIES})

include(CTest)

add_definitions(-DOPENCV_WINDOW=0)

add_executable(test_traj_solver test/test_traj_solver.cpp)
target_link_libraries(test_traj_solver poly_solver ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_test(test_traj_solver test_traj_solver)

add_executable(test_planner_2d test/test_planner_2d.cpp)
target_link_libraries(test_planner_2d map_planner ${YAMLCPP_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_test(test_planner_2d test_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_planner_2d_with_prior_traj test/test_planner_2d_with_prior_traj.cpp)
target_link_libraries(test_planner_2d_with_prior_traj map_planner ${YAMLCPP_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_test(test_planner_2d_with_prior_traj test_planner_2d_with_prior_traj ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_planner_2d_with_yaw test/test_planner_2d_with_yaw.cpp)
target_link_libraries(test_planner_2d_with_yaw map_planner ${YAMLCPP_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_test(test_planner_2d_with_yaw test_planner_2d_with_yaw ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_distance_map_planner_2d test/test_distance_map_planner_2d.cpp)
target_link_libraries(test_distance_map_planner_2d map_planner ${YAMLCPP_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_test(test_distance_map_planner_2d test_distance_map_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_distance_map_planner_2d_with_yaw test/test_distance_map_planner_2d_with_yaw.cpp)
target_link_libraries(test_distance_map_planner_2d_with_yaw map_planner ${YAMLCPP_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_test(test_distance_map_planner_2d_with_yaw test_distance_map_planner_2d_with_yaw ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_distance_map_planner_2d_iterative test/test_distance_map_planner_2d_iterative.cpp)
target_link_libraries(test_distance_map_planner_2d_iterative map_planner ${YAMLCPP_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_test(test_distance_map_planner_2d_iterative test_distance_map_planner_2d_iterative ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(view_utils_test test/view_utils_test.cpp)
# set_target_properties(view_utils_test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(view_utils_test ${catkin_LIBRARIES})
# add_dependencies(view_utils_test  ${catkin_EXPORTED_TARGETS})


install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS poly_solver map_planner
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/mpl_basis include/mpl_traj_solver include/mpl_planner include/mpl_collision
  DESTINATION include
  )
