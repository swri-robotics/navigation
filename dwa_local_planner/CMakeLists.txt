cmake_minimum_required(VERSION 2.8.3)
project(dwa_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            pcl_conversions
            roscpp
            tf
        )

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/DWAPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dwa_local_planner
    CATKIN_DEPENDS
        dynamic_reconfigure
        pluginlib
        roscpp
)

add_library(cost_functions 
    src/map_grid_cost_function.cpp
    src/goal_dist_cost_function.cpp
    src/path_dist_cost_function.cpp
    src/offset_grid_cost_function.cpp
    src/goal_align_cost_function.cpp
    src/path_align_cost_function.cpp
    src/obstacle_cost_function.cpp
    src/oscillation_cost_function.cpp
    src/prefer_forward_cost_function.cpp
) 

add_library(dwa_local_planner
    src/scale_manager.cpp
    src/dwa_planner.cpp
    src/dwa_planner_ros.cpp
    src/simple_scored_sampling_planner.cpp
)
target_link_libraries(cost_functions base_local_planner)
target_link_libraries(dwa_local_planner base_local_planner)

add_dependencies(dwa_local_planner dwa_local_planner_gencfg)
add_dependencies(dwa_local_planner nav_msgs_gencpp)

add_executable(standalone_planner src/standalone_planner.cpp)
target_link_libraries(standalone_planner dwa_local_planner)

install(TARGETS dwa_local_planner
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
