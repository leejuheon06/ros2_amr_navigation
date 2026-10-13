cmake_minimum_required(VERSION 3.8)
project(amr_navigation_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    maps
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/planning_benchmark.py
    scripts/generate_maze_assets.py
    scripts/plot_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
