# 设置最低 CMake 版本要求
cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(VehicleProject)

# 设置 C++ 标准为 C++11 或更高版本
set(CMAKE_CXX_STANDARD 11)

# 设置包含目录
include_directories(
    ${PROJECT_SOURCE_DIR}/vehicle_state
    ${PROJECT_SOURCE_DIR}/common
    ${PROJECT_SOURCE_DIR}/mcts_base
    ${PROJECT_SOURCE_DIR}/mcts_tree
    ${PROJECT_SOURCE_DIR}/tree_node
    ${PROJECT_SOURCE_DIR}/mcts_in_narrow_meeting
    ${PROJECT_SOURCE_DIR}/include
)

# 添加源文件列表
set(SOURCE_FILES
    behavior_planner.cc
)

# 添加可执行文件
add_executable(VehicleExecutable ${SOURCE_FILES})