# 设置最低 CMake 版本要求
cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(VehicleProject)
set(CMAKE_BUILD_TYPE Debug)
# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -fno-omit-frame-pointer") # 在CMakeLists.txt中添加
add_compile_options(-fsanitize=address)
add_link_options(-fsanitize=address)


# 设置包含目录
include_directories(
    ${PROJECT_SOURCE_DIR}/vehicle_state
    ${PROJECT_SOURCE_DIR}/common
    ${PROJECT_SOURCE_DIR}/mcts_base
    ${PROJECT_SOURCE_DIR}/mcts_tree
    ${PROJECT_SOURCE_DIR}/tree_node
    ${PROJECT_SOURCE_DIR}/mcts_in_narrow_meeting
    ${PROJECT_SOURCE_DIR}/behavior_planner
    ${PROJECT_SOURCE_DIR}/conf


)

# 添加所有源文件（包含 planner 子目录下的文件）
file(GLOB SOURCE_FILES 
    "${PROJECT_SOURCE_DIR}/bp_main.cc" 
    "${PROJECT_SOURCE_DIR}/behavior_planner/*.cc"
    "${PROJECT_SOURCE_DIR}/vehicle_state/*.cc"
    "${PROJECT_SOURCE_DIR}/common/*.cc"
    "${PROJECT_SOURCE_DIR}/mcts_base/*.cc"
    "${PROJECT_SOURCE_DIR}/mcts_tree/*.cc"
    "${PROJECT_SOURCE_DIR}/tree_node/*.cc"
    "${PROJECT_SOURCE_DIR}/mcts_in_narrow_meeting/*.cc"
    "${PROJECT_SOURCE_DIR}/conf/*.cc"
)
# 添加可执行文件
add_executable(VehicleExecutable ${SOURCE_FILES})