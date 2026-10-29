# 先设置 CMake 版本和组件目录，再加载 ESP-IDF 并声明工程。
cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/component")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Strain_sensor)
