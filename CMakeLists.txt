cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

list(APPEND EXTRA_COMPONENT_DIRS src)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
# set( CMAKE_CXX_STANDARD 23 REQUIRED )
# add_compile_options(-Wall -Wextra -Wformat -Wformat-security -Wpedantic -pedantic-errors -Werror -fexceptions)


project(esp32oscilloscope)
