cmake_minimum_required(VERSION 3.5)

project(botop_ws)
add_library(rai SHARED IMPORTED)
set_target_properties(rai PROPERTIES IMPORTED_LOCATION /home/jiayun/git/botop/build/librai.so)

#target_include_directories()
include_directories("/home/jiayun/git/botop/rai/rai")
include_directories("/home/jiayun/git/botop/src")

add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries(${PROJECT_NAME} rai)
