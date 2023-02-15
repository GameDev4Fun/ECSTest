cmake_minimum_required(VERSION 3.24)
project(ECSTest)

add_subdirectory(externals)

set(CMAKE_CXX_STANDARD 17)

add_executable(ECSTest main.cpp)
target_link_libraries(ECSTest PUBLIC EnTT::EnTT)
target_link_libraries(ECSTest PUBLIC fmt::fmt)
