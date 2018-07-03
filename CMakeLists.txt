cmake_minimum_required(VERSION 3.10)
project(MudBoxer)

set(CMAKE_CXX_STANDARD 11)
include_directories(/opt/ros/melodic/include/)
link_libraries(/opt/ros/melodic/lib/libroscpp.so)
add_library(MudBoxer SHARED mudboxer.cpp)
target_link_libraries(MudBoxer Crypt)
INSTALL(TARGETS MudBoxer
        LIBRARY DESTINATION lib
        )

