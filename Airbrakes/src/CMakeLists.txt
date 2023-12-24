cmake_minimum_required(VERSION 3.16)
project(Airbrakes)

file(GLOB SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/*.hpp
    ${CMAKE_CURRENT_LIST_DIR}/*.cpp
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(Airbrakes ${SOURCES})
target_link_libraries(Airbrakes Eigen3::Eigen)