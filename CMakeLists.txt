cmake_minimum_required(VERSION 3.2)

project(csc476-simulation-framework)

#Important CMAKE stuff
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#VTK
set(VTK_DIR "usr/local/lib")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

#Eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(usr/local/include/vtk-8.2)

#include source code
file(GLOB H_FILES ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB CPP_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(csc476-simulation-framework main.cpp ${H_FILES} ${CPP_FILES})
target_link_libraries(csc476-simulation-framework ${VTK_LIBRARIES} Eigen3::Eigen)
