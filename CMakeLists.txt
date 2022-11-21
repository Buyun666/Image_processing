cmake_minimum_required(VERSION 3.0)
project(aruco VERSION "3.1.15" LANGUAGES CXX)
set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11





add_subdirectory(de_distor)
add_subdirectory(image_generate)
add_subdirectory(crop_image)


# Package Generator  #######################################################
IF(BUILD_DEBPACKAGE)
include(cmake/cpack.cmake)
ENDIF()