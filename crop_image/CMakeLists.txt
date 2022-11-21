set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
SET(THREADLIB "pthread")
ENDIF()


include_directories(
    include
    src
    )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(crop_image src/crop_image.cpp)

target_link_libraries(crop_image ${OpenCV_LIBS} ${THREADLIB})

#add_executable(aruco_dummy aruco_dummy.cpp)
#target_link_libraries(aruco_dummy             aruco opencv_calib3d opencv_highgui ${THREADLIB})

#aruco_create_board
INSTALL(TARGETS crop_image      RUNTIME DESTINATION bin)