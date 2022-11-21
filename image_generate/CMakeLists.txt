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

add_executable(image_generate src/ShmReader2RGB.cpp)

target_link_libraries(image_generate ${OpenCV_LIBS} ${THREADLIB})


#aruco_create_board
#INSTALL(image_generater     RUNTIME DESTINATION bin)