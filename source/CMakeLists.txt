cmake_minimum_required(VERSION 2.8)

set(EXE_NAME playground)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(PLAYGROUND_INCLUDE_DIRS "")
set(PLAYGROUND_LIBRARIES "")

project(${EXE_NAME})

find_package(Bullet 2.82 REQUIRED)
list(APPEND PLAYGROUND_INCLUDE_DIRS ${BULLET_INCLUDE_DIR})
list(APPEND PLAYGROUND_LIBRARIES ${BULLET_LIBRARIES})

include_directories(${PLAYGROUND_INCLUDE_DIRS})

add_executable(${EXE_NAME}
  Main.cpp
)

target_link_libraries(${EXE_NAME} ${PLAYGROUND_LIBRARIES})
