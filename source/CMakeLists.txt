cmake_minimum_required(VERSION 2.8)
set(EXE_NAME bullet)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

project(${EXE_NAME})
link_libraries(
BulletDynamics BulletCollision LinearMath
)

add_executable(${EXE_NAME} 
  Main.cpp
  )

include_directories("./lib" "/usr/local/include" "/usr/local/include/bullet")
