cmake_minimum_required(VERSION 3.21)
project(VideoProcessing)

if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
	message("Building in release mode")
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	message("Building in debug mode")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(track "Source/main.cpp" "Source/CameraEmulator.cpp" "Source/Tracker.cpp" "Source/Utils/OutData.cpp")

target_include_directories(track PRIVATE "Source")
target_include_directories(track PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(track PRIVATE ${OpenCV_LIBS})

message("OpenCV libraries: ${OpenCV_LIBS}")