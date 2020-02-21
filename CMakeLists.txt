cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(PCL_Registration)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB PCL_Registration_SRC
    "src/registration.cpp"
)

file(GLOB PCL_openni_SRC
    "src/openni_capture.cpp"
)

add_executable(capture ${PCL_openni_SRC})
add_executable(registration ${PCL_Registration_SRC})
 
target_link_libraries (registration ${PCL_LIBRARIES})
target_link_libraries (capture ${PCL_LIBRARIES})