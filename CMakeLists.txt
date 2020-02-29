cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(PCL_Registration)
 
find_package(PCL 1.7 REQUIRED)
find_package(Threads)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB PCL_Registration_SRC
    "src/generate_model.cpp"
    "src/registration.cpp"
    "src/process.cpp"
)

file(GLOB PCL_openni_SRC
    "src/openni_capture.cpp"
)

add_executable(capture ${PCL_openni_SRC})
add_executable(generate_model ${PCL_Registration_SRC})
 
target_link_libraries (capture ${PCL_LIBRARIES})
target_link_libraries (generate_model ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})