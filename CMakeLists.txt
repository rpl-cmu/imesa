project(imesa CXX C)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)

# Dependencies 
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})


message(STATUS "================  iMESA  ======================")
# Build the iMESA library
file(GLOB_RECURSE imesa_srcs "*.cpp" "*.h")
add_library(imesa ${imesa_srcs})
target_link_libraries(imesa gtsam)
target_include_directories(imesa PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")