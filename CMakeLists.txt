cmake_minimum_required(VERSION 3.1)
project(CPP_ROBOTICS)

set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#############################
#Dependencies
#############################

#find_package(Eigen REQUIRED) # Eigen 3.3.7
find_package(OpenCV REQUIRED)
# find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

#include_directories(${EIGEN_INCLUDE_DIR})
include_directories(include)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# Create library
## Declare a cpp library
add_library(_cpp_robotics_core
    # Localization
    # src/localization/CODE.cpp

    # Planning
    src/planning/rrt.cpp

    # SLAM
    # src/slam/CODE.cpp

    # Other
    src/other/icp.cpp
)


# Localization
add_executable(rrt examples/planning/rrt_example.cpp)
target_link_libraries(rrt ${OpenCV_LIBS} _cpp_robotics_core)
# target_link_libraries(rrt ${OpenCV_LIBS} ${Boost_LIBRARIES} _cpp_robotics_core)

# Planning

# SLAM

# Other
add_executable(icp examples/other/icp_example.cpp)
target_link_libraries(icp ${OpenCV_LIBS} _cpp_robotics_core)
