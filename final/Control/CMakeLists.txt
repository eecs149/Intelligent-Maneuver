PROJECT(control)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only 
#    need to specify the top-most libraries your code depends on.
# --------------------------------------------------------------------------
FIND_PACKAGE( MRPT REQUIRED base obs slam gui )    # WARNING: Add all the MRPT libs used by your program: "gui", "obs", "slam",etc. 

# Declare the target (an executable)
ADD_EXECUTABLE(control control.cpp pathfinder.cpp feedback.cpp)
TARGET_LINK_LIBRARIES(control ${MRPT_LIBS} sfml-graphics sfml-window sfml-system)

IF(WIN32)
    TARGET_LINK_LIBRARIES(control ws2_32)
ENDIF(WIN32)

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
