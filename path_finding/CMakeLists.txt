PROJECT(mrpt_example1)

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
ADD_EXECUTABLE(mrpt_example1 read_log.cpp pathfinder.cpp)
TARGET_LINK_LIBRARIES(mrpt_example1 ${MRPT_LIBS} sfml-graphics sfml-window sfml-network sfml-system)

# Set optimized building:
SET(CMAKE_CXX_FLAGS "-g")
