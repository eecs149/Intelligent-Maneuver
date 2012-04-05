This is the repository of the "Intelligent Maneuver" team of EECS 149, Spring 2012.

Our team comprises of:
Hui Peng Hu
Lily Lin
Constance Lu
Jonathan Wong
Yoriyasu Yano

Requirements:
- ARDroneSDK (https://projects.ardrone.org/)
-- Please define the following two environment variables to compile the source code in the repo via Build/Makefile
    ARDRONE_SDK_PATH = Should point to ARDroneLib in the ARDrone SDK
    ARDRONE_TARGET_DIR = Should point to where you want the compiled binary
- MRPT (http://mrpt.org/)
- Boost C++ library (http://www.boost.org/)

Directories:
- ARDroneCode - Includes relevant code that works with the ARDrone
- path_finding - Includes a framework code that uses a past log of laser scan data (provided by MRPT) to run icp-slam and exposes the estimated map at each step. Can be used to test timing of icp-slam, and path-finding
