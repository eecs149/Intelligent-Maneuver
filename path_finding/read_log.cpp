#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

#include <mrpt/utils.h>
#include <mrpt/obs.h>
#include <mrpt/slam.h>
#include <mrpt/poses.h> 
using namespace std;
using namespace boost;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::slam;

double diffclock(clock_t, clock_t);

int main(int argc, char* argv[]) {
    ifstream laserLog, robotLog;
    string laserLine, robotLine;
    CConfigFile iniFile(argv[3]); // configurations file
    CMetricMapBuilderICP icp_slam;
    size_t rawlogEntry = 0;
    bool end = false;
    double accumX = 0.0, accumY = 0.0, accumPhi = 0.0;

    // Load configurations
    icp_slam.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
    icp_slam.ICP_params.loadFromConfigFile(iniFile, "ICP");
    icp_slam.initialize();

    laserLog.open(argv[1]); // log of laser scan
    robotLog.open(argv[2]); // log of robot odometer

    char_separator<char> sep(" ");
    while (laserLog.good()) {
        getline(laserLog, laserLine);
        getline(robotLog, robotLine);

        // Extract the laser scan info and convert it into a range scan observation to feed into icp-slam
        CObservation2DRangeScanPtr obs = CObservation2DRangeScan::Create();
        tokenizer< char_separator<char> > tokensLaser(laserLine, sep);
        BOOST_FOREACH(string t, tokensLaser)
        {
            obs->scan.push_back(atof(t.c_str()));
            obs->validRange.push_back(1);
        }
        icp_slam.processObservation(obs);

        // Extract the odometer values and convert it into an observation to feed into icp-slam
        int count = 0;
        tokenizer< char_separator<char> > tokensRobot(robotLine, sep);
        BOOST_FOREACH(string t, tokensRobot)
        {
            if (count == 0) {
                accumX += atof(t.c_str());
                count++;
            }
            else if (count == 1) {
                accumY += atof(t.c_str());
                count++;
            }
            else if (count == 2) {
                accumPhi += atof(t.c_str());
                count++;
            }

        }
        // Need the ABSOLUTE odometer readings, meaning the accumulated values
        CPose2DPtr rawOdo(new CPose2D(accumX, accumY, accumPhi));
        CObservationOdometryPtr obs2 = CObservationOdometry::Create();
        obs2->odometry = *rawOdo;
        obs2->hasEncodersInfo = false;
        obs2->hasVelocities = false;
        icp_slam.processObservation(obs2);

        // Extract current estimates
        // NOTE: coordinate points are in METERS
        // First get the grid map
        CMultiMetricMap* curMapEst = icp_slam.getCurrentlyBuiltMetricMap();
        /* Grid representation of the current map.
         * Grid X Range: [0, getSizeX()]
         * Grid Y Range: [0, getSizeY()]
         * Convert from coordinate to grid loc: x2idx(float), y2idx(float)
         * Convert from grid loc to coordinate: idx2x(int), idx2y(float)
         * Use getCell(int x, int y) to tell if the cell is empty or not. A real value [0,1], which is the probablity that cell is empty
         */
        COccupancyGridMap2DPtr gridMap = curMapEst->m_gridMaps[0];

        // Get the position estimates
        CPose3DPDFPtr curPosPDF = icp_slam.getCurrentPoseEstimation();
        CPose3D curPosEst;
        curPosPDF->getMean(curPosEst);
        // (estimated) X,Y coordinates of the robot, and robot yaw angle (direction)
        float robx = curPosEst.x();
        float roby = curPosEst.y();
        float robphi = curPosEst.yaw();
        // Convert real coordinate to grid coordinate points
        int gridRobX = gridMap->x2idx(robx);
        int gridRobY = gridMap->y2idx(roby);

        // Perform path finding
    }
    // Save map estimate to temp.png
    icp_slam.saveCurrentEstimationToImage("temp");
}
