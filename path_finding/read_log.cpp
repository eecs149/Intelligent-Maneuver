#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>

#include <mrpt/utils.h>
#include <mrpt/obs.h>
#include <mrpt/slam.h>
#include <mrpt/poses.h> 
#include <mrpt/opengl.h>
#include <mrpt/gui.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;

int main(int argc, char* argv[]) {
    ifstream laserLog, robotLog;
    string laserLine, robotLine;
    CConfigFile iniFile(argv[3]); // configurations file
    CMetricMapBuilderICP icp_slam;
    size_t rawlogEntry = 0;
    bool end = false;
    double accumX = 0.0, accumY = 0.0, accumPhi = 0.0;

    // Graphics stuff
    // Create 3D window if requested:
    CDisplayWindow3D*win3D = NULL;
    #if MRPT_HAS_WXWIDGETS
    win3D = new CDisplayWindow3D("ICP-SLAM @ MRPT C++ Library (C) 2004-2008", 600, 500);
    win3D->setCameraZoom(20);
    win3D->setCameraAzimuthDeg(-45);
    #endif

    // Load configurations
    icp_slam.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
    icp_slam.ICP_params.loadFromConfigFile(iniFile, "ICP");
    icp_slam.initialize();

    laserLog.open(argv[1]); // log of laser scan
    robotLog.open(argv[2]); // log of robot odometer

    while (laserLog.good()) {
        double f;
        getline(laserLog, laserLine);
        getline(robotLog, robotLine);

        // Extract the laser scan info and convert it into a range scan observation to feed into icp-slam
        CObservation2DRangeScanPtr obs = CObservation2DRangeScan::Create();
        stringstream ssLaser(laserLine);
        while (ssLaser >> f)
        {
            obs->scan.push_back(f);
            obs->validRange.push_back(1);
        }
        icp_slam.processObservation(obs);

        // Extract the odometer values and convert it into an observation to feed into icp-slam
        stringstream ssRobot(robotLine);
        ssRobot >> f;
        accumX += f;
        ssRobot >> f;
        accumY += f;
        ssRobot >> f;
        accumPhi += f;
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

        // Save a 3D scene view of the mapping process:
        if (win3D!=NULL)
        {
            COpenGLScenePtr scene = COpenGLScenePtr( new COpenGLScene() );
        
            COpenGLViewportPtr view = scene->getViewport("main");
            ASSERT_(view);
        
            COpenGLViewportPtr view_map = scene->createViewport("mini-map");
            view_map->setBorderSize(2);
            view_map->setViewportPosition(0.01,0.01,0.35,0.35);
            view_map->setTransparent(false);
        
            CCamera &cam = view_map->getCamera();
            cam.setAzimuthDegrees(-90);
            cam.setElevationDegrees(90);
            cam.setPointingAt(curPosEst.x(),curPosEst.y(),curPosEst.z());
            cam.setZoomDistance(20);
            cam.setOrthogonal();
        
            // The ground:
            CGridPlaneXYPtr groundPlane = CGridPlaneXY::Create(-200,200,-200,200,0,5);
            groundPlane->setColor(0.4,0.4,0.4);
            view->insert( groundPlane );
            view_map->insert( CRenderizablePtr( groundPlane) ); // A copy
        
            // The camera pointing to the current robot pose:
            scene->enableFollowCamera(true);
        
            cam = view_map->getCamera();
            cam.setAzimuthDegrees(-45);
            cam.setElevationDegrees(45);
            cam.setPointingAt(curPosEst.x(),curPosEst.y(),curPosEst.z());
        
            // The maps:
            CSetOfObjectsPtr obj = CSetOfObjects::Create();
            curMapEst->getAs3DObject( obj );
            view->insert(obj);
        
            // Only the point map:
            CSetOfObjectsPtr ptsMap = CSetOfObjects::Create();
            if (curMapEst->m_pointsMaps.size())
            {
                curMapEst->m_pointsMaps[0]->getAs3DObject(ptsMap);
                view_map->insert( ptsMap );
            }
        
            // Show 3D?
            if (win3D)
            {
                COpenGLScenePtr &ptrScene = win3D->get3DSceneAndLock();
                ptrScene = scene;
        
                win3D->unlockAccess3DScene();
        
                // Move camera:
                win3D->setCameraPointingToPoint( curPosEst.x(), curPosEst.y(), curPosEst.z() );
        
                // Update:
                win3D->forceRepaint();
            }
        }
        
    }
    // Save map estimate to temp.png
    icp_slam.saveCurrentEstimationToImage("temp");
}
