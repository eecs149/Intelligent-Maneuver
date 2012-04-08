#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>
#include <queue>

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



struct Node
{
    int x;
    int y;
    int g;
    int h;
    int f;
    int parentX;
    int parentY;

    Node(int x, int y, int g, int h, int parentX, int parentY):
    x(x), y(y), g(g), h(h), f(g+h), parentX(parentX), parentY(parentY)
    { }

    
    bool operator<(const Node& other) const
    {
        return f < other.f;
    }
};

class PathFinder
{
public:
    PathFinder(int resolution):
    resolution(resolution), gridWidth(0), gridHeight(0)
    {
    }

    bool findPath(const TPoint2D& start, const TPoint2D& end, deque<TPoint2D>& path)
    {
        int endX = end.x / resolution;
        int endY = end.y / resolution;

        vector<int> parentX(gridWidth*gridHeight, -1);
        vector<int> parentY(gridWidth*gridHeight, -1);
        priority_queue<Node> fringe;
        fringe.push(Node(start.x/resolution, start.y/resolution, 0, 0, -1, -1));

        while (fringe.size())
        {
            Node pt = fringe.top();
            fringe.pop();
            if (parentX[pt.y*gridWidth+pt.x] >= 0)
                continue;
            parentX[pt.y*gridWidth+pt.x] = pt.parentX;
            parentY[pt.y*gridWidth+pt.x] = pt.parentY;

            if (pt.x == endX && pt.y == endY)
            {
                int x = pt.x;
                int y = pt.y;
                while (parentX[y*gridWidth+x] >= 0)
                {
                    path.push_front(TPoint2D(x, y));
                    int oldX = x;
                    x = parentX[y*gridWidth+x];
                    y = parentY[y*gridWidth+oldX];
                }
                return true;
            }

            if (pt.x > 0 && !occupancyGrid[pt.y*gridWidth+pt.x-1])
                fringe.push(Node(pt.x-1, pt.y, pt.g+1, abs(endX-pt.x)+abs(endY-pt.y), pt.x, pt.y));
            if (pt.y > 0 && !occupancyGrid[(pt.y-1)*gridWidth+pt.x])
                fringe.push(Node(pt.x, pt.y-1, pt.g+1, abs(endX-pt.x)+abs(endY-pt.y), pt.x, pt.y));
            if (pt.x+1 < gridWidth && !occupancyGrid[pt.y*gridWidth+pt.x+1])
                fringe.push(Node(pt.x+1, pt.y, pt.g+1, abs(endX-pt.x)+abs(endY-pt.y), pt.x, pt.y));
            if (pt.y+1 < gridHeight && !occupancyGrid[(pt.y+1)*gridWidth+pt.x])
                fringe.push(Node(pt.x, pt.y+1, pt.g+1, abs(endX-pt.x)+abs(endY-pt.y), pt.x, pt.y));
        }

        return false;
    }

    void update(const COccupancyGridMap2D& gridMap)
    {
        if (gridMap.getSizeX() > gridWidth*resolution || gridMap.getSizeY() > gridHeight*resolution)
        {
            int newGridHeight = gridMap.getSizeY() / resolution;
            int newGridWidth = gridMap.getSizeX() / resolution;
            vector<unsigned char> newOccupancyMap(newGridHeight * newGridWidth, 0);
            for (int y = 0; y < gridHeight; ++y)
                for (int x = 0; x < gridWidth; ++x)
                    newOccupancyMap[y*newGridWidth+x] = occupancyGrid[y*gridWidth+x];
            gridWidth = newGridWidth;
            gridHeight = newGridHeight;
            occupancyGrid.swap(newOccupancyMap);
        }

        for (int y = 0; y < gridHeight; ++y)
        {
            for (int x = 0; x < gridWidth; ++x)
            {
                unsigned char& val = occupancyGrid[y*gridWidth+x];
                
                if (val) continue;
                for (int yy = 0; yy < resolution; ++yy)
                    for (int xx = 0; xx < resolution; ++xx)
                        val |= gridMap.getCell(y*resolution+yy, x*resolution+xx) > 0.5;
            }
        }
    }

    bool checkPathValid(deque<TPoint2D>& path)
    {
        for (int i = 0; i < path.size(); ++i)
        {
            int x = path[i].x / resolution;
            int y = path[i].y / resolution;

            if (occupancyGrid[y*gridWidth+x])
                return false;
        }
        return true;
    }

private:
    int gridHeight;
    int gridWidth;
    int resolution;
    std::vector<unsigned char> occupancyGrid;
};

int main(int argc, char* argv[]) {
    if (argc < 4)
    {
        puts("Not enough arguments.");
        return -1;
    }

    ifstream laserLog, robotLog;
    string laserLine, robotLine;
    CConfigFile iniFile(argv[3]); // configurations file
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
    CMetricMapBuilderICP icp_slam;
    icp_slam.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
    icp_slam.ICP_params.loadFromConfigFile(iniFile, "ICP");
    icp_slam.initialize();

    laserLog.open(argv[1]); // log of laser scan
    robotLog.open(argv[2]); // log of robot odometer

    // pathfinding
    CPathPlanningCircularRobot pathPlanner;

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
        double robx = curPosEst.x();
        double roby = curPosEst.y();
        double robphi = curPosEst.yaw();
        // Convert real coordinate to grid coordinate points
        int gridRobX = gridMap->x2idx(robx);
        int gridRobY = gridMap->y2idx(roby);


        cout << "robot location: " << gridRobX << ' ' << gridRobY << '\n';
        cout << "gridMap size: " << gridMap->getSizeX() << ' ' << gridMap->getSizeY() << '\n';

        // Perform path finding
        deque<TPoint2D> path;
        bool notFound;
        CPose2D origin(robx, roby, robphi);
        CPose2D target(gridMap->idx2x(890), gridMap->idx2y(270), robphi);
        pathPlanner.computePath(*gridMap, origin, target, path, notFound);
        cout << "path: " << notFound << ' ' << path.size();
        if (!notFound)
        {
            cout << " [";
            for (int i = 0; i < path.size(); ++i)
                cout << path[i].asString() << ' ';
            cout << ']';
        }
        cout << '\n';




        // Save a 3D scene view of the mapping process:
        if (win3D)
        {
            COpenGLScenePtr scene = COpenGLScenePtr( new COpenGLScene() );
        
            COpenGLViewportPtr view = scene->getViewport("main");
            ASSERT_(view);
        
            COpenGLViewportPtr view_map = scene->createViewport("mini-map");
            view_map->setBorderSize(2);
            view_map->setViewportPosition(0.01,0.01,0.35,0.35);
            view_map->setTransparent(false);
        
            {
                CCamera &cam = view_map->getCamera();
                cam.setAzimuthDegrees(-90);
                cam.setElevationDegrees(90);
                cam.setPointingAt(curPosEst.x(), curPosEst.y(), curPosEst.z());
                cam.setZoomDistance(20);
                cam.setOrthogonal();
            }
        
            // The ground:
            CGridPlaneXYPtr groundPlane = CGridPlaneXY::Create(-200,200,-200,200,0,5);
            groundPlane->setColor(0.4,0.4,0.4);
            view->insert( groundPlane );
            view_map->insert( CRenderizablePtr( groundPlane) ); // A copy
        
            // The camera pointing to the current robot pose:
            scene->enableFollowCamera(true);
        
            {
                CCamera &cam = view_map->getCamera();
                cam.setAzimuthDegrees(-45);
                cam.setElevationDegrees(45);
                cam.setPointingAt(curPosEst.x(),curPosEst.y(),curPosEst.z());
            }
        
            // The maps:
            {
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
            }
        
            // Show 3D?
            COpenGLScenePtr &ptrScene = win3D->get3DSceneAndLock();
            ptrScene = scene;
        
            win3D->unlockAccess3DScene();
        
            // Move camera:
            win3D->setCameraPointingToPoint( curPosEst.x(), curPosEst.y(), curPosEst.z() );
        
            // Update:
            win3D->forceRepaint();
        }
        
    }
    // Save map estimate to temp.png
//    icp_slam.saveCurrentEstimationToImage("temp");

    return 0;
}
