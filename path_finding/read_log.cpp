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

#include <SFML/Graphics.hpp>
#include "pathfinder.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::slam;


int main(int argc, char* argv[]) {
    if (argc < 4)
    {
        puts("Not enough arguments.");
        return -1;
    }

    ifstream laserLog, robotLog;
    string laserLine, robotLine;
    CConfigFile iniFile(argv[3]); // configurations file
    double accumX = 0.0, accumY = 0.0, accumPhi = 0.0;

    // pathfinding
    int resolution = 4;
    PathFinder pathFinder(resolution);
    deque<TPoint2D> path;


    // Load configurations
    CMetricMapBuilderICP icp_slam;
    icp_slam.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
    icp_slam.ICP_params.loadFromConfigFile(iniFile, "ICP");
    icp_slam.initialize();

    laserLog.open(argv[1]); // log of laser scan
    robotLog.open(argv[2]); // log of robot odometer

    sf::RenderWindow window(sf::VideoMode(800, 600), "bam!");
    sf::Texture texture;
    texture.create(800, 600);
    Matrix<sf::Color> pixels(600, 800);
    sf::Sprite sprite(texture);
    window.setVerticalSyncEnabled(true);
    bool paused = false;

    sf::Clock clock;
    unsigned frameCount = 0;
    while (laserLog.good() && window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                window.close();
                break;
            }
        }

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
        CPose3D curPosEst = icp_slam.getCurrentPoseEstimation()->getMeanVal();
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
        bool pathFound = true;
        pathFinder.update(*gridMap);
        pathFound = pathFinder.findPath(TPoint2D(gridRobX, gridRobY), TPoint2D(890, 500), path);
        printf("pathFound: %d\tpath length: %d\n", pathFound, path.size());


        // windows drawing
        window.clear(sf::Color::White);
        sf::View view;
        view.setSize(800, 600);
        view.setCenter(gridRobX, gridRobY);
        window.setView(view);

        // draw the grayscale probability map
        int yStart = max(0, gridRobY - 300);
        int yEnd = min((int)gridMap->getSizeY(), gridRobY + 300);
        int xStart = max(0, gridRobX - 400);
        int xEnd = min((int)gridMap->getSizeX(), gridRobX + 400);
        for (int y = yStart; y < yEnd; ++y)
        {
            for (int x = xStart; x < xEnd; ++x)
            {
                sf::Color &color = pixels(y-yStart, x-xStart);
                sf::Uint8 col = gridMap->getCell(x, y) * 255;
                color.r = col;
                color.g = col;
                color.b = col;
            }
        }
        texture.update((sf::Uint8*)pixels.getData());
        sprite.setPosition(xStart, yStart);
        window.draw(sprite);

        // draw the robot's position
        sf::CircleShape circle(5);
        circle.setPosition(gridRobX-resolution/2, gridRobY-resolution/2);
        circle.setOutlineColor(sf::Color::Red);
        circle.setFillColor(sf::Color::Red);
        window.draw(circle);

        // draw the path
        std::vector<sf::Vertex> verticies;
        verticies.resize(path.size() + 1);
        verticies[0].position.x = (gridRobX / resolution) * resolution + resolution/2;
        verticies[0].position.y = (gridRobY / resolution) * resolution + resolution/2;
        verticies[0].color = sf::Color::Blue;
        for (unsigned i = 0; i < path.size(); ++i)
        {
            verticies[i+1].position.x = path[i].x * resolution + resolution / 2;
            verticies[i+1].position.y = path[i].y * resolution + resolution / 2;
            verticies[i+1].color = sf::Color::Blue;
        }
        window.draw(&verticies[0], verticies.size(), sf::LinesStrip); 
        
        // draw the grid representation (only the occupied cells)
        /*
        sf::Color col = sf::Color::Yellow;
        col.a = 128;
        for (unsigned y = max(0, (gridRobY-400)/resolution); y < min<int>(pathFinder.occupancyGrid.height(), (gridRobY+400)/resolution); ++y)
            for (unsigned x = max(0, (gridRobX-400)/resolution); x < min<int>(pathFinder.occupancyGrid.width(), (gridRobX+400)/resolution); ++x)
            {
                if (!pathFinder.occupancyGrid(y, x)) continue;

                int xx = x * resolution;
                int yy = y * resolution;
                sf::RectangleShape rect;
                rect.setPosition(xx, yy);
                rect.setSize(sf::Vector2f(resolution, resolution));
                rect.setFillColor(col);
                window.draw(rect);
            }
            */

        window.display();

        frameCount++;
        if (clock.getElapsedTime().asSeconds() >= 1.0)
        {
            char timestr[16];
            sprintf(timestr, "%d fps", frameCount);
            window.setTitle(timestr);

            clock.restart();
            frameCount = 0;
        }
    }

    return 0;
}
