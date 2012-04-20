#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>
#include <queue>

// c libs
#include <cstring>
#include <cstdlib>

#include <mrpt/utils.h>
#include <mrpt/obs.h>
#include <mrpt/slam.h>
#include <mrpt/poses.h> 

#include <SFML/Graphics.hpp>

#include "memdb.h"
#include "pathfinder.h"
#include "feedback.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::slam;

int main(int argc, char* argv[]) {

    if (argc < 2)
    {
        puts("Not enough arguments.");
        return -1;
    }

    CConfigFile iniFile(argv[1]); // configurations file
    double accumX = 0.0, accumY = 0.0, accumPhi = 0.0;

    // Load configurations
    CMetricMapBuilderICP icp_slam;
    icp_slam.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
    icp_slam.ICP_params.loadFromConfigFile(iniFile, "ICP");
    icp_slam.initialize();

    // pathfinding
    int resolution = 4;
    PathFinder pathFinder(resolution);
    deque<TPoint2D> path;

    // connect to memdb
    db_t db = db_connect("8765");

    sf::RenderWindow window(sf::VideoMode(800, 600), "bam!");
    window.setVerticalSyncEnabled(true);
    bool paused = false;

    while (1) { // TODO: What will be our terminal case? target location reached?
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

        // Extract the laser scan info and convert it into a range scan observation to feed into icp-slam
        CObservation2DRangeScanPtr obs = CObservation2DRangeScan::Create();
        // Need to define 2 values
        // 1.) scan: a vector of floats signalling the distances. Each element is a degree
        // 2.) validRange: a vector of ints where 1 signals the reading is good and 0 means its bad (and won't be used)
        char buffer[1024];
        while (db_tryget(db, "lidar", buffer, sizeof(buffer)) != -1)
        {
            obs->scan.push_back(atof(buffer));
            obs->validRange.push_back(1);
        }
        icp_slam.processObservation(obs);

        // TODO: Determine and compensate for ardrone drift
        // Need the ABSOLUTE odometer readings, meaning the accumulated values

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
        pathFinder.update(*gridMap, gridRobX - 100, gridRobY - 100, gridRobX + 100, gridRobY + 100);
        bool pathFound = true;
        pathFound = pathFinder.findPath(TPoint2D(gridRobX, gridRobY), TPoint2D(890, 270), path);
        printf("pathFound: %d\tpath length: %lu\n", pathFound, path.size());

        //TODO: convert path to a vector (distance + direction)
        //use vector_t struct in feedback.h
        
        //TODO: feed vector to feedback loop
        //  need to translate commands => send using something like _db_send
        //  don't uncomment this yet
        /*
        initialize_feedback(time);  //TODO: need variable for current time (unsigned float)
        DroneMovement state = process_feedback(vector, time);  
        switch (state) {
        case FLY_FORWARD:    // positive x
            front rotor: omega - delta_b
            rear rotor:  omega + delta_a
            break;
        case FLY_BACKWARD:   // negative x
            front rotor: omega + delta a
            rear rotor:  omega - delta_b
            break;
        case FLY_LEFT:       // positive y
            left rotor:  omega - delta_b
            right rotor: omega + delta_a
            break;
        case FLY_RIGHT:      // negative y
            left rotor:  omega + delta_a
            right rotor: omega - delta_b
            break;
        case FLY_UP:         // positive z
            all rotors:  omega + delta_a
            break;
        case FLY_DOWN:       // negative z
            all rotors: omega - delta_b
            break;
        case TURN_LEFT:      // positive angle
            front rotor: omega - delta_b
            rear rotor:  omega - delta_b
            left rotor:  omega + delta_a
            right rotor: omega + delta_a
            break;
        case TURN_RIGHT:     // negative angle
            front rotor: omega + delta_a
            rear rotor:  omega + delta_a
            left rotor:  omega - delta_b
            right rotor: omega - delta_b
            break;
        default:             // default is hover
            break;
        }*/ 

        // windows drawing
        window.clear(sf::Color::White);
        sf::View view;
        view.setSize(800, 600);
        view.setCenter(gridRobX, gridRobY);
        window.setView(view);

        // draw the grayscale probability map
        sf::Image image;
        image.create(gridMap->getSizeX(), gridMap->getSizeY());
        for (unsigned y = 0; y < gridMap->getSizeY(); ++y)
            for (unsigned x = 0; x < gridMap->getSizeX(); ++x)
            {
                sf::Uint8 col = gridMap->getCell(x, y) * 255;
                image.setPixel(x, y, sf::Color(col, col, col));
            }
        sf::Texture texture;
        texture.create(gridMap->getSizeX(), gridMap->getSizeY());
        texture.update(image);
        window.draw(sf::Sprite(texture));

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
        sf::Color col = sf::Color::Yellow;
        col.a = 128;
        for (unsigned y = 0; y < pathFinder.occupancyGrid.height(); ++y)
            for (unsigned x = 0; x < pathFinder.occupancyGrid.width(); ++x)
            {
                if (!pathFinder.occupancyGrid(y, x)) continue;
                sf::RectangleShape rect;
                rect.setPosition(x * resolution, y * resolution);
                rect.setSize(sf::Vector2f(resolution, resolution));
                rect.setFillColor(col);
                window.draw(rect);
            }


        window.display();
    }

    return 0;
}
