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


enum State
{
    READ_LIDAR,
    PLAN,
    MOVE_FORWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    TURNING
};

int main(int argc, char* argv[]) {

    if (argc < 2)
    {
        puts("Not enough arguments.");
        return -1;
    }

    // connect to memdb
    db_t db = db_connect("8765");
    if (db == -1)
    {
        puts("failed to connect to memdb.");
        return -1;
    }
    
    CConfigFile iniFile(argv[1]); // configurations file
    float accumX = 0.0f, accumY = 0.0f, accumPhi = 0.0f;

    // Load configurations
    CMetricMapBuilderICP icp_slam;
    icp_slam.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
    icp_slam.ICP_params.loadFromConfigFile(iniFile, "ICP");
    icp_slam.initialize();

    // pathfinding
    int resolution = 4;
    PathFinder pathFinder(resolution);
    deque<TPoint2D> path;

    // THE state of the main control
    State state = READ_LIDAR;

    sf::RenderWindow window(sf::VideoMode(800, 600), "2:00am");
    sf::Clock globalClock;
    window.setVerticalSyncEnabled(true);
    sf::Texture texture;
    texture.create(800, 600);
    Matrix<sf::Color> pixels(600, 800);
    sf::Sprite sprite(texture);
    sf::Clock fpsClock;
    int frameCount = 0;

    while (window.isOpen()) { // TODO: What will be our terminal case? target location reached?
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

        // read the navadata from the ardrone
        char buffer[1024];
        while (db_tryget(db, "navdata", buffer, sizeof(buffer)) != -1)
        {
            unsigned time;
            float vx, vy, vz, accelx, accely, accelz, gyrox, gyroy, gyroz;
            sscanf(buffer, "%u,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &time, &vx, &vy, &vz, &accelx, &accely, &accelz, &gyrox, &gyroy, &gyroz);
            
            // here goes the terrible part....
            accumX += vx;
            accumY += vy;
            accumPhi += gyroz; // TODO: is it gyroz

            // TODO: Determine and compensate for ardrone drift
            // Need the ABSOLUTE odometer readings, meaning the accumulated values

            CObservationOdometryPtr obs = CObservationOdometry::Create();
            obs->odometry = CPose2D(accumX, accumY, accumPhi);
            obs->hasEncodersInfo = false;
            obs->hasVelocities = false;
            icp_slam.processObservation(obs);
        }

        // get the map object and the grid representation of the map
        CMultiMetricMap* curMapEst = icp_slam.getCurrentlyBuiltMetricMap();
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


        // state machine actions
        if (state == READ_LIDAR)
        {
            // Extract the laser scan info and convert it into a range scan observation to feed into icp-slam
            CObservation2DRangeScanPtr obs = CObservation2DRangeScan::Create();
            // Need to define 2 values
            // 1.) scan: a vector of floats signalling the distances. Each element is a degree
            // 2.) validRange: a vector of ints where 1 signals the reading is good and 0 means its bad (and won't be used)
            while (db_tryget(db, "lidar", buffer, sizeof(buffer)) != -1)
            {
                obs->scan.push_back(atof(buffer));
                obs->validRange.push_back(1);
            }
            icp_slam.processObservation(obs);
        }


        else if (state == PLAN)
        {
            // Perform path finding
            pathFinder.update(*gridMap);
            bool pathFound = true;
            pathFound = pathFinder.findPath(TPoint2D(gridRobX, gridRobY), TPoint2D(890, 270), path);
            printf("pathFound: %d\tpath length: %lu\n", pathFound, path.size());

            //TODO: convert path to a vector (distance + direction)
            //use vector_t struct in feedback.h
            vector<vector_t> movement_vectors;
            for (unsigned i = 1; i < path.size(); ++i)
            {

            }
        }

        else if (state == MOVE_FORWARD || 
                 state == MOVE_LEFT || 
                 state == MOVE_RIGHT || 
                 state == TURNING)
        {
            //TODO: feed vector to feedback loop
            //  need to translate commands => send using something like _db_send
            //  don't uncomment this yet
            /*
            initialize_feedback(globalClock.getElapsedTime().asMilliseconds());
            DroneMovement state = process_feedback(vector, globalClock.getElapsedTime().asMilliseconds());
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
        }

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
        sf::Color col = sf::Color::Yellow;
        col.a = 128;
        for (unsigned y = 0; y < pathFinder.occupancyGrid.height(); ++y)
        {
            for (unsigned x = 0; x < pathFinder.occupancyGrid.width(); ++x)
            {
                if (!pathFinder.occupancyGrid(y, x)) continue;
                sf::RectangleShape rect;
                rect.setPosition(x * resolution, y * resolution);
                rect.setSize(sf::Vector2f(resolution, resolution));
                rect.setFillColor(col);
                window.draw(rect);
            }
        }


        window.display();

        
        frameCount++;
        if (fpsClock.getElapsedTime().asSeconds() >= 1.0f)
        {
            char windowTitle[32];
            sprintf(windowTitle, "%d fps", frameCount);
            window.setTitle(windowTitle);

            fpsClock.restart();
            frameCount = 0;
        }
    }

    return 0;
}
