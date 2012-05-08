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

#include "../memdb/memdb.h"
#include "pathfinder.h"
#include "feedback.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::slam;


enum State
{
    INIT_HOVER,
    HOVER,
    LAND,
    READ_LIDAR,
    PLAN,
    MOVE_FORWARD,
    MOVE_SIDEWAY,
    TURNING,
    TRANSITION_HOVER
};

struct PathCommand
{
    float delta_phi;
    float delta_distance;
};


float median(const deque<float>& numbers)
{
    if (numbers.size() == 0)
        return 0.0f;

    deque<float> copy = numbers;
    sort(copy.begin(), copy.end());
    if (copy.size() % 2)
        return copy[copy.size()/2];
    else
        return 0.5f * (copy[copy.size()/2-1] + copy[copy.size()/2]);
}

float angle_diff(float a1, float a2)
{
    float angle_diff = a2 - a1;
    if (angle_diff > 0)
    {
        if (angle_diff > 180)
            return angle_diff - 360;
        else
            return angle_diff;
    }
    else
    {
        if (angle_diff < -180)
            return angle_diff + 360;
        else
            return angle_diff;
    }
}


db_t db;

// all of the following k's range from -1 to 1
void hover();
void moveSideway(float k); // left if k < 0, right if k > 0
void moveForward(float k); // backward if k < 0, forward if k > 0
void turn(float k); // ?? if k < 0, ?? if k > 0
void land();

float vx = 0.0f, vy = 0.0f, vz = 0.0f;
float gyrox = 0.0f, gyroy = 0.0f, gyroz = 0.0f;
float accumX = 0.0f, accumY = 0.0f, accumPhi = 0.0f;

int main(int argc, char* argv[]) {
    if (argc < 2)
    {
        puts("Not enough arguments.");
        return -1;
    }

    // connect to memdb
    db = db_connect("8766");
    if (db == -1)
    {
        puts("failed to connect to memdb.");
        return -1;
    }

   
    CConfigFile iniFile(argv[1]); // configurations file

    // Load configurations
    CMetricMapBuilderICP icp_slam;
    icp_slam.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
    icp_slam.ICP_params.loadFromConfigFile(iniFile, "ICP");
    icp_slam.initialize();

    // pathfinding
    int resolution = 4;
    PathFinder pathFinder(resolution);
    deque<TPoint2D> path;
    deque<PathCommand> pathCommands;

    // timing
    sf::Clock globalClock;
    double transitionHoverStartTime;
    double prevTime = 0.0;
    double dt = 0.0;

    // THE state of the main control
    State state = INIT_HOVER;
    State nextState;
    double delta_distance = 0.0;
    double delta_phi = 0.0;

    sf::RenderWindow window(sf::VideoMode(800, 600), "2:00am");
    window.setVerticalSyncEnabled(true);
    sf::Texture texture;
    texture.create(800, 600);
    Matrix<sf::Color> pixels(600, 800);
    sf::Sprite sprite(texture);
    sf::Clock fpsClock;
    int frameCount = 0;
    
    float prev_gyroz;
    deque<float> past_vx;
    deque<float> past_vy;

    globalClock.restart();

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


        double curTime = globalClock.getElapsedTime().asSeconds();
        dt = curTime - prevTime;
        prevTime = curTime;

        // read the navadata from the ardrone
        char buffer[1024];
        while (db_tryget(db, "navdata", buffer, sizeof(buffer)) != -1)
        {
            unsigned drone_dt_u;
            float cur_gyroz;
            sscanf(buffer, "%u,%f,%f,%f,%f,%f,%f",
                   &drone_dt_u, &vx, &vy, &vz, &gyrox, &gyroy, &cur_gyroz);

            // compute the angle
            gyrox /= 1000;
            gyroy /= 1000;
            cur_gyroz /= 1000;
            cur_gyroz = fmod(cur_gyroz + 360, 360);

            gyroz += angle_diff(prev_gyroz, cur_gyroz);
            accumPhi = gyroz;
            prev_gyroz = cur_gyroz;

            // here goes the terrible part....
            double drone_dt = (double)drone_dt_u / 1e6;
            past_vx.push_back(vx);
            past_vy.push_back(vy);
            if (past_vx.size() > 19)
            {
                past_vx.pop_front();
                past_vy.pop_front();
            }

            CObservationOdometryPtr obs = CObservationOdometry::Create();
            obs->odometry = CPose2D(accumX, accumY, accumPhi);
            obs->hasEncodersInfo = false;
            obs->hasVelocities = false;
//            icp_slam.processObservation(obs);
        }
        accumX += median(past_vx) * dt;
        accumY += median(past_vy) * dt;


        if (state == INIT_HOVER)
        {
            if (globalClock.getElapsedTime().asSeconds() >= 10.0)
            {
                initialize_feedback();
                state = PLAN;
            }
            else
                continue;
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
            // Need to define 2 values
            // 1.) scan: a vector of floats signalling the distances. Each element is a degree
            // 2.) validRange: a vector of ints where 1 signals the reading is good and 0 means its bad (and won't be used)

            while (db_tryget(db, "lidar", buffer, sizeof(buffer)) != -1)
            {
                CObservation2DRangeScanPtr obs = CObservation2DRangeScan::Create();
                obs->scan.resize(360, 0);
                obs->validRange.resize(360, 0);

                char sensorId;
                float distance;
                sscanf("%c,%f\n", &sensorId, &distance);

                int sensorIndex = (sensorId - 'A') * 90;
                int startIndex = (sensorIndex + 345) % 360;
                int endIndex = (sensorIndex + 375) % 360;
                for (int i = startIndex; i < endIndex; ++i)
                {
                    obs->scan[i] = distance;
                    obs->validRange[i] = 1;
                }

                icp_slam.processObservation(obs);
            }
        }


        else if (state == PLAN)
        {
            // Perform path finding
            /*
            pathFinder.update(*gridMap);
            bool pathFound = true;
            pathFound = pathFinder.findPath(TPoint2D(gridRobX, gridRobY), TPoint2D(gridRobX+1000, gridRobY), path);
            printf("pathFound: %d\tpath length: %lu\n", pathFound, path.size());
            */
            path.push_back(TPoint2D(gridRobX, gridRobY));
            path.push_back(TPoint2D(gridRobX+1000, gridRobY));
            path.push_back(TPoint2D(gridRobX+1000, gridRobY+1000));
            path.push_back(TPoint2D(gridRobX, gridRobY+1000));
            path.push_back(TPoint2D(gridRobX, gridRobY));
            float prevAngle = 0.0f;
            for (int i = 1; i < path.size(); i++)
            {
                float dx = path[i].x - path[i-1].x;
                float dy = path[i].y - path[i-1].y;
                float angle = (float)fmod(atan2(dy, dx)*57.2957795 + 360, 360);
                PathCommand command;
                command.delta_phi = angle_diff(prevAngle, angle);
                command.delta_distance = sqrt(dx*dx + dy*dy);
                pathCommands.push_back(command);
                prevAngle = angle;
            }

            for (int i = 0; i < path.size(); ++i)
                printf("%f, %f\n", pathCommands[i].delta_phi, pathCommands[i].delta_distance);

            delta_phi = pathCommands[0].delta_phi;
            delta_distance = pathCommands[0].delta_distance;
            pathCommands.pop_front();
            initialize_feedback();

            state = TURNING;
        }
        
        else if (state == TURNING)
        {
            float k = do_feedback_turn(delta_phi);
            printf("delta_phi: %f, turning k: %f\n", delta_phi, k);
            if (k != 0.0f)
                turn(k);
            else
            {
                nextState = MOVE_FORWARD;
                state = TRANSITION_HOVER;
                transitionHoverStartTime = curTime;
                state = MOVE_FORWARD;
            }
        }

        else if (state == MOVE_FORWARD)
        {
            float k = do_feedback_forward(delta_distance);
	    printf("delta_distance: %f, forward k: %f\n", delta_distance, k);
            if (k != 0.0f)
                moveForward(k);
            else if (pathCommands.size() == 0)
                state = LAND;
            else
            {
                delta_phi = pathCommands[0].delta_phi;
                delta_distance = pathCommands[0].delta_distance;
                pathCommands.pop_front();
                initialize_feedback();
				
                nextState = TURNING;
                state = TRANSITION_HOVER;
                transitionHoverStartTime = curTime;
            }
        }
        
        else if (state == HOVER)
        {
            hover();
        }

        else if (state == TRANSITION_HOVER)
        {
            hover();
            if (curTime - transitionHoverStartTime > 2.0)
            {
                state = nextState;
                initialize_feedback();
            }
        }

        else if (state == LAND)
        {
            land();
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

    puts("exiting?!");

    return 0;
}


void hover()
{
    puts("hovering!");
    db_printf(db, "drone_command", "%d,%f,%f,%f,%f", 1, 0.0f, 0.0f, 0.0f, 0.0f);
}

void moveSideway(float k) // left if k > 0, right if k < 0
{
    puts("moveSideway!");
    db_printf(db, "drone_command", "%d,%f,%f,%f,%f", 0, k, 0.0f, 0.0f, 0.0f);
}

void moveForward(float k) // forward if k > 0, backward if k < 0
{
    puts("moveForward!");
    db_printf(db, "drone_command", "%d,%f,%f,%f,%f", 0, 0.0f, k, 0.0f, 0.0f);
}

void turn(float k) // left if k < 0, right if k > 0
{
    puts("turn!");
    db_printf(db, "drone_command", "%d,%f,%f,%f,%f", 0, 0.0f, 0.0f, 0.0f, k);
}

void land()
{
    puts("landing!");
    db_printf(db, "drone_command", "%d,%f,%f,%f,%f", -1, 0.0f, 0.0f, 0.0f, 0.0f);
}
