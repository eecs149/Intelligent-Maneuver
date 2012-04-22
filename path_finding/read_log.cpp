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

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::slam;


template<typename T>
class Matrix
{
public:
    Matrix(): w(0), h(0)
    { }

    Matrix(unsigned height, unsigned width, const T& val = T()):
    w(width), h(height), data(w*h, val)
    { }

    const T& operator()(unsigned y, unsigned x) const
    {
        return data[y*this->w+x];
    }
    T& operator()(unsigned y, unsigned x)
    {
        return data[y*this->w+x];
    }

    void resize(unsigned height, unsigned width, const T& val = T())
    {
        vector<T> data(width*height, val);
        unsigned minHeight = min(height, h);
        unsigned minWidth = min(width, w);
        for (unsigned y = 0; y < minHeight; ++y)
            for (unsigned x = 0; x < minWidth; ++x)
                data[y*width+x] = this->data[y*w+x];
        w = width;
        h = height;
        this->data.swap(data);
    }
    unsigned width() const
    {
        return w;
    }
    unsigned height() const
    {
        return h;
    }

    const T* getData() const
    {
        return &data[0];
    }

    T* getData()
    {
        return &data[0];
    }

private:
    unsigned w;
    unsigned h;
    vector<T> data;
};


struct Node
{
    unsigned x;
    unsigned y;
    unsigned g;
    unsigned h;
    unsigned f;
    unsigned parentX;
    unsigned parentY;
    int dx;
    int dy;

    static unsigned endX;
    static unsigned endY;

    Node(unsigned x, unsigned y, unsigned g, unsigned parentX, unsigned parentY):
    x(x), y(y), g(g), parentX(parentX), parentY(parentY), dx(0), dy(0)
    {
        int dx = abs((int)endX - (int)x);
        int dy = abs((int)endY - (int)y);
        int diag_steps = min(dx, dy);
        h = diag_steps * 14 + (max(dx, dy) - diag_steps) * 10;
        f = g + h;
    }
    
    bool operator<(const Node& other) const
    {
        return f > other.f;
    }

    Node getSuccessor(int dx, int dy, int dg)
    {
        Node node(x+dx, y+dy, g+dg, x, y);
        
        // if it is moving in the same direction as the parent, subtract a bit from hits heuristic
        node.dx = dx;
        node.dy = dy;
        if (dx == this->dx && dy == this->dy)
        {
            node.f -= 0.1;
            node.g -= 0.05;
        }

        return node;
    }
};

unsigned Node::endX;
unsigned Node::endY;

class PathFinder
{
public:
    PathFinder(unsigned resolution):
    resolution(resolution)
    {
    }

    bool findPath(const TPoint2D& start, const TPoint2D& end, deque<TPoint2D>& path)
    {
        int startX = start.x / resolution;
        int startY = start.y / resolution;
        int endX = end.x / resolution;
        int endY = end.y / resolution;

        Node::endX = endX;
        Node::endY = endY;

        Matrix<int> parentX(occupancyGrid.width(), occupancyGrid.height(), -1);
        Matrix<int> parentY(occupancyGrid.width(), occupancyGrid.height(), -1);
        priority_queue<Node> fringe;
        fringe.push(Node(startX, startY, 0, startX, startY));

        printf("origin: %d,%d\tend: %d,%d\n", startX, startY, endX, endY);

        while (fringe.size())
        {
            Node pt = fringe.top();
            fringe.pop();

            if (parentX(pt.y, pt.x) >= 0)
                continue;
            parentX(pt.y, pt.x) = pt.parentX;
            parentY(pt.y, pt.x) = pt.parentY;

            if (pt.x == endX && pt.y == endY)
            {
                int x = pt.x;
                int y = pt.y;
                path.clear();
                while (y != startY || x != startX)
                {
                    path.push_front(TPoint2D(x, y));
                    int oldX = x;
                    x = parentX(y, x);
                    y = parentY(y, oldX);
                }
                return true;
            }

            // left
            if (pt.x > 0 && !occupancyGrid(pt.y, pt.x-1))
                fringe.push(pt.getSuccessor(-1, 0, 10));
            // up
            if (pt.y > 0 && !occupancyGrid(pt.y-1, pt.x))
                fringe.push(pt.getSuccessor(0, -1, 10));
            // right
            if (pt.x+1 < occupancyGrid.width() && !occupancyGrid(pt.y, pt.x+1))
                fringe.push(pt.getSuccessor(1, 0, 10));
            // down
            if (pt.y+1 < occupancyGrid.height() && !occupancyGrid(pt.y+1, pt.x))
                fringe.push(pt.getSuccessor(0, 1, 10));
            // upleft
            if (pt.x > 0 && pt.y > 0 && !occupancyGrid(pt.y-1, pt.x-1) && !occupancyGrid(pt.y-1, pt.x) && !occupancyGrid(pt.y, pt.x-1))
                fringe.push(pt.getSuccessor(-1, -1, 14));
            // downleft
            if (pt.x > 0 && pt.y+1 < occupancyGrid.height() && !occupancyGrid(pt.y-1, pt.x-1) && !occupancyGrid(pt.y-1, pt.x) && !occupancyGrid(pt.y, pt.x-1))
                fringe.push(pt.getSuccessor(-1, 1, 14));
            // upright
            if (pt.x+1 < occupancyGrid.width() && pt.y > 0 && !occupancyGrid(pt.y-1, pt.x+1) && !occupancyGrid(pt.y-1, pt.x) && !occupancyGrid(pt.y, pt.x+1))
                fringe.push(pt.getSuccessor(1, -1, 14));
            // downright
            if (pt.x+1 < occupancyGrid.width() && pt.y+1 < occupancyGrid.height() && !occupancyGrid(pt.y+1, pt.x+1) && !occupancyGrid(pt.y+1, pt.x) && !occupancyGrid(pt.y, pt.x+1))
                fringe.push(pt.getSuccessor(1, 1, 14));
        }

        return false;
    }

    void update(const COccupancyGridMap2D& gridMap)
    {
        update(gridMap, 0, 0, gridMap.getSizeX(), gridMap.getSizeY());
    }

    void update(const COccupancyGridMap2D& gridMap, int startX, int startY, int endX, int endY)
    {
        if (gridMap.getSizeY() > occupancyGrid.height()*resolution || gridMap.getSizeX() > occupancyGrid.width())
        {
            occupancyGrid = Matrix<unsigned char>(gridMap.getSizeY() / resolution, gridMap.getSizeX() / resolution);
            startX = 0;
            startY = 0;
            endX = gridMap.getSizeX();
            endY = gridMap.getSizeY();
        }

        startX = max(startX / (int)resolution, 0);
        startY = max(startY / (int)resolution, 0);
        endX = min(endX / resolution, occupancyGrid.width());
        endY = min(endY / resolution, occupancyGrid.height());
        for (int y = startY; y < endY; ++y)
        {
            for (int x = startX; x < endX; ++x)
            {
                unsigned char& val = occupancyGrid(y, x);
                
                if (val) continue;
                for (unsigned yy = 0; yy < resolution; ++yy)
                {
                    for (unsigned xx = 0; xx < resolution; ++xx)
                    {
                        double pOccupied = gridMap.getCell(x*resolution+xx, y*resolution+yy);
                        val |= pOccupied < 0.5;
                    }
                }

            }
        }
    }

    bool checkPathValid(deque<TPoint2D>& path)
    {
        for (unsigned i = 0; i < path.size(); ++i)
        {
            int x = path[i].x;
            int y = path[i].y;

            if (occupancyGrid(y, x))
                return false;
        }
        return true;
    }

//private:
    Matrix<unsigned char> occupancyGrid;
    unsigned resolution;
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
    double accumX = 0.0, accumY = 0.0, accumPhi = 0.0;


    // Load configurations
    CMetricMapBuilderICP icp_slam;
    icp_slam.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
    icp_slam.ICP_params.loadFromConfigFile(iniFile, "ICP");
    icp_slam.initialize();

    laserLog.open(argv[1]); // log of laser scan
    robotLog.open(argv[2]); // log of robot odometer

    // pathfinding
    int resolution = 4;
    PathFinder pathFinder(resolution);
    deque<TPoint2D> path;

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
        if (path.size() == 0 || !pathFinder.checkPathValid(path))
        {
            bool pathFound = true;
            pathFound = pathFinder.findPath(TPoint2D(gridRobX, gridRobY), TPoint2D(890, 270), path);
            printf("pathFound: %d\tpath length: %d\n", pathFound, path.size());
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
