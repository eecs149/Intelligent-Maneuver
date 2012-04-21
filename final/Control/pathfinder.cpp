#include <algorithm>
#include "pathfinder.h"
using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::slam;


/////////////////////////////////
// Node class implementation
/////////////////////////////////
unsigned Node::endX;
unsigned Node::endY;

Node::Node(unsigned x, unsigned y, unsigned g, unsigned parentX, unsigned parentY):
x(x), y(y), g(g), parentX(parentX), parentY(parentY), dx(0), dy(0)
{
    int dx = abs((int)endX - (int)x);
    int dy = abs((int)endY - (int)y);
    int diag_steps = min(dx, dy);
    h = diag_steps * 14 + (max(dx, dy) - diag_steps) * 10;
    f = g + h;
}

bool Node::operator<(const Node& other) const
{
    return f > other.f;
}

Node Node::getSuccessor(int dx, int dy, int dg)
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


/////////////////////////////////////
// Pathfinder class
/////////////////////////////////////
bool PathFinder::findPath(const TPoint2D& start, const TPoint2D& end, deque<TPoint2D>& path)
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

void PathFinder::update(const COccupancyGridMap2D& gridMap)
{
    update(gridMap, 0, 0, gridMap.getSizeX(), gridMap.getSizeY());
}

void PathFinder::update(const COccupancyGridMap2D& gridMap, int startX, int startY, int endX, int endY)
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

bool PathFinder::checkPathValid(deque<TPoint2D>& path)
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

void simplifyPath(std::deque<mrpt::utils::TPoint2D>& path)
{
    if (path.size() <= 1) return;

    std::deque<mrpt::utils::TPoint2D>& newpath;
    newpath.push_back(path[0]);
    int dx = path[1].x - path[0].x;
    int dy = path[1].y - path[0].y;
    for (int i = 2; i < path.size(); ++i)
    {
        if (path[i].x - path[i-1].x != dx || path[i].y - path[i-1].y != dy)
        {
            dx = path[i].x - path[i-1].x;
            dy = path[i].y - path[i-1].y;
            newpath.push_back(path[i-1]);
        }
    }
    newpath.push_back(path[path.size()-1]);

    path.swap(newpath);
}

