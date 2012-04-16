#ifndef PATHFINDING_H
#define PATHFINDING_H
#include <vector>
#include <deque>
#include <mrpt/utils.h>
#include <mrpt/obs.h>
#include <mrpt/slam.h>
#include <mrpt/poses.h> 


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

private:
    unsigned w;
    unsigned h;
    std::vector<T> data;
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

    Node(unsigned x, unsigned y, unsigned g, unsigned parentX, unsigned parentY);
    
    bool operator<(const Node& other) const;

    Node getSuccessor(int dx, int dy, int dg);
};

class PathFinder
{
public:
    PathFinder(unsigned resolution):
    resolution(resolution)
    { }

    bool findPath(const mrpt::utils::TPoint2D& start, const mrpt::utils::TPoint2D& end, std::deque<mrpt::utils::TPoint2D>& path);

    void update(const mrpt::utils::COccupancyGridMap2D& gridMap);

    void update(const mrpt::utils::COccupancyGridMap2D& gridMap, int startX, int startY, int endX, int endY);

    bool checkPathValid(std::deque<mrpt::utils::TPoint2D>& path);

//private:
    Matrix<unsigned char> occupancyGrid;
    unsigned resolution;
};


#endif

