#pragma once

#include <Eigen/Dense>

namespace RRT {

class ObstacleGrid {
public:
    ObstacleGrid(double width, double height, int discretizedWidth,
                 int discretizedHeight);
    ~ObstacleGrid();

    Eigen::Vector2i gridSquareForLocation(const Eigen::Vector2d& loc) const;

    //返回最近的障碍物的距离（包含边界），有一个上限距离
    double nearestObstacleDist(const Eigen::Vector2d& state,
                              double maxDist) const;
    void clear();
    bool& obstacleAt(int x, int y);
    bool obstacleAt(int x, int y) const;
    bool& obstacleAt(const Eigen::Vector2i& gridLoc);
    bool obstacleAt(const Eigen::Vector2i& gridLoc) const;

    int discretizedWidth() const;
    int discretizedHeight() const;
    double width() const;
    double height() const;

private:
    int _discretizedWidth, _discretizedHeight;
    double _width, _height;

    bool* _obstacles;
};

}  // namespace RRT
