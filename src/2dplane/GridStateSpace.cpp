#include <math.h>
#include <2dplane/GridStateSpace.hpp>
#include <stdexcept>

using namespace Eigen;
using namespace std;

#include <iostream>

namespace RRT {

GridStateSpace::GridStateSpace(double width, double height, int discretizedWidth,
                               int discretizedHeight)
    : PlaneStateSpace(width, height),
      _obstacleGrid(width, height, discretizedWidth, discretizedHeight) {}

bool GridStateSpace::stateValid(const Vector2d& pt) const {
    return PlaneStateSpace::stateValid(pt) &&
           !_obstacleGrid.obstacleAt(_obstacleGrid.gridSquareForLocation(pt));
}//当前点是否既在边界范围内，又不在障碍上

Vector2d GridStateSpace::intermediateState(const Vector2d& source,
                                           const Vector2d& target,
                                           double minStepSize,
                                           double maxStepSize) const {
    bool debug = false;

    Vector2d delta = target - source;
    delta = delta / delta.norm();
    double dist = _obstacleGrid.nearestObstacleDist(source, maxStepSize * 2);

    double stepSize = (dist / maxStepSize) * minStepSize;
    if (stepSize > maxStepSize) stepSize = maxStepSize;
    if (stepSize < minStepSize) stepSize = minStepSize;

    Vector2d val = source + delta * stepSize;
    return val;
}//寻找下一个撒的点

bool GridStateSpace::transitionValid(const Vector2d& from,
                                     const Vector2d& to) const {
    if (!stateValid(to)) return false;

    Vector2d delta = to - from;
    //将坐标统一转移到栅格坐标下
    Vector2i discreteFrom = _obstacleGrid.gridSquareForLocation(from);
    Vector2i discreteTo = _obstacleGrid.gridSquareForLocation(to);
    int x1 = discreteFrom.x(), y1 = discreteFrom.y();
    int x2 = discreteTo.x(), y2 = discreteTo.y();

    if (x1 > x2) swap<int>(x1, x2);
    if (y1 > y2) swap<int>(y1, y2);

    double gridSqWidth = width() / _obstacleGrid.discretizedWidth();
    double gridSqHeight = height() / _obstacleGrid.discretizedHeight();

    //  检查两个端点间连线的障碍情况
    for (int x = x1; x <= x2; x++) {
        for (int y = y1; y <= y2; y++) {
            if (_obstacleGrid.obstacleAt(x, y)) {
                //某一点被占据，代表整个矩形栅格被占据
                Vector2d ulCorner(x * gridSqWidth, y * gridSqHeight);
                Vector2d brCorner(ulCorner.x() + gridSqWidth,
                                  ulCorner.y() + gridSqHeight);

                if (delta.x() != 0) {
                    double slope = delta.y() / delta.x();
                    double b = to.y() - to.x() * slope;

                    double yInt = slope * ulCorner.x() + b;
                    if (inRange<double>(yInt, ulCorner.y(), brCorner.y()))
                        return false;
                    yInt = slope * brCorner.x() + b;
                    if (inRange<double>(yInt, ulCorner.y(), brCorner.y()))
                        return false;

                    if (slope == 0) return false;
                    double xInt = (ulCorner.y() - b) / slope;
                    if (inRange<double>(xInt, ulCorner.x(), brCorner.x()))
                        return false;
                    xInt = (brCorner.y() - b) / slope;
                    if (inRange<double>(xInt, ulCorner.x(), brCorner.x()))
                        return false;
                } else {
                    if (inRange<double>(from.x(), ulCorner.x(), brCorner.x())) {
                        Vector2d lower(from);
                        Vector2d higher(to);
                        if (higher.y() < lower.y())
                            swap<Vector2d>(lower, higher);

                        if (lower.y() < ulCorner.y() &&
                            higher.y() > ulCorner.y())
                            return false;
                        if (lower.y() < brCorner.y() &&
                            higher.y() > brCorner.y())
                            return false;
                    }
                }
            }
        }
    }
    return true;
}//检查两点是否可以直接相连

const ObstacleGrid& GridStateSpace::obstacleGrid() const {
    return _obstacleGrid;
}

ObstacleGrid& GridStateSpace::obstacleGrid() { return _obstacleGrid; }

}  // namespace RRT
