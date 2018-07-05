#pragma once

#include <Eigen/Dense>
#include <2dplane/ObstacleGrid.hpp>
#include <2dplane/PlaneStateSpace.hpp>

namespace RRT {

//将连续平面空间的类，扩展为平面上离散的格子，每个格子存在或不存在障碍
class GridStateSpace : public PlaneStateSpace<Eigen::Vector2d> {
public:
    GridStateSpace(double width, double height, int discretizedWidth,
                   int discretizedHeight);

    bool stateValid(const Eigen::Vector2d& pt) const;
    bool transitionValid(const Eigen::Vector2d& from,
                         const Eigen::Vector2d& to) const;

    Eigen::Vector2d intermediateState(const Eigen::Vector2d& source,
                                      const Eigen::Vector2d& target,
                                      double minStepSize,
                                      double maxStepSize) const;

    const ObstacleGrid& obstacleGrid() const;
    ObstacleGrid& obstacleGrid();

private:
    ObstacleGrid _obstacleGrid;
};

}  // namespace RRT
