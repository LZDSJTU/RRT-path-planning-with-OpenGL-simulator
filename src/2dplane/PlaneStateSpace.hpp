#pragma once

#include <Eigen/Dense>
#include <2dplane/StateSpace.hpp>

namespace RRT {

template <typename T>
bool inRange(T n, T min, T max) {
    return (n >= min) && (n <= max);
}

template <class POINT_CLASS = Eigen::Vector2d>//定义轨迹的状态空间为二维平面点
class PlaneStateSpace : public StateSpace<POINT_CLASS> {
public:
    PlaneStateSpace(double width, double height)
        : _width(width), _height(height) {}

    POINT_CLASS randomState() const {
        return POINT_CLASS(drand48() * width(), drand48() * height());
    }//随机在长宽范围内取点

    POINT_CLASS intermediateState(const POINT_CLASS& source,
                                  const POINT_CLASS& target,
                                  double stepSize) const {
        POINT_CLASS delta = target - source;
        delta = delta / delta.norm();  //  unit vector

        POINT_CLASS val = source + delta * stepSize;
        return val;
    }//返回两点中间某比例的点的状态

    double distance(const POINT_CLASS& from, const POINT_CLASS& to) const {
        POINT_CLASS delta = from - to;
        return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
    }//返回两点间距离

    bool stateValid(const POINT_CLASS& pt) const {
        return pt.x() >= 0 && pt.y() >= 0 && pt.x() < width() &&
               pt.y() < height();
    }//检查点是否在长宽边界范围内

    double width() const { return _width; }
    double height() const { return _height; }

private:
    double _width, _height;
};

}  // namespace RRT
