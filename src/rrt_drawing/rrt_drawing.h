#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <Eigen/Dense>
#include <2dplane/GridStateSpace.hpp>
#include <rrt/BiRRT.hpp>
//调用smooth函数
#include <rrt/Path.hpp>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/functional/hash.hpp>

class RRTdrawing{
public:
    RRTdrawing();

    ~RRTdrawing();

    void GenerateObstacle();
    void GridMapDilate();
    void GridMapGenerate();
    void ObstacleGenerate(int width, int height, cv::Point leftup);
    void DrawObstacle();
    void ReadObstacleFile(cv::Mat obstacle_in);
    void ObstacleOutput();
    void ObstacleShow();
    void _run();
    void _run_step();
    void _step(int numTimes);
    void drawTree(const RRT::Tree<Eigen::Vector2d>& rrt, const RRT::Node_t<Eigen::Vector2d>* solutionNode);
    void reduceTree();
    void smoothCurve();
    Eigen::Vector2d CalculateBezierPoint(std::vector<Eigen::Vector2d> ControlPoints, int n, float t);
    cv::Mat GridMapBinaryCallback(){ return _GridMapBinary; }
    cv::Mat GridMapCallback(){ return GridMap; }
    std::vector<Eigen::Vector2d> BezierCurvePointCallback() {return BezierCurvePoint;}

private:
//    std::vector<std::vector<int>> Obstacle;
    cv::Mat GridMap;
    cv::Mat _GridMapDilate;
    cv::Mat _GridMapBinary;
    cv::Mat GridMap_resize;
    std::shared_ptr<RRT::GridStateSpace> _stateSpace;
    std::unique_ptr<RRT::BiRRT<Eigen::Vector2d>> _biRRT;
    std::vector<Eigen::Vector2d> _previousSolution;

    Eigen::Vector2d _startVel, _goalVel;
    Eigen::Vector2d curr;
    std::vector<Eigen::Vector2d> BezierControlPoint;
    std::vector<Eigen::Vector2d> BezierCurvePoint;
};

namespace RRT {
    const int dimensions = 2;
    static size_t hash(Eigen::Vector2d state) {
        size_t seed = 0;
        boost::hash_combine(seed, state.x());
        boost::hash_combine(seed, state.y());
        return seed;
    }
}

//template<typename T, typename... Args>
//std::unique_ptr<T> make_unique(Args&&... args) {
//    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
//}
