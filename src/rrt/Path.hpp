#pragma once

#include <2dplane/StateSpace.hpp>
#include <vector>

namespace RRT {

template <typename T>
void DownSampleVector(std::vector<T>& states, size_t maxSize) {
    if (states.size() > maxSize) {
        int toDelete = states.size() - maxSize;
        double spacing = (double)states.size() / (double)toDelete;
        double i = 0.0;
        while (toDelete) {
            toDelete--;
            states.erase(states.begin() + (int)(i + 0.5));
            i += spacing - 1.0;
        }
    }
}//降采样，均匀减少路径上的点，使得轨迹上点的数目小于一个最大数目

template <typename T>
void SmoothPath(std::vector<T>& pts, const StateSpace<T>& stateSpace) {
    int span = 2;
    while (span < 10) {
        bool changed = false;
        for (int i = 0; i + span < pts.size(); i++) {
            if (stateSpace.transitionValid(pts[i], pts[i + span])) {
                for (int x = 1; x < span; x++) {
                    pts.erase(pts.begin() + i + 1);
                }
                changed = true;
            }
        }
        if (!changed) span++;
    }
}//把两点间的多个连线尽可能转化为一条直线，去除中间无用的线

}  // namespace RRT
