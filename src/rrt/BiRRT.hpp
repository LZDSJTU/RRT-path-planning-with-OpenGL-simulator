#pragma once

#include <limits.h>
#include <rrt/Tree.hpp>

namespace RRT {

template <typename T>
class BiRRT {
public:
    BiRRT(std::shared_ptr<StateSpace<T>> stateSpace,
          std::function<size_t(T)> hash, int dimensions,
          std::function<T(double*)> arrayToT = NULL,
          std::function<void(T, double*)> TToArray = NULL)
        : _startTree(stateSpace, hash, dimensions, arrayToT, TToArray),
          _goalTree(stateSpace, hash, dimensions, arrayToT, TToArray) {
        _minIterations = 0;
        reset();
    }

    void reset() {
        _startTree.reset();
        _goalTree.reset();

        _iterationCount = 0;

        _startSolutionNode = nullptr;
        _goalSolutionNode = nullptr;
        _solutionLength = INT_MAX;
    }

    const Tree<T>& startTree() const { return _startTree; }
    const Tree<T>& goalTree() const { return _goalTree; }

    bool isASCEnabled() const { return _startTree.isASCEnabled(); }
    void setASCEnabled(bool checked) {
        _startTree.setASCEnabled(checked);
        _goalTree.setASCEnabled(checked);
    }

    double goalBias() const { return _startTree.goalBias(); }
    void setGoalBias(double goalBias) {
        _startTree.setGoalBias(goalBias);
        _goalTree.setGoalBias(goalBias);
    }

    int maxIterations() const { return _startTree.maxIterations(); }
    void setMaxIterations(int itr) {
        _startTree.setMaxIterations(itr);
        _goalTree.setMaxIterations(itr);
    }

    int minIterations() const { return _minIterations; }
    void setMinIterations(int itr) { _minIterations = itr; }

    double waypointBias() const { return _startTree.waypointBias(); }
    void setWaypointBias(double waypointBias) {
        _startTree.setWaypointBias(waypointBias);
        _goalTree.setWaypointBias(waypointBias);
    }

    const std::vector<T>& waypoints() { return _startTree.waypoints(); }
    void setWaypoints(const std::vector<T>& waypoints) {
        _startTree.setWaypoints(waypoints);
        _goalTree.setWaypoints(waypoints);
    }

    double stepSize() const { return _startTree.stepSize(); }
    void setStepSize(double stepSize) {
        _startTree.setStepSize(stepSize);
        _goalTree.setStepSize(stepSize);
    }

    double maxStepSize() const { return _startTree.maxStepSize(); }
    void setMaxStepSize(double stepSize) {
        _startTree.setMaxStepSize(stepSize);
        _goalTree.setMaxStepSize(stepSize);
    }

    double goalMaxDist() const { return _startTree.goalMaxDist(); }
    void setGoalMaxDist(double maxDist) {
        _startTree.setGoalMaxDist(maxDist);
        _goalTree.setGoalMaxDist(maxDist);
    }

    std::vector<T> getPath() {
        std::vector<T> path;
        _startTree.getPath(&path, _startSolutionNode);
        _startTree.getPath(&path, _goalSolutionNode, true);
        return path;
    }

    void grow() {
        int depth;
        const Node_t<T>* otherNode;

        Node_t<T>* newStartNode = _startTree.grow();
        if (newStartNode) {
            otherNode = _findBestPath(newStartNode->state(), _goalTree, &depth);
            //other不为null表示已经找到了合适的路径
            if (otherNode && depth + newStartNode->depth() < _solutionLength &&
                _goalTree.stateSpace().transitionValid(newStartNode->state(),
                                                       otherNode->state())) {
                _startSolutionNode = newStartNode;
                _goalSolutionNode = otherNode;
                _solutionLength = newStartNode->depth() + depth;
            }
        }

        Node_t<T>* newGoalNode = _goalTree.grow();
        if (newGoalNode) {
            otherNode = _findBestPath(newGoalNode->state(), _startTree, &depth);
            if (otherNode && depth + newGoalNode->depth() < _solutionLength &&
                _goalTree.stateSpace().transitionValid(newGoalNode->state(),
                                                       otherNode->state())) {
                _startSolutionNode = otherNode;
                _goalSolutionNode = newGoalNode;
                _solutionLength = newGoalNode->depth() + depth;
            }
        }

        ++_iterationCount;
    }

    bool run() {
        for (int i = 0; i < _startTree.maxIterations(); i++) {
            grow();
            if (_startSolutionNode != nullptr && i >= minIterations())
                return true;
        }
        return false;
    }

    void setStartState(const T& start) {
        _startTree.setStartState(start);
        _goalTree.setGoalState(start);
    }
    const T& startState() const { return _startTree.startState(); }

    void setGoalState(const T& goal) {
        _startTree.setGoalState(goal);
        _goalTree.setStartState(goal);
    }
    const T& goalState() const { return _startTree.goalState(); }

    const Node_t<T>* startSolutionNode() { return _startSolutionNode; }

    const Node_t<T>* goalSolutionNode() { return _goalSolutionNode; }

    int iterationCount() const { return _iterationCount; }

protected:
    const Node_t<T>* _findBestPath(const T& targetState, Tree<T>& treeToSearch,
                                 int* depthOut) const {
        const Node_t<T>* bestNode = nullptr;
        int depth = INT_MAX;

        for (const Node_t<T>& other : treeToSearch.allNodes()) {
            double dist = _startTree.stateSpace().distance(other.state(), targetState);
            if (dist < goalMaxDist() && other.depth() < depth) {
                bestNode = &other;
                depth = other.depth();
            }
        }

        if (depthOut) *depthOut = depth;

        return bestNode;
    }

private:
    Tree<T> _startTree;
    Tree<T> _goalTree;

    int _iterationCount;
    int _minIterations;

    int _solutionLength;
    const Node_t<T>* _startSolutionNode, *_goalSolutionNode;
};

}  // namespace RRT
