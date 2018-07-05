#pragma once

#include <deque>
#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>
#include <functional>
#include <list>
#include <memory>
#include <2dplane/StateSpace.hpp>
#include <stdexcept>
#include <stdlib.h>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace RRT {

template <typename T>
class Node_t {
public:
    Node_t(const T& state, Node_t<T>* parent = nullptr, int dimensions = 2,
         std::function<void(T, double*)> TToArray = NULL)
        : _parent(parent), _state(state), _vec(dimensions) {
        if (_parent) {
            _parent->_children.push_back(this);
        }
        if (NULL == TToArray) {
            for (int i = 0; i < dimensions; i++) {
                _vec[i] = state[i];
            }
        } else {
            TToArray(state, _vec.data());
        }
    }

    const Node_t<T>* parent() const { return _parent; }

    int depth() const {
        int n = 0;
        for (Node_t<T>* ancestor = _parent; ancestor != nullptr;
             ancestor = ancestor->_parent) {
            n++;
        }
        return n;
    }

    const T& state() const { return _state; }

    std::vector<double>* coordinates() { return &_vec; }

private:
    std::vector<double> _vec;
    T _state;
    std::list<Node_t<T>*> _children;
    Node_t<T>* _parent;
};

template <typename T>
class Tree {
public:
    Tree(const Tree&) = delete;
    Tree& operator=(const Tree&) = delete;
    Tree(std::shared_ptr<StateSpace<T>> stateSpace,
         std::function<size_t(T)> hashT, int dimensions,
         std::function<T(double*)> arrayToT = NULL,
         std::function<void(T, double*)> TToArray = NULL)
        : _kdtree(flann::KDTreeSingleIndexParams()),
          _dimensions(dimensions),
          _nodemap(20, hashT) {
        _stateSpace = stateSpace;
        _arrayToT = arrayToT;
        _TToArray = TToArray;

        //  default values
        setStepSize(0.1);
        setMaxStepSize(5);
        setMaxIterations(1000);
        setASCEnabled(false);
        setGoalBias(0);
        setWaypointBias(0);
        setGoalMaxDist(0.1);
    }

    StateSpace<T>& stateSpace() { return *_stateSpace; }
    const StateSpace<T>& stateSpace() const { return *_stateSpace; }

    int maxIterations() const { return _maxIterations; }
    void setMaxIterations(int itr) { _maxIterations = itr; }

    bool isASCEnabled() const { return _isASCEnabled; }
    void setASCEnabled(bool checked) { _isASCEnabled = checked; }

    double goalBias() const { return _goalBias; }
    void setGoalBias(double goalBias) {
        if (goalBias < 0 || goalBias > 1) {
            throw std::invalid_argument(
                "The goal bias must be a number between 0.0 and 1.0");
        }
        _goalBias = goalBias;
    }

    double waypointBias() const { return _waypointBias; }
    void setWaypointBias(double waypointBias) {
        if (waypointBias < 0 || waypointBias > 1) {
            throw std::invalid_argument(
                "The waypoint bias must be a number between 0.0 and 1.0");
        }
        _waypointBias = waypointBias;
    }

    const std::vector<T>& waypoints() const { return _waypoints; }
    void setWaypoints(const std::vector<T>& waypoints) {
        _waypoints = waypoints;
    }
    void clearWaypoints() { _waypoints.clear(); }

    double stepSize() const { return _stepSize; }
    void setStepSize(double stepSize) { _stepSize = stepSize; }

    double maxStepSize() const { return _maxStepSize; }
    void setMaxStepSize(double maxStep) { _maxStepSize = maxStep; }

    double goalMaxDist() const { return _goalMaxDist; }
    void setGoalMaxDist(double maxDist) { _goalMaxDist = maxDist; }

    bool run() {
        for (int i = 0; i < _maxIterations; i++) {
            Node_t<T>* newNode = grow();

            if (newNode &&
                _stateSpace->distance(newNode->state(), _goalState) <
                    _goalMaxDist)
                return true;
        }
        return false;
    }

    void reset(bool eraseRoot = false) {
        _kdtree = flann::Index<flann::L2_Simple<double>>(
            flann::KDTreeSingleIndexParams());
        if (eraseRoot) {
            _nodes.clear();
            _nodemap.clear();
        } else if (_nodes.size() > 1) {
            T root = rootNode()->state();
            _nodemap.clear();
            _nodes.clear();
            _nodes.emplace_back(root, nullptr, _dimensions, _TToArray);
            _nodemap.insert(std::pair<T, Node_t<T>*>(root, &_nodes.back()));
            if (_TToArray) {
                std::vector<double> data(_dimensions);
                _TToArray(root, data.data());
                _kdtree.buildIndex(
                    flann::Matrix<double>(data.data(), 1, _dimensions));
            } else {
                _kdtree.buildIndex(flann::Matrix<double>(
                    (double*)&(rootNode()->state()), 1, _dimensions));
            }
        }
    }

    Node_t<T>* grow() {
        double r = rand() / (double)RAND_MAX;
        if (r < goalBias()) {
            return extend(goalState());
        } else if (r < goalBias() + waypointBias() && _waypoints.size() > 0) {
            //指向任意随机点，这个点是从过去成功的路径中选择的
            const T& waypoint = _waypoints[rand() % _waypoints.size()];
            return extend(waypoint);
        } else {
            return extend(_stateSpace->randomState());
        }
    }

    Node_t<T>* nearest(const T& state, double* distanceOut = nullptr) {
        Node_t<T>* best = nullptr;

        flann::Matrix<double> query;
        if (NULL == _TToArray) {
            query = flann::Matrix<double>((double*)&state, 1,
                                          sizeof(state) / sizeof(0.0));
        } else {
            std::vector<double> data(_dimensions);
            _TToArray(state, data.data());
            query = flann::Matrix<double>(data.data(), 1,
                                          sizeof(state) / sizeof(0.0));
        }
        std::vector<int> i(query.rows);
        flann::Matrix<int> indices(i.data(), query.rows, 1);
        std::vector<double> d(query.rows);
        flann::Matrix<double> dists(d.data(), query.rows, 1);

        int n =
            _kdtree.knnSearch(query, indices, dists, 1, flann::SearchParams());

        if (distanceOut)
            *distanceOut = _stateSpace->distance(state, best->state());

        T point;
        if (NULL == _arrayToT) {
            point = (T)_kdtree.getPoint(indices[0][0]);
        } else {
            point = _arrayToT(_kdtree.getPoint(indices[0][0]));
        }

        return _nodemap[point];
    }

    virtual Node_t<T>* extend(const T& target, Node_t<T>* source = nullptr) {
        if (!source) {
            source = nearest(target, nullptr);
            if (!source) {
                return nullptr;
            }
        }

        T intermediateState;
        if (_isASCEnabled) {
            intermediateState = _stateSpace->intermediateState(
                source->state(), target, stepSize(), maxStepSize());
        } else {
            intermediateState = _stateSpace->intermediateState(
                source->state(), target, stepSize());
        }

        if (!_stateSpace->transitionValid(source->state(), intermediateState)) {
            return nullptr;
        }

        _nodes.emplace_back(intermediateState, source, _dimensions, _TToArray);
        _kdtree.addPoints(flann::Matrix<double>(
            _nodes.back().coordinates()->data(), 1, _dimensions));
        _nodemap.insert(
            std::pair<T, Node_t<T>*>(intermediateState, &_nodes.back()));
        return &_nodes.back();
    }

    void getPath(std::function<void(const T& stateI)> callback,
                 const Node_t<T>* dest = nullptr, bool reverse = false) const {
        const Node_t<T>* node = (dest != nullptr) ? dest : lastNode();
        if (reverse) {
            while (node) {
                callback(node->state());
                node = node->parent();
            }
        } else {
            std::vector<const Node_t<T>*> nodes;
            while (node) {
                nodes.push_back(node);
                node = node->parent();
            }

            for (auto itr = nodes.rbegin(); itr != nodes.rend(); itr++) {
                callback((*itr)->state());
            }
        }
    }

    void getPath(std::vector<T>* vectorOut, const Node_t<T>* dest = nullptr,
                 bool reverse = false) const {
        getPath([&](const T& stateI) { vectorOut->push_back(stateI); }, dest,
                reverse);
    }

    std::vector<T> getPath(const Node_t<T>* dest = nullptr,
                           bool reverse = false) const {
        std::vector<T> path;
        getPath(&path, dest, reverse);
        return path;
    }

    const Node_t<T>* rootNode() const {
        if (_nodes.empty()) return nullptr;

        return &_nodes.front();
    }

    const Node_t<T>* lastNode() const {
        if (_nodes.empty()) return nullptr;

        return &_nodes.back();
    }

    const std::deque<Node_t<T>>& allNodes() const { return _nodes; }

    const T& startState() const {
        if (_nodes.empty())
            throw std::logic_error("No start state specified for RRT::Tree");
        else
            return rootNode()->state();
    }
    void setStartState(const T& startState) {
        reset(true);

        _nodes.emplace_back(startState, nullptr, _dimensions, _TToArray);
        _nodemap.insert(std::pair<T, Node_t<T>*>(startState, &_nodes.back()));
        if (_TToArray) {
            std::vector<double> data(_dimensions);
            _TToArray(rootNode()->state(), data.data());
            _kdtree.buildIndex(
                flann::Matrix<double>(data.data(), 1, _dimensions));
        } else {
            _kdtree.buildIndex(flann::Matrix<double>(
                (double*)&(rootNode()->state()), 1, _dimensions));
        }
    }

    const T& goalState() const { return _goalState; }
    void setGoalState(const T& goalState) { _goalState = goalState; }

protected:
    std::deque<Node_t<T>> _nodes{};

    std::unordered_map<T, Node_t<T>*, std::function<size_t(T)>> _nodemap;

    T _goalState;

    const int _dimensions;

    int _maxIterations;

    bool _isASCEnabled;

    double _goalBias;

    double _waypointBias;
    std::vector<T> _waypoints{};

    double _goalMaxDist;

    double _stepSize;
    double _maxStepSize;

    flann::Index<flann::L2_Simple<double>> _kdtree;

    std::function<T(double*)> _arrayToT;

    std::function<void(T, double*)> _TToArray;

    std::shared_ptr<StateSpace<T>> _stateSpace{};
};
}  // namespace RRT
