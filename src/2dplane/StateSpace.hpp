#pragma once

namespace RRT {

template <typename T>
class StateSpace {
public:
    StateSpace(){};
    virtual ~StateSpace(){};

    virtual T randomState() const = 0;

    virtual T intermediateState(const T& source, const T& target,
                                double stepSize) const = 0;

    virtual T intermediateState(const T& source, const T& target,
                                double minStepSize, double maxStepSize) const = 0;

    virtual double distance(const T& from, const T& to) const = 0;

    virtual bool stateValid(const T& state) const = 0;

    virtual bool transitionValid(const T& from, const T& to) const = 0;

protected:
    double _minStepSize;
    double _maxStepSize;
};

}  // namespace RRT
