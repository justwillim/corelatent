#pragma once
#include <Eigen/Eigen>
#include "simulator/rigid_object.hpp"

//! @todo rename file to jerk_2d_object
class Jerk2DObject : public Particle2DObject
{
    using State = Particle2DObject::State;
    using Control = Particle2DObject::Control;
    using Velocity = Eigen::Vector2d;
    using Acceleration = Eigen::Vector2d;

public:
    Jerk2DObject(const State &initial_state = State::Zero())
        : state_(initial_state), velocity_(Velocity::Zero()), acceleration_(Acceleration::Zero()), control_(Control::Zero())
    {
    }
    virtual ~Jerk2DObject() override = default;

public:
    virtual State step(const Control &control, double dt)
    {
        control_ = control;
        // clip the jerk
        auto jerk = control_.cwiseMax(-1.0).cwiseMin(1.0);

        acceleration_ += jerk * dt;
        velocity_ += acceleration_ * dt;
        state_ += velocity_ * dt;

        return state_;
    }

public:
    const State &state() const { return state_; }
    const Velocity &velocity() const { return velocity_; }
    const Acceleration &acceleration() const { return acceleration_; }

private:
    State state_;
    Velocity velocity_;
    Acceleration acceleration_;
    Control control_;
};