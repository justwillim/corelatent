#pragma once
#include "simulator/rigid_object.hpp"

class RigidJerkAngularVelocityObject : public RigidObject
{
    using State = RigidObject::State;
    // translation, rotation, 6dof.
    using Control = RigidObject::Control;
    using Velocity = Eigen::Vector3d;
    using Acceleration = Eigen::Vector3d;

public:
    RigidJerkAngularVelocityObject(const State &initial_state = State::Identity());
    virtual ~RigidJerkAngularVelocityObject() override = default;

public:
    virtual State step(const Control &control, double dt)
    {
        Eigen::Quaterniond q(state_.block<3, 3>(0, 0));
        Eigen::Vector3d pos = state_.block<3, 1>(0, 3);
        auto jerk = control.block<3, 1>(0, 0);
        auto w = control.block<3, 1>(3, 0);

        acceleration_ += jerk * dt;
        velocity_ += acceleration_ * dt;
        pos += velocity_ * dt;

        // quaternion integration
        const Eigen::Matrix4d omega = OmegaMat(w);
        const Eigen::Matrix4d q_intt = MatExp(omega, 4);
        const Eigen::Vector4d vq(q.w(), q.x(), q.y(), q.z());
        const Eigen::Vector4d cq = (q_intt * vq);

        q = Eigen::Quaterniond(cq.x(), cq.y(), cq.z(), cq.w());
        q.normalize();

        state_.block<3, 3>(0, 0) = q.toRotationMatrix();
        state_.block<3, 1>(0, 3) = pos;

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

private:
    Eigen::Matrix3d Skew(const Eigen::Vector3d &v) const
    {
        Eigen::Matrix3d res;
        res << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
        return res;
    }

    Eigen::Matrix4d OmegaMat(const Eigen::Vector3d &v) const
    {
        Eigen::Matrix4d res;
        res.setZero();

        res.block(0, 1, 1, 3) = -v.transpose();
        res.block(1, 0, 3, 1) = v;
        res.block(1, 1, 3, 3) = -Skew(v);

        return res;
    }

    Eigen::Matrix4d MatExp(const Eigen::Matrix4d &A, const int &order) const
    {
        Eigen::Matrix4d matexp(Eigen::Matrix4d::Identity()); // initial condition with k=0

        int div = 1;
        Eigen::Matrix4d a_loop = A;

        for (int k = 1; k <= order; k++)
        {
            div = div * k; // factorial(k)
            matexp = matexp + a_loop / div;
            a_loop = a_loop * A; // adding one exponent each iteration
        }

        return matexp;
    }
};