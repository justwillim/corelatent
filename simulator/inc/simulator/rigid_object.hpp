#pragma once
#include <Eigen/Eigen>

class RigidObject
{
    using State = Eigen::Matrix4d;
    using Control = Eigen::VectorXd;

public:
    virtual ~RigidObject() = 0;

public:
    virtual State step(const Control &control, double dt) = 0;
};