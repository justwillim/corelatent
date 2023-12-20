#pragma once
#include <Eigen/Eigen>

class RigidObject
{
public:
    using State = Eigen::Matrix4d;
    using Control = Eigen::VectorXd;

public:
    virtual ~RigidObject() = default;

public:
    virtual State step(const Control &control, double dt) = 0;
};

class Particle2DObject
{
public:
    using State = Eigen::Vector2d;
    using Control = Eigen::Vector2d;

public:
    virtual ~Particle2DObject() = default;

public:
    virtual State step(const Control &control, double dt) = 0;
};