#pragma once
#include <Eigen/Eigen>

class Rigid6dofController
{
public:
    using State = Eigen::Matrix4d;
    using Error = Eigen::Matrix4d;
    using Control = Eigen::VectorXd;

public:
    virtual Control control(const State &state, const State &target) = 0;
};