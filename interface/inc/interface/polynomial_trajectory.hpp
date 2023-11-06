#pragma once
#include "interface/trajectory.hpp"

class PolynomialTrajectory : public Trajectory<PolynomialTrajectory>
{
// constructor / destructor
public:
    PolynomialTrajectory(const Eigen::MatrixXd &coefficients = Eigen::MatrixXd::Zero(3, 1));

// inherited APIs
public:

    const Eigen::VectorXd sample(double t) const;
    const Eigen::MatrixXd sample(double t, int derivative_order) const;

    double curvature(double t) const;

    double arc_length(double t0, double t1) const;
    int dimension() const;

// special APIs
public:
    int order() const;

private:
    int dimension_;
    int order_;
    Eigen::MatrixXd coefficients_;
};
