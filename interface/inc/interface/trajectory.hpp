#pragma once
#include <Eigen/Eigen>
//! @todo add more trajectory types: spline, bezier, time-optimal, piecewise, etc.
// Base class for trajectory, for runtime polymorphism usage requirements
class TrajectoryBase
{
public:
    virtual ~TrajectoryBase() = default;

public:
    virtual double get_duration() const = 0;
    virtual const Eigen::VectorXd get_sample(double t) const = 0;
    virtual const Eigen::MatrixXd get_sample(double t, int derivative_order) const = 0;
    virtual int get_dimension() const = 0;
    virtual double get_arc_length(double t0, double t1) const = 0;
    virtual double get_curvature(double t) const = 0;
    //! @todo virtual Eigen::VectorXd get_curvature(double t, int derivative_order) = 0;
    //! @todo torsion
};

// CRTP for static polymorphism, faster than virtual functions
template <typename Derived>
class Trajectory : public TrajectoryBase
{
public:
    virtual ~Trajectory() override = default;
public:
    const Eigen::VectorXd sample(double t) const
    {
        return static_cast<const Derived *>(this)->sample(t);
    }
    const Eigen::MatrixXd sample(double t, int derivative_order) const
    {
        return static_cast<const Derived *>(this)->sample(t, derivative_order);
    }
    int dimension() const
    {
        return static_cast<const Derived *>(this)->dimension();
    }
    double arc_length(double t0, double t1) const
    {
        return static_cast<const Derived *>(this)->arc_length(t0, t1);
    }
    double curvature(double t) const
    {
        return static_cast<const Derived *>(this)->curvature(t);
    }
    double duration() const
    {
        return static_cast<const Derived *>(this)->duration();
    }

public:
    const Eigen::VectorXd get_sample(double t) const override
    {
        return sample(t);
    }
    const Eigen::MatrixXd get_sample(double t, int derivative_order) const override
    {
        return sample(t, derivative_order);
    }
    int get_dimension() const override
    {
        return dimension();
    }
    double get_arc_length(double t0, double t1) const override
    {
        return arc_length(t0, t1);
    }
    double get_curvature(double t) const override
    {
        return curvature(t);
    }
    double get_duration() const override
    {
        return static_cast<const Derived *>(this)->get_duration();
    }
};