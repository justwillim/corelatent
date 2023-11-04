#pragma once
#include <Eigen/Eigen>

//! @todo split into separate files for each class
//! @todo split definition and declaration
// Base class for trajectory, for runtime polymorphism usage requirements
class TrajectoryBase
{
public:
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
};

class Polynomial3dTrajectory : public Trajectory<Polynomial3dTrajectory>
{
// constructor / destructor
public:
    Polynomial3dTrajectory(const Eigen::MatrixXd &coefficients = Eigen::MatrixXd::Zero(3, 1));

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
    int order_;
    Eigen::MatrixXd coefficients_;
};
