#include <Eigen/Eigen>

//! @todo split into separate files for each class
//! @todo split definition and declaration
// Base class for trajectory, for runtime polymorphism usage requirements
class TrajectoryBase
{
public:
    virtual Eigen::VectorXd get_sample(double t) = 0;
    virtual Eigen::MatrixXd get_sample(double t, int derivative_order) = 0;
    virtual int get_dimension() = 0;
    virtual double get_arc_length(double t0, double t1) = 0;
    virtual double get_curvature(double t) = 0;
    //! @todo virtual Eigen::VectorXd get_curvature(double t, int derivative_order) = 0;
    //! @todo torsion
};

// CRTP for static polymorphism, faster than virtual functions
template <typename Derived>
class Trajectory : public TrajectoryBase
{
public:
    Eigen::VectorXd sample(double t)
    {
        return static_cast<Derived *>(this)->sample(t);
    }
    Eigen::MatrixXd sample(double t, int derivative_order)
    {
        return static_cast<Derived *>(this)->sample(t, derivative_order);
    }
    int dimension()
    {
        return static_cast<Derived *>(this)->dimension();
    }
    double arc_length(double t0, double t1)
    {
        return static_cast<Derived *>(this)->arc_length(t0, t1);
    }
    double curvature(double t)
    {
        return static_cast<Derived *>(this)->curvature(t);
    }

public:
    Eigen::VectorXd get_sample(double t) override
    {
        return sample(t);
    }
    Eigen::MatrixXd get_sample(double t, int derivative_order) override
    {
        return sample(t, derivative_order);
    }
    int get_dimension() override
    {
        return dimension();
    }
    double get_arc_length(double t0, double t1) override
    {
        return arc_length(t0, t1);
    }
    double get_curvature(double t) override
    {
        return curvature(t);
    }
};

template <>
class Polynomial3dTrajectory : public Trajectory<Polynomial3dTrajectory>
{
public:
    Polynomial3dTrajectory(int order, Eigen::MatrixXd coefficients)
    {
        order_ = order;
        coefficients_ = coefficients;
    }
    Eigen::VectorXd sample(double t)
    {
        Eigen::VectorXd sample = Eigen::VectorXd::Zero(3);
        for (int i = 0; i < order_ + 1; i++)
        {
            sample(0) += coefficients_(i, 0) * pow(t, i);
            sample(1) += coefficients_(i, 1) * pow(t, i);
            sample(2) += coefficients_(i, 2) * pow(t, i);
        }
        return sample;
    }
    Eigen::MatrixXd sample(double t, int derivative_order)
    {
        Eigen::MatrixXd sample = Eigen::MatrixXd::Zero(3, derivative_order + 1);
        for (int i = 0; i < order_ + 1; i++)
        {
            sample(0, 0) += coefficients_(i, 0) * pow(t, i);
            sample(1, 0) += coefficients_(i, 1) * pow(t, i);
            sample(2, 0) += coefficients_(i, 2) * pow(t, i);
            for (int j = 1; j < derivative_order + 1; j++)
            {
                sample(0, j) += coefficients_(i, 0) * pow(t, i - j);
                sample(1, j) += coefficients_(i, 1) * pow(t, i - j);
                sample(2, j) += coefficients_(i, 2) * pow(t, i - j);
            }
        }
        return sample;
    }
    double curvature(double t)
    {
        // curvature = |v x a| / |v|^3
        Eigen::MatrixXd sample = sample(t, 2);
        Eigen::Vector3d velocity = sample.col(1);
        Eigen::Vector3d acceleration = sample.col(2);
        double curvature = (velocity.cross(acceleration)).norm() / pow(velocity.norm(), 3);
        return curvature;
    }
    double arc_length(double t0, double t1)
    {
        static constexpr double dt = 0.001; // accuracy of numerical integration
        // arc length = integral of |v| dt
        Eigen::MatrixXd sample = sample(t0, 1);
        Eigen::Vector3d velocity = sample.col(1);
        // do numerical integration
        double arc_length = 0;
        for (double t = t0; t < t1; t += dt)
        {
            Eigen::MatrixXd sample = sample(t, 1);
            Eigen::Vector3d velocity = sample.col(1);
            arc_length += velocity.norm() * dt;
        }
        return arc_length;
    }
    int dimension()
    {
        return 3;
    }

private:
    Eigen::MatrixXd coefficients_;
};