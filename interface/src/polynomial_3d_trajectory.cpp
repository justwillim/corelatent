#include "interface/polynomial_3d_trajectory.hpp"
#include <vector>

Polynomial3dTrajectory::Polynomial3dTrajectory(const Eigen::MatrixXd &coefficients)
{
    assert(coefficients.rows() == dimension());
    order_ = coefficients.cols() - 1;
    coefficients_ = coefficients;
}

const Eigen::VectorXd Polynomial3dTrajectory::sample(double t) const
{
    Eigen::VectorXd sample = Eigen::VectorXd::Zero(dimension());
    double t_powers = 1;
    for (int i = 0; i < order_ + 1; i++)
    {
        sample += coefficients_.col(i) * t_powers;
        t_powers *= t;
    }
    return sample;
}

const Eigen::MatrixXd Polynomial3dTrajectory::sample(double t, int derivative_order) const
{
    Eigen::MatrixXd sample = Eigen::MatrixXd::Zero(dimension(), derivative_order + 1);
    //! @todo fix this wrong implementation
    std::vector<double> lazy_factorials = {1, 1, 2, 6, 24, 120, 720};
    auto factorial = [&lazy_factorials](int n) -> int
    {
        if (n < lazy_factorials.size())
        {
            return lazy_factorials[n];
        }
        else
        {
            lazy_factorials.reserve(n + 1);
            int result = lazy_factorials.back();
            for (int i = lazy_factorials.size(); i <= n; i++)
            {
                result *= i;
                lazy_factorials.push_back(result);
            }
            return result;
        }
    };
    derivative_order = std::min(derivative_order, order_);
    for (int j = 0; j < derivative_order + 1; j++)
    {
        for (int i = 0; i < order_ + 1; i++)
        {
            if (i - j < 0)
            {
                continue;
            }
            sample.col(j) += coefficients_.col(i) * pow(t, i-j);
        }
        sample.col(j) *= factorial(order_) / factorial(order_ - j);
    }
    return sample;
}

double Polynomial3dTrajectory::curvature(double t) const
{
    // curvature = |v x a| / |v|^3
    Eigen::MatrixXd sample_data = sample(t, 2);
    Eigen::VectorXd velocity = sample_data.col(1);
    Eigen::VectorXd acceleration = sample_data.col(2);
    // THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE, we cannot use cross
    // double curvature = (velocity.cross(acceleration)).norm() / pow(velocity.norm(), 3);
    double curvature = sqrt(pow(velocity(1) * acceleration(2) - velocity(2) * acceleration(1), 2) +
                            pow(velocity(2) * acceleration(0) - velocity(0) * acceleration(2), 2) +
                            pow(velocity(0) * acceleration(1) - velocity(1) * acceleration(0), 2)) /
                       pow(velocity.norm(), 3);
    return curvature;
}

double Polynomial3dTrajectory::arc_length(double t0, double t1) const
{
    static constexpr double dt = 0.001; // accuracy of numerical integration
    // arc length = integral of |v| dt
    // do numerical integration
    double arc_length = 0;
    for (double t = t0; t < t1; t += dt)
    {
        Eigen::MatrixXd sample_data = sample(t, 1);
        Eigen::VectorXd velocity = sample_data.col(1);
        arc_length += velocity.norm() * dt;
    }
    return arc_length;
}

int Polynomial3dTrajectory::dimension() const
{
    return 3;
}

int Polynomial3dTrajectory::order() const
{
    return order_;
}