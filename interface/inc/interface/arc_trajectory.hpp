#paragma once
#include "interface/trajectory.hpp"
#include <memory>
#include <map>

//! @brief ArcTrajectory
//! A trajectory container which holds only one internal trajector
//! Keeping the holding trajectory and makes the sampling variable t to represent the arc length of trajectory.
class ArcTrajectory : public Trajectory<ArcTrajectory>
{
    // type alias
public:
    using Piece = std::unique_ptr<TrajectoryBase>;

    // constructor / destructor
public:
    ArcTrajectory(Piece picec = nullptr, double t0 = 0, double epsilon = 0.01);

public:
    const Eigen::VectorXd sample(double t) const;
    const Eigen::MatrixXd sample(double t, int derivative_order) const;

    double curvature(double t) const;

    double arc_length(double t0, double t1) const;
    int dimension() const;
    double duration() const;

private:
    Piece trajectory_;
    double start_time_;
    double epsilon_; // error tolerance

    // auxiliary members
private:
    // arc-length & time mapping heuristic
    mutable std::map<double, double> heuristic_;

    // helper auxiliary functions
private:
    double interpolation(double t_0, double s_0, double t_1, double s_1, double target) const;
    double extrapolation(double t_0, double s_0, double t_1, double s_1, double target) const;
};