#pragma once
#include "interface/trajectory.hpp"
#include <vector>
#include <memory>

class PiecewiseTrajectory : public Trajectory<PiecewiseTrajectory>
{
    // type alias
public:
    using Piece = std::unique_ptr<TrajectoryBase>;
    using TimeRange = std::pair<double, double>;

    // constructor / destructor
public:
    PiecewiseTrajectory();
    // Vector of Piece is not copyable, so we use rvalue reference here.
    PiecewiseTrajectory(std::vector<Piece> &&trajectories, std::vector<double> &&time_allocation);

    // inherited APIs
public:
    const Eigen::VectorXd sample(double t) const;
    const Eigen::MatrixXd sample(double t, int derivative_order) const;

    double curvature(double t) const;

    double arc_length(double t0, double t1) const;
    int dimension() const;
    double duration() const;

    // special APIs
public:
    void insert_piece(int index, Piece &&new_piece, double new_time_allocation);
    Piece swap_piece(int index, Piece &&new_piece, double new_time_allocation);
    TimeRange get_piece_time_range(int index) const;
    TimeRange get_time_range() const;
    // assume all pieces is smoothable and is smooth internally.
    // chech the smoothness of the whole trajectory in the connection points.
    // smooth means the derivative of the position is continuous.
    // smoothable means the derivative norm of the position is continuous in the connection points.
    bool smooth(int derivative_order) const;
    bool smoothable(int derivative_order) const;
    const std::vector<double> nonsmooth_points(int derivative_order) const;
    const std::vector<double> nonsmoothable_points(int derivative_order) const;

    // data members
private:
    std::vector<Piece> trajectories_;
    std::vector<double> time_allocation_;

    // auxiliary members
private:
    std::vector<double> time_allocation_cumsum_; // auxiliary data member for fast search
    /// @brief dimension of the trajectory should be consistent
    /// @note this is mutable member because we need to update it lazily
    mutable bool dimension_validated_ = false;
    // helper auxiliary functions
private:
    void update_time_allocation_cumsum();
    bool validate_dimension() const;
};