#include "interface/piecewise_trajectory.hpp"
#include <algorithm>
#include <numeric>

PiecewiseTrajectory::PiecewiseTrajectory() : trajectories_(),
                                             time_allocation_()
{
}

PiecewiseTrajectory::PiecewiseTrajectory(std::vector<Piece> &&trajectories, std::vector<double> &&time_allocation) : trajectories_(std::move(trajectories)),
                                                                                                                     time_allocation_(std::move(time_allocation))
{
    if (trajectories_.size() != time_allocation_.size())
    {
        throw std::invalid_argument("The size of the trajectories and the time allocation are not consistent.");
    }
    if (!validate_dimension())
    {
        throw std::invalid_argument("The dimension of the trajectories are not consistent.");
    }
    if (time_allocation_.empty())
    {
        throw std::invalid_argument("The time allocation is empty.");
    }
    if (time_allocation_[0] <= 0.0)
    {
        throw std::invalid_argument("The time allocation is not positive.");
    }
    update_time_allocation_cumsum();
}

const Eigen::VectorXd PiecewiseTrajectory::sample(double t) const
{
    // find the piece index
    auto piece_iter = std::lower_bound(time_allocation_cumsum_.begin(), time_allocation_cumsum_.end(), t);
    if(piece_iter == std::end(time_allocation_cumsum_))
    {
        throw std::invalid_argument("The time is out of range.");
    }
    int piece_index = std::distance(time_allocation_cumsum_.begin(), piece_iter);
    // sample the piece
    return trajectories_[piece_index]->get_sample(t - time_allocation_cumsum_[piece_index]);
}

const Eigen::MatrixXd PiecewiseTrajectory::sample(double t, int derivative_order) const
{
    // find the piece index
    auto piece_iter = std::lower_bound(time_allocation_cumsum_.begin(), time_allocation_cumsum_.end(), t);
    
    if(piece_iter == std::end(time_allocation_cumsum_))
    {
        throw std::invalid_argument("The time is out of range.");
    }
    int piece_index = std::distance(time_allocation_cumsum_.begin(), piece_iter);
    // sample the piece
    return trajectories_[piece_index]->get_sample(t - time_allocation_cumsum_[piece_index], derivative_order);
}

double PiecewiseTrajectory::curvature(double t) const
{
    // find the piece index
    auto piece_iter = std::lower_bound(time_allocation_cumsum_.begin(), time_allocation_cumsum_.end(), t);
    if(piece_iter == std::end(time_allocation_cumsum_))
    {
        throw std::invalid_argument("The time is out of range.");
    }
    int piece_index = std::distance(time_allocation_cumsum_.begin(), piece_iter);
    // sample the piece
    return trajectories_[piece_index]->get_curvature(t - time_allocation_cumsum_[piece_index]);
    
}

double PiecewiseTrajectory::arc_length(double t0, double t1) const
{
    // find the piece index
    auto piece_iterator0 = std::lower_bound(time_allocation_cumsum_.begin(), time_allocation_cumsum_.end(), t0);
    auto piece_iterator1 = std::lower_bound(time_allocation_cumsum_.begin(), time_allocation_cumsum_.end(), t1);
    if(piece_iterator0 == std::end(time_allocation_cumsum_) || piece_iterator1 == std::end(time_allocation_cumsum_))
    {
        throw std::invalid_argument("The time is out of range.");
    }
    int piece_index0 = std::distance(time_allocation_cumsum_.begin(), piece_iterator0);
    int piece_index1 = std::distance(time_allocation_cumsum_.begin(), piece_iterator1);
    // sample the piece
    double arc_length = 0.0;
    for (int i = piece_index0; i < piece_index1; i++)
    {
        //! @todo wrong implementation, need to be fixed
        arc_length += trajectories_[i]->get_arc_length(t0 - time_allocation_cumsum_[i], t1 - time_allocation_cumsum_[i]);
    }
    return arc_length;
}

int PiecewiseTrajectory::dimension() const
{
    if (!dimension_validated_)
    {
        dimension_validated_ = validate_dimension();
    }
    return dimension_validated_ ? trajectories_[0]->get_dimension() : 0;
}

void PiecewiseTrajectory::insert_piece(int index, Piece &&new_piece, double new_time_allocation)
{
    if (not trajectories_.empty() && new_piece->get_dimension() != dimension())
    {
        throw std::invalid_argument("The dimension of the new piece is not consistent with the dimension of the trajectory.");
    }
    if (new_time_allocation <= 0.0)
    {
        throw std::invalid_argument("The time allocation of the new piece is not positive.");
    }
    // insert the piece
    trajectories_.insert(trajectories_.begin() + index, std::move(new_piece));
    time_allocation_.insert(time_allocation_.begin() + index, new_time_allocation);
    // update the time allocation cumsum
    update_time_allocation_cumsum();
}

PiecewiseTrajectory::Piece PiecewiseTrajectory::swap_piece(int index, Piece &&new_piece, double new_time_allocation)
{
    if (not trajectories_.empty() && new_piece->get_dimension() != dimension())
    {
        throw std::invalid_argument("The dimension of the new piece is not consistent with the dimension of the trajectory.");
    }
    if (new_time_allocation <= 0.0)
    {
        throw std::invalid_argument("The time allocation of the new piece is not positive.");
    }
    // swap the piece
    Piece old_piece = std::move(trajectories_[index]);
    trajectories_[index] = std::move(new_piece);
    time_allocation_[index] = new_time_allocation;
    // update the time allocation cumsum
    update_time_allocation_cumsum();
    return old_piece;
}

PiecewiseTrajectory::TimeRange PiecewiseTrajectory::get_piece_time_range(int index) const
{
    if (index < 0 || index >= (int)trajectories_.size())
    {
        throw std::invalid_argument("The index is out of range.");
    }
    double t0 = index == 0 ? 0.0 : time_allocation_cumsum_[index - 1];
    double t1 = time_allocation_cumsum_[index];
    return std::make_pair(t0, t1);
}

PiecewiseTrajectory::TimeRange PiecewiseTrajectory::get_time_range() const
{
    if (trajectories_.empty())
    {
        return std::make_pair(0.0, 0.0);
    }
    double t0 = 0.0;
    double t1 = time_allocation_cumsum_.back();
    return std::make_pair(t0, t1);
}

bool PiecewiseTrajectory::smooth(int derivative_order) const
{
    if (trajectories_.empty())
    {
        return true;
    }
    if (derivative_order < 0)
    {
        throw std::invalid_argument("The derivative order is negative.");
    }
    auto np = nonsmooth_points(derivative_order);
    return np.empty();
}

bool PiecewiseTrajectory::smoothable(int derivative_order) const
{
    if (trajectories_.empty())
    {
        return true;
    }
    if (derivative_order < 0)
    {
        throw std::invalid_argument("The derivative order is negative.");
    }
    auto np = nonsmoothable_points(derivative_order);
    return np.empty();
}

const std::vector<double> PiecewiseTrajectory::nonsmooth_points(int derivative_order) const
{
    if (trajectories_.empty())
    {
        return std::vector<double>();
    }
    if (derivative_order < 0)
    {
        throw std::invalid_argument("The derivative order is negative.");
    }
    std::vector<double> np;
    for (int i = 0; i < (int)trajectories_.size() - 1; i++)
    {
        double t = time_allocation_cumsum_[i];
        if (trajectories_[i]->get_sample(t, derivative_order) != trajectories_[i + 1]->get_sample(t, derivative_order))
        {
            np.push_back(t);
        }
    }
    return np;
}

const std::vector<double> PiecewiseTrajectory::nonsmoothable_points(int derivative_order) const
{
    if (trajectories_.empty())
    {
        return std::vector<double>();
    }
    if (derivative_order < 0)
    {
        throw std::invalid_argument("The derivative order is negative.");
    }
    std::vector<double> np;
    for (int i = 0; i < (int)trajectories_.size() - 1; i++)
    {
        double t = time_allocation_cumsum_[i];
        if (trajectories_[i]->get_sample(t, derivative_order).norm() != trajectories_[i + 1]->get_sample(t, derivative_order).norm())
        {
            np.push_back(t);
        }
    }
    return np;
}

void PiecewiseTrajectory::update_time_allocation_cumsum()
{
    time_allocation_cumsum_.resize(time_allocation_.size());
    std::partial_sum(time_allocation_.begin(), time_allocation_.end(), time_allocation_cumsum_.begin());
}

bool PiecewiseTrajectory::validate_dimension() const
{
    if (trajectories_.empty())
    {
        return true;
    }
    int dimension = trajectories_[0]->get_dimension();
    for (int i = 1; i < (int)trajectories_.size(); i++)
    {
        if (trajectories_[i]->get_dimension() != dimension)
        {
            return false;
        }
    }
    return true;
}