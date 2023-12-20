#pragma once
#include <Eigen/Eigen>
// #include <stdio.h>
#include <iostream>

class Jerk2DController
{
public:
    using State = Eigen::VectorXd; // x, y, vx, vy, ax, ay
    using Control = Eigen::Vector2d;
    using Pid3D = Eigen::Matrix3d; // Kp, Ki, Kd for p, v, a

public:
    Jerk2DController(const Pid3D &pid = Pid3D::Ones(), double clip = 1.0)
        : pid_(pid), clip_(clip)
    {
        prev_error_x_ = Eigen::Vector3d::Zero();
        prev_error_y_ = Eigen::Vector3d::Zero();
        intt_error_x_ = Eigen::Vector3d::Zero();
        intt_error_y_ = Eigen::Vector3d::Zero();
    }
    virtual ~Jerk2DController() = default;

public:
    virtual Control control(const State &current_state, const State &desired_state)
    {
        Eigen::VectorXd error = desired_state - current_state;

        // nan to 0
        for (int i = 0; i < error.size(); i++)
        {
            if (std::isnan(error(i)))
            {
                error(i) = 0.0;
            }
        }
        // x, y, vx, vy, ax, ay to x, vx, ax, y, vy, ay
        Eigen::PermutationMatrix<6> perm;
        perm.indices() = {0, 3, 1, 4, 2, 5};
        error =  perm * error;
        auto error_x = error.block<3, 1>(0, 0);
        auto error_y = error.block<3, 1>(3, 0);

        // update intt_error
        intt_error_x_ += error_x;
        intt_error_y_ += error_y;
        // clip the intt_error
        intt_error_x_ = intt_error_x_.cwiseMax(-clip_).cwiseMin(clip_);
        intt_error_y_ = intt_error_y_.cwiseMax(-clip_).cwiseMin(clip_);
        // e_x | intt_x | prev_x
        Eigen::Matrix3d error_matrix_x = Eigen::Matrix3d::Zero();
        error_matrix_x.block<3, 1>(0, 0) = error_x;
        error_matrix_x.block<3, 1>(0, 1) = intt_error_x_;
        error_matrix_x.block<3, 1>(0, 2) = error_x - prev_error_x_;
        // e_y | intt_y | prev_y
        Eigen::Matrix3d error_matrix_y = Eigen::Matrix3d::Zero();
        error_matrix_y.block<3, 1>(0, 0) = error_y;
        error_matrix_y.block<3, 1>(0, 1) = intt_error_y_;
        error_matrix_y.block<3, 1>(0, 2) = error_y - prev_error_y_;

        // pid
        auto control_x = (error_matrix_x * pid_).trace();
        auto control_y = (error_matrix_y * pid_).trace();
        std::cout << "desired_state: " << desired_state.transpose() << std::endl;
        std::cout << "current_state: " << current_state.transpose() << std::endl;
        std::cout << "error_x: " << error_matrix_x << std::endl;
        std::cout << "error_y: " << error_matrix_y << std::endl;
        std::cout << "control_x: " << control_x << std::endl;
        std::cout << "control_y: " << control_y << std::endl;

        // update prev_error
        prev_error_x_ = error_x;
        prev_error_y_ = error_y;

        return Control(control_x, control_y);
    }

public:
    Pid3D pid() const { return pid_; }
    double clip() const { return clip_; }
    Eigen::Vector3d prev_error_x() const { return prev_error_x_; }
    Eigen::Vector3d prev_error_y() const { return prev_error_y_; }
    Eigen::Vector3d intt_error_x() const { return intt_error_x_; }
    Eigen::Vector3d intt_error_y() const { return intt_error_y_; }

private:
    Eigen::Vector3d prev_error_x_;
    Eigen::Vector3d prev_error_y_;
    Eigen::Vector3d intt_error_x_;
    Eigen::Vector3d intt_error_y_;
    Pid3D pid_;
    double clip_ = 1.0;
};