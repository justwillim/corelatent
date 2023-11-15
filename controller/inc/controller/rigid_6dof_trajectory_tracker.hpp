#pragma once
#include "interface/trajectory.hpp"
#include <memory>
#include <Eigen/Eigen>

class Rigid6dofTrajectoryTracker
{
public:
    using State = Eigen::Matrix4d;
    using Error = Eigen::Matrix4d;

public:
    enum class SystemError{
        None,
        TranslationOutofRange,
        RotationOutofRange,
    };

public:
    virtual void applyTranslation(std::unique_ptr<TrajectoryBase> &&translation) = 0;
    virtual void applyRotation(std::unique_ptr<TrajectoryBase> &&rotation) = 0;
    virtual State generate(double t) = 0;
};