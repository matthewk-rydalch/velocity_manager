#include "include/velocity_manager/velocity_manager_class.h"

VelocityManager::VelocityManager(Eigen::Vector3d initialRelativePosition) :
    mRelativePosition{initialRelativePosition},
    mRelativeDerivative{0.0,0.0,0.0},
    mRelativeIntegrator{0.0,0.0,0.0},
    mDt{0.2},
    mSigma{0.3}
{}

void VelocityManager::set_gains(Eigen::Vector3d proportionalGains,Eigen::Vector3d derivativeGains,Eigen::Vector3d integrallGains)
{
    mKp = proportionalGains;
    mKd = derivativeGains;
    mKi = integralGains;
}

void VelocityManager::update_control_parameters(Eigen::Vector3d relativePosition,double dt)
{
    mDt = dt;
    Eigen::Vector3d relativeDerivative{calculate_derivatives(relativePosition,mRelativePosition,mDt)};
    mRelativeDerivative = low_pass_filter(relativeDerivative,mRelativeDerivative,mSigma,mDt);
    mRelativeIntegrator = calculate_integrators(relativePosition,mRelativePosition,mRelativeIntegrator,mDt);
    mRelativePosition = relativePosition;
}

Eigen::Vector3d VelocityManager::get_relative_position()
{
    return mRelativePosition;
}

Eigen::Vector3d VelocityManager::get_relative_derivative()
{
    return mRelativeDerivative;
}

Eigen::Vector3d VelocityManager::get_relative_integrator()
{
    return mRelativeIntegrator;
}
