#include "include/velocity_manager/velocity_manager.h"

Eigen::Vector3d apply_gains(Eigen::Vector3d gains, Eigen::Vector3d state)
{
    Eigen::Vector3d velocityCommandsComponent;
    velocityCommandsComponent = gains.array()*state.array();
    return velocityCommandsComponent;
}

Eigen::Vector3d calculate_derivatives(Eigen::Vector3d state, Eigen::Vector3d prevState, double dt)
{
    Eigen::Vector3d stateDerivatives;
    stateDerivatives = (state-prevState)/dt;
    return stateDerivatives;
}

Eigen::Vector3d low_pass_filter(Eigen::Vector3d derivatives, Eigen::Vector3d prevDerivates, double sigma, double dt)
{
    Eigen::Vector3d filteredDerivatives;
    filteredDerivatives = (derivatives*dt)/(sigma+dt) + (prevDerivates*sigma)/(sigma+dt);
    return filteredDerivatives;
}

Eigen::Vector3d calculate_integrators(Eigen::Vector3d state, Eigen::Vector3d prevState, Eigen::Vector3d prevIntegrators, double dt)
{
    Eigen::Vector3d integrators;
    integrators = prevIntegrators+(state-prevState)*dt;
    return integrators;
}


