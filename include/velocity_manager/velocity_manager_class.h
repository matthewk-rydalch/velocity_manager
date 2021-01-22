#include <Eigen/Dense>

#include "include/velocity_manager/velocity_manager.h"

class VelocityManager
{
public:
    VelocityManager(Eigen::Vector3d initialRelativePosition);
    void set_gains(Eigen::Vector3d Kp, Eigen::Vector3d Kd, Eigen::Vector3d Ki);
    void update_control_parameters(Eigen::Vector3d relativePosition,double dt);
    void compute_control();
    Eigen::Vector3d get_velocity_control();


    //For testing
    Eigen::Vector3d get_relative_position();
    Eigen::Vector3d get_relative_derivative();
    Eigen::Vector3d get_relative_integrator();

private:
    Eigen::Vector3d mRelativePosition;
    Eigen::Vector3d mRelativeDerivative;
    Eigen::Vector3d mRelativeIntegrator;
    double mDt;
    double mSigma;
};
