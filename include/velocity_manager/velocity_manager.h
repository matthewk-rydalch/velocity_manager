#include <iostream>
#include <array>
#include <Eigen/Dense>

#include "include/utils/math.h"

void hello_there();
Eigen::Vector3d apply_gains(Eigen::Vector3d gains, Eigen::Vector3d state);
Eigen::Vector3d calculate_derivatives(Eigen::Vector3d state, Eigen::Vector3d prevState, double dt);
Eigen::Vector3d low_pass_filter(Eigen::Vector3d derivatives, Eigen::Vector3d prevDerivates, double sigma, double dt);
Eigen::Vector3d calculate_integrators(Eigen::Vector3d state, Eigen::Vector3d prevState, Eigen::Vector3d prevIntegrators, double dt);

