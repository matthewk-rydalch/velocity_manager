#include "gtest/gtest.h"
#include "include/velocity_manager/velocity_manager.h"
#include "include/velocity_manager/velocity_manager_class.h"

void test_3d_vectors(Eigen::Vector3d vector1, Eigen::Vector3d vector2)
{
    double tolerance{0.001};
    EXPECT_NEAR(vector1[0],vector2[0],tolerance);
    EXPECT_NEAR(vector1[1],vector2[1],tolerance);
    EXPECT_NEAR(vector1[2],vector2[2],tolerance);
}

TEST(VelocityManager,ApplyGains)
{
    Eigen::Vector3d gains{1.2,1.2,0.8};
    Eigen::Vector3d state{0.5,-1.1,3.0};
    Eigen::Vector3d velocityCommandsComponent{apply_gains(gains,state)};

    Eigen::Vector3d expectedVelocityCommandsComponent{0.6,-1.32,2.4};
    test_3d_vectors(velocityCommandsComponent,expectedVelocityCommandsComponent);
}

TEST(VelocityManager,CalculateDerivatives)
{
    Eigen::Vector3d state{1.2,-0.3,0.8};
    Eigen::Vector3d prevState{1.1,-0.6,1.0};
    double dt{0.2};
    Eigen::Vector3d stateDerivatives{calculate_derivatives(state,prevState,dt)};

    Eigen::Vector3d expectedStateDerivatives{0.5,1.5,-1.0};
    test_3d_vectors(stateDerivatives,expectedStateDerivatives);
}

TEST(VelocityManager,LowPassFilterDerivatives)
{
    Eigen::Vector3d derivatives{0.5,1.5,-1.0};
    Eigen::Vector3d prevDerivates{0.7,1.1,-0.7};
    double dt{0.2};
    double sigma{0.3};
    Eigen::Vector3d filteredDerivatives{low_pass_filter(derivatives,prevDerivates,sigma,dt)};

    Eigen::Vector3d expectedFilteredDerivatives{3.1/5.0,6.3/5.0,-4.1/5.0};
    test_3d_vectors(filteredDerivatives,expectedFilteredDerivatives);
}

TEST(VelocityManager,CalculateIntegrators)
{
    Eigen::Vector3d state{-1.0,-0.5,1.8};
    Eigen::Vector3d prevState{-1.1,-0.6,1.4};
    Eigen::Vector3d prevIntegrators{3.1,-0.02,0.11};
    double dt{0.2};
    Eigen::Vector3d integrators{calculate_integrators(state,prevState,prevIntegrators,dt)};

    Eigen::Vector3d expectedIntegrators{3.12,0.0,0.19};
    test_3d_vectors(integrators,expectedIntegrators);
}

TEST(VelocityManagerClass,updateControlParameters)
{
    Eigen::Vector3d initialRelativePosition{1.0,1.1,1.0};
    VelocityManager velMan(initialRelativePosition);
    Eigen::Vector3d relativePosition{1.2,1.2,0.8};
    double dt{0.1};
    velMan.update_control_parameters(relativePosition,dt);

    Eigen::Vector3d expectedRelativePosition{1.2,1.2,0.8};
    Eigen::Vector3d expectedRelativeDerivative{0.5,0.25,-0.5};
    Eigen::Vector3d expectedRelativeIntegrator{0.02,0.01,-0.02};
    test_3d_vectors(velMan.get_relative_position(),expectedRelativePosition);
    test_3d_vectors(velMan.get_relative_derivative(),expectedRelativeDerivative);
    test_3d_vectors(velMan.get_relative_integrator(),expectedRelativeIntegrator);
}

//TEST(VelocityManagerClass,calculateProportionalControl)
//{
//    Eigen::Vector3d relativePosition{1.2,1.2,0.8};
//    Eigen::Vector3d proportionalGains{0.5,-1.1,3.0};
//    Eigen::Vector3d velocityCommandsComponent{computeControl()};

//    Eigen::Vector3d expectedVelocityCommandsComponent{0.6,-1.32,2.4};
//    test_3d_vectors(velocityCommandsComponent,expectedVelocityCommandsComponent);
//}


