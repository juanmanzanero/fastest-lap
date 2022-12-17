#include "gtest/gtest.h"
#include "src/core/tire/tire_pacejka.h"

class Tire_pacejka_simple_test : public ::testing::Test
{
 protected:
    Tire_pacejka_simple_test() 
    {
        tire.get_frame().set_parent(inertial_frame);
    }

    sFrame inertial_frame = {};

    Xml_document database = {"./database/vehicles/f1/limebeer-2014-f1.xml", true};
    Tire_pacejka_simple<scalar,0,0> tire = { "tire-test", database, "vehicle/front-tire/"};
};


TEST_F(Tire_pacejka_simple_test, longitudinal_force)
{
    const scalar kappa = -0.05;
    const scalar lambda = 0.01;

    const scalar Fz = 4000.0;

    const scalar Fx = tire.get_model().force_combined_longitudinal_magic(kappa, lambda, Fz);

    const scalar kappa_max = (Fz - 2000.0)*(0.10-0.11)/(6000.0-2000.0) + 0.11;
    const scalar lambda_max = (Fz - 2000.0)*(8.0*DEG - 9.0*DEG)/(6000.0-2000.0) + 9.0*DEG;
    const scalar mu_x_max  = smooth_pos((Fz - 2000.0)*(1.40-1.75)/(6000.0-2000.0) + 1.75-1.0,1.0e-5)+1.0;

    const scalar kappa_n = kappa/kappa_max;
    const scalar lambda_n = lambda/lambda_max;

    const scalar rho = sqrt(kappa_n*kappa_n + lambda_n*lambda_n + 1.0e-12);

    const scalar mu_x = mu_x_max*sin(1.9*atan(0.5*pi*rho/atan(1.9)));

    const scalar Fx_computed = mu_x*Fz*kappa_n/rho;

    EXPECT_DOUBLE_EQ(Fx,Fx_computed);
};


TEST_F(Tire_pacejka_simple_test, lateral_force)
{
    const scalar kappa = 0.04;
    const scalar lambda = -0.06;

    const scalar Fz = 3000.0;

    const scalar Fy = tire.get_model().force_combined_lateral_magic(kappa, lambda, Fz);


    const scalar kappa_max = (Fz - 2000.0)*(0.10-0.11)/(6000.0-2000.0) + 0.11;
    const scalar lambda_max = (Fz - 2000.0)*(8.0*DEG - 9.0*DEG)/(6000.0-2000.0) + 9.0*DEG;
    const scalar mu_y_max  = smooth_pos((Fz - 2000.0)*(1.45-1.80)/(6000.0-2000.0) + 1.80-1.0,1.0e-5)+1.0;

    const scalar kappa_n = kappa/kappa_max;
    const scalar lambda_n = lambda/lambda_max;

    const scalar rho = sqrt(kappa_n*kappa_n + lambda_n*lambda_n + 1.0e-12);

    const scalar mu_y = mu_y_max*sin(1.9*atan(0.5*pi*rho/atan(1.9)));

    const scalar Fy_computed = mu_y*Fz*lambda_n/rho;

    EXPECT_DOUBLE_EQ(Fy,Fy_computed);
};


TEST_F(Tire_pacejka_simple_test, update)
{
    const sVector3d x0 = {0.0,0.0,0.0};
    const sVector3d v0 = {cos(15.0*DEG),sin(15.0*DEG),0.0};
    tire.get_frame().set_origin(x0,v0,sFrame::Frame_velocity_types::parent_frame);
    const scalar Fz = 5555.0;

    const scalar kappa = (1.0-v0[0])/v0[0];
    const scalar lambda = -v0[1]/v0[0]; 

    tire.update(Fz,kappa/tire.get_model().maximum_kappa(Fz), inertial_frame); 

    EXPECT_DOUBLE_EQ(tire.get_omega(), 1/0.33);
    EXPECT_DOUBLE_EQ(tire.get_kappa(), kappa);
    EXPECT_DOUBLE_EQ(tire.get_lambda(), lambda);

    const scalar kappa_max = (Fz - 2000.0)*(0.10-0.11)/(6000.0-2000.0) + 0.11;
    const scalar lambda_max = (Fz - 2000.0)*(8.0*DEG - 9.0*DEG)/(6000.0-2000.0) + 9.0*DEG;
    const scalar mu_x_max  = smooth_pos((Fz - 2000.0)*(1.40-1.75)/(6000.0-2000.0) + 1.75-1.0,1.0e-5)+1.0;
    const scalar mu_y_max  = smooth_pos((Fz - 2000.0)*(1.45-1.80)/(6000.0-2000.0) + 1.80-1.0,1.0e-5)+1.0;

    const scalar kappa_n = kappa/kappa_max;
    const scalar lambda_n = lambda/lambda_max;

    const scalar rho = sqrt(kappa_n*kappa_n + lambda_n*lambda_n + 1.0e-12);

    const scalar mu_x = mu_x_max*sin(1.9*atan(0.5*pi*rho/atan(1.9)));
    const scalar mu_y = mu_y_max*sin(1.9*atan(0.5*pi*rho/atan(1.9)));

    const scalar Fx_computed = mu_x*Fz*kappa_n/rho;
    const scalar Fy_computed = mu_y*Fz*lambda_n/rho;

    EXPECT_DOUBLE_EQ(tire.get_force()[0],Fx_computed);
    EXPECT_DOUBLE_EQ(tire.get_force()[1],Fy_computed);
    EXPECT_DOUBLE_EQ(tire.get_force()[2], -5555.0);
}
