#include "gtest/gtest.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "helper_functions.h"

// Get alias to the limebeer2014f1 dynamic model
using Front_left_tire_t  = limebeer2014f1<scalar>::Front_left_tire_type;
using Front_right_tire_t = limebeer2014f1<scalar>::Front_right_tire_type;
using Rear_left_tire_t   = limebeer2014f1<scalar>::Rear_left_tire_type;
using Rear_right_tire_t  = limebeer2014f1<scalar>::Rear_right_tire_type;
using Front_axle_t       = limebeer2014f1<scalar>::Front_axle_t;
using Rear_axle_t        = limebeer2014f1<scalar>::Rear_axle_t;
using Chassis_t          = limebeer2014f1<scalar>::Chassis_t;
using Road_t             = limebeer2014f1<scalar>::Road_cartesian_t;

class Chassis_car_3dof_test : public ::testing::Test
{
 protected:
    Chassis_car_3dof_test () 
    {
        std::array<scalar,limebeer2014f1<scalar>::cartesian::NSTATE> q_in;
        std::array<scalar,limebeer2014f1<scalar>::cartesian::NCONTROL> u_in;
        std::array<scalar,limebeer2014f1<scalar>::cartesian::NALGEBRAIC> qa_in;

        q_in[Front_axle_t::IKAPPA_LEFT]  = kappa_fl;
        q_in[Front_axle_t::IKAPPA_RIGHT] = kappa_fr;
        q_in[Rear_axle_t::IKAPPA_LEFT]   = kappa_rl;
        q_in[Rear_axle_t::IKAPPA_RIGHT]  = kappa_rr;
        q_in[Chassis_t::IU]              = u;
        q_in[Chassis_t::IV]              = v;
        q_in[Chassis_t::IOMEGA]          = omega;
        q_in[Road_t::IX]                 = x;
        q_in[Road_t::IY]                 = y;
        q_in[Road_t::IPSI]               = psi;

        u_in[Front_axle_t::ISTEERING] = delta;
        u_in[Chassis_t::ITHROTTLE]    = throttle;

        qa_in[Chassis_t::IFZFL] = Fz_fl/(9.81*660.0);
        qa_in[Chassis_t::IFZFR] = Fz_fr/(9.81*660.0);
        qa_in[Chassis_t::IFZRL] = Fz_rl/(9.81*660.0);
        qa_in[Chassis_t::IFZRR] = Fz_rr/(9.81*660.0);

        chassis.set_state_and_controls(q_in,qa_in,u_in);
        chassis.update(x,y,psi);
    }

    // Database containing the model parameters from the reference
    Xml_document database   = {"./database/vehicles/f1/limebeer-2014-f1.xml", true};

    // Chassis object to be tested
    Chassis_t chassis       = {database};

    const Front_left_tire_t& tire_fl = chassis.get_front_axle().template get_tire<0>();
    const Front_right_tire_t& tire_fr = chassis.get_front_axle().template get_tire<1>();
    const Rear_left_tire_t& tire_rl = chassis.get_rear_axle().template get_tire<0>();
    const Rear_right_tire_t& tire_rr = chassis.get_rear_axle().template get_tire<1>();

    // Choose a state to evaluate the dynamics
    const scalar vel = 4.0;

    const scalar omega_fl = vel*0.95/0.33;
    const scalar omega_fr = vel*1.00/0.33;
    const scalar omega_rl = vel*1.05/0.33;
    const scalar omega_rr = vel*0.95/0.33;
    const scalar x = 0.0;
    const scalar y = 0.0;
    const scalar psi = 15.0*DEG;
    const scalar u = vel*cos(psi);
    const scalar v = vel*sin(psi);
    const scalar omega = 0.3;

    const scalar delta = -10.0*DEG;
    const scalar throttle = 0.5;

    const scalar Fz_fl = -2222.0;
    const scalar Fz_fr = -3333.0;
    const scalar Fz_rl = -4444.0;
    const scalar Fz_rr = -5555.0;

    const scalar neg_Fz_fl = 0.5*(Fz_fl - sqrt(Fz_fl*Fz_fl + 1.0));
    const scalar neg_Fz_fr = 0.5*(Fz_fr - sqrt(Fz_fr*Fz_fr + 1.0));
    const scalar neg_Fz_rl = 0.5*(Fz_rl - sqrt(Fz_rl*Fz_rl + 1.0));
    const scalar neg_Fz_rr = 0.5*(Fz_rr - sqrt(Fz_rr*Fz_rr + 1.0));

    // Formulas directly taken from Limebeer 2014 et al.
    const scalar kappa_fl = -(1.0 - 0.33*omega_fl/(cos(delta)*(u+omega*0.73) + sin(delta)*(omega*1.8+v)));
    const scalar kappa_fr = -(1.0 - 0.33*omega_fr/(cos(delta)*(u-omega*0.73) + sin(delta)*(omega*1.8+v)));
    const scalar kappa_rl = -(1.0 - 0.33*omega_rl/(u+omega*0.73));
    const scalar kappa_rr = -(1.0 - 0.33*omega_rr/(u-omega*0.73));

    const scalar lambda_fl = -(cos(delta)*(omega*1.8+v)-sin(delta)*(omega*0.73+u))/(cos(delta)*(omega*0.73+u)+sin(delta)*(omega*1.8+v));
    const scalar lambda_fr = -(cos(delta)*(omega*1.8+v)+sin(delta)*(omega*0.73-u))/(cos(delta)*(u-omega*0.73)+sin(delta)*(omega*1.8+v));
    const scalar lambda_rl = -(v-omega*1.6)/(u+omega*0.73);
    const scalar lambda_rr = -(v-omega*1.6)/(u-omega*0.73);

    // Compute tire forces
    const sVector3d F_fl = get_tire_forces(kappa_fl, lambda_fl, -neg_Fz_fl);
    const sVector3d F_fr = get_tire_forces(kappa_fr, lambda_fr, -neg_Fz_fr);
    const sVector3d F_rl = get_tire_forces(kappa_rl, lambda_rl, -neg_Fz_rl);
    const sVector3d F_rr = get_tire_forces(kappa_rr, lambda_rr, -neg_Fz_rr);

    // Compute aerodynamic forces
    const scalar F_lift = 0.5*1.2*u*u*1.5*3.0;
    const scalar F_drag = -0.5*1.2*u*u*1.5*0.9;

    // Compute total forces
    const scalar Fx = cos(delta)*(F_fr[0]+F_fl[0])-sin(delta)*(F_fr[1]+F_fl[1]) + F_rr[0] + F_rl[0] + F_drag;
    const scalar Fy = cos(delta)*(F_fr[1]+F_fl[1])+sin(delta)*(F_fr[0]+F_fl[0]) + F_rr[1] + F_rl[1];
};


TEST_F(Chassis_car_3dof_test, positions)
{
    // Position of the front axle center
    EXPECT_DOUBLE_EQ(chassis.get_front_axle().get_frame().get_origin()[0], 1.8);
    EXPECT_DOUBLE_EQ(chassis.get_front_axle().get_frame().get_origin()[1], 0.0);
    EXPECT_DOUBLE_EQ(chassis.get_front_axle().get_frame().get_origin()[2], -0.33);

    // Position of the rear axle center
    EXPECT_DOUBLE_EQ(chassis.get_rear_axle().get_frame().get_origin()[0], -1.6);
    EXPECT_DOUBLE_EQ(chassis.get_rear_axle().get_frame().get_origin()[1], 0.0);
    EXPECT_DOUBLE_EQ(chassis.get_rear_axle().get_frame().get_origin()[2], -0.33);

    // Position of the front left tire center
    EXPECT_DOUBLE_EQ(std::get<0>(tire_fl.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[0], 1.8);
    EXPECT_DOUBLE_EQ(std::get<0>(tire_fl.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[1], -0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(tire_fl.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[2], -0.33);

    // Position of the front right tire center
    EXPECT_DOUBLE_EQ(std::get<0>(tire_fr.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[0], 1.8);
    EXPECT_DOUBLE_EQ(std::get<0>(tire_fr.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[1],  0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(tire_fr.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[2], -0.33);

    // Position of the rear left tire center
    EXPECT_DOUBLE_EQ(std::get<0>(tire_rl.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[0], -1.6);
    EXPECT_DOUBLE_EQ(std::get<0>(tire_rl.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[1], -0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(tire_rl.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[2], -0.33);

    // Position of the rear right tire center
    EXPECT_DOUBLE_EQ(std::get<0>(tire_rr.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[0], -1.6);
    EXPECT_DOUBLE_EQ(std::get<0>(tire_rr.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[1],  0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(tire_rr.get_frame().get_position_and_velocity_in_target(chassis.get_chassis_frame()))[2], -0.33);
}


TEST_F(Chassis_car_3dof_test, velocities)
{
    // The model represents a 3DOF dynamics, hence all relative velocities of the parts must be zero 
    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_DOUBLE_EQ(chassis.get_front_axle().get_frame().get_relative_velocity()[i], 0.0);
        EXPECT_DOUBLE_EQ(chassis.get_rear_axle().get_frame().get_relative_velocity()[i], 0.0);
    
        EXPECT_DOUBLE_EQ(tire_fl.get_frame().get_relative_velocity()[i], 0.0);
        EXPECT_DOUBLE_EQ(tire_fr.get_frame().get_relative_velocity()[i], 0.0);
        EXPECT_DOUBLE_EQ(tire_rl.get_frame().get_relative_velocity()[i], 0.0);
        EXPECT_DOUBLE_EQ(tire_rr.get_frame().get_relative_velocity()[i], 0.0);
    }
}


TEST_F(Chassis_car_3dof_test, check_set_state)
{
    EXPECT_DOUBLE_EQ(chassis.get_front_axle().template get_tire<0>().get_omega() , omega_fl);
    EXPECT_DOUBLE_EQ(chassis.get_front_axle().template get_tire<1>().get_omega(), omega_fr);
    EXPECT_DOUBLE_EQ(chassis.get_rear_axle().template get_tire<0>().get_omega()  , omega_rl);
    EXPECT_DOUBLE_EQ(chassis.get_rear_axle().template get_tire<1>().get_omega() , omega_rr);

    EXPECT_DOUBLE_EQ(tire_fl.get_omega(), omega_fl); 
    EXPECT_DOUBLE_EQ(tire_fr.get_omega(), omega_fr); 
    EXPECT_DOUBLE_EQ(tire_rl.get_omega(), omega_rl); 
    EXPECT_DOUBLE_EQ(tire_rr.get_omega(), omega_rr); 

    EXPECT_DOUBLE_EQ(chassis.get_negative_normal_force(Chassis_t::IFZFL), neg_Fz_fl);
    EXPECT_DOUBLE_EQ(chassis.get_negative_normal_force(Chassis_t::IFZFR), neg_Fz_fr);
    EXPECT_DOUBLE_EQ(chassis.get_negative_normal_force(Chassis_t::IFZRL), neg_Fz_rl);
    EXPECT_DOUBLE_EQ(chassis.get_negative_normal_force(Chassis_t::IFZRR), neg_Fz_rr);

    EXPECT_DOUBLE_EQ(tire_fl.get_force()[Z], neg_Fz_fl);
    EXPECT_DOUBLE_EQ(tire_fr.get_force()[Z], neg_Fz_fr);
    EXPECT_DOUBLE_EQ(tire_rl.get_force()[Z], neg_Fz_rl);
    EXPECT_DOUBLE_EQ(tire_rr.get_force()[Z], neg_Fz_rr);

    EXPECT_DOUBLE_EQ(chassis.get_u(), u);
    EXPECT_DOUBLE_EQ(chassis.get_v(), v);
    EXPECT_DOUBLE_EQ(chassis.get_omega(), omega);

    EXPECT_DOUBLE_EQ(chassis.get_throttle(), throttle);
    EXPECT_DOUBLE_EQ(chassis.get_front_axle().get_steering_angle(), delta);
}


TEST_F(Chassis_car_3dof_test, kappa_and_lambda)
{
    EXPECT_NEAR(tire_fl.get_kappa(), kappa_fl, 1.0e-12);
    EXPECT_NEAR(tire_fr.get_kappa(), kappa_fr, 1.0e-12);
    EXPECT_NEAR(tire_rl.get_kappa(), kappa_rl, 1.0e-12);
    EXPECT_NEAR(tire_rr.get_kappa(), kappa_rr, 1.0e-12);

    EXPECT_NEAR(tire_fl.get_lambda(), lambda_fl, 1.0e-12);
    EXPECT_NEAR(tire_fr.get_lambda(), lambda_fr, 1.0e-12);
    EXPECT_NEAR(tire_rl.get_lambda(), lambda_rl, 1.0e-12);
    EXPECT_NEAR(tire_rr.get_lambda(), lambda_rr, 1.0e-12);
}


TEST_F(Chassis_car_3dof_test, tire_forces)
{
    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(tire_fl.get_force()[i], F_fl[i], 2.0e-11);
        EXPECT_NEAR(tire_fr.get_force()[i], F_fr[i], 2.0e-11);
        EXPECT_NEAR(tire_rl.get_force()[i], F_rl[i], 2.0e-11);
        EXPECT_NEAR(tire_rr.get_force()[i], F_rr[i], 2.0e-11);
    }
}


TEST_F(Chassis_car_3dof_test, newton_equations)
{
    std::array<scalar,4> dqa;
    chassis.get_algebraic_constraints(dqa);
    const scalar du = omega*v + Fx/660.0;
    const scalar dv = -omega*u + Fy/660.0;
    const scalar dw = F_fl[2]+F_fr[2]+F_rl[2]+F_rr[2]+660.0*9.81+F_lift; 

    EXPECT_NEAR(chassis.get_du(), du, 2.0e-11);
    EXPECT_NEAR(chassis.get_dv(), dv, 2.0e-11);
    EXPECT_NEAR(dqa[0]*9.81*660.0, dw, 2.0e-11);
}


TEST_F(Chassis_car_3dof_test, euler_equations)
{
    std::array<scalar,4> dqa;
    chassis.get_algebraic_constraints(dqa);

    const scalar domega = (1.8*(cos(delta)*(F_fr[1]+F_fl[1])+sin(delta)*(F_fr[0]+F_fl[0])) + 0.73*(sin(delta)*F_fr[1]-cos(delta)*F_fr[0]) - 0.73*F_rr[0] + 0.73*(cos(delta)*F_fl[0] - sin(delta)*F_fl[1]) + 0.73*F_rl[0] - 1.6*(F_rr[1]+F_rl[1]))/450.0;

    EXPECT_NEAR(dqa[1]*9.81*660.0, 0.73*(neg_Fz_rl-neg_Fz_rr)+0.73*(neg_Fz_fl-neg_Fz_fr) + 0.3*Fy, 2.0e-11); 
    EXPECT_NEAR(dqa[2]*9.81*660.0, 1.6*(neg_Fz_rr+neg_Fz_rl)-1.8*(neg_Fz_fr+neg_Fz_fl)+ 0.3*Fx + 0.1*F_lift, 2.0e-11);
    EXPECT_NEAR(chassis.get_domega(), domega, 2.0e-11);
}


TEST_F(Chassis_car_3dof_test, suspension_compliance_equation)
{
    std::array<scalar,4> dqa;
    chassis.get_algebraic_constraints(dqa);

    EXPECT_NEAR(dqa[3]*9.81*660.0, Fz_fr - Fz_fl - 0.5*(Fz_fr + Fz_rr - Fz_fl - Fz_rl), 2.0e-11);
}
