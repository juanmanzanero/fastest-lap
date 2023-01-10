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
        std::array<scalar,limebeer2014f1<scalar>::cartesian::number_of_inputs> q_in;
        std::array<scalar,limebeer2014f1<scalar>::cartesian::number_of_controls> u_in;

        q_in[Front_axle_t::input_names::KAPPA_LEFT]  = kappa_fl/tire_fl.get_model().maximum_kappa(-neg_Fz_fl);
        q_in[Front_axle_t::input_names::KAPPA_RIGHT] = kappa_fr/tire_fr.get_model().maximum_kappa(-neg_Fz_fr);
        q_in[Rear_axle_t::input_names::KAPPA_LEFT]   = kappa_rl/tire_rl.get_model().maximum_kappa(-neg_Fz_rl);
        q_in[Rear_axle_t::input_names::KAPPA_RIGHT]  = kappa_rr/tire_rr.get_model().maximum_kappa(-neg_Fz_rr);
        q_in[Chassis_t::input_names::velocity_x_mps] = u;
        q_in[Chassis_t::input_names::velocity_y_mps] = v;
        q_in[Chassis_t::input_names::yaw_rate_radps] = omega;
        q_in[Chassis_t::input_names::force_z_fl_g] = Fz_fl/(9.81*660.0);
        q_in[Chassis_t::input_names::force_z_fr_g] = Fz_fr/(9.81*660.0);
        q_in[Chassis_t::input_names::force_z_rl_g] = Fz_rl/(9.81*660.0);
        q_in[Chassis_t::input_names::force_z_rr_g] = Fz_rr/(9.81*660.0);
        q_in[Road_t::input_names::X]                 = x;
        q_in[Road_t::input_names::Y]                 = y;
        q_in[Road_t::input_names::PSI]               = psi;

        u_in[Front_axle_t::control_names::STEERING] = delta;
        u_in[Rear_axle_t::control_names::boost]     = 0.0;
        u_in[Chassis_t::control_names::throttle]    = throttle;
        u_in[Chassis_t::control_names::brake_bias]  = 0.6;


        chassis.set_state_and_controls(q_in,u_in);
        chassis.update({ x,y,0 }, { 0,0,0 }, psi, { 0,0,0}, omega, 0.0);
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
    const scalar Fx_drag = -0.5*1.2*sqrt(u*u+v*v)*u*1.5*0.9;
    const scalar Fy_drag = -0.5*1.2*sqrt(u*u+v*v)*v*1.5*0.9;

    // Compute total forces
    const scalar Fx = cos(delta)*(F_fr[0]+F_fl[0])-sin(delta)*(F_fr[1]+F_fl[1]) + F_rr[0] + F_rl[0] + Fx_drag;
    const scalar Fy = cos(delta)*(F_fr[1]+F_fl[1])+sin(delta)*(F_fr[0]+F_fl[0]) + F_rr[1] + F_rl[1] + Fy_drag;
};


TEST_F(Chassis_car_3dof_test, positions)
{
    // Position of the front axle center
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_front_axle().get_frame().get_origin()[0], 1.8);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_front_axle().get_frame().get_origin()[1], 0.0);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_front_axle().get_frame().get_origin()[2], -0.33);

    // Position of the rear axle center
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_rear_axle().get_frame().get_origin()[0], -1.6);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_rear_axle().get_frame().get_origin()[1], 0.0);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_rear_axle().get_frame().get_origin()[2], -0.33);

    // Position of the front left tire center
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[0], 1.8);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[1], -0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[2], -0.33);

    // Position of the front right tire center
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[0], 1.8);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[1],  0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[2], -0.33);

    // Position of the rear left tire center
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[0], -1.6);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[1], -0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[2], -0.33);

    // Position of the rear right tire center
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[0], -1.6);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[1],  0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame()))[2], -0.33);
}

TEST_F(Chassis_car_3dof_test, transform_states_to_input_states)
{
    // Call to transform states to input states shall always result in an exception
    try
    {
        std::array<scalar, 0> x;
        chassis.transform_states_to_inputs(x,x,x);
        FAIL();
    }
    catch (const fastest_lap_exception& ex)
    {
        SUCCEED();
    }
    catch (...)
    {
        FAIL();
    }
}

TEST_F(Chassis_car_3dof_test, velocities)
{
    // The model represents a 3DOF dynamics, hence all relative velocities of the parts must be zero 
    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_DOUBLE_EQ(std::as_const(chassis).get_front_axle().get_frame().get_relative_velocity_in_parent()[i], 0.0);
        EXPECT_DOUBLE_EQ(std::as_const(chassis).get_rear_axle().get_frame().get_relative_velocity_in_parent()[i], 0.0);
    
        EXPECT_DOUBLE_EQ(std::as_const(tire_fl).get_frame().get_relative_velocity_in_parent()[i], 0.0);
        EXPECT_DOUBLE_EQ(std::as_const(tire_fr).get_frame().get_relative_velocity_in_parent()[i], 0.0);
        EXPECT_DOUBLE_EQ(std::as_const(tire_rl).get_frame().get_relative_velocity_in_parent()[i], 0.0);
        EXPECT_DOUBLE_EQ(std::as_const(tire_rr).get_frame().get_relative_velocity_in_parent()[i], 0.0);
    }
}


TEST_F(Chassis_car_3dof_test, get_gravity_force)
{
    EXPECT_DOUBLE_EQ(chassis.get_gravity_force().x(), 0.0);
    EXPECT_DOUBLE_EQ(chassis.get_gravity_force().y(), 0.0);
    EXPECT_DOUBLE_EQ(chassis.get_gravity_force().z(), 660.0*9.81);
}

TEST_F(Chassis_car_3dof_test, tires_absolute_velocities)
{
    EXPECT_DOUBLE_EQ(std::as_const(tire_fr).get_frame().get_absolute_velocity_in_body().x(), cos(delta) * (u - omega * 0.73) + sin(delta) * (omega * 1.8 + v));
    EXPECT_DOUBLE_EQ(std::as_const(tire_fr).get_frame().get_absolute_velocity_in_body().y(), sin(delta) * (omega * 0.73 - u) + cos(delta) * (omega * 1.8 + v));
    EXPECT_DOUBLE_EQ(std::as_const(tire_fr).get_frame().get_absolute_velocity_in_body().z(), 0.0);

    EXPECT_DOUBLE_EQ(std::as_const(tire_fl).get_frame().get_absolute_velocity_in_body().x(), cos(delta) * (u + omega * 0.73) + sin(delta) * (omega * 1.8 + v));
    EXPECT_DOUBLE_EQ(std::as_const(tire_fl).get_frame().get_absolute_velocity_in_body().y(), -sin(delta) * (omega * 0.73 + u) + cos(delta) * (omega * 1.8 + v));
    EXPECT_DOUBLE_EQ(std::as_const(tire_fl).get_frame().get_absolute_velocity_in_body().z(), 0.0);
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

    EXPECT_DOUBLE_EQ(chassis.get_negative_normal_force(Chassis_t::input_names::force_z_fl_g), neg_Fz_fl);
    EXPECT_DOUBLE_EQ(chassis.get_negative_normal_force(Chassis_t::input_names::force_z_fr_g), neg_Fz_fr);
    EXPECT_DOUBLE_EQ(chassis.get_negative_normal_force(Chassis_t::input_names::force_z_rl_g), neg_Fz_rl);
    EXPECT_DOUBLE_EQ(chassis.get_negative_normal_force(Chassis_t::input_names::force_z_rr_g), neg_Fz_rr);

    EXPECT_DOUBLE_EQ(tire_fl.get_force()[Z], neg_Fz_fl);
    EXPECT_DOUBLE_EQ(tire_fr.get_force()[Z], neg_Fz_fr);
    EXPECT_DOUBLE_EQ(tire_rl.get_force()[Z], neg_Fz_rl);
    EXPECT_DOUBLE_EQ(tire_rr.get_force()[Z], neg_Fz_rr);

    EXPECT_DOUBLE_EQ(chassis.get_u(), u);
    EXPECT_DOUBLE_EQ(chassis.get_v(), v);
    EXPECT_DOUBLE_EQ(chassis.get_yaw_rate_radps(), omega);

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
    const scalar du = omega*v + Fx/660.0;
    const scalar dv = -omega*u + Fy/660.0;
    const scalar dw = F_fl[2]+F_fr[2]+F_rl[2]+F_rr[2]+660.0*9.81+F_lift; 

    EXPECT_NEAR(chassis.get_com_velocity_x_dot_mps2(), du, 2.0e-11);
    EXPECT_NEAR(chassis.get_com_velocity_y_dot_mps2(), dv, 2.0e-11);
    EXPECT_NEAR(chassis.get_com_velocity_z_dot_mps2()*660.0, dw, 2.0e-11);
}


TEST_F(Chassis_car_3dof_test, euler_equations)
{
    const scalar domega = (-0.1*Fy_drag + 1.8*(cos(delta)*(F_fr[1]+F_fl[1])+sin(delta)*(F_fr[0]+F_fl[0])) + 0.73*(sin(delta)*F_fr[1]-cos(delta)*F_fr[0]) - 0.73*F_rr[0] + 0.73*(cos(delta)*F_fl[0] - sin(delta)*F_fl[1]) + 0.73*F_rl[0] - 1.6*(F_rr[1]+F_rl[1]))/450.0;

    EXPECT_NEAR(-chassis.get_roll_angular_momentum_dot_Nm(), 0.73*(neg_Fz_rl-neg_Fz_rr)+0.73*(neg_Fz_fl-neg_Fz_fr) + 0.3*Fy, 2.0e-11); 
    EXPECT_NEAR(chassis.get_pitch_angular_momentum_dot_Nm(), 1.6*(neg_Fz_rr+neg_Fz_rl)-1.8*(neg_Fz_fr+neg_Fz_fl)+ 0.3*Fx + 0.1*F_lift, 2.0e-11);
    EXPECT_NEAR(chassis.get_yaw_rate_dot_radps2(), domega, 2.0e-11);
}


TEST_F(Chassis_car_3dof_test, suspension_compliance_equation)
{
    EXPECT_NEAR(chassis.get_roll_balance_equation_N(), Fz_fr - Fz_fl - 0.5*(Fz_fr + Fz_rr - Fz_fl - Fz_rl), 2.0e-11);
}


TEST_F(Chassis_car_3dof_test, aerodynamic_force_air_in_calm)
{
    EXPECT_NEAR(Fx_drag, chassis.get_aerodynamic_force().drag.x(), 2.0e-11);
    EXPECT_NEAR(Fy_drag, chassis.get_aerodynamic_force().drag.y(), 2.0e-11);
    EXPECT_NEAR(0.0,     chassis.get_aerodynamic_force().drag.z(), 2.0e-11);
    EXPECT_NEAR(0.0,     chassis.get_aerodynamic_force().lift.x(), 2.0e-11);
    EXPECT_NEAR(0.0,     chassis.get_aerodynamic_force().lift.y(), 2.0e-11);
    EXPECT_NEAR(F_lift,  chassis.get_aerodynamic_force().lift.z(), 2.0e-11);
}


TEST_F(Chassis_car_3dof_test, aerodynamic_force_wind)
{
    const double northward_wind = 50.0 * KMH;
    const double eastward_wind  = 30.0 * KMH;

    // Set wind
    chassis.set_parameter("vehicle/chassis/aerodynamics/wind_velocity/northward", northward_wind);
    chassis.set_parameter("vehicle/chassis/aerodynamics/wind_velocity/eastward",  eastward_wind);

    // Compute wind velocity in body axes
    const auto wind_velocity_x_body = eastward_wind * cos(psi) - northward_wind * sin(psi);
    const auto wind_velocity_y_body = -northward_wind * cos(psi) - eastward_wind * sin(psi);

    // Compute aero velocity
    const auto aero_velocity_x = - u + wind_velocity_x_body;
    const auto aero_velocity_y = - v + wind_velocity_y_body;

    // Compute aero forces
    const auto drag_x = 0.5 * 1.2 * sqrt(aero_velocity_x * aero_velocity_x + aero_velocity_y * aero_velocity_y) * aero_velocity_x * 1.5 * 0.9;
    const auto drag_y = 0.5 * 1.2 * sqrt(aero_velocity_x * aero_velocity_x + aero_velocity_y * aero_velocity_y) * aero_velocity_y * 1.5 * 0.9;
    const auto lift = 0.5 * 1.2 * aero_velocity_x * aero_velocity_x * 1.5 * 3.0;

    // Get aerodynamic forces
    const auto aero_forces = chassis.get_aerodynamic_force();

    EXPECT_NEAR(wind_velocity_x_body, aero_forces.wind_velocity_body.x(), 2.0e-11);
    EXPECT_NEAR(wind_velocity_y_body, aero_forces.wind_velocity_body.y(), 2.0e-11);
    EXPECT_NEAR(0.0,                  aero_forces.wind_velocity_body.z(), 2.0e-11);

    EXPECT_NEAR(aero_velocity_x, aero_forces.aerodynamic_velocity.x(), 2.0e-11);
    EXPECT_NEAR(aero_velocity_y, aero_forces.aerodynamic_velocity.y(), 2.0e-11);
    EXPECT_NEAR(0.0,             aero_forces.aerodynamic_velocity.z(), 2.0e-11);

    EXPECT_NEAR(drag_x, aero_forces.drag.x(), 2.0e-11);
    EXPECT_NEAR(drag_y, aero_forces.drag.y(), 2.0e-11);
    EXPECT_NEAR(0.0,    aero_forces.drag.z(), 2.0e-11);

    EXPECT_NEAR(0.0,    aero_forces.lift.x(), 2.0e-11);
    EXPECT_NEAR(0.0,    aero_forces.lift.y(), 2.0e-11);
    EXPECT_NEAR(lift,   aero_forces.lift.z(), 2.0e-11);
}
