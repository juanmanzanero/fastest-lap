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

class Chassis_car_3dof_3d_test : public ::testing::Test
{
 protected:
    Chassis_car_3dof_3d_test () 
    {
        std::array<scalar,limebeer2014f1<scalar>::cartesian::number_of_inputs> q_in;
        std::array<scalar,limebeer2014f1<scalar>::cartesian::number_of_controls> u_in;

        q_in[Front_axle_t::input_names::KAPPA_LEFT]  = kappa_fl/tire_fl.get_model().maximum_kappa(-neg_Fz_fl);
        q_in[Front_axle_t::input_names::KAPPA_RIGHT] = kappa_fr/tire_fr.get_model().maximum_kappa(-neg_Fz_fr);
        q_in[Rear_axle_t::input_names::KAPPA_LEFT]   = kappa_rl/tire_rl.get_model().maximum_kappa(-neg_Fz_rl);
        q_in[Rear_axle_t::input_names::KAPPA_RIGHT]  = kappa_rr/tire_rr.get_model().maximum_kappa(-neg_Fz_rr);
        q_in[Chassis_t::input_names::velocity_x_mps] = u;
        q_in[Chassis_t::input_names::velocity_y_mps] = v;
        q_in[Chassis_t::input_names::yaw_rate_radps] = yaw_rate;
        q_in[Chassis_t::input_names::force_z_fl_g] = Fz_fl/(9.81*660.0);
        q_in[Chassis_t::input_names::force_z_fr_g] = Fz_fr/(9.81*660.0);
        q_in[Chassis_t::input_names::force_z_rl_g] = Fz_rl/(9.81*660.0);
        q_in[Chassis_t::input_names::force_z_rr_g] = Fz_rr/(9.81*660.0);

        u_in[Front_axle_t::control_names::STEERING] = delta;
        u_in[Rear_axle_t::control_names::boost]     = 0.0;
        u_in[Chassis_t::control_names::throttle]    = throttle;
        u_in[Chassis_t::control_names::brake_bias]  = 0.6;


        chassis.set_state_and_controls(q_in,u_in);
        chassis.update({ x,y,z }, { yaw, pitch, roll }, track_heading_angle, { yaw_dot, pitch_dot, roll_dot}, track_heading_angle_dot, w);
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
    const scalar x = 20.0;
    const scalar y = 100.0;
    const scalar z = -5.0;

    const scalar u = vel*cos(11.0*DEG);
    const scalar v = vel*sin(11.0*DEG);
    const scalar w = 1.0;

    const scalar yaw = 45.0 * DEG;
    const scalar pitch = -5.0 * DEG;
    const scalar roll = 11.0 * DEG;

    const scalar yaw_dot = 30.0 * DEG;
    const scalar pitch_dot = 1.0 * DEG;
    const scalar roll_dot = 2.0 * DEG;

    const scalar track_heading_angle = -50.0 * DEG;
    const scalar track_heading_angle_dot = 20.0 * DEG;

    const scalar delta = -10.0*DEG;
    const scalar throttle = 0.5;

    const scalar Fz_fl = -2222.0;
    const scalar Fz_fr = -3333.0;
    const scalar Fz_rl = -4444.0;
    const scalar Fz_rr = -5555.0;

    // Computed quantities
    const scalar roll_rate_road  = roll_dot - sin(pitch) * yaw_dot;
    const scalar pitch_rate_road = cos(roll) * pitch_dot + cos(pitch) * sin(roll) * yaw_dot;
    const scalar roll_rate       = cos(track_heading_angle) * roll_rate_road + sin(track_heading_angle) * pitch_rate_road;
    const scalar pitch_rate      = -sin(track_heading_angle) * roll_rate_road + cos(track_heading_angle) * pitch_rate_road;
    const scalar yaw_rate        = -sin(roll) * pitch_dot + cos(pitch) * cos(roll) * yaw_dot + track_heading_angle_dot;

    const scalar neg_Fz_fl = 0.5*(Fz_fl - sqrt(Fz_fl*Fz_fl + 1.0));
    const scalar neg_Fz_fr = 0.5*(Fz_fr - sqrt(Fz_fr*Fz_fr + 1.0));
    const scalar neg_Fz_rl = 0.5*(Fz_rl - sqrt(Fz_rl*Fz_rl + 1.0));
    const scalar neg_Fz_rr = 0.5*(Fz_rr - sqrt(Fz_rr*Fz_rr + 1.0));

    // Formulas directly taken from Limebeer 2014 et al.
    const scalar kappa_fl = -(1.0 - 0.33*omega_fl/(cos(delta)*(u+yaw_rate*0.73) + sin(delta)*(yaw_rate*1.8+v)));
    const scalar kappa_fr = -(1.0 - 0.33*omega_fr/(cos(delta)*(u-yaw_rate*0.73) + sin(delta)*(yaw_rate*1.8+v)));
    const scalar kappa_rl = -(1.0 - 0.33*omega_rl/(u+yaw_rate*0.73));
    const scalar kappa_rr = -(1.0 - 0.33*omega_rr/(u-yaw_rate*0.73));

    const scalar lambda_fl = -(cos(delta)*(yaw_rate*1.8+v)-sin(delta)*(yaw_rate*0.73+u))/(cos(delta)*(yaw_rate*0.73+u)+sin(delta)*(yaw_rate*1.8+v));
    const scalar lambda_fr = -(cos(delta)*(yaw_rate*1.8+v)+sin(delta)*(yaw_rate*0.73-u))/(cos(delta)*(u-yaw_rate*0.73)+sin(delta)*(yaw_rate*1.8+v));
    const scalar lambda_rl = -(v-yaw_rate*1.6)/(u+yaw_rate*0.73);
    const scalar lambda_rr = -(v-yaw_rate*1.6)/(u-yaw_rate*0.73);

    // Compute tire forces
    const sVector3d F_fl = get_tire_forces(kappa_fl, lambda_fl, -neg_Fz_fl);
    const sVector3d F_fr = get_tire_forces(kappa_fr, lambda_fr, -neg_Fz_fr);
    const sVector3d F_rl = get_tire_forces(kappa_rl, lambda_rl, -neg_Fz_rl);
    const sVector3d F_rr = get_tire_forces(kappa_rr, lambda_rr, -neg_Fz_rr);

    // Compute aerodynamic forces
    const scalar F_lift = 0.5*1.2*u*u*1.5*3.0;
    const scalar Fx_drag = -0.5*1.2*sqrt(u*u+v*v)*u*1.5*0.9;
    const scalar Fy_drag = -0.5*1.2*sqrt(u*u+v*v)*v*1.5*0.9;
    const scalar Fz_drag = -0.5*1.2*sqrt(u*u+v*v)*w*1.5*0.9;

    // Compute total forces
    const scalar Fx = cos(delta)*(F_fr.x()+F_fl.x())-sin(delta)*(F_fr.y()+F_fl.y()) + F_rr.x() + F_rl.x() + Fx_drag;
    const scalar Fy = cos(delta)*(F_fr.y()+F_fl.y())+sin(delta)*(F_fr.x()+F_fl.x()) + F_rr.y() + F_rl.y() + Fy_drag;

    const sVector3d gravity = 660.0 * 9.81 * sVector3d{ sin(track_heading_angle) * sin(roll) * cos(pitch) - cos(track_heading_angle) * sin(pitch),
        sin(track_heading_angle) * sin(pitch) + cos(track_heading_angle) * sin(roll) * cos(pitch),
        cos(roll) * cos(pitch) };
};


TEST_F(Chassis_car_3dof_3d_test, positions)
{
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_origin().x(), x);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_origin().y(), y);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_origin().z(), z);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_origin().x(), 0.0);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_origin().y(), 0.0);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_origin().z(), 0.0);

    // Position of the front axle center
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_front_axle().get_frame().get_origin().x(), 1.8);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_front_axle().get_frame().get_origin().y(), 0.0);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_front_axle().get_frame().get_origin().z(), -0.33);

    // Position of the rear axle center
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_rear_axle().get_frame().get_origin().x(), -1.6);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_rear_axle().get_frame().get_origin().y(), 0.0);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_rear_axle().get_frame().get_origin().z(), -0.33);

    // Position of the front left tire center
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).x(), 1.8);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).y(), -0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).z(), -0.33);

    // Position of the front right tire center
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).x(), 1.8);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).y(),  0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_fr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).z(), -0.33);

    // Position of the rear left tire center
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).x(), -1.6);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).y(), -0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rl).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).z(), -0.33);

    // Position of the rear right tire center
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).x(), -1.6);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).y(),  0.73);
    EXPECT_DOUBLE_EQ(std::get<0>(std::as_const(tire_rr).get_frame().get_position_and_velocity_in_target(std::as_const(chassis).get_chassis_frame())).z(), -0.33);
}


TEST_F(Chassis_car_3dof_3d_test, transform_states_to_input_states)
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

TEST_F(Chassis_car_3dof_3d_test, velocities)
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


TEST_F(Chassis_car_3dof_3d_test, get_gravity_force)
{

    EXPECT_DOUBLE_EQ(chassis.get_gravity_force().x(), gravity.x());
    EXPECT_DOUBLE_EQ(chassis.get_gravity_force().y(), gravity.y());
    EXPECT_DOUBLE_EQ(chassis.get_gravity_force().z(), gravity.z());
}


TEST_F(Chassis_car_3dof_3d_test, omega_absolute)
{
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_omega_absolute_in_body().x(), roll_rate);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_omega_absolute_in_body().y(), pitch_rate);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_omega_absolute_in_body().z(), yaw_rate);
}

TEST_F(Chassis_car_3dof_3d_test, tires_absolute_velocities)
{
    EXPECT_DOUBLE_EQ(std::as_const(tire_fr).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).x(), cos(delta) * (u - yaw_rate * 0.73) + sin(delta) * (yaw_rate * 1.8 + v));
    EXPECT_DOUBLE_EQ(std::as_const(tire_fr).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).y(), sin(delta) * (yaw_rate * 0.73 - u) + cos(delta) * (yaw_rate * 1.8 + v));
    EXPECT_DOUBLE_EQ(std::as_const(tire_fr).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).z(), w - 1.8 * pitch_rate + 0.73 * roll_rate);

    EXPECT_DOUBLE_EQ(std::as_const(tire_fl).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).x(), cos(delta) * (u + yaw_rate * 0.73) + sin(delta) * (yaw_rate * 1.8 + v));
    EXPECT_DOUBLE_EQ(std::as_const(tire_fl).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).y(), -sin(delta) * (yaw_rate * 0.73 + u) + cos(delta) * (yaw_rate * 1.8 + v));
    EXPECT_DOUBLE_EQ(std::as_const(tire_fl).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).z(), w - 1.8*pitch_rate - 0.73*roll_rate);

    EXPECT_DOUBLE_EQ(std::as_const(tire_rr).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).x(), u - yaw_rate * 0.73);
    EXPECT_DOUBLE_EQ(std::as_const(tire_rr).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).y(), - yaw_rate * 1.6 + v);
    EXPECT_DOUBLE_EQ(std::as_const(tire_rr).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).z(), w + 1.6*pitch_rate + 0.73*roll_rate);

    EXPECT_DOUBLE_EQ(std::as_const(tire_rl).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).x(), u + yaw_rate * 0.73);
    EXPECT_DOUBLE_EQ(std::as_const(tire_rl).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).y(), -yaw_rate * 1.6 + v);
    EXPECT_DOUBLE_EQ(std::as_const(tire_rl).get_frame().get_absolute_velocity_in_body({ 0.0,0.0,0.33 }).z(), w + 1.6*pitch_rate - 0.73*roll_rate);
}

TEST_F(Chassis_car_3dof_3d_test, check_set_state)
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
    EXPECT_DOUBLE_EQ(chassis.get_yaw_rate_radps(), yaw_rate);

    EXPECT_DOUBLE_EQ(chassis.get_throttle(), throttle);
    EXPECT_DOUBLE_EQ(chassis.get_front_axle().get_steering_angle(), delta);
}


TEST_F(Chassis_car_3dof_3d_test, kappa_and_lambda)
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


TEST_F(Chassis_car_3dof_3d_test, tire_forces)
{
    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(tire_fl.get_force()[i], F_fl[i], 2.0e-11);
        EXPECT_NEAR(tire_fr.get_force()[i], F_fr[i], 2.0e-11);
        EXPECT_NEAR(tire_rl.get_force()[i], F_rl[i], 2.0e-11);
        EXPECT_NEAR(tire_rr.get_force()[i], F_rr[i], 2.0e-11);
    }
}

TEST_F(Chassis_car_3dof_3d_test, newton_equations)
{
    const scalar du = yaw_rate*(v + 0.3*roll_rate) - w*pitch_rate + gravity.x()/660.0 + Fx/660.0;
    const scalar dv = w*roll_rate - yaw_rate*(u - 0.3*pitch_rate) + gravity.y()/660.0 + Fy/660.0;
    const scalar dw = F_fl.z()+F_fr.z()+F_rl.z()+F_rr.z() + gravity.z() + Fz_drag + F_lift + 660.0*(u-0.3*pitch_rate)*pitch_rate - 660.0*(v + 0.3*roll_rate)*roll_rate;

    EXPECT_DOUBLE_EQ(chassis.get_com_velocity_x_mps(), u - 0.3 * pitch_rate);
    EXPECT_DOUBLE_EQ(chassis.get_com_velocity_y_mps(), v + 0.3 * roll_rate);
    EXPECT_DOUBLE_EQ(chassis.get_com_velocity_z_mps(), w);

    EXPECT_NEAR(chassis.get_com_velocity_x_dot_mps2(), du, 2.0e-11);
    EXPECT_NEAR(chassis.get_com_velocity_y_dot_mps2(), dv, 2.0e-11);
    EXPECT_NEAR(chassis.get_com_velocity_z_dot_mps2()*660.0, dw, 2.0e-11);
}


TEST_F(Chassis_car_3dof_3d_test, euler_equations)
{
    const scalar domega = (-0.1*Fy_drag + 1.8*(cos(delta)*(F_fr.y()+F_fl.y())+sin(delta)*(F_fr.x()+F_fl.x())) + 0.73*(sin(delta)*F_fr.y()-cos(delta)*F_fr.x()) - 0.73*F_rr.x() + 0.73*(cos(delta)*F_fl.x() - sin(delta)*F_fl.y()) + 0.73*F_rl.x() - 1.6*(F_rr.y()+F_rl.y()) + (112.5-425.0)*roll_rate*pitch_rate)/450.0;

    EXPECT_NEAR(chassis.get_roll_angular_momentum_dot_Nm(), 0.73*(neg_Fz_rr-neg_Fz_rl)+0.73*(neg_Fz_fr-neg_Fz_fl) - 0.3*Fy + (425.0 - 450.0)*yaw_rate*pitch_rate, 2.0e-11);
    EXPECT_NEAR(chassis.get_pitch_angular_momentum_dot_Nm(), 1.6*(neg_Fz_rr+neg_Fz_rl)-1.8*(neg_Fz_fr+neg_Fz_fl)+ 0.3*Fx + 0.1*(F_lift + Fz_drag) + (450.0 - 112.5)*yaw_rate*roll_rate, 2.0e-11);
    EXPECT_NEAR(chassis.get_yaw_rate_dot_radps2(), domega, 2.0e-11);
}


TEST_F(Chassis_car_3dof_3d_test, suspension_compliance_equation)
{
    EXPECT_NEAR(chassis.get_roll_balance_equation_N(), Fz_fr - Fz_fl - 0.5*(Fz_fr + Fz_rr - Fz_fl - Fz_rl), 2.0e-11);
}


TEST_F(Chassis_car_3dof_3d_test, aerodynamic_force_air_in_calm)
{
    EXPECT_NEAR(Fx_drag, chassis.get_aerodynamic_force().drag.x(), 2.0e-11);
    EXPECT_NEAR(Fy_drag, chassis.get_aerodynamic_force().drag.y(), 2.0e-11);
    EXPECT_NEAR(Fz_drag, chassis.get_aerodynamic_force().drag.z(), 2.0e-11);
    EXPECT_NEAR(0.0,     chassis.get_aerodynamic_force().lift.x(), 2.0e-11);
    EXPECT_NEAR(0.0,     chassis.get_aerodynamic_force().lift.y(), 2.0e-11);
    EXPECT_NEAR(F_lift,  chassis.get_aerodynamic_force().lift.z(), 2.0e-11);
}


TEST_F(Chassis_car_3dof_3d_test, aerodynamic_force_wind)
{
    const double northward_wind = 50.0 * KMH;
    const double eastward_wind  = 30.0 * KMH;

    // Set wind
    chassis.set_parameter("vehicle/chassis/aerodynamics/wind_velocity/northward", northward_wind);
    chassis.set_parameter("vehicle/chassis/aerodynamics/wind_velocity/eastward",  eastward_wind);

    // Compute wind velocity in body axes
    const auto wind_velocity_body = transpose(std::as_const(chassis).get_road_frame().get_absolute_rotation_matrix()) * sVector3d{ eastward_wind, -northward_wind, 0.0 };
    const auto& wind_velocity_x_body = wind_velocity_body.x();
    const auto& wind_velocity_y_body = wind_velocity_body.y();
    const auto& wind_velocity_z_body = wind_velocity_body.z();


    // Compute aero velocity
    const auto aero_velocity_x = - u + wind_velocity_x_body;
    const auto aero_velocity_y = - v + wind_velocity_y_body;
    const auto aero_velocity_z = - w + wind_velocity_z_body;

    // Compute aero forces
    const auto drag_x = 0.5 * 1.2 * sqrt(aero_velocity_x * aero_velocity_x + aero_velocity_y * aero_velocity_y) * aero_velocity_x * 1.5 * 0.9;
    const auto drag_y = 0.5 * 1.2 * sqrt(aero_velocity_x * aero_velocity_x + aero_velocity_y * aero_velocity_y) * aero_velocity_y * 1.5 * 0.9;
    const auto drag_z = 0.5 * 1.2 * sqrt(aero_velocity_x * aero_velocity_x + aero_velocity_y * aero_velocity_y) * aero_velocity_z * 1.5 * 0.9;
    const auto lift = 0.5 * 1.2 * aero_velocity_x * aero_velocity_x * 1.5 * 3.0;

    // Get aerodynamic forces
    const auto aero_forces = chassis.get_aerodynamic_force();

    EXPECT_NEAR(wind_velocity_x_body, aero_forces.wind_velocity_body.x(), 2.0e-11);
    EXPECT_NEAR(wind_velocity_y_body, aero_forces.wind_velocity_body.y(), 2.0e-11);
    EXPECT_NEAR(wind_velocity_z_body, aero_forces.wind_velocity_body.z(), 2.0e-11);

    EXPECT_NEAR(aero_velocity_x, aero_forces.aerodynamic_velocity.x(), 2.0e-11);
    EXPECT_NEAR(aero_velocity_y, aero_forces.aerodynamic_velocity.y(), 2.0e-11);
    EXPECT_NEAR(aero_velocity_z, aero_forces.aerodynamic_velocity.z(), 2.0e-11);

    EXPECT_NEAR(drag_x, aero_forces.drag.x(), 2.0e-11);
    EXPECT_NEAR(drag_y, aero_forces.drag.y(), 2.0e-11);
    EXPECT_NEAR(drag_z, aero_forces.drag.z(), 2.0e-11);

    EXPECT_NEAR(0.0,    aero_forces.lift.x(), 2.0e-11);
    EXPECT_NEAR(0.0,    aero_forces.lift.y(), 2.0e-11);
    EXPECT_NEAR(lift,   aero_forces.lift.z(), 2.0e-11);
}
