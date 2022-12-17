#include "gtest/gtest.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "helper_functions.h"

using Front_left_tire_type  = limebeer2014f1<scalar>::Front_left_tire_type;
using Front_right_tire_type = limebeer2014f1<scalar>::Front_right_tire_type;
using Rear_left_tire_type   = limebeer2014f1<scalar>::Rear_left_tire_type;
using Rear_right_tire_type  = limebeer2014f1<scalar>::Rear_right_tire_type;

using Front_axle_t          = limebeer2014f1<scalar>::Front_axle_t;
using Rear_axle_t           = limebeer2014f1<scalar>::Rear_axle_t;

static_assert(Front_axle_t::LEFT == 0);
static_assert(Front_axle_t::RIGHT == 1);

static_assert(Rear_axle_t::LEFT == 0);
static_assert(Rear_axle_t::RIGHT == 1);

static_assert(Front_axle_t::input_names::KAPPA_LEFT  == 0);
static_assert(Front_axle_t::input_names::KAPPA_RIGHT == 1);

static_assert(Rear_axle_t::input_names::KAPPA_LEFT  == 2);
static_assert(Rear_axle_t::input_names::KAPPA_RIGHT == 3);

static_assert(Front_axle_t::state_names::angular_momentum_left  == 0);
static_assert(Front_axle_t::state_names::angular_momentum_right== 1);

static_assert(Rear_axle_t::state_names::angular_momentum_left == 2);
static_assert(Rear_axle_t::state_names::angular_momentum_right == 3);
 
class Axle_car_3dof_test : public testing::Test
{
 protected:
    Axle_car_3dof_test ()
    {
        front_axle.get_frame().set_parent(axle_frame);
        rear_axle.get_frame().set_parent(axle_frame);

        std::array<scalar, 4> q = { kappa_fl / tire_fl.get_model().maximum_kappa(-Fz_fl),
                                    kappa_fr / tire_fr.get_model().maximum_kappa(-Fz_fr),
                                    kappa_rl / tire_rl.get_model().maximum_kappa(-Fz_rl),
                                    kappa_rr / tire_rr.get_model().maximum_kappa(-Fz_rr) };
        std::array<scalar,2> u = {delta, 0.0};

        front_axle.set_state_and_controls(q,u);
        rear_axle.set_state_and_controls(q,u);
    }
    // Read database
    Xml_document database         = {"./database/vehicles/f1/limebeer-2014-f1.xml", true};

    // Construct tires
    Front_left_tire_type tire_fl  = { "front-left" , database, "vehicle/front-tire/"};
    Front_right_tire_type tire_fr = { "front-right", database, "vehicle/front-tire/"};
    Rear_left_tire_type tire_rl   = { "rear-left"  , database, "vehicle/front-tire/"};
    Rear_right_tire_type tire_rr  = { "rear-right" , database, "vehicle/front-tire/"};

    // Construct axles
    Front_axle_t front_axle = { "front-axle", tire_fl, tire_fr, database, "vehicle/front-axle/" };
    Rear_axle_t rear_axle = { "rear-axle", tire_fl, tire_fr, database, "vehicle/rear-axle/" };

    // Geometry
    const scalar half_track = 0.73;

    // Axle velocity and omega
    const scalar u = 1.0;
    const scalar v = 0.1;
    const scalar omega = 0.2;

    // Wheels angular speeds
    const scalar omega_fl = 1.1/0.33;
    const scalar omega_fr = 1.2/0.33;
    const scalar omega_rl = 1.05/0.33;
    const scalar omega_rr = 0.95/0.33;

    // Steering angle
    const scalar delta = 10.0*DEG;

    // Tires absolute velocities
    const sVector3d v_fl = {u+omega*half_track,v,0.0};
    const sVector3d v_fr = {u-omega*half_track,v,0.0};

    const sVector3d v_fl_body = {v_fl[0]*cos(delta) + v_fl[1]*sin(delta), v_fl[1]*cos(delta) - v_fl[0]*sin(delta), v_fl[2]};
    const sVector3d v_fr_body = {v_fr[0]*cos(delta) + v_fr[1]*sin(delta), v_fr[1]*cos(delta) - v_fr[0]*sin(delta), v_fr[2]};

    const sVector3d v_rl = {u+omega*half_track,v,0.0};
    const sVector3d v_rr = {u-omega*half_track,v,0.0};

    // Tires kappa and lambda
    const scalar kappa_fl = (omega_fl*0.33 - v_fl_body[0])/v_fl_body[0];
    const scalar kappa_fr = (omega_fr*0.33 - v_fr_body[0])/v_fr_body[0];
    const scalar kappa_rl = (omega_rl*0.33 - v_rl[0])/v_rl[0];
    const scalar kappa_rr = (omega_rr*0.33 - v_rr[0])/v_rr[0];

    const scalar lambda_fl = -v_fl_body[1]/v_fl_body[0];
    const scalar lambda_fr = -v_fr_body[1]/v_fr_body[0];
    const scalar lambda_rl = -v_rl[1]/v_rl[0];
    const scalar lambda_rr = -v_rr[1]/v_rr[0];

    const scalar Fz_fl = -3333.0;
    const scalar Fz_fr = -2222.0;
    const scalar Fz_rl = -4444.0;
    const scalar Fz_rr = -5555.0;

    // Maximum braking torque
    const scalar T_max_brake = 200.0;

    // Maximum engine power
    const scalar P_max_engine = 20.1;

    // Construct frames
    sFrame inertial_frame = {};
    sFrame axle_frame = {{0.0,0.0,-0.33},{u,v,0.0},{0.0},{omega},{Z},inertial_frame, sFrame::Frame_velocity_types::parent_frame}; 
};



TEST_F(Axle_car_3dof_test, front_axle_constructor)
{
    // Check tires position
    EXPECT_DOUBLE_EQ(front_axle.get_tire_position(Front_axle_t::LEFT)[0], 0.0);
    EXPECT_DOUBLE_EQ(front_axle.get_tire_position(Front_axle_t::LEFT)[1], -0.73);
    EXPECT_DOUBLE_EQ(front_axle.get_tire_position(Front_axle_t::LEFT)[2], 0.0);

    EXPECT_DOUBLE_EQ(front_axle.get_tire_position(Front_axle_t::RIGHT)[0], 0.0);
    EXPECT_DOUBLE_EQ(front_axle.get_tire_position(Front_axle_t::RIGHT)[1], 0.73);
    EXPECT_DOUBLE_EQ(front_axle.get_tire_position(Front_axle_t::RIGHT)[2], 0.0);

    // Check tires velocity
    EXPECT_DOUBLE_EQ(front_axle.get_tire_velocity(Front_axle_t::LEFT)[0], 0.0);
    EXPECT_DOUBLE_EQ(front_axle.get_tire_velocity(Front_axle_t::LEFT)[1], 0.0);
    EXPECT_DOUBLE_EQ(front_axle.get_tire_velocity(Front_axle_t::LEFT)[2], 0.0);

    EXPECT_DOUBLE_EQ(front_axle.get_tire_velocity(Front_axle_t::RIGHT)[0], 0.0);
    EXPECT_DOUBLE_EQ(front_axle.get_tire_velocity(Front_axle_t::RIGHT)[1], 0.0);
    EXPECT_DOUBLE_EQ(front_axle.get_tire_velocity(Front_axle_t::RIGHT)[2], 0.0);

    // Check the tires frame
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_origin()[0], 0.0); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_origin()[1], -0.73); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_origin()[2], 0.0); 

    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_origin()[0], 0.0); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_origin()[1], 0.73); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_origin()[2], 0.0); 

    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_relative_velocity_in_parent()[0], 0.0); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_relative_velocity_in_parent()[1], 0.0); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_relative_velocity_in_parent()[2], 0.0); 

    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_relative_velocity_in_parent()[0], 0.0); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_relative_velocity_in_parent()[1], 0.0); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_relative_velocity_in_parent()[2], 0.0); 

    EXPECT_EQ(front_axle.template get_tire<0>().get_frame().get_rotation_angles().size(), 1);
    EXPECT_EQ(front_axle.template get_tire<1>().get_frame().get_rotation_angles().size(), 1);

    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_rotation_angles().front(), delta);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_rotation_angles().front(), delta);
}


TEST_F(Axle_car_3dof_test, rear_axle_constructor)
{
    // Check tires position
    EXPECT_DOUBLE_EQ(rear_axle.get_tire_position(Rear_axle_t::LEFT)[0], 0.0);
    EXPECT_DOUBLE_EQ(rear_axle.get_tire_position(Rear_axle_t::LEFT)[1], -0.73);
    EXPECT_DOUBLE_EQ(rear_axle.get_tire_position(Rear_axle_t::LEFT)[2], 0.0);

    EXPECT_DOUBLE_EQ(rear_axle.get_tire_position(Rear_axle_t::RIGHT)[0], 0.0);
    EXPECT_DOUBLE_EQ(rear_axle.get_tire_position(Rear_axle_t::RIGHT)[1], 0.73);
    EXPECT_DOUBLE_EQ(rear_axle.get_tire_position(Rear_axle_t::RIGHT)[2], 0.0);

    // Check tires velocity
    EXPECT_DOUBLE_EQ(rear_axle.get_tire_velocity(Rear_axle_t::LEFT)[0], 0.0);
    EXPECT_DOUBLE_EQ(rear_axle.get_tire_velocity(Rear_axle_t::LEFT)[1], 0.0);
    EXPECT_DOUBLE_EQ(rear_axle.get_tire_velocity(Rear_axle_t::LEFT)[2], 0.0);

    EXPECT_DOUBLE_EQ(rear_axle.get_tire_velocity(Rear_axle_t::RIGHT)[0], 0.0);
    EXPECT_DOUBLE_EQ(rear_axle.get_tire_velocity(Rear_axle_t::RIGHT)[1], 0.0);
    EXPECT_DOUBLE_EQ(rear_axle.get_tire_velocity(Rear_axle_t::RIGHT)[2], 0.0);

    // Check the tires frame
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_origin()[0], 0.0); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_origin()[1], -0.73); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_origin()[2], 0.0); 

    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_origin()[0], 0.0); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_origin()[1], 0.73); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_origin()[2], 0.0); 

    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_relative_velocity_in_parent()[0], 0.0); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_relative_velocity_in_parent()[1], 0.0); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_relative_velocity_in_parent()[2], 0.0); 

    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_relative_velocity_in_parent()[0], 0.0); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_relative_velocity_in_parent()[1], 0.0); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_relative_velocity_in_parent()[2], 0.0); 

    EXPECT_EQ(rear_axle.template get_tire<0>().get_frame().get_rotation_angles().size(), 0);
    EXPECT_EQ(rear_axle.template get_tire<1>().get_frame().get_rotation_angles().size(), 0);
}


TEST_F(Axle_car_3dof_test, set_state_and_controls)
{
    EXPECT_EQ(front_axle.get_kappa_dimensionless_left(), kappa_fl/tire_fl.get_model().maximum_kappa(-Fz_fl));
    EXPECT_EQ(front_axle.get_kappa_dimensionless_right(), kappa_fr/tire_fr.get_model().maximum_kappa(-Fz_fr));

    EXPECT_EQ(front_axle.get_steering_angle(), delta);

    EXPECT_EQ(rear_axle.get_kappa_dimensionless_left(), kappa_rl/tire_rl.get_model().maximum_kappa(-Fz_rl));
    EXPECT_EQ(rear_axle.get_kappa_dimensionless_right(), kappa_rr/tire_rr.get_model().maximum_kappa(-Fz_rr));
}


TEST_F(Axle_car_3dof_test, update_front_axle)
{
    const scalar throttle = 1.0;
    const scalar brake_bias = 0.0;
    front_axle.update(Fz_fl, Fz_fr, throttle, brake_bias, inertial_frame);

    // Check omega of the tires
    EXPECT_EQ(front_axle.template get_tire<0>().get_omega(), omega_fl);    
    EXPECT_EQ(front_axle.template get_tire<1>().get_omega(), omega_fr);    

    // Check tires absolute velocity in inertial
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_inertial()[0], v_fl[0]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_inertial()[1], v_fl[1]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_inertial()[2], v_fl[2]);

    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_inertial()[0], v_fr[0]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_inertial()[1], v_fr[1]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_inertial()[2], v_fr[2]);

    // Check tires absolute velocity in body
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_body()[0], v_fl_body[0]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_body()[1], v_fl_body[1]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_body()[2], v_fl_body[2]);

    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_body()[0], v_fr_body[0]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_body()[1], v_fr_body[1]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_body()[2], v_fr_body[2]);

    // Check kappa and lambda
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_kappa(), kappa_fl);    
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_kappa(), kappa_fr);    
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_lambda(), lambda_fl);    
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_lambda(), lambda_fr);    

    // Check tires load
    const sVector3d F_fl = get_tire_forces(kappa_fl,lambda_fl,-Fz_fl);
    const sVector3d F_fr = get_tire_forces(kappa_fr,lambda_fr,-Fz_fr);

    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_force()[0], F_fl[0]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_force()[1], F_fl[1]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_force()[2], F_fl[2]);
    
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_force()[0], F_fr[0]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_force()[1], F_fr[1]);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_force()[2], F_fr[2]);
    

    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_force()[2], Fz_fl);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_force()[2], Fz_fr);

    // Check axle forces
    const sVector3d F_total_tire = F_fl + F_fr;
    const sVector3d F_total = { F_total_tire[0]*cos(delta) - F_total_tire[1]*sin(delta), 
                                F_total_tire[1]*cos(delta) + F_total_tire[0]*sin(delta), 
                                F_total_tire[2]};

    EXPECT_NEAR(front_axle.get_force()[0], F_total[0],1.0e-11);
    EXPECT_NEAR(front_axle.get_force()[1], F_total[1],1.0e-11);
    EXPECT_DOUBLE_EQ(front_axle.get_force()[2], F_total[2]);

    // Check axle torques
    const sVector3d F_fl_ax = {F_fl[0]*cos(delta)-F_fl[1]*sin(delta), F_fl[1]*cos(delta) + F_fl[0]*sin(delta), F_fl[2]};
    const sVector3d F_fr_ax = {F_fr[0]*cos(delta)-F_fr[1]*sin(delta), F_fr[1]*cos(delta) + F_fr[0]*sin(delta), F_fr[2]};
    EXPECT_DOUBLE_EQ(front_axle.get_torque()[0], F_fr_ax[2]*0.73 - F_fl_ax[2]*0.73 - (F_fl_ax[1]+F_fr_ax[1])*0.33);
    EXPECT_DOUBLE_EQ(front_axle.get_torque()[1], (F_fl_ax[0]+F_fr_ax[0])*0.33);
    EXPECT_DOUBLE_EQ(front_axle.get_torque()[2], F_fl_ax[0]*0.73 - F_fr_ax[0]*0.73);

    // Check the tires rotational dynamics
    EXPECT_DOUBLE_EQ(front_axle.get_dangular_momentum_dt_left(), -0.33*F_fl[0]);
    EXPECT_DOUBLE_EQ(front_axle.get_dangular_momentum_dt_right(), -0.33*F_fr[0]);
}


TEST_F(Axle_car_3dof_test, update_rear_axle)
{
    const scalar throttle = -1e10;
    const scalar brake_bias = 0.0;
    rear_axle.update(Fz_rl, Fz_rr, throttle, brake_bias, inertial_frame);

    // Check omega of the tires
    EXPECT_EQ(rear_axle.template get_tire<0>().get_omega(), omega_rl);    
    EXPECT_NEAR(rear_axle.template get_tire<1>().get_omega(), omega_rr, 1.0e-12);    

    // Check tires absolute velocity in inertial
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_inertial()[0], v_rl[0]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_inertial()[1], v_rl[1]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_inertial()[2], v_rl[2]);

    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_inertial()[0], v_rr[0]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_inertial()[1], v_rr[1]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_inertial()[2], v_rr[2]);

    // Check tires absolute velocity in body
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_body()[0], v_rl[0]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_body()[1], v_rl[1]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_absolute_velocity_in_body()[2], v_rl[2]);

    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_body()[0], v_rr[0]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_body()[1], v_rr[1]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_absolute_velocity_in_body()[2], v_rr[2]);

    // Check kappa and lambda
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_kappa(), kappa_rl);    
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_kappa(), kappa_rr);    
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_lambda(), lambda_rl);    
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_lambda(), lambda_rr);    

    // Check tires load
    const sVector3d F_rl = get_tire_forces(kappa_rl,lambda_rl,-Fz_rl);
    const sVector3d F_rr = get_tire_forces(kappa_rr,lambda_rr,-Fz_rr);

    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_force()[0], F_rl[0]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_force()[1], F_rl[1]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_force()[2], F_rl[2]);
    
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_force()[0], F_rr[0]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_force()[1], F_rr[1]);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_force()[2], F_rr[2]);
    

    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_force()[2], Fz_rl);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_force()[2], Fz_rr);

    // Check axle forces
    const sVector3d F_total = F_rl + F_rr;
    EXPECT_NEAR(rear_axle.get_force()[0], F_total[0],1.0e-11);
    EXPECT_NEAR(rear_axle.get_force()[1], F_total[1],1.0e-11);
    EXPECT_DOUBLE_EQ(rear_axle.get_force()[2], F_total[2]);

    // Check axle torques
    EXPECT_NEAR(rear_axle.get_torque()[0], F_rr[2]*0.73 - F_rl[2]*0.73 - (F_rl[1]+F_rr[1])*0.33,1.0e-12);
    EXPECT_NEAR(rear_axle.get_torque()[1], (F_rl[0]+F_rr[0])*0.33,1.0e-12);
    EXPECT_NEAR(rear_axle.get_torque()[2], F_rl[0]*0.73 - F_rr[0]*0.73,2.0e-12);

    // Check the tires rotational dynamics
    EXPECT_DOUBLE_EQ(rear_axle.get_dangular_momentum_dt_left(), (-10.47*(omega_rl-omega_rr) - 0.33*F_rl[0]));
    EXPECT_DOUBLE_EQ(rear_axle.get_dangular_momentum_dt_right(),( 10.47*(omega_rl-omega_rr) - 0.33*F_rr[0]));
}
