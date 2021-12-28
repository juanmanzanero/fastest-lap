#include "gtest/gtest.h"
#include "src/core/vehicles/limebeer2014f1.h"

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

static_assert(Front_axle_t::IOMEGA_LEFT  == 0);
static_assert(Front_axle_t::IOMEGA_RIGHT == 1);

static_assert(Rear_axle_t::IOMEGA_LEFT  == 2);
static_assert(Rear_axle_t::IOMEGA_RIGHT == 3);
 
class Axle_car_3dof_test : public testing::Test
{
 protected:
    Axle_car_3dof_test ()
    {
        front_axle.get_frame().set_parent(axle_frame);
        rear_axle.get_frame().set_parent(axle_frame);

        std::array<scalar,4> q = {omega_fl, omega_fr, omega_rl, omega_rr};
        std::array<scalar,1> u = {delta};

        front_axle.set_state_and_controls(q,u);
        rear_axle.set_state_and_controls(q,u);
    }
    // Read database
    Xml_document database         = {"./database/limebeer-2014-f1.xml", true};

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

    // Maximum braking torque
    const scalar T_max_brake = 200.0;

    // Maximum engine power
    const scalar P_max_engine = 20.1;

    // Construct frames
    sFrame inertial_frame = {};
    sFrame axle_frame = {{0.0,0.0,0.0},{u,v,0.0},{0.0},{omega},{Z},inertial_frame}; 
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

    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_relative_velocity()[0], 0.0); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_relative_velocity()[1], 0.0); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_frame().get_relative_velocity()[2], 0.0); 

    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_relative_velocity()[0], 0.0); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_relative_velocity()[1], 0.0); 
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_frame().get_relative_velocity()[2], 0.0); 

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

    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_relative_velocity()[0], 0.0); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_relative_velocity()[1], 0.0); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_frame().get_relative_velocity()[2], 0.0); 

    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_relative_velocity()[0], 0.0); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_relative_velocity()[1], 0.0); 
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_frame().get_relative_velocity()[2], 0.0); 

    EXPECT_EQ(rear_axle.template get_tire<0>().get_frame().get_rotation_angles().size(), 0);
    EXPECT_EQ(rear_axle.template get_tire<1>().get_frame().get_rotation_angles().size(), 0);
}


TEST_F(Axle_car_3dof_test, set_state_and_controls)
{
    EXPECT_EQ(front_axle.get_omega_left(), omega_fl);
    EXPECT_EQ(front_axle.get_omega_right(), omega_fr);

    EXPECT_EQ(front_axle.get_steering_angle(), delta);

    EXPECT_EQ(rear_axle.get_omega_left(), omega_rl);
    EXPECT_EQ(rear_axle.get_omega_right(), omega_rr);
}


TEST_F(Axle_car_3dof_test, update_front_axle)
{
    const scalar Fz_left = -3333.0;
    const scalar Fz_right = -2222.0;
    const scalar throttle = 1.0;
    const scalar brake_bias = 0.0;
    front_axle.update(Fz_left, Fz_right, throttle, brake_bias);

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

    // Check tires normal load
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<0>().get_force()[2], Fz_left);
    EXPECT_DOUBLE_EQ(front_axle.template get_tire<1>().get_force()[2], Fz_right);

    std::cout << front_axle.get_torque_left() << std::endl;
    std::cout << front_axle.get_torque_right() << std::endl;
}


TEST_F(Axle_car_3dof_test, update_rear_axle)
{
    const scalar Fz_left = -4444.0;
    const scalar Fz_right = -5555.0;
    const scalar throttle = 1.0;
    const scalar brake_bias = 0.0;
    rear_axle.update(Fz_left, Fz_right, throttle, brake_bias);

    // Check omega of the tires
    EXPECT_EQ(rear_axle.template get_tire<0>().get_omega(), omega_rl);    
    EXPECT_EQ(rear_axle.template get_tire<1>().get_omega(), omega_rr);    

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

    // Check tires normal load
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<0>().get_force()[2], Fz_left);
    EXPECT_DOUBLE_EQ(rear_axle.template get_tire<1>().get_force()[2], Fz_right);
}
