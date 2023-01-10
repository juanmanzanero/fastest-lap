#include "gtest/gtest.h"

#include "lion/frame/frame.h"
#include "src/core/chassis/chassis_car_6dof.h"
#include "src/core/chassis/axle_car_6dof.h"
#include "src/core/tire/tire_pacejka.h"


using Front_left_tire_type  = Tire_pacejka_std<scalar,0,0>;
using Front_right_tire_type = Tire_pacejka_std<scalar,Front_left_tire_type ::input_names::end, Front_left_tire_type ::control_names::end>;
using Rear_left_tire_type   = Tire_pacejka_std<scalar,Front_right_tire_type::input_names::end, Front_right_tire_type::control_names::end>;
using Rear_right_tire_type  = Tire_pacejka_std<scalar,Rear_left_tire_type  ::input_names::end, Rear_left_tire_type  ::control_names::end>;

using Front_axle_type = Axle_car_6dof<scalar,Front_left_tire_type,Front_right_tire_type,STEERING_FREE_ROLL,Rear_right_tire_type::input_names::end,Rear_right_tire_type::control_names::end>;
using Rear_axle_type  = Axle_car_6dof<scalar,Rear_left_tire_type,Rear_right_tire_type,POWERED_WITHOUT_DIFFERENTIAL,Front_axle_type::input_names::end,Front_axle_type::control_names::end>;
using Chassis_t = Chassis_car_6dof<scalar,Front_axle_type,Rear_axle_type,Rear_axle_type::input_names::end,Rear_axle_type::control_names::end>;

template class Axle_car_6dof<scalar,Front_left_tire_type,Front_right_tire_type,STEERING_FREE_ROLL,Rear_right_tire_type::input_names::end,Rear_right_tire_type::control_names::end>;
template class Axle_car_6dof<scalar,Rear_left_tire_type,Rear_right_tire_type,POWERED_WITHOUT_DIFFERENTIAL,Front_axle_type::input_names::end,Front_axle_type::control_names::end>;
template class Chassis_car_6dof<scalar,Front_axle_type,Rear_axle_type,Rear_axle_type::input_names::end,Rear_axle_type::control_names::end>;
template class Chassis<scalar,Front_axle_type,Rear_axle_type,Rear_axle_type::input_names::end,Rear_axle_type::control_names::end>;

  
class Chassis_test : public ::testing::Test
{
 protected:
    Chassis_test() 
    {
        chassis.get_front_axle().set_steering_angle(delta);
        chassis.get_rear_axle().set_torque_and_omega(T,omega_axle);
        chassis.set_state(u,v,omega,z,dz,mu,dmu,phi,dphi);
        chassis.update({ x,y,0 }, { 0,0,0 }, psi, { 0,0,0 }, omega, 0);
    }

    const scalar x = 2.0;
    const scalar y = 4.0;

    const scalar psi = pi    /4.0;
    const scalar omega = 0.1;

    const scalar u = 0.95;
    const scalar v = 0.1;

    const scalar dx = u*cos(psi) - v*sin(psi);
    const scalar dy = u*sin(psi) + v*cos(psi);

    const scalar z    = 0.1;
    const scalar dz   = 0.03;
    const scalar mu   = -5.0*DEG;
    const scalar dmu  = 0.4;
    const scalar phi  = 4.0*DEG;
    const scalar dphi = 0.2;

    const scalar delta = -16.0*DEG;

    const scalar T = 0.1*17.6;    // Maximum torque at 10.250rpm
    const scalar omega_axle = 10.0;

    const scalar rho = 1.2;
    const scalar CdA = 0.7;

    std::map<std::string,scalar> front_axle_parameters = { std::pair("track", 1.055),
                                                           std::pair("chassis_stiffness", 17.7e3),
                                                           std::pair("antiroll_stiffness", 0.0),
                                                           std::pair("beta_steering", 0.058)
                                                          };


    std::map<std::string,scalar> rear_axle_parameters = { std::pair("track", 1.200),
                                                          std::pair("chassis_stiffness", 60.0e3),
                                                          std::pair("antiroll_stiffness", 0.0),
                                                          std::pair("inertia", 0.2)
                                                        };

    std::map<std::string,scalar> chassis_parameters = { std::pair("cog_height", 0.250),
                                                        std::pair("front_axle_x", 0.645),
                                                        std::pair("rear_axle_x", -0.400),
                                                        std::pair("front_axle_z",0.111),
                                                        std::pair("rear_axle_z",0.111),
                                                        std::pair("mass", 165.0),
                                                        std::pair("inertia_xx", 20.0),
                                                        std::pair("inertia_yy", 15.0),
                                                        std::pair("inertia_zz", 25.0),
                                                        std::pair("inertia_xz", 5.0),
                                                        std::pair("inertia_xy", 0.0),
                                                        std::pair("inertia_yz", 0.0),
                                                        std::pair("rho_air", 1.2),
                                                        std::pair("CdA", 0.7),
                                                       };


    Xml_document database = { "./database/vehicles/kart/roberto-lot-kart-2016.xml", true };

    Chassis_t construct_chassis();

    Chassis_t chassis = construct_chassis();
};

Chassis_t Chassis_test::construct_chassis()
{
//  Construct tires
    Front_left_tire_type tire_fl("front left", database, "vehicle/front-tire/");
    Front_right_tire_type tire_fr("front right", database, "vehicle/front-tire/");
    Rear_left_tire_type tire_rl("rear left", database, "vehicle/rear-tire/");
    Rear_right_tire_type tire_rr("rear right", database, "vehicle/rear-tire/");
    Front_axle_type front_axle("front axle", 
                        tire_fl, tire_fr, 
                        database, "vehicle/front-axle/"
                       );

    Rear_axle_type rear_axle("rear axle",
                       tire_rl, tire_rr, 
                       database, "vehicle/rear-axle/"
                      );

    return Chassis_t(front_axle, rear_axle, database, "vehicle/chassis/");
}


TEST_F(Chassis_test, copy_constructor_frame_trees_sanity)
{
    const Front_axle_type& front_axle = chassis.get_front_axle();
    const Rear_axle_type& rear_axle   = chassis.get_rear_axle();

    const Front_left_tire_type& tire_fl = front_axle.get_tire<Front_axle_type::LEFT>();
    const Front_right_tire_type& tire_fr = front_axle.get_tire<Front_axle_type::RIGHT>();

    const Rear_left_tire_type& tire_rl = rear_axle.get_tire<Rear_axle_type::LEFT>();
    const Rear_right_tire_type& tire_rr = rear_axle.get_tire<Rear_axle_type::RIGHT>();

    EXPECT_EQ(&std::as_const(chassis).get_inertial_frame(), std::as_const(chassis).get_road_frame().get_parent_ptr());
    EXPECT_EQ(&std::as_const(chassis).get_road_frame(), std::as_const(chassis).get_chassis_frame().get_parent_ptr());

    EXPECT_EQ(&std::as_const(chassis).get_chassis_frame(), std::as_const(front_axle).get_frame().get_parent_ptr());
    EXPECT_EQ(&std::as_const(chassis).get_chassis_frame(), std::as_const(rear_axle).get_frame().get_parent_ptr());

    EXPECT_EQ(&std::as_const(front_axle).get_frame(), std::as_const(tire_fl).get_frame().get_parent_ptr());
    EXPECT_EQ(&std::as_const(front_axle).get_frame(), std::as_const(tire_fr).get_frame().get_parent_ptr());

    EXPECT_EQ(&std::as_const(rear_axle).get_frame(), std::as_const(tire_rl).get_frame().get_parent_ptr());
    EXPECT_EQ(&std::as_const(rear_axle).get_frame(), std::as_const(tire_rr).get_frame().get_parent_ptr());
}


TEST_F(Chassis_test, copy_assignment_frame_trees_sanity)
{
    Chassis_t chassis_copy;

    chassis_copy = chassis;

    const Front_axle_type& front_axle = chassis_copy.get_front_axle();
    const Rear_axle_type& rear_axle   = chassis_copy.get_rear_axle();

    const Front_left_tire_type& tire_fl = front_axle.get_tire<Front_axle_type::LEFT>();
    const Front_right_tire_type& tire_fr = front_axle.get_tire<Front_axle_type::RIGHT>();

    const Rear_left_tire_type& tire_rl = rear_axle.get_tire<Rear_axle_type::LEFT>();
    const Rear_right_tire_type& tire_rr = rear_axle.get_tire<Rear_axle_type::RIGHT>();

    EXPECT_EQ(&std::as_const(chassis_copy).get_inertial_frame(), std::as_const(chassis_copy).get_road_frame().get_parent_ptr());

    EXPECT_EQ(&std::as_const(chassis_copy).get_road_frame(), std::as_const(chassis_copy).get_chassis_frame().get_parent_ptr());

    EXPECT_EQ(&std::as_const(chassis_copy).get_chassis_frame(), std::as_const(front_axle).get_frame().get_parent_ptr());
    EXPECT_EQ(&std::as_const(chassis_copy).get_chassis_frame(), std::as_const(rear_axle).get_frame().get_parent_ptr());

    EXPECT_EQ(&std::as_const(front_axle).get_frame(), std::as_const(tire_fl).get_frame().get_parent_ptr());
    EXPECT_EQ(&std::as_const(front_axle).get_frame(), std::as_const(tire_fr).get_frame().get_parent_ptr());

    EXPECT_EQ(&std::as_const(rear_axle).get_frame(), std::as_const(tire_rl).get_frame().get_parent_ptr());
    EXPECT_EQ(&std::as_const(rear_axle).get_frame(), std::as_const(tire_rr).get_frame().get_parent_ptr());
}
  

TEST_F(Chassis_test, chassis_position)
{
    const scalar x_chassis = x;
    const scalar y_chassis = y;
    const scalar z_chassis = z + database.get_element("vehicle/chassis/com").get_value(sVector3d())[2];
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_origin().at(0), x_chassis);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_origin().at(1), y_chassis);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_origin().at(2),       0.0);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_absolute_position().at(0), x_chassis);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_absolute_position().at(1), y_chassis);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_absolute_position().at(2),       0.0);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_rotation_angles().size(), 4);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_rotation_angles().at(3), psi);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_rotation_angles_derivative().at(3), omega);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_rotation_axis().at(3), Z);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_origin().at(0), 0.0);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_origin().at(1), 0.0);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_origin().at(2), z_chassis);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_absolute_position().at(0), x_chassis);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_absolute_position().at(1), y_chassis);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_absolute_position().at(2), z_chassis);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_rotation_angles().size(), 0);

}


TEST_F(Chassis_test, chassis_velocity)
{
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_relative_velocity_in_parent().at(0), dx);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_relative_velocity_in_parent().at(1), dy);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_relative_velocity_in_parent().at(2),0.0);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_absolute_velocity_in_inertial().at(0), dx);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_absolute_velocity_in_inertial().at(1), dy);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_absolute_velocity_in_inertial().at(2),0.0);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_absolute_velocity_in_body().at(0),  u);
    EXPECT_NEAR     (std::as_const(chassis).get_road_frame().get_absolute_velocity_in_body().at(1),  v, 1.0e-16);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_road_frame().get_absolute_velocity_in_body().at(2),0.0);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_relative_velocity_in_parent().at(0), 0.0);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_relative_velocity_in_parent().at(1), 0.0);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_relative_velocity_in_parent().at(2), dz);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_absolute_velocity_in_inertial().at(0), dx);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_absolute_velocity_in_inertial().at(1), dy);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_absolute_velocity_in_inertial().at(2), dz);

    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_absolute_velocity_in_body().at(0),  u);
    EXPECT_NEAR     (std::as_const(chassis).get_chassis_frame().get_absolute_velocity_in_body().at(1),  v, 1.0e-16);
    EXPECT_DOUBLE_EQ(std::as_const(chassis).get_chassis_frame().get_absolute_velocity_in_body().at(2), dz);
}


TEST_F(Chassis_test, front_axle_position)
{
    const scalar  R0              = 0.139;
    const Front_axle_type& front_axle = chassis.get_front_axle();

    const scalar& a = chassis_parameters.at("front_axle_x");
    const scalar& h = chassis_parameters.at("cog_height");

    const scalar x_axle = x + a*cos(psi);
    const scalar y_axle = y + a*sin(psi);
    const scalar z_axle = z - R0 -mu*a ;

    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_origin().at(0), a);
    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_origin().at(1),  0.0);
    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_origin().at(2), h-R0-a*mu);
                    
    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_absolute_position().at(0), x_axle);
    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_absolute_position().at(1), y_axle);
    EXPECT_NEAR(std::as_const(front_axle).get_frame().get_absolute_position().at(2), z_axle, 3.0e-17);

    EXPECT_EQ(std::as_const(front_axle).get_frame().get_rotation_angles().size(), 0);
}


TEST_F(Chassis_test, rear_axle_position)
{
    const scalar  R0            = 0.139;
    const Rear_axle_type& rear_axle = chassis.get_rear_axle();

    const scalar& a = chassis_parameters.at("rear_axle_x");
    const scalar& h = chassis_parameters.at("cog_height");

    const scalar x_axle = x + a*cos(psi);
    const scalar y_axle = y + a*sin(psi);
    const scalar z_axle = z - a*mu - R0;

    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_origin().at(0), a);
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_origin().at(1),  0.0);
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_origin().at(2), h-R0-a*mu);
                    
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_absolute_position().at(0), x_axle);
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_absolute_position().at(1), y_axle);
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_absolute_position().at(2), z_axle);

    EXPECT_EQ(std::as_const(rear_axle).get_frame().get_rotation_angles().size(), 0);
}


TEST_F(Chassis_test, front_axle_velocity)
{
    const Front_axle_type& front_axle = chassis.get_front_axle();

    const scalar& a = chassis_parameters.at("front_axle_x");

    const scalar u_axle = u;
    const scalar v_axle = v + a*omega;

    const scalar dx_axle = u_axle*cos(psi) - v_axle*sin(psi);
    const scalar dy_axle = u_axle*sin(psi) + v_axle*cos(psi);

    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_relative_velocity_in_parent().at(0),  0.0);
    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_relative_velocity_in_parent().at(1),  0.0);
    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_relative_velocity_in_parent().at(2), -a*dmu);

    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_absolute_velocity_in_body().at(0), u);
    EXPECT_NEAR     (std::as_const(front_axle).get_frame().get_absolute_velocity_in_body().at(1), v+a*omega, 2.0e-16);
    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_absolute_velocity_in_body().at(2), dz-a*dmu);
                    
    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_absolute_velocity_in_inertial().at(0), dx_axle);
    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_absolute_velocity_in_inertial().at(1), dy_axle);
    EXPECT_DOUBLE_EQ(std::as_const(front_axle).get_frame().get_absolute_velocity_in_inertial().at(2), dz-a*dmu);
}


TEST_F(Chassis_test, rear_axle_velocity)
{
    const Rear_axle_type& rear_axle = chassis.get_rear_axle();

    const scalar& a = chassis_parameters.at("rear_axle_x");

    const scalar u_axle = u;
    const scalar v_axle = v + a*omega;

    const scalar dx_axle = u_axle*cos(psi) - v_axle*sin(psi);
    const scalar dy_axle = u_axle*sin(psi) + v_axle*cos(psi);

    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_relative_velocity_in_parent().at(0),  0.0);
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_relative_velocity_in_parent().at(1),  0.0);
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_relative_velocity_in_parent().at(2), -a*dmu);

    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_absolute_velocity_in_body().at(0), u);
    EXPECT_NEAR     (std::as_const(rear_axle).get_frame().get_absolute_velocity_in_body().at(1), v+a*omega, 2.0e-16);
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_absolute_velocity_in_body().at(2), dz-a*dmu);
                    
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_absolute_velocity_in_inertial().at(0), dx_axle);
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_absolute_velocity_in_inertial().at(1), dy_axle);
    EXPECT_DOUBLE_EQ(std::as_const(rear_axle).get_frame().get_absolute_velocity_in_inertial().at(2), dz-a*dmu);
}


TEST_F(Chassis_test, front_left_tire_position)
{
    const scalar& R0         = 0.139;
    const Front_axle_type& axle  = chassis.get_front_axle();
    const Front_left_tire_type& tire = axle.get_tire<Front_axle_type::LEFT>();

    const scalar& a = chassis_parameters.at("front_axle_x");
    const scalar& t = front_axle_parameters.at("track");
    const scalar& beta = front_axle_parameters.at("beta_steering");

    const scalar& s = axle.get_chassis_deformation(Front_axle_type::LEFT);

    const scalar x_tire = x + a*cos(psi) + 0.5*t*sin(psi);
    const scalar y_tire = y + a*sin(psi) - 0.5*t*cos(psi);
    const scalar z_tire = z - R0 - a*mu - 0.5*t*phi - beta*delta + s;

    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(0), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(1), -0.5*t);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(2), -0.5*t*phi - beta*delta + s );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(0), x_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(1), y_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(2), z_tire );
}


TEST_F(Chassis_test, front_right_tire_position)
{
    const scalar& R0         = 0.139;
    const Front_axle_type& axle  = chassis.get_front_axle();
    const Front_right_tire_type& tire = axle.get_tire<Front_axle_type::RIGHT>();

    const scalar& a = chassis_parameters.at("front_axle_x");
    const scalar& t = front_axle_parameters.at("track");
    const scalar& beta = front_axle_parameters.at("beta_steering");

    const scalar& s = axle.get_chassis_deformation(Front_axle_type::RIGHT);

    const scalar x_tire = x + a*cos(psi) - 0.5*t*sin(psi);
    const scalar y_tire = y + a*sin(psi) + 0.5*t*cos(psi);
    const scalar z_tire = z - R0 - a*mu + 0.5*t*phi + beta*delta + s;

    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(0), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(1), 0.5*t);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(2), 0.5*t*phi + beta*delta + s );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(0), x_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(1), y_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(2), z_tire );
}


TEST_F(Chassis_test, rear_left_tire_position)
{
    const scalar& R0         = 0.139;
    const Rear_axle_type& axle   = chassis.get_rear_axle();
    const Rear_left_tire_type& tire = axle.get_tire<Rear_axle_type::LEFT>();

    const scalar& a = chassis_parameters.at("rear_axle_x");
    const scalar& t = rear_axle_parameters.at("track");

    const scalar& s = axle.get_chassis_deformation(Rear_axle_type::LEFT);

    const scalar x_tire = x + a*cos(psi) + 0.5*t*sin(psi);
    const scalar y_tire = y + a*sin(psi) - 0.5*t*cos(psi);
    const scalar z_tire = z - R0 - a*mu - 0.5*t*phi + s;

    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(0), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(1), -0.5*t);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(2), -0.5*t*phi + s );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(0), x_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(1), y_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(2), z_tire );
}


TEST_F(Chassis_test, rear_right_tire_position)
{
    const scalar& R0         = 0.139;
    const Rear_axle_type& axle   = chassis.get_rear_axle();
    const Rear_right_tire_type& tire = axle.get_tire<Rear_axle_type::RIGHT>();

    const scalar& a = chassis_parameters.at("rear_axle_x");
    const scalar& t = rear_axle_parameters.at("track");

    const scalar& s = axle.get_chassis_deformation(Rear_axle_type::RIGHT);

    const scalar x_tire = x + a*cos(psi) - 0.5*t*sin(psi);
    const scalar y_tire = y + a*sin(psi) + 0.5*t*cos(psi);
    const scalar z_tire = z - R0 - a*mu + 0.5*t*phi + s;

    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(0), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(1), 0.5*t);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_origin().at(2), 0.5*t*phi + s );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(0), x_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(1), y_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_position().at(2), z_tire );
}


TEST_F(Chassis_test, front_axle_deformations)
{
    const scalar& R0               = 0.139;
    const Front_axle_type& axle        = chassis.get_front_axle();
    const Front_left_tire_type& tire_left  = axle.get_tire<Front_axle_type::LEFT>();
    const Front_right_tire_type& tire_right = axle.get_tire<Front_axle_type::RIGHT>();

    const scalar& a = chassis_parameters.at("front_axle_x");
    const scalar& t = front_axle_parameters.at("track");
    const scalar& beta = front_axle_parameters.at("beta_steering");

    const scalar& kchassis  = front_axle_parameters.at("chassis_stiffness");
    const scalar& kantiroll = front_axle_parameters.at("antiroll_stiffness");
    const scalar& ktire     = 64.5e3;

    const scalar wl = std::as_const(tire_left).get_frame().get_absolute_position({0.0, 0.0, R0}).at(Z);
    const scalar wr = std::as_const(tire_right).get_frame().get_absolute_position({0.0, 0.0, R0}).at(Z);

    const scalar sl = axle.get_chassis_deformation(Front_axle_type::LEFT);
    const scalar sr = axle.get_chassis_deformation(Front_axle_type::RIGHT);

    EXPECT_NEAR(kchassis*sl + kantiroll*(sl-sr) + ktire*wl, 0.0, 1.0e-16*ktire);
    EXPECT_NEAR(kchassis*sr + kantiroll*(sr-sl) + ktire*wr, 0.0, 1.0e-16*ktire);
    EXPECT_DOUBLE_EQ(wl, z - a*mu - 0.5*t*phi - beta*delta + sl);
    EXPECT_DOUBLE_EQ(wr, z - a*mu + 0.5*t*phi + beta*delta + sr);
    
    EXPECT_DOUBLE_EQ(wl, tire_left.get_vertical_deformation());
    EXPECT_DOUBLE_EQ(wr, tire_right.get_vertical_deformation());
}


TEST_F(Chassis_test, rear_axle_deformations)
{
    const scalar& R0               = 0.139;
    const Rear_axle_type& axle         = chassis.get_rear_axle();
    const Rear_left_tire_type& tire_left  = axle.get_tire<Rear_axle_type::LEFT>();
    const Rear_right_tire_type& tire_right = axle.get_tire<Rear_axle_type::RIGHT>();

    const scalar& a = chassis_parameters.at("rear_axle_x");
    const scalar& t = rear_axle_parameters.at("track");

    const scalar& kchassis  = rear_axle_parameters.at("chassis_stiffness");
    const scalar& kantiroll = rear_axle_parameters.at("antiroll_stiffness");
    const scalar& ktire     = 61.3e3;

    const scalar wl = std::as_const(tire_left).get_frame().get_absolute_position({0.0, 0.0, R0}).at(Z);
    const scalar wr = std::as_const(tire_right).get_frame().get_absolute_position({0.0, 0.0, R0}).at(Z);

    const scalar sl = axle.get_chassis_deformation(Rear_axle_type::LEFT);
    const scalar sr = axle.get_chassis_deformation(Rear_axle_type::RIGHT);

    EXPECT_NEAR(kchassis*sl + kantiroll*(sl-sr) + ktire*wl, 0.0, 1.0e-16*ktire);
    EXPECT_NEAR(kchassis*sr + kantiroll*(sr-sl) + ktire*wr, 0.0, 1.0e-16*ktire);
    EXPECT_DOUBLE_EQ(wl, z - a*mu - 0.5*t*phi + sl);
    EXPECT_DOUBLE_EQ(wr, z - a*mu + 0.5*t*phi + sr);

    EXPECT_DOUBLE_EQ(wl, tire_left.get_vertical_deformation());
    EXPECT_DOUBLE_EQ(wr, tire_right.get_vertical_deformation());
}


TEST_F(Chassis_test, front_axle_deformations_velocity)
{
    const scalar& R0                        = 0.139;
    const Front_axle_type& axle             = chassis.get_front_axle();
    const Front_left_tire_type& tire_left   = axle.get_tire<Front_axle_type::LEFT>();
    const Front_right_tire_type& tire_right = axle.get_tire<Front_axle_type::RIGHT>();

    const scalar& a = chassis_parameters.at("front_axle_x");
    const scalar& t = front_axle_parameters.at("track");

    const scalar& kchassis  = front_axle_parameters.at("chassis_stiffness");
    const scalar& kantiroll = front_axle_parameters.at("antiroll_stiffness");
    const scalar  ktire     = 64.5e3;

    const scalar dwl = std::as_const(tire_left).get_frame().get_absolute_velocity_in_body({0.0, 0.0, R0}).at(Z);
    const scalar dwr = std::as_const(tire_right).get_frame().get_absolute_velocity_in_body({0.0, 0.0, R0}).at(Z);

    const scalar dsl = axle.get_chassis_deformation_velocity(Front_axle_type::LEFT);
    const scalar dsr = axle.get_chassis_deformation_velocity(Front_axle_type::RIGHT);

    EXPECT_NEAR(kchassis*dsl + kantiroll*(dsl-dsr) + ktire*dwl, 0.0, 1.0e-16*ktire);
    EXPECT_NEAR(kchassis*dsr + kantiroll*(dsr-dsl) + ktire*dwr, 0.0, 1.0e-16*ktire);
    EXPECT_DOUBLE_EQ(dwl, dz - a*dmu - 0.5*t*dphi + dsl);
    EXPECT_DOUBLE_EQ(dwr, dz - a*dmu + 0.5*t*dphi + dsr);
    
    EXPECT_DOUBLE_EQ(dwl, tire_left.get_vertical_deformation_velocity());
    EXPECT_DOUBLE_EQ(dwr, tire_right.get_vertical_deformation_velocity());
}


TEST_F(Chassis_test, rear_axle_deformations_velocity)
{
    const scalar& R0               = 0.139;
    const Rear_axle_type& axle         = chassis.get_rear_axle();
    const Rear_left_tire_type& tire_left  = axle.get_tire<Rear_axle_type::LEFT>();
    const Rear_right_tire_type& tire_right = axle.get_tire<Rear_axle_type::RIGHT>();

    const scalar& a = chassis_parameters.at("rear_axle_x");
    const scalar& t = rear_axle_parameters.at("track");

    const scalar& kchassis  = rear_axle_parameters.at("chassis_stiffness");
    const scalar& kantiroll = rear_axle_parameters.at("antiroll_stiffness");
    const scalar  ktire     = 61.3e3;

    const scalar dwl = std::as_const(tire_left).get_frame().get_absolute_velocity_in_body({0.0, 0.0, R0}).at(Z);
    const scalar dwr = std::as_const(tire_right).get_frame().get_absolute_velocity_in_body({0.0, 0.0, R0}).at(Z);

    const scalar dsl = axle.get_chassis_deformation_velocity(Rear_axle_type::LEFT);
    const scalar dsr = axle.get_chassis_deformation_velocity(Rear_axle_type::RIGHT);

    EXPECT_NEAR(kchassis*dsl + kantiroll*(dsl-dsr) + ktire*dwl, 0.0, 1.0e-16*ktire);
    EXPECT_NEAR(kchassis*dsr + kantiroll*(dsr-dsl) + ktire*dwr, 0.0, 1.0e-16*ktire);
    EXPECT_DOUBLE_EQ(dwl, dz - a*dmu - 0.5*t*dphi + dsl);
    EXPECT_DOUBLE_EQ(dwr, dz - a*dmu + 0.5*t*dphi + dsr);
    
    EXPECT_DOUBLE_EQ(dwl, tire_left.get_vertical_deformation_velocity());
    EXPECT_DOUBLE_EQ(dwr, tire_right.get_vertical_deformation_velocity());
}


TEST_F(Chassis_test, front_left_tire_velocity)
{
    const Front_axle_type& axle  = chassis.get_front_axle();
    const Front_left_tire_type& tire = axle.get_tire<Front_axle_type::LEFT>();

    const scalar& a = chassis_parameters.at("front_axle_x");
    const scalar& t = front_axle_parameters.at("track");

    const scalar ds = axle.get_chassis_deformation_velocity(Front_axle_type::LEFT);

    const scalar u_tire = u + 0.5*t*omega;
    const scalar v_tire = v + a*omega;

    const scalar u_body = u_tire*cos(delta) + v_tire*sin(delta);
    const scalar v_body = v_tire*cos(delta) - u_tire*sin(delta);
    

    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(0), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(1), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(2), ds - 0.5*t*dphi );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_parent().at(0), u_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_parent().at(1), v_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_parent().at(2), dz - a*dmu + ds - 0.5*t*dphi );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_body().at(0), u_body);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_body().at(1), v_body);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_body().at(2), dz - a*dmu + ds - 0.5*t*dphi );

    EXPECT_DOUBLE_EQ(tire.get_lambda(), -v_body/u_body);
    EXPECT_DOUBLE_EQ(tire.get_kappa(),  0.0);
}


TEST_F(Chassis_test, front_right_tire_velocity)
{
    const Front_axle_type& axle  = chassis.get_front_axle();
    const Front_right_tire_type& tire = axle.get_tire<Front_axle_type::RIGHT>();

    const scalar& a = chassis_parameters.at("front_axle_x");
    const scalar& t = front_axle_parameters.at("track");

    const scalar ds = axle.get_chassis_deformation_velocity(Front_axle_type::RIGHT);

    const scalar u_tire = u - 0.5*t*omega;
    const scalar v_tire = v + a*omega;

    const scalar u_body = u_tire*cos(delta) + v_tire*sin(delta);
    const scalar v_body = v_tire*cos(delta) - u_tire*sin(delta);

    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(0), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(1), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(2), ds + 0.5*t*dphi );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_parent().at(0), u_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_parent().at(1), v_tire);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_parent().at(2), dz - a*dmu + ds + 0.5*t*dphi );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_body().at(0), u_body);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_body().at(1), v_body);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_body().at(2), dz - a*dmu + ds + 0.5*t*dphi );

    EXPECT_DOUBLE_EQ(tire.get_lambda(), -v_body/u_body);
    EXPECT_NEAR     (tire.get_kappa(),  0.0, 3.0e-16);
}


TEST_F(Chassis_test, rear_left_tire_velocity)
{
    const scalar& R0         = 0.139;
    const Rear_axle_type& axle   = chassis.get_rear_axle();
    const Rear_left_tire_type& tire = axle.get_tire<Rear_axle_type::LEFT>();

    const scalar& a = chassis_parameters.at("rear_axle_x");
    const scalar& t = rear_axle_parameters.at("track");

    const scalar ds = axle.get_chassis_deformation_velocity(Rear_axle_type::LEFT);

    const scalar u_tire = u + 0.5*t*omega;
    const scalar v_tire = v + a*omega;

    const scalar u_body = u_tire;
    const scalar v_body = v_tire;
    

    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(0), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(1), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(2), ds - 0.5*t*dphi );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_parent().at(0), u_tire);
    EXPECT_NEAR     (tire.get_frame().get_absolute_velocity_in_parent().at(1), v_tire, 1.0e-16);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_parent().at(2), dz - a*dmu + ds - 0.5*t*dphi );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_body().at(0), u_body);
    EXPECT_NEAR     (tire.get_frame().get_absolute_velocity_in_body().at(1), v_body, 1.0e-16);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_body().at(2), dz - a*dmu + ds - 0.5*t*dphi );

    EXPECT_NEAR     (tire.get_lambda(), -v_body/u_body,1.0e-16);
    EXPECT_NEAR     (tire.get_kappa(),  omega_axle*R0/u_body - 1.0,3.0e-16);
}


TEST_F(Chassis_test, rear_right_tire_velocity)
{
    const scalar& R0         = 0.139;
    const Rear_axle_type& axle   = chassis.get_rear_axle();
    const Rear_right_tire_type& tire = axle.get_tire<Rear_axle_type::RIGHT>();

    const scalar& a = chassis_parameters.at("rear_axle_x");
    const scalar& t = rear_axle_parameters.at("track");

    const scalar ds = axle.get_chassis_deformation_velocity(Rear_axle_type::RIGHT);

    const scalar u_tire = u - 0.5*t*omega;
    const scalar v_tire = v + a*omega;

    const scalar u_body = u_tire;
    const scalar v_body = v_tire;

    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(0), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(1), 0.0);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_relative_velocity_in_parent().at(2), ds + 0.5*t*dphi );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_parent().at(0), u_tire);
    EXPECT_NEAR     (tire.get_frame().get_absolute_velocity_in_parent().at(1), v_tire, 1.0e-16);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_parent().at(2), dz - a*dmu + ds + 0.5*t*dphi );

    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_body().at(0), u_body);
    EXPECT_NEAR     (tire.get_frame().get_absolute_velocity_in_body().at(1), v_body, 1.0e-16);
    EXPECT_DOUBLE_EQ(tire.get_frame().get_absolute_velocity_in_body().at(2), dz - a*dmu + ds + 0.5*t*dphi );

    EXPECT_NEAR     (tire.get_lambda(), -v_body/u_body,1.0e-16);
    EXPECT_DOUBLE_EQ(tire.get_kappa(),  omega_axle*R0/u_body - 1.0);
}


TEST_F(Chassis_test, total_force)
{
    const Front_axle_type& front_axle = chassis.get_front_axle();
    const Rear_axle_type& rear_axle   = chassis.get_rear_axle();

    const Front_left_tire_type& tire_fl = front_axle.get_tire<Front_axle_type::LEFT>();
    const Front_right_tire_type& tire_fr = front_axle.get_tire<Front_axle_type::RIGHT>();
    const Rear_left_tire_type& tire_rl = rear_axle.get_tire<Rear_axle_type::LEFT>();
    const Rear_right_tire_type& tire_rr = rear_axle.get_tire<Rear_axle_type::RIGHT>();
    
    const sVector3d& F_fl = tire_fl.get_force();
    const sVector3d& F_fr = tire_fr.get_force();
    const sVector3d& F_rl = tire_rl.get_force();
    const sVector3d& F_rr = tire_rr.get_force();
    
    // Forces as written in Lot 2016 (except the normal, we use the other sign convention)
    const scalar Fx = F_rl[X] + F_rr[X] - (F_fl[Y]+F_fr[Y])*sin(delta);
    const scalar Fy = F_rl[Y] + F_rr[Y] + (F_fl[Y]+F_fr[Y])*cos(delta);
    const scalar Fz = chassis.get_mass()*g0 + F_fl[Z] + F_fr[Z] + F_rl[Z] + F_rr[Z];

    const scalar Fx_aero = 0.5*chassis_parameters.at("rho_air")*chassis_parameters.at("CdA")*sqrt(u*u+v*v)*u;
    const scalar Fy_aero = 0.5*chassis_parameters.at("rho_air")*chassis_parameters.at("CdA")*sqrt(u*u+v*v)*v;

    EXPECT_DOUBLE_EQ(chassis.get_force().at(X), Fx-Fx_aero);
    EXPECT_DOUBLE_EQ(chassis.get_force().at(Y), Fy-Fy_aero);
    EXPECT_DOUBLE_EQ(chassis.get_force().at(Z), Fz);

    EXPECT_EQ(F_fl[Z] <= 0.0, true);
    EXPECT_EQ(F_fr[Z] <= 0.0, true);
    EXPECT_EQ(F_rl[Z] <= 0.0, true);
    EXPECT_EQ(F_rr[Z] <= 0.0, true);
}


TEST_F(Chassis_test, total_torque)
{
    const Front_axle_type& front_axle = chassis.get_front_axle();
    const Rear_axle_type& rear_axle   = chassis.get_rear_axle();

    const Front_left_tire_type& tire_fl = front_axle.get_tire<Front_axle_type::LEFT>();
    const Front_right_tire_type& tire_fr = front_axle.get_tire<Front_axle_type::RIGHT>();
    const Rear_left_tire_type& tire_rl = rear_axle.get_tire<Rear_axle_type::LEFT>();
    const Rear_right_tire_type& tire_rr = rear_axle.get_tire<Rear_axle_type::RIGHT>();

    const scalar  tf = 0.5*front_axle_parameters.at("track");
    const scalar  tr = 0.5*rear_axle_parameters.at("track");

    const scalar& a = chassis_parameters.at("front_axle_x");
    const scalar& b = - chassis_parameters.at("rear_axle_x");
    const scalar& h = chassis_parameters.at("cog_height");
    
    const sVector3d& F_fl = tire_fl.get_force();
    const sVector3d& F_fr = tire_fr.get_force();
    const sVector3d& F_rl = tire_rl.get_force();
    const sVector3d& F_rr = tire_rr.get_force();
    
    // Torques as written in Lot 2016
    const scalar Tx = - tf*(F_fl[Z] - F_fr[Z]) - tr*(F_rl[Z] - F_rr[Z]) + 0.5*rho*CdA*sqrt(u*u+v*v)*v*(z-h);
    const scalar Ty = - a*(F_fr[Z] + F_fl[Z]) + b*(F_rr[Z] + F_rl[Z]) - 0.5*rho*CdA*sqrt(u*u+v*v)*u*(z-h);
    const scalar Tz = a*(F_fl[Y]+F_fr[Y])*cos(delta) - b*(F_rl[Y]+F_rr[Y])
                          +tf*(F_fr[Y]-F_fl[Y])*sin(delta) + tr*(F_rl[X] - F_rr[X]);

    EXPECT_DOUBLE_EQ(chassis.get_torque().at(X), Tx);
    EXPECT_DOUBLE_EQ(chassis.get_torque().at(Y), Ty);
    EXPECT_DOUBLE_EQ(chassis.get_torque().at(Z), Tz);

    EXPECT_EQ(F_fl[Z] <= 0.0, true);
    EXPECT_EQ(F_fr[Z] <= 0.0, true);
    EXPECT_EQ(F_rl[Z] <= 0.0, true);
    EXPECT_EQ(F_rr[Z] <= 0.0, true);
}


TEST_F(Chassis_test, newton_equations)
{
    const scalar m     = chassis_parameters.at("mass");
    const sVector3d dv = chassis.get_acceleration();

    EXPECT_DOUBLE_EQ(m*(dv[X] - omega*v), chassis.get_force().at(X));
    EXPECT_DOUBLE_EQ(m*(dv[Y] + omega*u), chassis.get_force().at(Y));
    EXPECT_DOUBLE_EQ(m*dv[Z], chassis.get_force().at(Z));

}


TEST_F(Chassis_test, euler_equations)
{
    const scalar m        = chassis_parameters.at("mass");
    const sVector3d dv    = chassis.get_acceleration();
    const sVector3d d2phi = chassis.get_angles_acceleration();
    const scalar& h       = chassis_parameters.at("cog_height");

    const scalar Ixx = chassis_parameters.at("inertia_xx");
    const scalar Iyy = chassis_parameters.at("inertia_yy");
    const scalar Izz = chassis_parameters.at("inertia_zz");
    const scalar Ixz = chassis_parameters.at("inertia_xz");

    EXPECT_DOUBLE_EQ(m*(dv[Y]*h + (h-z)*omega*u) + Ixx*d2phi[X] - (Iyy-Izz+Ixx)*omega*dmu
                     -(Iyy-Izz)*omega*omega*phi - (Ixx-Izz)*d2phi[Z]*mu - Ixz*d2phi[Z], chassis.get_torque().at(X));

    EXPECT_DOUBLE_EQ(m*((z-h)*dv[X] + h*omega*v) + Iyy*d2phi[Y] + (Iyy-Izz+Ixx)*omega*dphi
                     -(Ixx-Izz)*omega*omega*mu+(Iyy-Izz)*d2phi[Z]*phi - Ixz*omega*omega,chassis.get_torque().at(Y));

    EXPECT_DOUBLE_EQ(Izz*d2phi[Z] - Ixz*(d2phi[X]-2.0*mu*d2phi[Z]-2.0*dmu*omega), chassis.get_torque().at(Z));

}
