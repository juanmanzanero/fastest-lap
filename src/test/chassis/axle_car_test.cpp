#include "gtest/gtest.h"
#include "src/core/chassis/axle_car_6dof.h"
#include "src/core/tire/tire_pacejka.h"

using Tire_t       = Tire_pacejka_std<scalar,0,0>;
using Front_axle_t = Axle_car_6dof<scalar,Tire_t,Tire_t,STEERING_FREE_ROLL,0,0>;
using Rear_axle_t  = Axle_car_6dof<scalar,Tire_t,Tire_t,POWERED_WITHOUT_DIFFERENTIAL,0,0>;

Front_axle_t construct_front_axle(const sFrame& car_frame, const std::map<std::string,scalar>& parameters)
{
    // Construct tires
    Xml_document database = { "database/vehicles/kart/roberto-lot-kart-2016.xml", true };
    
    // set the 'smooth' max parameter to 0
    database.get_element("vehicle/front-tire/Fz-max-ref2").set_value("0.0");
    database.get_element("vehicle/rear-tire/Fz-max-ref2").set_value("0.0");

    Tire_t tire_fl("front left", database, "vehicle/front-tire/");
    Tire_t tire_fr("front right", database, "vehicle/front-tire/");

    // A custom database because we included antiroll stifness
    Xml_document front_axle_data;
    front_axle_data.parse("<vehicle>"
                          "    <front-axle>"
                          "        <track>1.055</track>"
                          "        <stiffness>"
                          "            <chassis>17.7e3</chassis>"
                          "            <antiroll>30.0e3</antiroll>"
                          "        </stiffness>"
                          "        <beta-steering>"
                          "            <left>-0.058</left>"
                          "            <right>0.058</right>"
                          "        </beta-steering>"
                          "    </front-axle>"
                          "</vehicle>");

    return Front_axle_t("front axle", 
                    tire_fl, tire_fr, 
                    front_axle_data, "vehicle/front-axle/"
                   );


}


Rear_axle_t construct_rear_axle(const sFrame& car_frame, const std::map<std::string,scalar>& parameters)
{
    // Construct tires
    Xml_document database = { "database/vehicles/kart/roberto-lot-kart-2016.xml", true };

    // set the 'smooth' max parameter to 0
    database.get_element("vehicle/front-tire/Fz-max-ref2").set_value("0.0");
    database.get_element("vehicle/rear-tire/Fz-max-ref2").set_value("0.0");

    Tire_t tire_rl("rear left", database, "vehicle/rear-tire/");
    Tire_t tire_rr("rear right", database, "vehicle/rear-tire/");

    // A custom database because we included antiroll stifness
    Xml_document rear_axle_data;
    rear_axle_data.parse( "<vehicle>"
                          "    <rear-axle>"
                          "        <track>1.200</track>"
                          "        <stiffness>"
                          "            <chassis>60.0e3</chassis>"
                          "            <antiroll>35.0e3</antiroll>"
                          "        </stiffness>"
                          "        <inertia>0.2</inertia>"
                          "        <smooth_throttle_coeff> 0.0 </smooth_throttle_coeff>"
                          "    </rear-axle>"
                          "</vehicle>");

    return Rear_axle_t("rear axle", 
                    tire_rl, tire_rr, 
                    rear_axle_data, "vehicle/rear-axle/"
                   );
}


//class Axle_car_test : public ::testing::Test
class Axle_car_test : public ::testing::TestWithParam<std::tuple<double,double,double,double,double,double,
                                                                 double,double,double,double,double,double>>
{
 protected:
    Axle_car_test() 
        { 
            front_axle.get_frame().set_parent(car_frame);

            front_axle.set_steering_angle(delta);
            front_axle.update({front_axle_parameters.at("position"), 0.0,z_front},{0.0, 0.0, dz_front},
                              phi, dphi);

            rear_axle.get_frame().set_parent(car_frame);
            rear_axle.set_torque_and_omega(T, omega_axle);
            rear_axle.update({-rear_axle_parameters.at("position"), 0.0,z_rear}, {0.0, 0.0, dz_rear},
                              phi, dphi);
        }
    
    const scalar x = 2.0;
    const scalar y = 4.0;

    const scalar psi = std::get<0>(GetParam());
    const scalar omega = std::get<1>(GetParam());

    const scalar u = std::get<2>(GetParam());
    const scalar v = std::get<3>(GetParam());

    const scalar dx = u*cos(psi) - v*sin(psi);
    const scalar dy = u*sin(psi) + v*cos(psi);

    const sFrame inertial_frame;
    const sFrame car_frame = sFrame({x,y,0.0}, {dx,dy,0}, {psi}, {omega}, {Z}, inertial_frame, sFrame::Frame_velocity_types::parent_frame);

    // State of the front axle
    const scalar z_front = std::get<4>(GetParam()); 
    const scalar dz_front = std::get<5>(GetParam());

    // State of the rear axle
    const scalar z_rear = std::get<6>(GetParam());
    const scalar dz_rear = std::get<7>(GetParam());
    const scalar T = 0.1*17.6;    // Maximum torque at 10.250rpm
    const scalar omega_axle = std::get<8>(GetParam());

    const scalar phi = std::get<9>(GetParam());
    const scalar dphi = std::get<10>(GetParam());

    const scalar delta = std::get<11>(GetParam());

    std::map<std::string,scalar> front_axle_parameters = { std::pair("position", 0.645),
                                                           std::pair("track", 1.055),
                                                           std::pair("chassis_stiffness", 17.7e3),
                                                           // Antiroll stiffness = 0 in Lot2016
                                                           std::pair("antiroll_stiffness",30.0e3),
                                                           std::pair("beta_steering", 0.058)
                                                         };

    std::map<std::string,scalar> rear_axle_parameters = { std::pair("position", 0.400), 
                                                          std::pair("track", 1.200),
                                                          std::pair("chassis_stiffness", 60.0e3),
                                                          // Antiroll stiffness = 0 in Lot2016
                                                          std::pair("antiroll_stiffness",35.0e3),
                                                          std::pair("inertia", 0.2) 
                                                         };




    Front_axle_t front_axle = construct_front_axle(car_frame, front_axle_parameters);

    Rear_axle_t rear_axle = construct_rear_axle(car_frame, rear_axle_parameters);
};

INSTANTIATE_TEST_SUITE_P(Axle_car_test_conf1, Axle_car_test, ::testing::Values(
//                         psi   omega u   v    zfr  dzfr  zre  dzre omegaaxle phi    dphi   delta
           std::make_tuple(pi/4.0,0.1,0.95,0.1,-0.134,0.1,-0.039,0.2,10.0,3.0*DEG,0.1,15.0*DEG) ));

INSTANTIATE_TEST_SUITE_P(Axle_car_test_conf2, Axle_car_test, ::testing::Values(
//                         psi   omega u   v    zfr    dzfr  zre  dzre omegaaxle phi    dphi   delta
   std::make_tuple(135.0*DEG,0.2,4.00,0.2,-0.080,-0.04,-0.120,-0.06,20.0,5.0*DEG,0.05,30.0*DEG) ));





TEST_P(Axle_car_test, front_frame_tree_sanity)
{
    const Tire_t& tire_fl = front_axle.get_tire<Front_axle_t::LEFT>();
    const Tire_t& tire_fr = front_axle.get_tire<Front_axle_t::RIGHT>();

    EXPECT_EQ(&front_axle.get_frame(), tire_fl.get_frame().get_parent_ptr());
    EXPECT_EQ(&front_axle.get_frame(), tire_fr.get_frame().get_parent_ptr());
}


TEST_P(Axle_car_test, rear_frame_tree_sanity)
{
    const Tire_t& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const Tire_t& tire_rr = rear_axle.get_tire<Rear_axle_t::RIGHT>();

    EXPECT_EQ(&rear_axle.get_frame(), tire_rl.get_frame().get_parent_ptr());
    EXPECT_EQ(&rear_axle.get_frame(), tire_rr.get_frame().get_parent_ptr());
}


TEST_P(Axle_car_test, front_frame_tree_sanity_copy_assignment)
{
    Front_axle_t front_axle_copy;

    front_axle_copy = front_axle;

    const Tire_t& tire_fl = front_axle_copy.get_tire<Front_axle_t::LEFT>();
    const Tire_t& tire_fr = front_axle_copy.get_tire<Front_axle_t::RIGHT>();

    EXPECT_EQ(&front_axle_copy.get_frame(), tire_fl.get_frame().get_parent_ptr());
    EXPECT_EQ(&front_axle_copy.get_frame(), tire_fr.get_frame().get_parent_ptr());
}


TEST_P(Axle_car_test, rear_frame_tree_sanity_copy_assignment)
{
    Rear_axle_t rear_axle_copy;

    rear_axle_copy = rear_axle;

    const Tire_t& tire_rl = rear_axle_copy.get_tire<Rear_axle_t::LEFT>();
    const Tire_t& tire_rr = rear_axle_copy.get_tire<Rear_axle_t::RIGHT>();

    EXPECT_EQ(&rear_axle_copy.get_frame(), tire_rl.get_frame().get_parent_ptr());
    EXPECT_EQ(&rear_axle_copy.get_frame(), tire_rr.get_frame().get_parent_ptr());
}


TEST_P(Axle_car_test, rear_set_torque_and_omega)
{
    rear_axle.set_torque_and_omega(2.0,4.0);

    EXPECT_DOUBLE_EQ(rear_axle.get_axle_torque(), 2.0);
    EXPECT_DOUBLE_EQ(rear_axle.get_omega(), 4.0);
}


TEST_P(Axle_car_test, front_set_delta)
{
    front_axle.set_steering_angle(10.0*DEG);
    EXPECT_EQ(front_axle.get_steering_angle(), 10.0*DEG);

    const Tire_t& tire_fl = front_axle.get_tire<Front_axle_t::LEFT>();
    const Tire_t& tire_fr = front_axle.get_tire<Front_axle_t::RIGHT>();

    const std::vector<scalar>& left_rot_angles = tire_fl.get_frame().get_rotation_angles();
    const std::vector<scalar>& right_rot_angles = tire_fr.get_frame().get_rotation_angles();

    EXPECT_EQ(left_rot_angles.size(), 1);
    EXPECT_EQ(right_rot_angles.size(), 1);

    EXPECT_EQ(left_rot_angles.at(0), 10.0*DEG);
    EXPECT_EQ(right_rot_angles.at(0), 10.0*DEG);
}


TEST_P(Axle_car_test, front_axle_position)
{
    const scalar x_front = x + cos(psi)*front_axle_parameters.at("position");
    const scalar y_front = y + sin(psi)*front_axle_parameters.at("position");

    EXPECT_DOUBLE_EQ(front_axle.get_frame().get_absolute_position().at(X), x_front);
    EXPECT_DOUBLE_EQ(front_axle.get_frame().get_absolute_position().at(Y), y_front);
    EXPECT_DOUBLE_EQ(front_axle.get_frame().get_absolute_position().at(Z), z_front);
}


TEST_P(Axle_car_test, rear_axle_position)
{
    const scalar x_rear = x - cos(psi)*rear_axle_parameters.at("position");
    const scalar y_rear = y - sin(psi)*rear_axle_parameters.at("position");

    EXPECT_DOUBLE_EQ(rear_axle.get_frame().get_absolute_position().at(X), x_rear);
    EXPECT_DOUBLE_EQ(rear_axle.get_frame().get_absolute_position().at(Y), y_rear);
    EXPECT_DOUBLE_EQ(rear_axle.get_frame().get_absolute_position().at(Z), z_rear);
}


TEST_P(Axle_car_test, front_axle_velocity)
{
    const scalar& a = front_axle_parameters.at("position");

    EXPECT_DOUBLE_EQ(front_axle.get_frame().get_absolute_velocity_in_body().at(X), u);
    EXPECT_NEAR     (front_axle.get_frame().get_absolute_velocity_in_body().at(Y), v+omega*a,2.0e-16);
    EXPECT_DOUBLE_EQ(front_axle.get_frame().get_absolute_velocity_in_body().at(Z), dz_front);
}


TEST_P(Axle_car_test, rear_axle_velocity)
{
    const scalar& a = rear_axle_parameters.at("position");

    EXPECT_DOUBLE_EQ(rear_axle.get_frame().get_absolute_velocity_in_body().at(X), u);
    EXPECT_NEAR     (rear_axle.get_frame().get_absolute_velocity_in_body().at(Y), v-omega*a,5.0e-16);
    EXPECT_DOUBLE_EQ(rear_axle.get_frame().get_absolute_velocity_in_body().at(Z), dz_rear);
}


TEST_P(Axle_car_test, front_left_tire_position)
{
    const Tire_t& tire_fl = front_axle.get_tire<Front_axle_t::LEFT>();
    const scalar& a = front_axle_parameters.at("position");
    const scalar& t = front_axle_parameters.at("track");
    const scalar& beta = front_axle_parameters.at("beta_steering");

    const scalar x_fl = x + cos(psi)*a + 0.5*sin(psi)*t;
    const scalar y_fl = y + sin(psi)*a - 0.5*cos(psi)*t;
    const scalar z_fl = z_front - 0.5*t*phi - beta*delta
                            + front_axle.get_chassis_deformation(Front_axle_t::LEFT);

    EXPECT_DOUBLE_EQ(tire_fl.get_frame().get_absolute_position().at(X), x_fl);
    EXPECT_DOUBLE_EQ(tire_fl.get_frame().get_absolute_position().at(Y), y_fl);
    EXPECT_DOUBLE_EQ(tire_fl.get_frame().get_absolute_position().at(Z), z_fl);
}


TEST_P(Axle_car_test, rear_left_tire_position)
{
    const Tire_t& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const scalar& a = rear_axle_parameters.at("position");
    const scalar& t = rear_axle_parameters.at("track");

    const scalar x_rl = x - cos(psi)*a + 0.5*sin(psi)*t;
    const scalar y_rl = y - sin(psi)*a - 0.5*cos(psi)*t;
    const scalar z_rl = z_rear - 0.5*t*phi
                            + rear_axle.get_chassis_deformation(Rear_axle_t::LEFT);

    EXPECT_DOUBLE_EQ(tire_rl.get_frame().get_absolute_position().at(X), x_rl);
    EXPECT_DOUBLE_EQ(tire_rl.get_frame().get_absolute_position().at(Y), y_rl);
    EXPECT_DOUBLE_EQ(tire_rl.get_frame().get_absolute_position().at(Z), z_rl);
}


TEST_P(Axle_car_test, front_right_tire_position)
{
    const Tire_t& tire_fr = front_axle.get_tire<Front_axle_t::RIGHT>();
    const scalar& a = front_axle_parameters.at("position");
    const scalar& t = front_axle_parameters.at("track");
    const scalar& beta = front_axle_parameters.at("beta_steering");

    const scalar x_fr = x + cos(psi)*a - 0.5*sin(psi)*t;
    const scalar y_fr = y + sin(psi)*a + 0.5*cos(psi)*t;
    const scalar z_fr = z_front + 0.5*t*phi + beta*delta 
                            + front_axle.get_chassis_deformation(Front_axle_t::RIGHT);

    EXPECT_DOUBLE_EQ(tire_fr.get_frame().get_absolute_position().at(X), x_fr);
    EXPECT_DOUBLE_EQ(tire_fr.get_frame().get_absolute_position().at(Y), y_fr);
    EXPECT_DOUBLE_EQ(tire_fr.get_frame().get_absolute_position().at(Z), z_fr);
}


TEST_P(Axle_car_test, rear_right_tire_position)
{
    const Tire_t& tire_rr = rear_axle.get_tire<Front_axle_t::RIGHT>();
    const scalar& a = rear_axle_parameters.at("position");
    const scalar& t = rear_axle_parameters.at("track");

    const scalar x_rr = x - cos(psi)*a - 0.5*sin(psi)*t;
    const scalar y_rr = y - sin(psi)*a + 0.5*cos(psi)*t;
    const scalar z_rr = z_rear + 0.5*t*phi
                            + rear_axle.get_chassis_deformation(Rear_axle_t::RIGHT);

    EXPECT_DOUBLE_EQ(tire_rr.get_frame().get_absolute_position().at(X), x_rr);
    EXPECT_DOUBLE_EQ(tire_rr.get_frame().get_absolute_position().at(Y), y_rr);
    EXPECT_DOUBLE_EQ(tire_rr.get_frame().get_absolute_position().at(Z), z_rr);
}


TEST_P(Axle_car_test, front_left_tire_velocity)
{
    const Tire_t& tire_fl = front_axle.get_tire<Front_axle_t::LEFT>();
    const scalar& a             = front_axle_parameters.at("position");
    const scalar& t             = front_axle_parameters.at("track");

    const scalar vz = dz_front + front_axle.get_chassis_deformation_velocity(Front_axle_t::LEFT)
                            - 0.5*dphi*t ;

    EXPECT_DOUBLE_EQ(tire_fl.get_frame().get_absolute_velocity_in_parent().at(X), u+0.5*omega*t);
    EXPECT_NEAR     (tire_fl.get_frame().get_absolute_velocity_in_parent().at(Y), v+omega*a,4.0e-16);
    EXPECT_DOUBLE_EQ(tire_fl.get_frame().get_absolute_velocity_in_parent().at(Z), vz);
}


TEST_P(Axle_car_test, rear_left_tire_velocity)
{
    const Tire_t& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const scalar& a             = rear_axle_parameters.at("position");
    const scalar& t             = rear_axle_parameters.at("track");

    const scalar vz = dz_rear + rear_axle.get_chassis_deformation_velocity(Rear_axle_t::LEFT)
                            - 0.5*dphi*t ;

    EXPECT_DOUBLE_EQ(tire_rl.get_frame().get_absolute_velocity_in_parent().at(X), u+0.5*omega*t);
    EXPECT_NEAR     (tire_rl.get_frame().get_absolute_velocity_in_parent().at(Y), v-omega*a,2.0e-16);
    EXPECT_DOUBLE_EQ(tire_rl.get_frame().get_absolute_velocity_in_parent().at(Z), vz);
}


TEST_P(Axle_car_test, front_right_tire_velocity)
{
    const Tire_t& tire_fr = front_axle.get_tire<Front_axle_t::RIGHT>();
    const scalar& a = front_axle_parameters.at("position");
    const scalar& t = front_axle_parameters.at("track");

    const scalar vz = dz_front + front_axle.get_chassis_deformation_velocity(Front_axle_t::RIGHT)
                            + 0.5*dphi*t ;

    EXPECT_DOUBLE_EQ(tire_fr.get_frame().get_absolute_velocity_in_parent().at(X), u-0.5*omega*t);
    EXPECT_NEAR     (tire_fr.get_frame().get_absolute_velocity_in_parent().at(Y), v+omega*a,2.0e-16);
    EXPECT_DOUBLE_EQ(tire_fr.get_frame().get_absolute_velocity_in_parent().at(Z), vz);
}


TEST_P(Axle_car_test, rear_right_tire_velocity)
{
    const Tire_t& tire_rr = rear_axle.get_tire<Rear_axle_t::RIGHT>();
    const scalar& a = rear_axle_parameters.at("position");
    const scalar& t = rear_axle_parameters.at("track");

    const scalar vz = dz_rear + rear_axle.get_chassis_deformation_velocity(Rear_axle_t::RIGHT)
                            + 0.5*dphi*t ;

    EXPECT_DOUBLE_EQ(tire_rr.get_frame().get_absolute_velocity_in_parent().at(X), u-0.5*omega*t);
    EXPECT_NEAR     (tire_rr.get_frame().get_absolute_velocity_in_parent().at(Y), v-omega*a,5.0e-16);
    EXPECT_DOUBLE_EQ(tire_rr.get_frame().get_absolute_velocity_in_parent().at(Z), vz);
}


TEST_P(Axle_car_test, front_chassis_deformations)
{
    const Tire_t& tire_fl = front_axle.get_tire<Front_axle_t::LEFT>();
    const Tire_t& tire_fr = front_axle.get_tire<Front_axle_t::RIGHT>();
    const scalar& R0            = tire_fl.get_radius();

    const scalar& wl = front_axle.get_tire<Front_axle_t::LEFT>().get_vertical_deformation();
    const scalar& wr = front_axle.get_tire<Front_axle_t::RIGHT>().get_vertical_deformation();

    const scalar& sl = front_axle.get_chassis_deformation(Front_axle_t::LEFT);
    const scalar& sr = front_axle.get_chassis_deformation(Front_axle_t::RIGHT);

    const scalar& dwl = front_axle.get_tire<Front_axle_t::LEFT>().get_vertical_deformation_velocity();
    const scalar& dwr = front_axle.get_tire<Front_axle_t::RIGHT>().get_vertical_deformation_velocity();

    const scalar& dsl = front_axle.get_chassis_deformation_velocity(Front_axle_t::LEFT);
    const scalar& dsr = front_axle.get_chassis_deformation_velocity(Front_axle_t::RIGHT);

    // Tire deformations are a result of solving the system:
    //         wl = zsym - zasym + sl
    //         wr = zsym + zasym + sr
    //         k_chassis sl + k_antiroll (sl - sr) + k_tire wl = 0
    //         k_chassis sr + k_antiroll (sr - sl) + k_tire wr = 0
    const scalar zaxle  = z_front + R0;
    const scalar dzaxle = dz_front;

    const scalar zsym = zaxle;
    const scalar zasym = 0.5*front_axle_parameters.at("track")*phi + front_axle_parameters.at("beta_steering")*delta;

    const scalar dzsym = dzaxle;
    const scalar dzasym = 0.5*front_axle_parameters.at("track")*dphi ;

    const scalar& kchassis  = front_axle_parameters.at("chassis_stiffness");
    const scalar& kantiroll = front_axle_parameters.at("antiroll_stiffness");
    const scalar  ktire     = 64.5e3;
    const scalar  ctire     = 1.0e3;

    EXPECT_DOUBLE_EQ(wl - zsym + zasym - sl, 0.0);
    EXPECT_NEAR(wr - zsym - zasym - sr, 0.0, 1.0e-16);
    EXPECT_NEAR(kchassis*sl + kantiroll*(sl-sr) + ktire*wl, 0.0, ktire*1.0e-16);
    EXPECT_NEAR(kchassis*sr + kantiroll*(sr-sl) + ktire*wr, 0.0, ktire*1.0e-16);

    EXPECT_DOUBLE_EQ(dwl - dzsym + dzasym - dsl, 0.0);
    EXPECT_DOUBLE_EQ(dwr - dzsym - dzasym - dsr, 0.0);
    EXPECT_NEAR     (kchassis*dsl + kantiroll*(dsl-dsr) + ktire*dwl, 0.0, ktire*1.0e-16);
    EXPECT_NEAR     (kchassis*dsr + kantiroll*(dsr-dsl) + ktire*dwr, 0.0, ktire*1.0e-16);

    // The tire normal force must be kt w + ct dw
    EXPECT_DOUBLE_EQ(tire_fl.get_force().at(Z), -std::max(0.0,ktire*wl + ctire*dwl));
    EXPECT_DOUBLE_EQ(tire_fr.get_force().at(Z), -std::max(0.0,ktire*wr + ctire*dwr));
}


TEST_P(Axle_car_test, rear_chassis_deformations)
{
    const Tire_t& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();

    const scalar& R0 = tire_rl.get_radius();

    const scalar& wl = rear_axle.get_tire<Rear_axle_t::LEFT>().get_vertical_deformation();
    const scalar& wr = rear_axle.get_tire<Rear_axle_t::RIGHT>().get_vertical_deformation();

    const scalar& sl = rear_axle.get_chassis_deformation(Rear_axle_t::LEFT);
    const scalar& sr = rear_axle.get_chassis_deformation(Rear_axle_t::RIGHT);

    const scalar& dwl = rear_axle.get_tire<Rear_axle_t::LEFT>().get_vertical_deformation_velocity();
    const scalar& dwr = rear_axle.get_tire<Rear_axle_t::RIGHT>().get_vertical_deformation_velocity();

    const scalar& dsl = rear_axle.get_chassis_deformation_velocity(Rear_axle_t::LEFT);
    const scalar& dsr = rear_axle.get_chassis_deformation_velocity(Rear_axle_t::RIGHT);

    // Tire deformations are a result of solving the system:
    //         wl = zsym - zasym + sl
    //         wr = zsym + zasym + sr
    //         k_chassis sl + k_antiroll (sl - sr) + k_tire wl = 0
    //         k_chassis sr + k_antiroll (sr - sl) + k_tire wr = 0
    const scalar zsym = z_rear + R0;
    const scalar zasym = 0.5*rear_axle_parameters.at("track")*phi ;

    const scalar dzsym = dz_rear;
    const scalar dzasym = 0.5*rear_axle_parameters.at("track")*dphi ;

    const scalar& kchassis  = rear_axle_parameters.at("chassis_stiffness");
    const scalar& kantiroll = rear_axle_parameters.at("antiroll_stiffness");
    const scalar  ktire     = 61.3e3;

    EXPECT_NEAR(wl - zsym + zasym - sl, 0.0, ktire*1.0e-16);
    EXPECT_NEAR(wr - zsym - zasym - sr, 0.0, ktire*1.0e-16);
    EXPECT_NEAR(kchassis*sl + kantiroll*(sl-sr) + ktire*wl, 0.0, ktire*1.0e-16);
    EXPECT_NEAR(kchassis*sr + kantiroll*(sr-sl) + ktire*wr, 0.0, ktire*1.0e-16);

    EXPECT_DOUBLE_EQ(dwl - dzsym + dzasym - dsl, 0.0);
    EXPECT_DOUBLE_EQ(dwr - dzsym - dzasym - dsr, 0.0);
    EXPECT_NEAR     (kchassis*dsl + kantiroll*(dsl-dsr) + ktire*dwl, 0.0, ktire*1.0e-16);
    EXPECT_NEAR     (kchassis*dsr + kantiroll*(dsr-dsl) + ktire*dwr, 0.0, ktire*1.0e-16);
}


TEST_P(Axle_car_test, rear_tires_normal_load)
{
    const Tire_t& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const Tire_t& tire_rr = rear_axle.get_tire<Rear_axle_t::RIGHT>();
    const scalar  ktire   = 61.3e3;                                   
    const scalar  ctire   = 1.0e3;                                       

    const scalar& wl = rear_axle.get_tire<Rear_axle_t::LEFT>().get_vertical_deformation();
    const scalar& wr = rear_axle.get_tire<Rear_axle_t::RIGHT>().get_vertical_deformation();
    const scalar& dwl = rear_axle.get_tire<Rear_axle_t::LEFT>().get_vertical_deformation_velocity();
    const scalar& dwr = rear_axle.get_tire<Rear_axle_t::RIGHT>().get_vertical_deformation_velocity();

    // The tire normal force must be kt w + ct dw
    EXPECT_DOUBLE_EQ(tire_rl.get_force().at(Z), -std::max(0.0,ktire*wl + ctire*dwl));
    EXPECT_DOUBLE_EQ(tire_rr.get_force().at(Z), -std::max(0.0,ktire*wr + ctire*dwr));
}


TEST_P(Axle_car_test, rear_axle_kappa_lambda)
{
    const Tire_t& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const Tire_t& tire_rr = rear_axle.get_tire<Rear_axle_t::RIGHT>();

    const scalar& dwl = rear_axle.get_tire<Rear_axle_t::LEFT>().get_vertical_deformation_velocity();
    const scalar& dwr = rear_axle.get_tire<Rear_axle_t::RIGHT>().get_vertical_deformation_velocity();

    // Check kappa and lambda
    const sVector3d vl = { u + omega*0.5*rear_axle_parameters.at("track"), v - omega*rear_axle_parameters.at("position"), dwl}; 
    const sVector3d vr = { u - omega*0.5*rear_axle_parameters.at("track"), v - omega*rear_axle_parameters.at("position"), dwr}; 

    for (size_t i = 0; i < 3; ++i)
    {
        EXPECT_NEAR     (tire_rl.get_velocity().at(i), vl.at(i), 5.0e-16);
        EXPECT_NEAR     (tire_rr.get_velocity().at(i), vr.at(i), 5.0e-16);
    }

    EXPECT_NEAR     (tire_rl.get_kappa(), omega_axle*0.139/vl.at(X)-1.0,5.0e-16);
    EXPECT_DOUBLE_EQ(tire_rr.get_kappa(), omega_axle*0.139/vr.at(X)-1.0);

    EXPECT_NEAR     (tire_rl.get_lambda(), -vl.at(Y)/vl.at(X), 3.0e-16);
    EXPECT_NEAR     (tire_rr.get_lambda(), -vr.at(Y)/vr.at(X), 3.0e-16);
}

TEST_P(Axle_car_test, front_axle_magic_forces)
{
    const Tire_t& tire_fl = front_axle.get_tire<Front_axle_t::LEFT>();
    const Tire_t& tire_fr = front_axle.get_tire<Front_axle_t::RIGHT>();
    
    // Check the forces generated
    EXPECT_DOUBLE_EQ(tire_fl.get_force().at(X), 0.0);
    EXPECT_DOUBLE_EQ(tire_fr.get_force().at(X), 0.0);

    const scalar Fl = tire_fl.get_model().force_pure_lateral_magic(tire_fl.get_lambda(), 
                                                                  -tire_fl.get_force().at(Z));

    const scalar Fr = tire_fr.get_model().force_pure_lateral_magic(tire_fr.get_lambda(), 
                                                                  -tire_fr.get_force().at(Z));

    EXPECT_DOUBLE_EQ(tire_fl.get_force().at(Y), Fl);
    EXPECT_DOUBLE_EQ(tire_fr.get_force().at(Y), Fr);
}



TEST_P(Axle_car_test, rear_axle_magic_forces)
{
    const Tire_t& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const Tire_t& tire_rr = rear_axle.get_tire<Rear_axle_t::RIGHT>();
    
    // Check the forces generated
    const scalar Sl = tire_rl.get_model().force_combined_longitudinal_magic(tire_rl.get_kappa(), 
                                                                    tire_rl.get_lambda(), 
                                                                    -tire_rl.get_force().at(Z));

    const scalar Sr = tire_rr.get_model().force_combined_longitudinal_magic(tire_rr.get_kappa(), 
                                                                    tire_rr.get_lambda(), 
                                                                    -tire_rr.get_force().at(Z));

    EXPECT_DOUBLE_EQ(tire_rl.get_force().at(X), Sl);
    EXPECT_DOUBLE_EQ(tire_rr.get_force().at(X), Sr);

    const scalar Fl = tire_rl.get_model().force_combined_lateral_magic(tire_rl.get_kappa(), 
                                                               tire_rl.get_lambda(), 
                                                               -tire_rl.get_force().at(Z));

    const scalar Fr = tire_rr.get_model().force_combined_lateral_magic(tire_rr.get_kappa(), 
                                                               tire_rr.get_lambda(), 
                                                               -tire_rr.get_force().at(Z));

    EXPECT_DOUBLE_EQ(tire_rl.get_force().at(Y), Fl);
    EXPECT_DOUBLE_EQ(tire_rr.get_force().at(Y), Fr);
}

TEST_P(Axle_car_test, rear_axle_torque_equation)
{
    const Tire_t& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const Tire_t& tire_rr = rear_axle.get_tire<Rear_axle_t::RIGHT>();

    const scalar& wl = rear_axle.get_tire<Rear_axle_t::LEFT>().get_vertical_deformation();
    const scalar& wr = rear_axle.get_tire<Rear_axle_t::RIGHT>().get_vertical_deformation();

    const scalar& Sl = tire_rl.get_model().force_combined_longitudinal_magic(tire_rl.get_kappa(), 
                                                                     tire_rl.get_lambda(), 
                                                                     -tire_rl.get_force().at(Z));

    const scalar& Sr = tire_rr.get_model().force_combined_longitudinal_magic(tire_rr.get_kappa(), 
                                                                     tire_rr.get_lambda(), 
                                                                     -tire_rr.get_force().at(Z));


    // Check the momentum equation at the wheel axle
    EXPECT_DOUBLE_EQ(rear_axle.get_omega_derivative()*rear_axle_parameters.at("inertia"), 
                     T - Sr*(0.139-wr) - Sl*(0.139-wl) );
}

