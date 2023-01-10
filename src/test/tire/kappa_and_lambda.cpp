#include "gtest/gtest.h"
#include "lion/frame/frame.h"
#include "src/core/tire/tire.h"
#include "src/core/tire/tire_pacejka.h"
#include <fstream>


using Tire_t = Tire<scalar,0,0>;
using Tire_pacejka_std_t = Tire_pacejka_std<scalar,0,0>;

TEST(Tire_test, base_lambda_and_kappa)
{
    sFrame inertial_frame;

    constexpr const scalar x = 2.0;
    constexpr const scalar y = 4.0;

    constexpr const scalar psi = pi/4.0;
    constexpr const scalar omega = 0.1;

    constexpr const scalar u = 0.95;
    constexpr const scalar v = 0.1;

    sFrame chassis_frame({x,y,0.0}, {u,v,0.0}, {psi}, {omega}, {Z}, inertial_frame, sFrame::Frame_velocity_types::this_frame);

    constexpr const scalar a = 0.645;
    constexpr const scalar b = 0.4;
    constexpr const scalar tr = 1.2/2.0;
    constexpr const scalar tf = 1.055/2.0;

    constexpr const scalar delta = -pi/10.0;

    constexpr const scalar R(0.139);

    sFrame tire_rr_frame({-b,tr,-R},{0.0,0.0,0.0},{}, {}, {}, chassis_frame, sFrame::Frame_velocity_types::parent_frame);
    sFrame tire_rl_frame({-b,-tr,-R},{0.0,0.0,0.0},{}, {}, {}, chassis_frame, sFrame::Frame_velocity_types::parent_frame);
    sFrame tire_fr_frame({+a,tf,-R},{0.0,0.0,0.0},{delta}, {0.0}, {Z}, chassis_frame, sFrame::Frame_velocity_types::parent_frame);
    sFrame tire_fl_frame({+a,-tf,-R},{0.0,0.0,0.0},{delta}, {0.0}, {Z}, chassis_frame, sFrame::Frame_velocity_types::parent_frame);

    Xml_document database = { "database/vehicles/kart/roberto-lot-kart-2016.xml", true };
    Tire_t tire_rr("Rear right",database,"vehicle/rear-tire/");
    Tire_t tire_rl("Rear left",database,"vehicle/rear-tire/");
    Tire_t tire_fr("Front right",database,"vehicle/front-tire/");
    Tire_t tire_fl("Front left",database,"vehicle/front-tire/");

    tire_fl.get_frame().set_parent(tire_fl_frame);
    tire_fr.get_frame().set_parent(tire_fr_frame);
    tire_rl.get_frame().set_parent(tire_rl_frame);
    tire_rr.get_frame().set_parent(tire_rr_frame);

    constexpr const scalar omega_rr = 0.4;
    constexpr const scalar omega_rl = 0.3;

    tire_rr.update(omega_rr);
    tire_rl.update(omega_rl);
    tire_fr.update(0.0);
    tire_fl.update(0.0);

    constexpr const scalar kappa_rr(omega_rr*R/(u-omega*tr)-1.0);
    constexpr const scalar kappa_rl(omega_rl*R/(u+omega*tr)-1.0);

    constexpr const scalar lambda_rr(-(v-omega*b)/(u-omega*tr));
    constexpr const scalar lambda_rl(-(v-omega*b)/(u+omega*tr));
    const scalar lambda_fr(tan(delta-atan2(v+omega*a,u-omega*tf)));
    const scalar lambda_fl(tan(delta-atan2(v+omega*a,u+omega*tf)));

    EXPECT_DOUBLE_EQ( tire_rr.get_kappa(), kappa_rr);
    EXPECT_DOUBLE_EQ( tire_rl.get_kappa(), kappa_rl);

    EXPECT_NEAR      ( tire_rr.get_lambda(), lambda_rr, 1.0e-16);
    EXPECT_NEAR      ( tire_rl.get_lambda(), lambda_rl, 1.0e-16);
    EXPECT_DOUBLE_EQ ( tire_fr.get_lambda(), lambda_fr);
    EXPECT_DOUBLE_EQ ( tire_fl.get_lambda(), lambda_fl);
}


TEST(Tire_test, pacejka_lambda_and_kappa)
{
    sFrame inertial_frame;

    constexpr const scalar x = 2.0;
    constexpr const scalar y = 4.0;

    constexpr const scalar psi = pi/4.0;
    constexpr const scalar omega = 0.1;

    constexpr const scalar u = 0.95;
    constexpr const scalar v = 0.1;

    const scalar dx = u*cos(psi) - v*sin(psi);
    const scalar dy = u*sin(psi) + v*cos(psi);
    
    constexpr const scalar R(0.139);

    sFrame chassis_frame({x,y,0.0}, {dx,dy,0.0}, {psi}, {omega}, {Z}, inertial_frame, sFrame::Frame_velocity_types::parent_frame);

    constexpr const scalar a = 0.645;
    constexpr const scalar b = 0.4;
    constexpr const scalar tr = 1.2/2.0;
    constexpr const scalar tf = 1.055/2.0;

    constexpr const scalar delta = -pi/10.0;

    sFrame tire_rr_frame({-b,tr,-R},{0.0,0.0,0.0},{}, {}, {}, chassis_frame, sFrame::Frame_velocity_types::parent_frame);
    sFrame tire_rl_frame({-b,-tr,-R},{0.0,0.0,0.0},{}, {}, {}, chassis_frame, sFrame::Frame_velocity_types::parent_frame);
    sFrame tire_fr_frame({+a,tf,-R},{0.0,0.0,0.0},{delta}, {0.0}, {Z}, chassis_frame, sFrame::Frame_velocity_types::parent_frame);
    sFrame tire_fl_frame({+a,-tf,-R},{0.0,0.0,0.0},{delta}, {0.0}, {Z}, chassis_frame, sFrame::Frame_velocity_types::parent_frame);

    Xml_document database = { "database/vehicles/kart/roberto-lot-kart-2016.xml", true };
    Tire_pacejka_std_t tire_rr("Rear right",database,"vehicle/rear-tire/");
    Tire_pacejka_std_t tire_rl("Rear left",database,"vehicle/rear-tire/");
    Tire_pacejka_std_t tire_fr("Front right",database,"vehicle/front-tire/");
    Tire_pacejka_std_t tire_fl("Front left",database,"vehicle/front-tire/");

    tire_fl.get_frame().set_parent(tire_fl_frame);
    tire_fr.get_frame().set_parent(tire_fr_frame);
    tire_rl.get_frame().set_parent(tire_rl_frame);
    tire_rr.get_frame().set_parent(tire_rr_frame);

    constexpr const scalar omega_rr = 0.4;
    constexpr const scalar omega_rl = 0.3;

    tire_rr.update(omega_rr);
    tire_rl.update(omega_rl);
    tire_fr.update(0.0);
    tire_fl.update(0.0);

    constexpr const scalar kappa_rr(omega_rr*R/(u-omega*tr)-1.0);
    constexpr const scalar kappa_rl(omega_rl*R/(u+omega*tr)-1.0);

    constexpr const scalar lambda_rr(-(v-omega*b)/(u-omega*tr));
    constexpr const scalar lambda_rl(-(v-omega*b)/(u+omega*tr));
    const scalar lambda_fr(tan(delta-atan2(v+omega*a,u-omega*tf)));
    const scalar lambda_fl(tan(delta-atan2(v+omega*a,u+omega*tf)));

    EXPECT_DOUBLE_EQ( tire_rr.get_kappa(), kappa_rr);
    EXPECT_DOUBLE_EQ( tire_rl.get_kappa(), kappa_rl);

    EXPECT_NEAR      ( tire_rr.get_lambda(), lambda_rr, 1.0e-16);
    EXPECT_NEAR      ( tire_rl.get_lambda(), lambda_rl, 1.0e-16);
    EXPECT_DOUBLE_EQ ( tire_fr.get_lambda(), lambda_fr);
    EXPECT_DOUBLE_EQ ( tire_fl.get_lambda(), lambda_fl);
}
