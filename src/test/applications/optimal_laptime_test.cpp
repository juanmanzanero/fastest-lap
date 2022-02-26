#include "gtest/gtest.h"
#include "lion/math/matrix_extensions.h"
#include "src/core/applications/optimal_laptime.h"
#include "src/core/vehicles/lot2016kart.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "src/core/applications/steady_state.h"

extern bool is_valgrind;

class Optimal_laptime_test : public ::testing::Test
{
 protected:
    Xml_document database = {"./database/roberto-lot-kart-2016.xml", true};
    Xml_document results  = {"./data/optimal_laptime.xml", true};
    lot2016kart<CppAD::AD<scalar>>::cartesian car_cartesian = { database };
    lot2016kart<scalar>::cartesian car_cartesian_scalar = { database };
};


TEST_F(Optimal_laptime_test, maximum_acceleration)
{
    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document straight_xml("./data/straight.xml",true);
    Track_by_arcs straight(straight_xml,false);

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, {straight});
    car.get_chassis().get_rear_axle().enable_direct_torque();

    constexpr const size_t n = 30;

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dqdt[0]*0.2;
   
    ss.u[1] = T;
    
    ss.dqdt = car_cartesian_scalar(ss.q, ss.u, 0.0);

    Optimal_laptime opt_laptime(n, false, true, car, ss.q, ss.qa, ss.u, {0.0, 4.0e-5});


    // Check that the car accelerates
    constexpr const size_t IU = lot2016kart<scalar>::curvilinear<Track_by_arcs>::Chassis_type::IU;
    for (size_t i = 1; i < n; ++i)
    {
        EXPECT_TRUE(opt_laptime.q.at(i).at(IU) > opt_laptime.q.at(i-1).at(IU) );
    }

    // Check that kappa is between [0.090,0.096] from the third onwards
    for (size_t i = 4; i < n; ++i)
    {
        car_cartesian_scalar(opt_laptime.q.at(i), opt_laptime.u.at(i),0.0);
        const auto& kappa_left  = car_cartesian_scalar.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = car_cartesian_scalar.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        EXPECT_TRUE( (kappa_left > 0.090) && (kappa_left < 0.096) ) << ", with i = " << i;
        EXPECT_TRUE( (kappa_right > 0.090) && (kappa_right < 0.096) ) << ", with i = " << i;
    }

    // Check that steering is zero
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(Value(opt_laptime.u.at(i)[lot2016kart<scalar>::curvilinear<Track_by_arcs>::Chassis_type::front_axle_type::ISTEERING]), 0.0, 1.0e-14);

    std::vector<scalar> T_saved = results.get_root_element().get_child("maximum_acceleration/T").get_value(std::vector<scalar>());

    EXPECT_EQ(opt_laptime.u.size(), T_saved.size());

    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][1], T_saved[i],1.0e-2);
}

TEST_F(Optimal_laptime_test, Ovaltrack_open)
{
    if ( is_valgrind ) GTEST_SKIP();

    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document ovaltrack_xml("./database/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,0.2,true);
    
    constexpr const size_t n = 400;

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dqdt[0]*0.2;
   
    ss.u[1] = T;
    
    ss.dqdt = car_cartesian_scalar(ss.q, ss.u, 0.0);

    Optimal_laptime opt_laptime(n, false, true, car, ss.q, ss.qa, ss.u, {1.0e2,2.0e-7});

    std::vector<scalar> delta_saved = results.get_root_element().get_child("ovaltrack_open/delta").get_value(std::vector<scalar>());
    std::vector<scalar> T_saved = results.get_root_element().get_child("ovaltrack_open/T").get_value(std::vector<scalar>());

    EXPECT_EQ(opt_laptime.u.size(), delta_saved.size());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.u[i][0], delta_saved[i],5.0e-7);
        EXPECT_NEAR(opt_laptime.u[i][1], T_saved[i],1.0e-2);
    }

}


TEST_F(Optimal_laptime_test, Ovaltrack_closed)
{
    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document ovaltrack_xml("./database/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,0.2,true);
    
    constexpr const size_t n = 100;

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dqdt[0]*0.2;
   
    ss.u[1] = T;
    
    ss.dqdt = car_cartesian_scalar(ss.q, ss.u, 0.0);

    Optimal_laptime opt_laptime(n, true, true, car, ss.q, ss.qa, ss.u, {1.0e2,2.0e-7});

    std::vector<scalar> delta_saved = results.get_root_element().get_child("ovaltrack_closed/delta").get_value(std::vector<scalar>());
    std::vector<scalar> T_saved = results.get_root_element().get_child("ovaltrack_closed/T").get_value(std::vector<scalar>());

    EXPECT_EQ(opt_laptime.u.size(), delta_saved.size());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.u[i][0], delta_saved[i],5.0e-7);
        EXPECT_NEAR(opt_laptime.u[i][1], T_saved[i],1.0e-2);
    }

}

TEST_F(Optimal_laptime_test, Ovaltrack_derivative)
{
    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document ovaltrack_xml("./database/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,0.2,true);
    
    constexpr const size_t n = 100;

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dqdt[0]*0.2;
   
    ss.u[1] = T;
    
    ss.dqdt = car_cartesian_scalar(ss.q, ss.u, 0.0);

    Optimal_laptime opt_laptime(n, true, false, car, ss.q, ss.qa, ss.u, {1.0e-3,2.0e-9});

    // Check the results with a saved simulation
    Xml_document opt_saved("data/ovaltrack_derivative.xml", true);

    // omega axle
    auto omega_axle_saved = opt_saved.get_element("optimal_laptime/axle-omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Rear_axle_t::IOMEGA_AXLE], omega_axle_saved[i], 5.0e-5);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // z
    auto z_saved = opt_saved.get_element("optimal_laptime/z").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IZ], z_saved[i], 1.0e-6);

    // phi
    auto phi_saved = opt_saved.get_element("optimal_laptime/phi").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IPHI], phi_saved[i], 1.0e-6);

    // mu
    auto mu_saved = opt_saved.get_element("optimal_laptime/mu").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IMU], mu_saved[i], 1.0e-6);

    // dzdt
    auto dzdt_saved = opt_saved.get_element("optimal_laptime/dzdt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDZ], dzdt_saved[i], 1.0e-6);

    // dphidt
    auto dphidt_saved = opt_saved.get_element("optimal_laptime/dphidt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDPHI], dphidt_saved[i], 1.0e-6);

    // dmudt
    auto dmudt_saved = opt_saved.get_element("optimal_laptime/dmudt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDMU], dmudt_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][lot2016kart<scalar>::Front_axle_t::ISTEERING], delta_saved[i], 1.0e-6);

    // torque
    auto torque_saved = opt_saved.get_element("optimal_laptime/torque").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][lot2016kart<scalar>::Rear_axle_t::ITORQUE], torque_saved[i], 5.0e-5);
}


TEST_F(Optimal_laptime_test, Catalunya_direct)
{
    if ( is_valgrind ) GTEST_SKIP();

    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document catalunya_xml("./database/catalunya.xml",true);
    Track_by_arcs catalunya(catalunya_xml,0.2,true);
    
    constexpr const size_t n = 500;

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(catalunya);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dqdt[0]*0.2;
   
    ss.u[1] = T;

    ss.dqdt = car_cartesian_scalar(ss.q, ss.u, 0.0);

    Optimal_laptime opt_laptime(n, true, true, car, ss.q, ss.qa, ss.u, {5.0e0,8.0e-6});

    std::vector<scalar> delta_saved = results.get_root_element().get_child("catalunya/delta").get_value(std::vector<scalar>());
    std::vector<scalar> T_saved = results.get_root_element().get_child("catalunya/T").get_value(std::vector<scalar>());

    EXPECT_EQ(opt_laptime.u.size(), delta_saved.size());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.u[i][0], delta_saved[i],5.0e-7);
        EXPECT_NEAR(opt_laptime.u[i][1], T_saved[i],1.0e-2);
    }

}


TEST_F(Optimal_laptime_test, Catalunya_derivative)
{
    #ifndef NDEBUG
        GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document catalunya_xml("./database/catalunya.xml",true);
    Track_by_arcs catalunya(catalunya_xml,0.2,true);
    
    constexpr const size_t n = 500;

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(catalunya);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dqdt[0]*0.2;
   
    ss.u[1] = T;

    ss.dqdt = car_cartesian_scalar(ss.q, ss.u, 0.0);

    Optimal_laptime opt_laptime(n, true, false, car, ss.q, ss.qa, ss.u, {1.0e-2,1.0e-10});

    // Check the results with a saved simulation
    Xml_document opt_saved("data/catalunya_derivative.xml", true);

    // omega axle
    auto omega_axle_saved = opt_saved.get_element("optimal_laptime/axle-omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Rear_axle_t::IOMEGA_AXLE], omega_axle_saved[i], 5.0e-5);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // z
    auto z_saved = opt_saved.get_element("optimal_laptime/z").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IZ], z_saved[i], 1.0e-6);

    // phi
    auto phi_saved = opt_saved.get_element("optimal_laptime/phi").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IPHI], phi_saved[i], 1.0e-6);

    // mu
    auto mu_saved = opt_saved.get_element("optimal_laptime/mu").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IMU], mu_saved[i], 1.0e-6);

    // dzdt
    auto dzdt_saved = opt_saved.get_element("optimal_laptime/dzdt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDZ], dzdt_saved[i], 1.0e-6);

    // dphidt
    auto dphidt_saved = opt_saved.get_element("optimal_laptime/dphidt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDPHI], dphidt_saved[i], 1.0e-6);

    // dmudt
    auto dmudt_saved = opt_saved.get_element("optimal_laptime/dmudt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDMU], dmudt_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][lot2016kart<scalar>::Front_axle_t::ISTEERING], delta_saved[i], 1.0e-6);

    // torque
    auto torque_saved = opt_saved.get_element("optimal_laptime/torque").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][lot2016kart<scalar>::Rear_axle_t::ITORQUE], torque_saved[i], 5.0e-5);
}

TEST_F(Optimal_laptime_test, Catalunya_derivative_throttle)
{
    #ifndef NDEBUG
        GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document catalunya_xml("./database/catalunya.xml",true);
    Track_by_arcs catalunya(catalunya_xml,0.2,true);
    
    constexpr const size_t n = 500;

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(catalunya);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = 0.0;
   
    ss.u[1] = T;

    ss.dqdt = car_cartesian_scalar(ss.q, ss.u, 0.0);

    Optimal_laptime opt_laptime(n, true, false, car, ss.q, ss.qa, ss.u, {1.0e-2,200*200.0*1.0e-10});

    // Check the results with a saved simulation
    Xml_document opt_saved("data/catalunya_derivative_throttle.xml", true);

    // omega axle
    auto omega_axle_saved = opt_saved.get_element("optimal_laptime/axle-omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Rear_axle_t::IOMEGA_AXLE], omega_axle_saved[i], 5.0e-5);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // z
    auto z_saved = opt_saved.get_element("optimal_laptime/z").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IZ], z_saved[i], 1.0e-6);

    // phi
    auto phi_saved = opt_saved.get_element("optimal_laptime/phi").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IPHI], phi_saved[i], 1.0e-6);

    // mu
    auto mu_saved = opt_saved.get_element("optimal_laptime/mu").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IMU], mu_saved[i], 1.0e-6);

    // dzdt
    auto dzdt_saved = opt_saved.get_element("optimal_laptime/dzdt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDZ], dzdt_saved[i], 1.0e-6);

    // dphidt
    auto dphidt_saved = opt_saved.get_element("optimal_laptime/dphidt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDPHI], dphidt_saved[i], 1.0e-6);

    // dmudt
    auto dmudt_saved = opt_saved.get_element("optimal_laptime/dmudt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDMU], dmudt_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][lot2016kart<scalar>::Front_axle_t::ISTEERING], delta_saved[i], 1.0e-6);

    // torque
    auto torque_saved = opt_saved.get_element("optimal_laptime/torque").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][lot2016kart<scalar>::Rear_axle_t::ITORQUE], torque_saved[i], 5.0e-5);
}


TEST_F(Optimal_laptime_test, Vendrell)
{
    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document vendrell_xml("./database/vendrell.xml",true);
    Circuit_preprocessor vendrell_pproc(vendrell_xml);
    Track_by_polynomial vendrell(vendrell_pproc);
    
    constexpr const size_t n = 500;

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(vendrell);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 30.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = 0.0;
   
    ss.u[1] = T;

    ss.dqdt = car_cartesian_scalar(ss.q, ss.u, 0.0);

    Optimal_laptime opt_laptime(n, true, false, car, ss.q, ss.qa, ss.u, {1.0e-2,200*200.0*1.0e-10});

    opt_laptime.xml()->save("vendrell_optimal.xml");

    // Check the results with a saved simulation
    Xml_document opt_saved("data/vendrell_optimal.xml", true);

    // omega axle
    auto omega_axle_saved = opt_saved.get_element("optimal_laptime/axle-omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Rear_axle_t::IOMEGA_AXLE], omega_axle_saved[i], 5.0e-5);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // z
    auto z_saved = opt_saved.get_element("optimal_laptime/z").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IZ], z_saved[i], 1.0e-6);

    // phi
    auto phi_saved = opt_saved.get_element("optimal_laptime/phi").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IPHI], phi_saved[i], 1.0e-6);

    // mu
    auto mu_saved = opt_saved.get_element("optimal_laptime/mu").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IMU], mu_saved[i], 1.0e-6);

    // dzdt
    auto dzdt_saved = opt_saved.get_element("optimal_laptime/dzdt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDZ], dzdt_saved[i], 1.0e-6);

    // dphidt
    auto dphidt_saved = opt_saved.get_element("optimal_laptime/dphidt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDPHI], dphidt_saved[i], 1.0e-6);

    // dmudt
    auto dmudt_saved = opt_saved.get_element("optimal_laptime/dmudt").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::Chassis_t::IDMU], dmudt_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][lot2016kart<scalar>::curvilinear_a::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][lot2016kart<scalar>::Front_axle_t::ISTEERING], delta_saved[i], 1.0e-6);

    // torque
    auto torque_saved = opt_saved.get_element("optimal_laptime/torque").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][lot2016kart<scalar>::Rear_axle_t::ITORQUE], torque_saved[i], 5.0e-5);

}


