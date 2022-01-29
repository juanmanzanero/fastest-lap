#include "gtest/gtest.h"
#include "lion/math/matrix_extensions.h"
#include "src/core/applications/optimal_laptime.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "src/core/applications/steady_state.h"

extern bool is_valgrind;

class F1_optimal_laptime_test : public ::testing::Test
{
 protected:
    Xml_document database = {"./database/limebeer-2014-f1.xml", true};
    Xml_document results  = {"./data/optimal_laptime.xml", true};
    limebeer2014f1<CppAD::AD<scalar>>::cartesian car_cartesian = { database };
    limebeer2014f1<scalar>::cartesian car_cartesian_scalar = { database };
};

TEST_F(F1_optimal_laptime_test, Catalunya_polyseg_direct)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/catalunya-polyseg.xml",true);
    Track_by_polynomial catalunya(catalunya_xml);
    
    constexpr const size_t n = 500;

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    Optimal_laptime opt_laptime(n, true, true, car, ss.q, ss.qa, ss.u, {5.0e0,8.0e-4});

    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial> car_scalar(database, {catalunya});
    for (size_t i = 0; i < n; ++i)
    {

        const scalar& L = car_scalar.get_road().track_length();
        car_scalar(opt_laptime.q.at(i), opt_laptime.qa.at(i), opt_laptime.u.at(i),((double)i)*L/((double)n));
        const auto& kappa_left  = car_scalar.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = car_scalar.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        std::cout << opt_laptime.u.at(i)[1] << ", " << kappa_left << ", " << kappa_right << ", " << opt_laptime.u.at(i)[0] << ", " << car_scalar.get_road().get_x() << ", " << car_scalar.get_road().get_y() << std::endl;
    }
}




TEST_F(F1_optimal_laptime_test, maximum_acceleration)
{
    Xml_document straight_xml("./data/straight.xml",true);
    Track_by_arcs straight(straight_xml,10.0,false);

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, {straight});

    constexpr const size_t n = 50;

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 80.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    Optimal_laptime opt_laptime(n, false, true, car, ss.q, ss.qa, ss.u, {0.0, 4.0e-7});

    // Check that the car accelerates
    constexpr const size_t IU = limebeer2014f1<scalar>::curvilinear<Track_by_arcs>::Chassis_type::IU;
    for (size_t i = 1; i < n; ++i)
    {
        EXPECT_TRUE(opt_laptime.q.at(i).at(IU) > opt_laptime.q.at(i-1).at(IU) );
    }

/*
    // Check that kappa is between [0.090,0.096] from the third onwards
    for (size_t i = 4; i < n; ++i)
    {
        car_cartesian_scalar(opt_laptime.q.at(i), opt_laptime.u.at(i),0.0);
        const auto& kappa_left  = car_cartesian_scalar.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = car_cartesian_scalar.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        EXPECT_TRUE( (kappa_left > 0.090) && (kappa_left < 0.096) ) << ", with i = " << i;
        EXPECT_TRUE( (kappa_right > 0.090) && (kappa_right < 0.096) ) << ", with i = " << i;
    }
*/

    // Check that steering is zero
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(Value(opt_laptime.u.at(i)[limebeer2014f1<scalar>::curvilinear<Track_by_arcs>::Chassis_type::front_axle_type::ISTEERING]), 0.0, 1.0e-14);

/*
    std::vector<scalar> T_saved = results.get_root_element().get_child("maximum_acceleration/T").get_value(std::vector<scalar>());

    EXPECT_EQ(opt_laptime.u.size(), T_saved.size());

    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][1], T_saved[i],1.0e-2);
*/



    // Check that steering is zero
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(Value(opt_laptime.u.at(i)[limebeer2014f1<scalar>::curvilinear<Track_by_arcs>::Chassis_type::front_axle_type::ISTEERING]), 0.0, 1.0e-14);

/*
    std::vector<scalar> T_saved = results.get_root_element().get_child("maximum_acceleration/T").get_value(std::vector<scalar>());

    EXPECT_EQ(opt_laptime.u.size(), T_saved.size());

    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.u[i][1], T_saved[i],1.0e-2);
*/

    for (size_t i = 0; i < n; ++i)
    {
        car_cartesian_scalar(opt_laptime.q.at(i), opt_laptime.qa.at(i), opt_laptime.u.at(i),0.0);
        const auto& kappa_left  = car_cartesian_scalar.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = car_cartesian_scalar.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        std::cout << opt_laptime.u.at(i)[1] << ", " << kappa_left << ", " << kappa_right << std::endl;
    }
}

TEST_F(F1_optimal_laptime_test, Ovaltrack_open)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document ovaltrack_xml("./database/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,1.0,true);
    
    constexpr const size_t n = 400;

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 100.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    Optimal_laptime opt_laptime(n, false, true, car, ss.q, ss.qa, ss.u, {1.0e2,2.0e-3});

    limebeer2014f1<scalar>::curvilinear<Track_by_arcs> car_scalar(database, {ovaltrack});
    for (size_t i = 0; i < n; ++i)
    {

        const scalar& L = car_scalar.get_road().track_length();
        car_scalar(opt_laptime.q.at(i), opt_laptime.qa.at(i), opt_laptime.u.at(i),((double)i)*L/((double)n));
        const auto& kappa_left  = car_scalar.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = car_scalar.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        std::cout << opt_laptime.u.at(i)[1] << ", " << kappa_left << ", " << kappa_right << ", " << opt_laptime.u.at(i)[0] << ", " << car_scalar.get_road().get_x() << ", " << car_scalar.get_road().get_y() << std::endl;
    }

}


TEST_F(F1_optimal_laptime_test, Ovaltrack_closed)
{
    Xml_document ovaltrack_xml("./database/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,1.0,true);
    
    constexpr const size_t n = 100;

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    Optimal_laptime opt_laptime(n, true, true, car, ss.q, ss.qa, ss.u, {1.0e2,2.0e-3});

    limebeer2014f1<scalar>::curvilinear<Track_by_arcs> car_scalar(database, {ovaltrack});
    for (size_t i = 0; i < n; ++i)
    {

        const scalar& L = car_scalar.get_road().track_length();
        car_scalar(opt_laptime.q.at(i), opt_laptime.qa.at(i), opt_laptime.u.at(i),((double)i)*L/((double)n));
        const auto& kappa_left  = car_scalar.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = car_scalar.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        std::cout << opt_laptime.u.at(i)[1] << ", " << kappa_left << ", " << kappa_right << ", " << opt_laptime.u.at(i)[0] << ", " << car_scalar.get_road().get_x() << ", " << car_scalar.get_road().get_y() << std::endl;
    }
}

/*
TEST_F(F1_optimal_laptime_test, Ovaltrack_derivative)
{
    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document ovaltrack_xml("./database/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,0.2,true);
    
    constexpr const size_t n = 100;

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack, 2.0);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dqdt[0]*0.2;
   
    ss.u[1] = T;
    
    ss.dqdt = car_cartesian_scalar(ss.q, ss.u, 0.0);

    Optimal_laptime opt_laptime(n, true, false, car, ss.q, ss.qa, ss.u, {1.0e-3,2.0e-9});

    std::vector<scalar> delta_saved = results.get_root_element().get_child("ovaltrack_derivative/delta").get_value(std::vector<scalar>());
    std::vector<scalar> T_saved = results.get_root_element().get_child("ovaltrack_derivative/T").get_value(std::vector<scalar>());

    EXPECT_EQ(opt_laptime.u.size(), delta_saved.size());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.u[i][0], delta_saved[i],5.0e-7);
        EXPECT_NEAR(opt_laptime.u[i][1], T_saved[i],1.0e-2);
    }


}

*/


TEST_F(F1_optimal_laptime_test, Catalunya_direct)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/catalunya.xml",true);
    Track_by_arcs catalunya(catalunya_xml,1.0,true);
    
    constexpr const size_t n = 500;

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    Optimal_laptime opt_laptime(n, true, true, car, ss.q, ss.qa, ss.u, {5.0e0,8.0e-4});

    limebeer2014f1<scalar>::curvilinear<Track_by_arcs> car_scalar(database, {catalunya});
    for (size_t i = 0; i < n; ++i)
    {

        const scalar& L = car_scalar.get_road().track_length();
        car_scalar(opt_laptime.q.at(i), opt_laptime.qa.at(i), opt_laptime.u.at(i),((double)i)*L/((double)n));
        const auto& kappa_left  = car_scalar.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = car_scalar.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        std::cout << opt_laptime.u.at(i)[1] << ", " << kappa_left << ", " << kappa_right << ", " << opt_laptime.u.at(i)[0] << ", " << car_scalar.get_road().get_x() << ", " << car_scalar.get_road().get_y() << std::endl;
    }
}


/*
TEST_F(F1_optimal_laptime_test, Catalunya_derivative)
{
    #ifndef NDEBUG
        GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/catalunya.xml",true);
    Track_by_arcs catalunya(catalunya_xml,1.0,true);
    
    constexpr const size_t n = 500;

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    Optimal_laptime opt_laptime(n, true, false, car, ss.q, ss.qa, ss.u, {1.0e-2,400*400*1.0e-10});

    limebeer2014f1<scalar>::curvilinear<Track_by_arcs> car_scalar(database, {catalunya,10.0});
    for (size_t i = 0; i < n; ++i)
    {

        const scalar& L = car_scalar.get_road().track_length();
        car_scalar(opt_laptime.q.at(i), opt_laptime.qa.at(i), opt_laptime.u.at(i),((double)i)*L/((double)n));
        const auto& kappa_left  = car_scalar.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = car_scalar.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        std::cout << opt_laptime.u.at(i)[1] << ", " << kappa_left << ", " << kappa_right << ", " << opt_laptime.u.at(i)[0] << ", " << car_scalar.get_road().get_x() << ", " << car_scalar.get_road().get_y() << std::endl;
    }
}

TEST_F(F1_optimal_laptime_test, Catalunya_derivative_throttle)
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

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(catalunya, 2.0);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = 0.0;
   
    ss.u[1] = T;

    ss.dqdt = car_cartesian_scalar(ss.q, ss.u, 0.0);

    Optimal_laptime opt_laptime(n, true, false, car, ss.q, ss.qa, ss.u, {1.0e-2,200*200.0*1.0e-10});

    std::vector<scalar> delta_saved = results.get_root_element().get_child("catalunya_derivative_throttle/delta").get_value(std::vector<scalar>());
    std::vector<scalar> T_saved = results.get_root_element().get_child("catalunya_derivative_throttle/T").get_value(std::vector<scalar>());

    EXPECT_EQ(opt_laptime.u.size(), delta_saved.size());

    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.u[i][0], delta_saved[i],6.0e-7);
        EXPECT_NEAR(opt_laptime.u[i][1], T_saved[i],1.0e-2);
    }
}
*/
