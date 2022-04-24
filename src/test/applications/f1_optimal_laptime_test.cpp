#include "gtest/gtest.h"
#include "lion/math/matrix_extensions.h"
#include "src/core/applications/optimal_laptime.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "src/core/applications/steady_state.h"
#include "src/core/applications/circuit_preprocessor.h"

extern bool is_valgrind;

class F1_optimal_laptime_test : public ::testing::Test
{
 protected:
    Xml_document database = {"./database/vehicles/f1/limebeer-2014-f1.xml", true};
    limebeer2014f1<CppAD::AD<scalar>>::cartesian car_cartesian = { database };
    limebeer2014f1<scalar>::cartesian car_cartesian_scalar = { database };
};


TEST_F(F1_optimal_laptime_test, Catalunya_discrete)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_discrete.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = catalunya_pproc.s;
    const auto& n = s.size();
    
    EXPECT_EQ(n, 500);

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]), 5.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::ITHROTTLE]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::ITHROTTLE]), 8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.print_level = 0;
    Optimal_laptime opt_laptime(s, true, true, car, {n,ss.q}, {n,ss.qa}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_discrete.xml", true);

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/throttle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Catalunya_adapted)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = catalunya_pproc.s;

    const size_t n = s.size();

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::ITHROTTLE]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::ITHROTTLE]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 


    Optimal_laptime opt_laptime(s, true, true, car, {n,ss.q}, {n,ss.qa}, control_variables, {});
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_adapted.xml", true);

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/control_variables/delta/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/throttle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);

    // Fz_fl
    auto Fz_fl_saved = opt_saved.get_element("optimal_laptime/Fz_fl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFL], Fz_fl_saved[i], 1.0e-6);

    // Fz_fr
    auto Fz_fr_saved = opt_saved.get_element("optimal_laptime/Fz_fr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFR], Fz_fr_saved[i], 1.0e-6);

    // Fz_rl
    auto Fz_rl_saved = opt_saved.get_element("optimal_laptime/Fz_rl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRL], Fz_rl_saved[i], 1.0e-6);

    // Fz_rr
    auto Fz_rr_saved = opt_saved.get_element("optimal_laptime/Fz_rr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRR], Fz_rr_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Catalunya_variable_parameter)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    sPolynomial variable_engine_power({0.0, 2900.0,3100.0,5000.0},{735.499,735.499,1000.0,1000.0},1,false);
    car.add_variable_parameter("vehicle/rear-axle/engine/maximum-power", variable_engine_power);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = catalunya_pproc.s;

    const size_t n = s.size();

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::ITHROTTLE]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::ITHROTTLE]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.print_level = 0;
    Optimal_laptime opt_laptime(s, true, true, car, {n,ss.q}, {n,ss.qa}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_variable_parameter.xml", true);

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/throttle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);

    // Fz_fl
    auto Fz_fl_saved = opt_saved.get_element("optimal_laptime/Fz_fl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFL], Fz_fl_saved[i], 1.0e-6);

    // Fz_fr
    auto Fz_fr_saved = opt_saved.get_element("optimal_laptime/Fz_fr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFR], Fz_fr_saved[i], 1.0e-6);

    // Fz_rl
    auto Fz_rl_saved = opt_saved.get_element("optimal_laptime/Fz_rl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRL], Fz_rl_saved[i], 1.0e-6);

    // Fz_rr
    auto Fz_rr_saved = opt_saved.get_element("optimal_laptime/Fz_rr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRR], Fz_rr_saved[i], 1.0e-6);
}
  

TEST_F(F1_optimal_laptime_test, Catalunya_chicane)
{
    // The chicane test uses the adapted mesh from i=533 to i=677

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 70.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    // Get arclength: s(i0:i1)
    const size_t i0 = 533;
    const size_t i1 = 677;
    std::vector<scalar> s(i1-i0+1);
    std::copy(catalunya_pproc.s.cbegin()+i0,catalunya_pproc.s.cbegin()+i1+1,s.begin());        

    const size_t n = s.size();

    // Set initial condtion
    std::vector<std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::NSTATE>>       q0(n,ss.q);
    std::vector<std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::NALGEBRAIC>>   qa0(n,ss.qa);
    
    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::ITHROTTLE]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::ITHROTTLE]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    // Set starting condition
    Xml_document opt_full_lap("data/f1_optimal_laptime_catalunya_adapted.xml", true);
    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p> opt_laptime_full_lap(opt_full_lap);

    std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::NSTATE>       q_start(opt_laptime_full_lap.q[i0]);
    std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::NALGEBRAIC>   qa_start(opt_laptime_full_lap.qa[i0]);
    auto u_start(opt_laptime_full_lap.control_variables.control_array_at_s(car,i0,s.front()));

    q0.front()  = q_start;
    qa0.front() = qa_start;

    for (size_t i = 0; i < decltype(car)::NCONTROL; ++i)
    {
        if ( control_variables[i].optimal_control_type != Optimal_laptime<decltype(car)>::DONT_OPTIMIZE )
            control_variables[i].u.front() = u_start[i];
    }

    Optimal_laptime opt_laptime(s, false, true, car, q0, qa0, control_variables, {});
    opt_laptime.xml();

    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_chicane.xml", true);

    // Check the results with a saved simulation

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/throttle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);

    // Fz_fl
    auto Fz_fl_saved = opt_saved.get_element("optimal_laptime/Fz_fl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFL], Fz_fl_saved[i], 1.0e-6);

    // Fz_fr
    auto Fz_fr_saved = opt_saved.get_element("optimal_laptime/Fz_fr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFR], Fz_fr_saved[i], 1.0e-6);

    // Fz_rl
    auto Fz_rl_saved = opt_saved.get_element("optimal_laptime/Fz_rl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRL], Fz_rl_saved[i], 1.0e-6);

    // Fz_rr
    auto Fz_rr_saved = opt_saved.get_element("optimal_laptime/Fz_rr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRR], Fz_rr_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Catalunya_warm_start)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const auto& s = catalunya_pproc.s;
    const size_t n = s.size();

    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_adapted.xml", true);
    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p> opt_laptime_saved(opt_saved);
    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts;

    opts.print_level = 0;

    // Set the dissipation
    opt_laptime_saved.control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING].dissipation = 50.0;
    opt_laptime_saved.control_variables[decltype(car)::Chassis_type::ITHROTTLE].dissipation = 20.0*8.0e-4;

    Optimal_laptime opt_laptime(s, true, true, car, opt_laptime_saved.q, opt_laptime_saved.qa, opt_laptime_saved.control_variables, 
        opt_laptime_saved.optimization_data.zl, opt_laptime_saved.optimization_data.zu, opt_laptime_saved.optimization_data.lambda, opts);


    EXPECT_EQ(opt_laptime.iter_count, 0);

    opt_laptime.xml();

    // Check the results with a saved simulation

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/control_variables/delta/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/throttle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);

    // Fz_fl
    auto Fz_fl_saved = opt_saved.get_element("optimal_laptime/Fz_fl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFL], Fz_fl_saved[i], 1.0e-6);

    // Fz_fr
    auto Fz_fr_saved = opt_saved.get_element("optimal_laptime/Fz_fr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFR], Fz_fr_saved[i], 1.0e-6);

    // Fz_rl
    auto Fz_rl_saved = opt_saved.get_element("optimal_laptime/Fz_rl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRL], Fz_rl_saved[i], 1.0e-6);

    // Fz_rr
    auto Fz_rr_saved = opt_saved.get_element("optimal_laptime/Fz_rr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRR], Fz_rr_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Catalunya_chicane_warm_start)
{
    // The chicane test uses the adapted mesh from i=533 to i=677

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Set initial condtion
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_adapted.xml", true);
    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p> opt_laptime_saved(opt_saved);

    // Get arclength: s(i0:i1)
    const size_t i0         = 533;
    const size_t i1         = 677;
    const size_t n_elements = i1-i0;
    const size_t n_points   = n_elements + 1;
    const size_t n_vars_per_point = Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::n_variables_per_point<true>(opt_laptime_saved.control_variables);
    const size_t n_cons_per_point = Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::n_constraints_per_element<true>(opt_laptime_saved.control_variables);
    std::vector<scalar> s(i1-i0+1);
    std::copy(catalunya_pproc.s.cbegin()+i0,catalunya_pproc.s.cbegin()+i1+1,s.begin());        

    std::vector<std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::NSTATE>> q(n_points);
    std::vector<std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::NALGEBRAIC>> qa(n_points);

    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING].optimal_control_type = Optimal_laptime<decltype(car)>::FULL_MESH;
    control_variables[decltype(car)::Chassis_type::ITHROTTLE].optimal_control_type                  = Optimal_laptime<decltype(car)>::FULL_MESH;
    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS].optimal_control_type                = Optimal_laptime<decltype(car)>::DONT_OPTIMIZE;

    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING].u = std::vector<scalar>(n_points);
    control_variables[decltype(car)::Chassis_type::ITHROTTLE].u = std::vector<scalar>(n_points);
        
    std::vector<scalar> zl(n_vars_per_point*n_elements);
    std::vector<scalar> zu(n_vars_per_point*n_elements);
    std::vector<scalar> lambda(n_cons_per_point*n_elements);

    std::copy(opt_laptime_saved.q.cbegin()+i0 , opt_laptime_saved.q.cbegin()+i1+1 , q.begin());
    std::copy(opt_laptime_saved.qa.cbegin()+i0, opt_laptime_saved.qa.cbegin()+i1+1, qa.begin());
    for (auto i : std::array<size_t,2>{decltype(car)::Chassis_type::front_axle_type::ISTEERING,decltype(car)::Chassis_type::ITHROTTLE})
        std::copy(opt_laptime_saved.control_variables[i].u.cbegin()+i0, 
                  opt_laptime_saved.control_variables[i].u.cbegin()+i1+1, 
                  control_variables[i].u.begin());

    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS].u = opt_laptime_saved.control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS].u;

    std::copy(opt_laptime_saved.optimization_data.zl.cbegin() + (i0+1)*n_vars_per_point,
              opt_laptime_saved.optimization_data.zl.cbegin() + (i1+1)*n_vars_per_point,
              zl.begin());
    std::copy(opt_laptime_saved.optimization_data.zu.cbegin() + (i0+1)*n_vars_per_point,
              opt_laptime_saved.optimization_data.zu.cbegin() + (i1+1)*n_vars_per_point,
              zu.begin());
    std::copy(opt_laptime_saved.optimization_data.lambda.cbegin() + (i0)*n_cons_per_point,
              opt_laptime_saved.optimization_data.lambda.cbegin() + (i1)*n_cons_per_point,
              lambda.begin());

    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING].dissipation = 50.0;
    control_variables[decltype(car)::Chassis_type::ITHROTTLE].dissipation = 20.0*8.0e-4;

    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts;

    opts.print_level = 0;

    Optimal_laptime opt_laptime(s, false, true, car, q, qa, control_variables, zl, zu, lambda, opts);

    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_chicane_saved("data/f1_optimal_laptime_catalunya_chicane.xml", true);

    // kappa front left
    auto kappa_fl_saved = opt_chicane_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_chicane_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_chicane_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_chicane_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_chicane_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_chicane_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_chicane_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_chicane_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_chicane_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_chicane_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_chicane_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_chicane_saved.get_element("optimal_laptime/throttle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);

    // Fz_fl
    auto Fz_fl_saved = opt_chicane_saved.get_element("optimal_laptime/Fz_fl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFL], Fz_fl_saved[i], 1.0e-6);

    // Fz_fr
    auto Fz_fr_saved = opt_chicane_saved.get_element("optimal_laptime/Fz_fr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFR], Fz_fr_saved[i], 1.0e-6);

    // Fz_rl
    auto Fz_rl_saved = opt_chicane_saved.get_element("optimal_laptime/Fz_rl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRL], Fz_rl_saved[i], 1.0e-6);

    // Fz_rr
    auto Fz_rr_saved = opt_chicane_saved.get_element("optimal_laptime/Fz_rr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n_points; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRR], Fz_rr_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, maximum_acceleration)
{
    Xml_document straight_xml("./data/straight.xml",true);
    Track_by_arcs straight(straight_xml,10.0,false);

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, {straight});

    constexpr const size_t n = 50;
    auto s = linspace(0.0,straight.get_total_length(),n+1);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 80.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]), 0.0e0);

    control_variables[decltype(car)::Chassis_type::ITHROTTLE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.u[decltype(car)::Chassis_type::ITHROTTLE]), 4.0e-7);

    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS] 
        = Optimal_laptime<decltype(car)>::create_dont_optimize();

    Optimal_laptime opt_laptime(s, false, true, car, {n+1,ss.q}, {n+1,ss.qa}, control_variables, {});

    opt_laptime.xml();

    // Check that the car accelerates
    constexpr const size_t IU = limebeer2014f1<scalar>::curvilinear<Track_by_arcs>::Chassis_type::IU;
    for (size_t i = 1; i < n; ++i)
    {
        EXPECT_TRUE(opt_laptime.q.at(i).at(IU) > opt_laptime.q.at(i-1).at(IU) );
    }

    // Check that steering is zero
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(Value(opt_laptime.control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING].u.at(i)), 0.0, 1.0e-14);

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_maximum_acceleration.xml", true);

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/throttle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Ovaltrack_open)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document ovaltrack_xml("./data/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,1.0,true);
    
    constexpr const size_t n = 400;

    auto s = linspace(0.0,ovaltrack.get_total_length(), n+1);

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 100.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]), 1.0e2);

    control_variables[decltype(car)::Chassis_type::ITHROTTLE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.u[decltype(car)::Chassis_type::ITHROTTLE]), 2.0e-3);

    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS] 
        = Optimal_laptime<decltype(car)>::create_dont_optimize();

    Optimal_laptime opt_laptime(s, false, true, car, {n+1,ss.q}, {n+1,ss.qa}, control_variables, {});
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_ovaltrack_open.xml", true);

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/throttle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Ovaltrack_closed)
{
    Xml_document ovaltrack_xml("./data/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,1.0,true);
    
    constexpr const size_t n = 100;

    auto s = linspace(0.0, ovaltrack.get_total_length(), n+1);
    s.pop_back();

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]), 1.0e2);

    control_variables[decltype(car)::Chassis_type::ITHROTTLE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::ITHROTTLE]), 2.0e-3);

    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS] 
        = Optimal_laptime<decltype(car)>::create_dont_optimize();

    Optimal_laptime opt_laptime(s, true, true, car, {n,ss.q}, {n,ss.qa}, control_variables, {});

    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_ovaltrack_closed.xml", true);

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/throttle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Melbourne_direct)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document database = {"./database/vehicles/f1/ferrari-2022-australia.xml", true};

    Xml_document melbourne_xml("./database/tracks/melbourne/melbourne_700.xml",true);
    Circuit_preprocessor melbourne_pproc(melbourne_xml);
    Track_by_polynomial melbourne(melbourne_pproc);
    
    constexpr const size_t n = 700;
    auto s = linspace(0.0,melbourne.get_total_length(),n+1);
    s.pop_back();

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(melbourne);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);
    limebeer2014f1<CppAD::AD<scalar>>::cartesian car_cartesian(database);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]), 5.0);

    control_variables[decltype(car)::Chassis_type::ITHROTTLE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::ITHROTTLE]), 8.0e-4);

    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS] 
        = Optimal_laptime<decltype(car)>::create_dont_optimize();

    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts; opts.print_level = 0;
    Optimal_laptime opt_laptime(s, true, true, car, {n,ss.q}, {n,ss.qa}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_melbourne_700_direct.xml", true);

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/throttle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Catalunya_chicane_derivative)
{
    // The chicane test uses the adapted mesh from i=533 to i=677

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 70.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    // Get arclength: s(i0:i1)
    const size_t i0 = 533;
    const size_t i1 = 677;
    std::vector<scalar> s(i1-i0+1);
    std::copy(catalunya_pproc.s.cbegin()+i0,catalunya_pproc.s.cbegin()+i1+1,s.begin());        

    const size_t n = s.size();

    // Set initial condtion
    std::vector<std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::NSTATE>>       q0(n,ss.q);
    std::vector<std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::NALGEBRAIC>>   qa0(n,ss.qa);
    
    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]),
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-1); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::ITHROTTLE]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::ITHROTTLE]),
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-5); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 
    
    Xml_document opt_full_lap("data/f1_optimal_laptime_catalunya_adapted.xml", true);
    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p> opt_laptime_full_lap(opt_full_lap);

    auto q_start = opt_laptime_full_lap.q[i0];
    auto qa_start = opt_laptime_full_lap.qa[i0];
    auto u_start = opt_laptime_full_lap.control_variables.control_array_at_s(car,i0,s.front());

    q0.front()  = q_start;
    qa0.front() = qa_start;

    for (size_t i = 0; i < decltype(car)::NCONTROL; ++i)
    {
        if ( control_variables[i].optimal_control_type != Optimal_laptime<decltype(car)>::DONT_OPTIMIZE )
            control_variables[i].u.front() = u_start[i];
    }

    Optimal_laptime<typename limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts;
    Optimal_laptime opt_laptime(s, false, false, car, q0, qa0, control_variables, opts);

    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_chicane_derivative.xml", true);

    // Check the results with a saved simulation

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6) << ", with i = " << i;

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/delta").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/throttle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);

    // Fz_fl
    auto Fz_fl_saved = opt_saved.get_element("optimal_laptime/Fz_fl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFL], Fz_fl_saved[i], 1.0e-6);

    // Fz_fr
    auto Fz_fr_saved = opt_saved.get_element("optimal_laptime/Fz_fr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFR], Fz_fr_saved[i], 1.0e-6);

    // Fz_rl
    auto Fz_rl_saved = opt_saved.get_element("optimal_laptime/Fz_rl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRL], Fz_rl_saved[i], 1.0e-6);

    // Fz_rr
    auto Fz_rr_saved = opt_saved.get_element("optimal_laptime/Fz_rr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRR], Fz_rr_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Melbourne_derivative)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document database = {"./database/vehicles/f1/ferrari-2022-australia.xml", true};

    Xml_document melbourne_xml("./database/tracks/melbourne/melbourne_700.xml",true);
    Circuit_preprocessor melbourne_pproc(melbourne_xml);
    Track_by_polynomial melbourne(melbourne_pproc);
    
    const auto& s = melbourne_pproc.s;
    constexpr const size_t n = 700;

    EXPECT_EQ(s.size(), n);

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(melbourne);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);
    limebeer2014f1<CppAD::AD<scalar>>::cartesian car_cartesian(database);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]),
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-1); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::ITHROTTLE]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::ITHROTTLE]),
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-5); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 
 

    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts; opts.print_level = 0;
    Optimal_laptime opt_laptime(s, true, false, car, {n,ss.q}, {n,ss.qa}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_melbourne_700_derivative.xml", true);

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/control_variables/delta/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/throttle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Melbourne_derivative_warm_start)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document database = {"./database/vehicles/f1/ferrari-2022-australia.xml", true};

    Xml_document melbourne_xml("./database/tracks/melbourne/melbourne_700.xml",true);
    Circuit_preprocessor melbourne_pproc(melbourne_xml);
    Track_by_polynomial melbourne(melbourne_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(melbourne);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    const auto& s = melbourne_pproc.s;
    const size_t n = s.size();

    Xml_document opt_saved("data/f1_optimal_laptime_melbourne_700_derivative.xml", true);
    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p> opt_laptime_saved(opt_saved);


    opt_laptime_saved.control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING].dissipation = 1.0e-1;
    opt_laptime_saved.control_variables[decltype(car)::Chassis_type::ITHROTTLE].dissipation = 1.0e-5;

    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts;

    opts.print_level = 0;

    Optimal_laptime opt_laptime(s, true, false, car, opt_laptime_saved.q, opt_laptime_saved.qa, opt_laptime_saved.control_variables, 
        opt_laptime_saved.optimization_data.zl, opt_laptime_saved.optimization_data.zu, opt_laptime_saved.optimization_data.lambda, opts);

    opt_laptime.xml();

    EXPECT_EQ(opt_laptime.iter_count, 0);

    // Check the results with a saved simulation

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // Fz_fl
    auto Fz_fl_saved = opt_saved.get_element("optimal_laptime/Fz_fl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFL], Fz_fl_saved[i], 1.0e-6);

    // Fz_fr
    auto Fz_fr_saved = opt_saved.get_element("optimal_laptime/Fz_fr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFR], Fz_fr_saved[i], 1.0e-6);

    // Fz_rl
    auto Fz_rl_saved = opt_saved.get_element("optimal_laptime/Fz_rl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRL], Fz_rl_saved[i], 1.0e-6);

    // Fz_rr
    auto Fz_rr_saved = opt_saved.get_element("optimal_laptime/Fz_rr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRR], Fz_rr_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/control_variables/delta/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/throttle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, imola_adapted_hypermesh)
{
    // Keep valgrind, at least for one go
    //if ( is_valgrind ) GTEST_SKIP();

    Xml_document database = {"./database/vehicles/f1/redbull-2022-imola-wet.xml", true};

    Xml_document imola_xml("./database/tracks/imola/imola_adapted.xml",true);
    Circuit_preprocessor imola_pproc(imola_xml);
    Track_by_polynomial imola(imola_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(imola);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = imola_pproc.s;

    const size_t n = s.size();

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::ISTEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::front_axle_type::ISTEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::ITHROTTLE]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.u[decltype(car)::Chassis_type::ITHROTTLE]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::IBRAKE_BIAS]
        = Optimal_laptime<decltype(car)>::create_hypermesh(std::vector<scalar>{0.0, 990.0, 1526.0, 1925.0, 2589.0, 3024.0, 3554.0},
                                                           std::vector<scalar>(7,ss.u[decltype(car)::Chassis_type::IBRAKE_BIAS])); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    Optimal_laptime opt_laptime(s, true, true, car, {n,ss.q}, {n,ss.qa}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_imola_adapted.xml", true);

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/steering-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/steering-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/powered-kappa-left").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/powered-kappa-right").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/u").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IU], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/v").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IV], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::Chassis_t::IOMEGA], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/n").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/alpha").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/control_variables/delta/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Front_axle_t::ISTEERING].u[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/throttle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::ITHROTTLE].u[i], throttle_saved[i], 1.0e-6);

    // brake-bias
    auto brake_bias_saved = opt_saved.get_element("optimal_laptime/control_variables/brake-bias/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < brake_bias_saved.size(); ++i)
        EXPECT_NEAR(opt_laptime.control_variables[limebeer2014f1<scalar>::Chassis_t::IBRAKE_BIAS].u[i], brake_bias_saved[i], 1.0e-6);

    // Fz_fl
    auto Fz_fl_saved = opt_saved.get_element("optimal_laptime/Fz_fl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFL], Fz_fl_saved[i], 1.0e-6);

    // Fz_fr
    auto Fz_fr_saved = opt_saved.get_element("optimal_laptime/Fz_fr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZFR], Fz_fr_saved[i], 1.0e-6);

    // Fz_rl
    auto Fz_rl_saved = opt_saved.get_element("optimal_laptime/Fz_rl").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRL], Fz_rl_saved[i], 1.0e-6);

    // Fz_rr
    auto Fz_rr_saved = opt_saved.get_element("optimal_laptime/Fz_rr").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.qa[i][limebeer2014f1<scalar>::Chassis_t::IFZRR], Fz_rr_saved[i], 1.0e-6);
}
