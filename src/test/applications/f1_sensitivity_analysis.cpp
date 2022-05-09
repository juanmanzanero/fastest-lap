#include "gtest/gtest.h"
#include "lion/math/matrix_extensions.h"
#include "src/core/applications/optimal_laptime.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "src/core/applications/steady_state.h"
#include "src/core/applications/circuit_preprocessor.h"

extern bool is_valgrind;

class F1_sensitivity_analysis : public ::testing::Test
{
 protected:
    Xml_document database = {"./database/vehicles/f1/limebeer-2014-f1.xml", true};
    limebeer2014f1<CppAD::AD<scalar>>::cartesian car_cartesian = { database };
    limebeer2014f1<scalar>::cartesian car_cartesian_scalar = { database };
};

template<typename Dynamic_model_t>
inline static void check_case(const Optimal_laptime<Dynamic_model_t>& opt_laptime, const Optimal_laptime<Dynamic_model_t>& dopt_laptime, const scalar dpar, Xml_document& opt_saved, const size_t n, const double max_error)
{
    // Check the full gradient
    for (size_t i = 0; i < opt_laptime.optimization_data.x.size(); ++i)
        EXPECT_NEAR(opt_laptime.dxdp.front()[i], (dopt_laptime.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dpar), max_error);

    EXPECT_NEAR(opt_laptime.dlaptimedp.front(), (dopt_laptime.laptime - opt_laptime.laptime)/dpar, 1.0e-6);

    // kappa fl
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT] - opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_LEFT])/dpar, 1.0e-5);
    }

    // kappa fr
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT] - opt_laptime.q[i][limebeer2014f1<scalar>::Front_axle_t::IKAPPA_RIGHT])/dpar, 1.0e-5);
    }

    // kappa rl
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT] - opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_LEFT])/dpar, 1.0e-5);
    }

    // kappa rr
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT] - opt_laptime.q[i][limebeer2014f1<scalar>::Rear_axle_t::IKAPPA_RIGHT])/dpar, 1.0e-5);
    }

    // u
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IU], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IU] - opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IU])/dpar, 1.0e-5);
    }

    // v
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IV], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IV] - opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IV])/dpar, 2.0e-5);
    }

    // omega
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IOMEGA], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IOMEGA] - opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IOMEGA])/dpar, 2.0e-5);
    }

    // n
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN] - opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IN])/dpar, 2.0e-5);
    }

    // alpha
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA] - opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::IALPHA])/dpar, 1.0e-6);
    }

    // time
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME] - opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::ITIME])/dpar, 1.0e-6);
    }

    // steering
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::Front_axle_type::ISTEERING], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::Front_axle_type::ISTEERING] - opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::Front_axle_type::ISTEERING])/dpar, 1.0e-6);
    }

    // throttle
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqdp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::ITHROTTLE], 
                    (dopt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::ITHROTTLE] - opt_laptime.q[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::ITHROTTLE])/dpar, 1.0e-6);
    }


    // Fz_fl
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqadp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZFL], 
                    (dopt_laptime.qa[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZFL] - opt_laptime.qa[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZFL])/dpar, 4.0e-6);
    }

    // Fz_fr
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqadp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZFR], 
                    (dopt_laptime.qa[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZFR] - opt_laptime.qa[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZFR])/dpar, 4.0e-6);
    }

    // Fz_rl
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqadp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZRL], 
                    (dopt_laptime.qa[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZRL] - opt_laptime.qa[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZRL])/dpar, 4.0e-6);
    }

    // Fz_rl
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dqadp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZRR], 
                    (dopt_laptime.qa[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZRR] - opt_laptime.qa[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::IFZRR])/dpar, 4.0e-6);
    }

    // Check the case to confirm that the sensitivity computation does not change the optimal laptime solution -------------------------------------------

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


TEST_F(F1_sensitivity_analysis, Catalunya_chicane)
{
    // The chicane test uses the adapted mesh from i=533 to i=677
    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Set the mass as car parameter
    car.add_parameter("vehicle/chassis/mass", 660.0);

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

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.check_optimality = true;
    Optimal_laptime opt_laptime(s, false, true, car, q0, qa0, control_variables, opts);
    
    const scalar dmass = 1.0e-2;
    car.set_parameter("vehicle/chassis/mass",660.0 + dmass);

    opts.check_optimality = false;
    Optimal_laptime dopt_laptime(s, false, true, car, opt_laptime.q, opt_laptime.qa, opt_laptime.control_variables, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_chicane.xml", true);

    check_case(opt_laptime, dopt_laptime, dmass, opt_saved, n, 1.0e-6);
}


TEST_F(F1_sensitivity_analysis, maximum_acceleration)
{
    Xml_document straight_xml("./data/straight.xml",true);
    Track_by_arcs straight(straight_xml,10.0,false);

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, {straight});

    // Set the mass as car parameter
    car.add_parameter("vehicle/chassis/mass", 660.0);

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

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.check_optimality = true;
    Optimal_laptime opt_laptime(s, false, true, car, {n+1,ss.q}, {n+1,ss.qa}, control_variables, opts);
    opt_laptime.xml();
    
    const scalar dmass = 1.0e-3;
    car.set_parameter("vehicle/chassis/mass",660.0 + dmass);

    opts.check_optimality = false;
    Optimal_laptime dopt_laptime(s, false, true, car, opt_laptime.q, opt_laptime.qa, opt_laptime.control_variables, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);
    opt_laptime.xml();

    car.set_parameter("vehicle/chassis/mass",659.999);
    Optimal_laptime dopt_laptime_m(s, false, true, car, opt_laptime.q, opt_laptime.qa, opt_laptime.control_variables, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);
    opt_laptime.xml();

    Xml_document opt_saved("data/f1_maximum_acceleration.xml", true);

    check_case(opt_laptime, dopt_laptime, dmass, opt_saved, n, 1.0e-6);
}


TEST_F(F1_sensitivity_analysis, Catalunya_discrete)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_discrete.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Set the mass as car parameter
    car.add_parameter("vehicle/chassis/mass", 660.0);

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
    opts.check_optimality = true;
    Optimal_laptime opt_laptime(s, true, true, car, {n,ss.q}, {n,ss.qa}, control_variables, opts);

    const scalar dmass = 1.0e-4;
    car.set_parameter("vehicle/chassis/mass",660.0 + dmass);

    opts.check_optimality = false;
    Optimal_laptime dopt_laptime(s, true, true, car, opt_laptime.q, opt_laptime.qa, opt_laptime.control_variables, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);


    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_discrete.xml", true);

    check_case(opt_laptime, dopt_laptime, dmass, opt_saved, n, 1.0e-4);
}
