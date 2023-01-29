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
inline static void check_derivatives(const Optimal_laptime<Dynamic_model_t>& opt_laptime, const Optimal_laptime<Dynamic_model_t>& dopt_laptime, const scalar dpar, const size_t n, const double max_error)
{
    // Check the full gradient
    for (size_t i = 0; i < opt_laptime.optimization_data.x.size(); ++i)
        EXPECT_NEAR(opt_laptime.dx_dp.front()[i], (dopt_laptime.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dpar), max_error);

    EXPECT_NEAR(opt_laptime.dlaptime_dp.front(), (dopt_laptime.laptime - opt_laptime.laptime)/dpar, 1.0e-6);

    // kappa fl
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_LEFT], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_LEFT] - opt_laptime.inputs[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_LEFT])/dpar, 1.0e-5);
    }

    // kappa fr
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_RIGHT], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_RIGHT] - opt_laptime.inputs[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_RIGHT])/dpar, 1.0e-5);
    }

    // kappa rl
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_LEFT], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_LEFT] - opt_laptime.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_LEFT])/dpar, 1.0e-5);
    }

    // kappa rr
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_RIGHT], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_RIGHT] - opt_laptime.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_RIGHT])/dpar, 1.0e-5);
    }

    // u
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::velocity_x_mps], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::velocity_x_mps] - opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::velocity_x_mps])/dpar, 1.0e-5);
    }

    // v
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::velocity_y_mps], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::velocity_y_mps] - opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::velocity_y_mps])/dpar, 2.0e-5);
    }

    // omega
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::yaw_rate_radps], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::yaw_rate_radps] - opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::yaw_rate_radps])/dpar, 2.0e-5);
    }

    // n
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::lateral_displacement], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::lateral_displacement] - opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::lateral_displacement])/dpar, 2.0e-5);
    }

    // alpha
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::track_heading_angle], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::track_heading_angle] - opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::track_heading_angle])/dpar, 1.0e-6);
    }

    // time
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::time], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::time] - opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::time])/dpar, 1.0e-6);
    }

    // steering
    for (size_t i = 1; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dcontrols_dp.front()[limebeer2014f1<scalar>::Chassis_t::Front_axle_type::control_names::STEERING].controls[i], 
                    (dopt_laptime.controls[limebeer2014f1<scalar>::Chassis_t::Front_axle_type::control_names::STEERING].controls[i] 
                      - opt_laptime.controls[limebeer2014f1<scalar>::Chassis_t::Front_axle_type::control_names::STEERING].controls[i])/dpar, 7.0e-5);
    }

    // throttle
    for (size_t i = 1; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dcontrols_dp.front()[limebeer2014f1<scalar>::Chassis_t::control_names::throttle].controls[i], 
                    (dopt_laptime.controls[limebeer2014f1<scalar>::Chassis_t::control_names::throttle].controls[i] 
                      - opt_laptime.controls[limebeer2014f1<scalar>::Chassis_t::control_names::throttle].controls[i])/dpar, 2.0e-6);
    }

    // Fz_fl
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_fl_g], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_fl_g] - opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_fl_g])/dpar, 4.0e-6);
    }

    // Fz_fr
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_fr_g], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_fr_g] - opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_fr_g])/dpar, 4.0e-6);
    }

    // Fz_rl
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_rl_g], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_rl_g] - opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_rl_g])/dpar, 4.0e-6);
    }

    // Fz_rl
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.dinputs_dp.front()[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_rr_g], 
                    (dopt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_rr_g] - opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Chassis_type::input_names::force_z_rr_g])/dpar, 4.0e-6);
    }

}


template<typename Dynamic_model_t>
inline static void check_case(const Optimal_laptime<Dynamic_model_t>& opt_laptime, const Optimal_laptime<Dynamic_model_t>& dopt_laptime, const scalar dpar, Xml_document& opt_saved, const size_t n, const double max_error)
{
    check_derivatives(opt_laptime, dopt_laptime, dpar, n, max_error);

    // Check the case to confirm that the sensitivity computation does not change the optimal laptime solution -------------------------------------------

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/front-axle.left-tire.kappa").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_LEFT], kappa_fl_saved[i], 1.0e-6);
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/front-axle.right-tire.kappa").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_RIGHT], kappa_fr_saved[i], 1.0e-6);
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/rear-axle.left-tire.kappa").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_LEFT], kappa_rl_saved[i], 1.0e-6);
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/rear-axle.right-tire.kappa").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_RIGHT], kappa_rr_saved[i], 1.0e-6);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/chassis.velocity.x").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::velocity_x_mps], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/chassis.velocity.y").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::velocity_y_mps], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/chassis.omega.z").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::yaw_rate_radps], omega_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::time], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/road.lateral-displacement").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::lateral_displacement], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/road.track-heading-angle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::track_heading_angle], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/control_variables/front-axle.steering-angle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.controls[limebeer2014f1<scalar>::Front_axle_t::control_names::STEERING].controls[i], delta_saved[i], 1.0e-6);

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/chassis.throttle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.controls[limebeer2014f1<scalar>::Chassis_t::control_names::throttle].controls[i], throttle_saved[i], 1.0e-6);
}


TEST_F(F1_sensitivity_analysis, Catalunya_chicane)
{

    // The chicane test uses the adapted mesh from i=533 to i=677
    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    car.set_parameter("vehicle/front-axle/inertia", 1.00);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);

    // Set the mass as car parameter
    car.add_parameter("vehicle/chassis/mass", "mass", 660.0);

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
    std::vector<std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::number_of_inputs>>       q0(n,ss.inputs);
    
    // Construct control variables
    auto controls = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    controls[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    controls[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    // Set starting condition
    Xml_document opt_full_lap("data/f1_optimal_laptime_catalunya_adapted.xml", true);
    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p> opt_laptime_full_lap(opt_full_lap);

    std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::number_of_inputs>       q_start(opt_laptime_full_lap.inputs[i0]);
    auto u_start(opt_laptime_full_lap.controls.control_array_at_s(car,i0,s.front()));

    q0.front()  = q_start;

    for (size_t i = 0; i < decltype(car)::number_of_controls; ++i)
    {
        if ( controls[i].optimal_control_type != Optimal_control_type::DONT_OPTIMIZE )
            controls[i].controls.front() = u_start[i];
    }

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.check_optimality = true;
    Optimal_laptime<decltype(car)> opt_laptime(s, false, true, car, q0, controls, opts);
    
    const scalar dmass = 1.0e-2;
    car.set_parameter("vehicle/chassis/mass",660.0 + dmass);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime(s, false, true, car, opt_laptime.inputs, opt_laptime.controls, 
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
    car.add_parameter("vehicle/chassis/mass", "mass", 660.0);

    constexpr const size_t n = 50;
    auto s = linspace(0.0,straight.get_total_length(),n+1);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 80.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    auto controls = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 0.0e0);

    controls[decltype(car)::Chassis_type::control_names::throttle] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 4.0e-7);

    controls[decltype(car)::Chassis_type::control_names::brake_bias] 
        = Optimal_laptime<decltype(car)>::create_dont_optimize();

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.check_optimality = true;
    Optimal_laptime<decltype(car)> opt_laptime(s, false, true, car, {n+1,ss.inputs}, controls, opts);
    opt_laptime.xml();
    
    const scalar dmass = 1.0e-3;
    car.set_parameter("vehicle/chassis/mass",660.0 + dmass);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime(s, false, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);
    opt_laptime.xml();

    car.set_parameter("vehicle/chassis/mass",659.999);
    Optimal_laptime<decltype(car)> dopt_laptime_m(s, false, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);
    opt_laptime.xml();

    Xml_document opt_saved("data/f1_maximum_acceleration.xml", true);

    check_case(opt_laptime, dopt_laptime, dmass, opt_saved, n, 1.0e-6);
}


TEST_F(F1_sensitivity_analysis, Catalunya_discrete)
{

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    car.set_parameter("vehicle/front-axle/inertia", 1.00);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);

    // Set the mass as car parameter
    car.add_parameter("vehicle/chassis/mass", "mass", 660.0);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = catalunya_pproc.s;
    const auto& n = s.size();
    
    EXPECT_EQ(n, 500);

    // Construct control variables
    auto controls = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    controls[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    controls[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.check_optimality = true;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, controls, opts);

    const scalar dmass = 1.0e-4;
    car.set_parameter("vehicle/chassis/mass",660.0 + dmass);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    // Check the results with a saved simulation
    opt_laptime.xml()->save("f1_optimal_laptime_catalunya_discrete_with_wheel_inertia.xml");
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_discrete_with_wheel_inertia.xml", true);

    check_case(opt_laptime, dopt_laptime, dmass, opt_saved, n, 1.0e-4);
}


TEST_F(F1_sensitivity_analysis, Catalunya_discrete_optimization_variables)
{

    if ( is_valgrind ) GTEST_SKIP();

    // This test takes the derivatives w.r.t. selected variables, used in the parameter estimation method
    // from real F1 data
    // 
    // The variables are: [differential_stiffness(10^p), power/mass,cd,aero_eff,x_cog,h_cog,x_press,z_press, roll_balance, ...
    //      mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, max_torque/(mass.g0)]

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    car.set_parameter("vehicle/front-axle/inertia", 1.00);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);

    // Create all the variables
    const scalar differential_stiffness_log = log10(10.47);
    const scalar differential_stiffness     = std::pow(10.0, differential_stiffness_log);
    const scalar power_dimensionless        = 735.499/660.0;
    const scalar power                      = power_dimensionless*660.0;
    const scalar cd                         = 0.9;
    const scalar aero_efficiency            = 3.0/0.9;
    const scalar cl                         = aero_efficiency*cd;
    const scalar x_front                    = 1.8;
    const scalar x_rear                     = -1.6;
    const scalar wheelbase                  = x_front - x_rear;
    const scalar x_press_dimensionless      = -0.1/wheelbase;
    const scalar x_press                    = x_press_dimensionless*wheelbase;
    const scalar z_press                    = 0.3;
    const scalar roll_balance               = 0.5;
    const scalar mu_front_1                 = 1.8;
    const scalar mu_front_2                 = 1.45;
    const scalar mu_rear_1                  = 1.8;
    const scalar mu_rear_2                  = 1.45;
    const scalar max_torque_dimensionless   = 5000.0/(9.81*660.0);
    const scalar max_torque                 = max_torque_dimensionless*(9.81*660.0);

    car.add_parameter("vehicle/rear-axle/differential_stiffness", "differential-stiffness", differential_stiffness);
    car.add_parameter("vehicle/rear-axle/engine/maximum-power"  , "power"                 , power);
    car.add_parameter("vehicle/chassis/aerodynamics/cd"         , "cd"                    , cd);
    car.add_parameter("vehicle/chassis/aerodynamics/cl"         , "cl"                    , cl);
    car.add_parameter("vehicle/chassis/front_axle/x"            , "x_front"               , x_front);
    car.add_parameter("vehicle/chassis/rear_axle/x"             , "x_rear"                , x_rear);
    car.add_parameter("vehicle/chassis/pressure_center/x"       , "x_press"               , x_press);
    car.add_parameter("vehicle/chassis/pressure_center/z"       , "z_press"               , -z_press);
    car.add_parameter("vehicle/chassis/roll_balance_coefficient", "roll_balance"          , roll_balance);
    car.add_parameter("vehicle/front-tire/mu-x-max-1"           , "mu-x-1"                , mu_front_1);
    car.add_parameter("vehicle/front-tire/mu-y-max-1"           , "mu-y-1"                , mu_front_1);
    car.add_parameter("vehicle/front-tire/mu-x-max-2"           , "mu-x-2"                , mu_front_2);
    car.add_parameter("vehicle/front-tire/mu-y-max-2"           , "mu-y-2"                , mu_front_2);
    car.add_parameter("vehicle/rear-tire/mu-x-max-1"            , "mu-x-1"                , mu_rear_1);
    car.add_parameter("vehicle/rear-tire/mu-y-max-1"            , "mu-y-1"                , mu_rear_1);
    car.add_parameter("vehicle/rear-tire/mu-x-max-2"            , "mu-x-2"                , mu_rear_2);
    car.add_parameter("vehicle/rear-tire/mu-y-max-2"            , "mu-y-2"                , mu_rear_2);
    car.add_parameter("vehicle/front-axle/brakes/max_torque"    , "max-torque-front"      , max_torque);
    car.add_parameter("vehicle/rear-axle/brakes/max_torque"     , "max-torque-rear"       , max_torque);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = catalunya_pproc.s;
    const auto& n = s.size();
    
    EXPECT_EQ(n, 500);

    // Construct control variables
    auto controls = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    controls[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    controls[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.check_optimality = true;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, controls, opts);

    // (1) Derivatives w.r.t. differential stiffness
    const scalar d_differential_stiffness = 1.0e-4;
    car.set_parameter("vehicle/rear-axle/differential_stiffness", pow(10.0, differential_stiffness_log + d_differential_stiffness));
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_diff_stiffness(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    const size_t total_size = opt_laptime.optimization_data.x.size();
    EXPECT_EQ(opt_laptime.dx_dp.size(), 19);

    for (const auto& dx_dp_i : opt_laptime.dx_dp)
        EXPECT_EQ(dx_dp_i.size(), total_size);

    std::vector<scalar> dx_dp(total_size,0.0);
    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = opt_laptime.dx_dp[0][i]*log(10)*differential_stiffness;

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_diff_stiffness.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(d_differential_stiffness), max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. 10^(differential_stiffness)";

    // (2) Derivatives w.r.t. power
    const scalar dpower = 1.0e-6;
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power + 660.0*dpower);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_power(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = opt_laptime.dx_dp[1][i]*660.0;

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_power.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dpower), max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. 660.0*power";

    // (3) Derivatives w.r.t. cd
    const scalar dcd = 1.0e-5;
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd + dcd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl + aero_efficiency*dcd);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_cd(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = (opt_laptime.dx_dp[2][i] + opt_laptime.dx_dp[3][i]*aero_efficiency);

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_cd.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dcd), max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. cd";

    // (4) Derivatives w.r.t. aero_eff
    const scalar daero_eff = 1.0e-4;
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl + daero_eff*cd);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_cl(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = opt_laptime.dx_dp[3][i]*cd;

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_cl.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(daero_eff), max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. aero_efficiency";

    // (5) Derivatives w.r.t. x_cog
    const scalar dxcog = 1.0e-6;
    car.set_parameter("vehicle/rear-axle/differential_stiffness" , differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"   , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"          , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"          , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"             , x_front - wheelbase*dxcog);
    car.set_parameter("vehicle/chassis/rear_axle/x"              , x_rear - wheelbase*dxcog);
    car.set_parameter("vehicle/chassis/pressure_center/x"        , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"        , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient" , roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"            , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"            , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"            , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"            , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"             , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"             , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"             , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"             , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"     , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"      , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_xcog(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = -wheelbase*(opt_laptime.dx_dp[4][i] + opt_laptime.dx_dp[5][i]);

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_xcog.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dxcog),max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. x_cog";

    // (6) Derivatives w.r.t. x_press
    const scalar dx_press = 1.0e-7;
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press + dx_press*wheelbase);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_x_press(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press - dx_press*wheelbase);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_x_press_m(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);



    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = opt_laptime.dx_dp[6][i]*wheelbase;

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_x_press.optimization_data.x[i] - dopt_laptime_x_press_m.optimization_data.x[i])/(2.0*dx_press),max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. x_press.wheelbase";

    // (7) Derivatives w.r.t. z_press
    const scalar dz_press = 1.0e-5;
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press-dz_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_z_press(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = -opt_laptime.dx_dp[7][i];

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_z_press.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dz_press),max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. z_press";

    // (8) Derivatives w.r.t. roll_balance
    const scalar droll_balance = 1.0e-6;
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance + droll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_roll_balance(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = opt_laptime.dx_dp[8][i];

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_roll_balance.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(droll_balance),max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. roll_balance";


    // (9) Derivatives w.r.t. mu_front_1
    const scalar dmu = 1.0e-6;
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1 + dmu);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1 + dmu);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_grip(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = (opt_laptime.dx_dp[9][i] + opt_laptime.dx_dp[10][i]);

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_grip.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dmu),max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. mu_front_1";

    // (10) Derivatives w.r.t. mu_front_2
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2 + dmu);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2 + dmu);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_grip_front_2(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = (opt_laptime.dx_dp[11][i] + opt_laptime.dx_dp[12][i]);

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_grip_front_2.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dmu),max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. mu_front_2";


    // (11) Derivatives w.r.t. mu_rear_1
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1 + dmu);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1 + dmu);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_grip_rear_1(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = (opt_laptime.dx_dp[13][i] + opt_laptime.dx_dp[14][i]);

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_grip_rear_1.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dmu),max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. mu_rear_1";

    // (12) Derivatives w.r.t. mu_rear_2
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2 + dmu);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2 + dmu);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_grip_rear_2(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = (opt_laptime.dx_dp[15][i] + opt_laptime.dx_dp[16][i]);

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_grip_rear_2.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dmu),max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. mu_rear_2";

    // (13) Derivatives w.r.t. max_torque
    const scalar dmax_torque = 1.0e-6;
    car.set_parameter("vehicle/rear-axle/differential_stiffness", differential_stiffness);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power"  , power);
    car.set_parameter("vehicle/chassis/aerodynamics/cd"         , cd);
    car.set_parameter("vehicle/chassis/aerodynamics/cl"         , cl);
    car.set_parameter("vehicle/chassis/front_axle/x"            , x_front);
    car.set_parameter("vehicle/chassis/rear_axle/x"             , x_rear);
    car.set_parameter("vehicle/chassis/pressure_center/x"       , x_press);
    car.set_parameter("vehicle/chassis/pressure_center/z"       , -z_press);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", roll_balance);
    car.set_parameter("vehicle/front-tire/mu-x-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-y-max-1"           , mu_front_1);
    car.set_parameter("vehicle/front-tire/mu-x-max-2"           , mu_front_2);
    car.set_parameter("vehicle/front-tire/mu-y-max-2"           , mu_front_2);
    car.set_parameter("vehicle/rear-tire/mu-x-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-y-max-1"            , mu_rear_1);
    car.set_parameter("vehicle/rear-tire/mu-x-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/rear-tire/mu-y-max-2"            , mu_rear_2);
    car.set_parameter("vehicle/front-axle/brakes/max_torque"    , max_torque + 660.0*9.81*dmax_torque);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque"     , max_torque + 660.0*9.81*dmax_torque);

    opts.check_optimality = false;
    Optimal_laptime<decltype(car)> dopt_laptime_max_torque(s, true, true, car, opt_laptime.inputs, opt_laptime.controls, 
        opt_laptime.optimization_data.zl, opt_laptime.optimization_data.zu, opt_laptime.optimization_data.lambda, opts);

    for (size_t i = 0; i < total_size; ++i)
        dx_dp[i] = (opt_laptime.dx_dp[17][i] + opt_laptime.dx_dp[18][i])*660.0*9.81;

    // Check the errors
    for (size_t i = 0; i < total_size; ++i)
        EXPECT_NEAR(dx_dp[i], (dopt_laptime_max_torque.optimization_data.x[i] - opt_laptime.optimization_data.x[i])/(dmax_torque),max(1.0, std::abs(dx_dp[i]))*5.0e-3) << ", with i = " << i << ", in derivative w.r.t. 660.0*9.81*max_torque";
}
