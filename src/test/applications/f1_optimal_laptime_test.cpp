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

template<typename Dynamic_model_t>
static void check_optimal_laptime(const Optimal_laptime<Dynamic_model_t>& opt_laptime, Xml_document& opt_saved, const size_t n, const scalar maximum_error)
{
    // Cover also the constructor from Xml_document
    Optimal_laptime<Dynamic_model_t> opt_laptime_saved(opt_saved);

    const auto new_reference_file = std::regex_replace(opt_saved.get_file_name(), std::regex("data\\/"), "new-references/");
    std::ifstream fin( new_reference_file );
 
    if( fin.fail() )
    {
        opt_laptime.xml()->save(new_reference_file);
    }

    EXPECT_TRUE(opt_laptime.iter_count < static_cast<size_t>(opt_saved.get_element("optimal_laptime/optimization_data/number_of_iterations").get_value(int()) + 5)) << "New iter: " << opt_laptime.iter_count << ", saved iters: " << opt_saved.get_element("optimal_laptime/optimization_data/number_of_iterations").get_value(int());

    EXPECT_NEAR(opt_laptime.laptime, opt_laptime_saved.laptime, maximum_error);
    EXPECT_NEAR(opt_laptime.laptime, opt_saved.get_element("optimal_laptime/laptime").get_value(scalar()), maximum_error);

    // kappa front left
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/front-axle.left-tire.kappa").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs.at(i).at(limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_LEFT), kappa_fl_saved.at(i), maximum_error);
        EXPECT_NEAR(opt_laptime.inputs.at(i).at(limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_LEFT), opt_laptime_saved.inputs.at(i).at(limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_LEFT), maximum_error);
    }
    
    // kappa front right
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/front-axle.right-tire.kappa").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_RIGHT], kappa_fr_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_RIGHT], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::Front_axle_t::input_names::KAPPA_RIGHT], maximum_error);
    }
    
    // kappa rear left
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/rear-axle.left-tire.kappa").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_LEFT], kappa_rl_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_LEFT], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_LEFT], maximum_error);
    }
    
    // kappa rear right
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/rear-axle.right-tire.kappa").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_RIGHT], kappa_rr_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_RIGHT], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::Rear_axle_t::input_names::KAPPA_RIGHT], maximum_error);
    }

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/chassis.velocity.x").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::velocity_x_mps], u_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::velocity_x_mps], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::velocity_x_mps], maximum_error);
    }

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/chassis.velocity.y").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::velocity_y_mps], v_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::velocity_y_mps], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::velocity_y_mps], maximum_error);
    }

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/chassis.omega.z").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::yaw_rate_radps], omega_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::yaw_rate_radps], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::yaw_rate_radps], maximum_error);
    }

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::time], time_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::time], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::time], maximum_error);
    }

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/road.lateral-displacement").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::lateral_displacement], n_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::lateral_displacement], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::lateral_displacement], maximum_error);
    }

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/road.track-heading-angle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::track_heading_angle], alpha_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::track_heading_angle], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::curvilinear_p::Road_t::input_names::track_heading_angle], maximum_error);
    }

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/control_variables/front-axle.steering-angle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.controls[limebeer2014f1<scalar>::Front_axle_t::control_names::STEERING].controls[i], delta_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.controls[limebeer2014f1<scalar>::Front_axle_t::control_names::STEERING].controls[i], opt_laptime_saved.controls[limebeer2014f1<scalar>::Front_axle_t::control_names::STEERING].controls[i], maximum_error);
    }

    // throttle
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/chassis.throttle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.controls[limebeer2014f1<scalar>::Chassis_t::control_names::throttle].controls[i], throttle_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.controls[limebeer2014f1<scalar>::Chassis_t::control_names::throttle].controls[i], opt_laptime_saved.controls[limebeer2014f1<scalar>::Chassis_t::control_names::throttle].controls[i], maximum_error);
    }

    // Fz_fl
    auto Fz_fl_saved = opt_saved.get_element("optimal_laptime/chassis.force_z_fl_g").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_fl_g], Fz_fl_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_fl_g], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_fl_g], maximum_error);
    }

    // Fz_fr
    auto Fz_fr_saved = opt_saved.get_element("optimal_laptime/chassis.force_z_fr_g").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_fr_g], Fz_fr_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_fr_g], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_fr_g], maximum_error);
    }

    // Fz_rl
    auto Fz_rl_saved = opt_saved.get_element("optimal_laptime/chassis.force_z_rl_g").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_rl_g], Fz_rl_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_rl_g], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_rl_g], maximum_error);
    }

    // Fz_rr
    auto Fz_rr_saved = opt_saved.get_element("optimal_laptime/chassis.force_z_rr_g").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_rr_g], Fz_rr_saved[i], maximum_error);
        EXPECT_NEAR(opt_laptime.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_rr_g], opt_laptime_saved.inputs[i][limebeer2014f1<scalar>::Chassis_t::input_names::force_z_rr_g], maximum_error);
    }
}


TEST_F(F1_optimal_laptime_test, Catalunya_discrete)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    car.set_parameter("vehicle/chassis/inertia/Ixx", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Iyy", 0.0);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = catalunya_pproc.s;
    const auto& n = s.size();
    
    EXPECT_EQ(n, 500);

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.print_level = 0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_discrete.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
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
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 


    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, {});
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_adapted.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Catalunya_variable_parameter)
{

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    car.set_parameter("vehicle/front-axle/inertia", 1.00);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);

    std::vector<std::string> parameter_aliases = {"power-1", "power-2"};
    std::vector<scalar> parameter_values = {735.499, 1000.0}; 
    std::vector<std::pair<scalar,size_t>> parameter_mesh = { {0.0, 0}, {2900.0, 0}, {3100.0,1}, {5000.0,1} };
    car.add_parameter("vehicle/rear-axle/engine/maximum-power", parameter_aliases, parameter_values, parameter_mesh);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = catalunya_pproc.s;

    const size_t n = s.size();

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.print_level = 0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_variable_parameter.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
}
  

TEST_F(F1_optimal_laptime_test, Catalunya_chicane)
{

    // The chicane test uses the adapted mesh from i=533 to i=677

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    car.set_parameter("vehicle/front-axle/inertia", 1.00);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);

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
    std::vector<std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::number_of_inputs>> inputs_start(n,ss.inputs);
    
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

    std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::number_of_inputs> q_start(opt_laptime_full_lap.inputs[i0]);
    auto controls_start(opt_laptime_full_lap.controls.control_array_at_s(car,i0,s.front()));

    inputs_start.front()  = q_start;

    for (size_t i = 0; i < decltype(car)::number_of_controls; ++i)
    {
        if ( controls[i].optimal_control_type != Optimal_control_type::DONT_OPTIMIZE )
            controls[i].controls.front() = controls_start[i];
    }

    Optimal_laptime<decltype(car)>::Options opts;

    if ( is_valgrind ) 
    {
        opts.maximum_iterations = 1;
        opts.throw_if_fail = false;
    }

    Optimal_laptime<decltype(car)> opt_laptime(s, false, true, car, inputs_start, controls, opts);
    opt_laptime.xml();

    if ( is_valgrind ) return;

    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_chicane.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
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

    // Set the dissipation
    opt_laptime_saved.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING].dissipation = 50.0;
    opt_laptime_saved.controls[decltype(car)::Chassis_type::control_names::throttle].dissipation = 20.0*8.0e-4;

    // Provisionally, flip the variable bound multipliers because the order has changed
    std::cout << "[WARNING] Flipped optimization data to fit new ordering, to remove!" << std::endl;
    std::vector<double> zl_flipped(opt_laptime_saved.optimization_data.zl);
    std::vector<double> zu_flipped(opt_laptime_saved.optimization_data.zu);

    for (size_t i_element = 0; i_element < s.size() ; ++i_element)
    {
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 0] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 0];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 1] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 1];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 2] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 2];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 3] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 3];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 4] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 4];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 5] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 5];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 6] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 6];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 7] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 9];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 8] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 10];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 9] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 11];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 10] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 12];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 11] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 7];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 12] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 8];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 13] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 13];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 14] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 1) * i_element + 14];

        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 0] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 0];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 1] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 1];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 2] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 2];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 3] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 3];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 4] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 4];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 5] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 5];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 6] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 6];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 7] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 9];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 8] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 10];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 9] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 11];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 10] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 12];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 11] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 7];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 12] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 8];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 13] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 13];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 14] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 1) * i_element + 14];
    }

    // Provisionally, flip sign to the roll moment equation, as it was the convention used in the previous version
    std::vector<scalar> lambda_flipped(opt_laptime_saved.optimization_data.lambda);

    for (size_t i_element = 0; i_element < s.size(); ++i_element)
    {
        lambda_flipped[19 * i_element + 10] *= -1.0;
    }

    opts.print_level = 0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, opt_laptime_saved.inputs, 
        opt_laptime_saved.controls, 
        zl_flipped, zu_flipped, lambda_flipped, opts);

    EXPECT_EQ(opt_laptime.iter_count, 0);

    opt_laptime.xml();

    // Check the results with a saved simulation
    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Catalunya_chicane_warm_start)
{

    // The chicane test uses the adapted mesh from i=533 to i=677

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    car.set_parameter("vehicle/front-axle/inertia", 1.00);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);

    // Set initial condtion
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_adapted.xml", true);
    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p> opt_laptime_saved(opt_saved);

    // Get arclength: s(i0:i1)
    const size_t i0         = 533;
    const size_t i1         = 677;
    const size_t n_elements = i1-i0;
    const size_t n_points   = n_elements + 1;
    const size_t n_vars_per_point = Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::base_type::n_variables_per_point<true>(opt_laptime_saved.controls);
    const size_t n_cons_per_point = Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::base_type::n_constraints_per_element<true>(opt_laptime_saved.controls, limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::N_OL_EXTRA_CONSTRAINTS);
    std::vector<scalar> s(i1-i0+1);
    std::copy(catalunya_pproc.s.cbegin()+i0,catalunya_pproc.s.cbegin()+i1+1,s.begin());        

    std::vector<std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::number_of_inputs>> q(n_points);

    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING].optimal_control_type = Optimal_control_type::FULL_MESH;
    control_variables[decltype(car)::Chassis_type::control_names::throttle].optimal_control_type                  = Optimal_control_type::FULL_MESH;
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias].optimal_control_type                = Optimal_control_type::DONT_OPTIMIZE;

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING].controls = std::vector<scalar>(n_points);
    control_variables[decltype(car)::Chassis_type::control_names::throttle].controls = std::vector<scalar>(n_points);
        
    std::vector<scalar> zl(n_vars_per_point*n_elements);
    std::vector<scalar> zu(n_vars_per_point*n_elements);
    std::vector<scalar> lambda(n_cons_per_point*n_elements);

    std::copy(opt_laptime_saved.inputs.cbegin()+i0 , opt_laptime_saved.inputs.cbegin()+i1+1 , q.begin());
    for (auto i : std::array<size_t,2>{decltype(car)::Chassis_type::front_axle_type::control_names::STEERING,decltype(car)::Chassis_type::control_names::throttle})
        std::copy(opt_laptime_saved.controls[i].controls.cbegin()+i0, 
                  opt_laptime_saved.controls[i].controls.cbegin()+i1+1, 
                  control_variables[i].controls.begin());

    control_variables[decltype(car)::Chassis_type::control_names::brake_bias].controls = opt_laptime_saved.controls[decltype(car)::Chassis_type::control_names::brake_bias].controls;

    std::copy(opt_laptime_saved.optimization_data.zl.cbegin() + (i0+1)*n_vars_per_point,
              opt_laptime_saved.optimization_data.zl.cbegin() + (i1+1)*n_vars_per_point,
              zl.begin());
    std::copy(opt_laptime_saved.optimization_data.zu.cbegin() + (i0+1)*n_vars_per_point,
              opt_laptime_saved.optimization_data.zu.cbegin() + (i1+1)*n_vars_per_point,
              zu.begin());
    std::copy(opt_laptime_saved.optimization_data.lambda.cbegin() + (i0)*n_cons_per_point,
              opt_laptime_saved.optimization_data.lambda.cbegin() + (i1)*n_cons_per_point,
              lambda.begin());


    // Provisionally, flip the variable bound multipliers because the order has changed
    std::cout << "[WARNING] Flipped optimization data to fit new ordering, to remove!" << std::endl;
    std::vector<double> zl_flipped(zl);
    std::vector<double> zu_flipped(zu);

    for (size_t i_element = 0; i_element < s.size()-1 ; ++i_element)
    {
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 0] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 0];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 1] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 1];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 2] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 2];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 3] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 3];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 4] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 4];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 5] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 5];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 6] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 6];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 7] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 9];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 8] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 10];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 9] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 11];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 10] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 12];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 11] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 7];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 12] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 8];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 13] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 13];
        zl_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 14] = zl[(decltype(car)::number_of_inputs + 1) * i_element + 14];

        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 0] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 0];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 1] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 1];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 2] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 2];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 3] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 3];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 4] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 4];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 5] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 5];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 6] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 6];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 7] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 9];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 8] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 10];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 9] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 11];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 10] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 12];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 11] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 7];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 12] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 8];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 13] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 13];
        zu_flipped[(decltype(car)::number_of_inputs + 1) * i_element + 14] = zu[(decltype(car)::number_of_inputs + 1) * i_element + 14];
    }

    // Provisionally, flip sign to the roll moment equation, as it was the convention used in the previous version
    std::vector<scalar> lambda_flipped(lambda);

    for (size_t i_element = 0; i_element < s.size()-1; ++i_element)
    {
        lambda_flipped[19 * i_element + 10] *= -1.0;
    }

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING].dissipation = 50.0;
    control_variables[decltype(car)::Chassis_type::control_names::throttle].dissipation = 20.0*8.0e-4;

    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts;

    Optimal_laptime<decltype(car)> opt_laptime(s, false, true, car, q, control_variables, zl_flipped, zu_flipped, lambda_flipped, opts);

    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_chicane_saved("data/f1_optimal_laptime_catalunya_chicane.xml", true);

    check_optimal_laptime(opt_laptime, opt_chicane_saved, s.size(), 1.0e-6);
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

    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1, 0.0), 0.0e0);

    control_variables[decltype(car)::Chassis_type::control_names::throttle] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 4.0e-7);

    control_variables[decltype(car)::Chassis_type::control_names::brake_bias] 
        = Optimal_laptime<decltype(car)>::create_dont_optimize();

    Optimal_laptime<decltype(car)> opt_laptime(s, false, true, car, {n+1,ss.inputs}, control_variables, {});

    opt_laptime.xml();

    // Check that the car accelerates
    const auto& IU = limebeer2014f1<scalar>::curvilinear<Track_by_arcs>::Chassis_type::input_names::velocity_x_mps;
    for (size_t i = 1; i < n; ++i)
    {
        EXPECT_TRUE(opt_laptime.inputs.at(i).at(IU) > opt_laptime.inputs.at(i-1).at(IU) );
    }

    // Check that steering is zero
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(Value(opt_laptime.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING].controls.at(i)), 0.0, 1.0e-13);

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_maximum_acceleration.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
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

    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 1.0e2);

    control_variables[decltype(car)::Chassis_type::control_names::throttle] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 2.0e-3);

    control_variables[decltype(car)::Chassis_type::control_names::brake_bias] 
        = Optimal_laptime<decltype(car)>::create_dont_optimize();

    Optimal_laptime<decltype(car)> opt_laptime(s, false, true, car, {n+1,ss.inputs}, control_variables, {});
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_ovaltrack_open.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
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

    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 1.0e2);

    control_variables[decltype(car)::Chassis_type::control_names::throttle] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 2.0e-3);

    control_variables[decltype(car)::Chassis_type::control_names::brake_bias] 
        = Optimal_laptime<decltype(car)>::create_dont_optimize();

    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, {});

    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_ovaltrack_closed.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Melbourne_direct)
{

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document database = {"./database/vehicles/f1/ferrari-2022-australia.xml", true};

    Xml_document melbourne_xml("./database/tracks/melbourne/melbourne.xml",true);
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

    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0);

    control_variables[decltype(car)::Chassis_type::control_names::throttle] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 20*8.0e-4);

    control_variables[decltype(car)::Chassis_type::control_names::brake_bias] 
        = Optimal_laptime<decltype(car)>::create_dont_optimize();

    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts; 
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_melbourne_700_direct.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Catalunya_chicane_derivative)
{

    // The chicane test uses the adapted mesh from i=533 to i=677

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_adapted.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    car.set_parameter("vehicle/front-axle/inertia", 1.00);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);

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
    std::vector<std::array<scalar,limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p::number_of_inputs>> q0(n,ss.inputs);
    
    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]),
                                                           std::vector<scalar>(n,0.0),
                                                           2.0e-1); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]),
                                                           std::vector<scalar>(n,0.0),
                                                           2.0e-5); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 
    
    Xml_document opt_full_lap("data/f1_optimal_laptime_catalunya_adapted.xml", true);
    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p> opt_laptime_full_lap(opt_full_lap);

    auto q_start = opt_laptime_full_lap.inputs[i0];
    auto u_start = opt_laptime_full_lap.controls.control_array_at_s(car,i0,s.front());

    q0.front()  = q_start;

    for (size_t i = 0; i < decltype(car)::number_of_controls; ++i)
    {
        if ( control_variables[i].optimal_control_type != Optimal_control_type::DONT_OPTIMIZE )
            control_variables[i].controls.front() = u_start[i];
    }

    Optimal_laptime<typename limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts;
    Optimal_laptime<decltype(car)> opt_laptime(s, false, false, car, q0, control_variables, opts);

    opt_laptime.xml();
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_chicane_derivative.xml", true);

    // Check the results with a saved simulation
    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-4);
}


TEST_F(F1_optimal_laptime_test, Melbourne_derivative)
{

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document database = {"./database/vehicles/f1/ferrari-2022-australia.xml", true};

    Xml_document melbourne_xml("./database/tracks/melbourne/melbourne.xml",true);
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
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]),
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-1); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]),
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-5); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 
 

    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts; opts.print_level = 0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, false, car, {n,ss.inputs}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_melbourne_700_derivative.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Melbourne_derivative_warm_start)
{

    if ( is_valgrind ) GTEST_SKIP();

    Xml_document database = {"./database/vehicles/f1/ferrari-2022-australia.xml", true};

    Xml_document melbourne_xml("./database/tracks/melbourne/melbourne.xml",true);
    Circuit_preprocessor melbourne_pproc(melbourne_xml);
    Track_by_polynomial melbourne(melbourne_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(melbourne);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    const auto& s = melbourne_pproc.s;
    const size_t n = s.size();

    Xml_document opt_saved("data/f1_optimal_laptime_melbourne_700_derivative.xml", true);
    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p> opt_laptime_saved(opt_saved);


    opt_laptime_saved.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING].dissipation = 1.0e-1;
    opt_laptime_saved.controls[decltype(car)::Chassis_type::control_names::throttle].dissipation = 1.0e-5;

    Optimal_laptime<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::Options opts;

    // Provisionally, flip the variable bound multipliers because the order has changed
    std::cout << "[WARNING] Flipped optimization data to fit new ordering, to remove!" << std::endl;
    std::vector<double> zl_flipped(opt_laptime_saved.optimization_data.zl);
    std::vector<double> zu_flipped(opt_laptime_saved.optimization_data.zu);

    for (size_t i_element = 0; i_element < s.size() ; ++i_element)
    {
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 0] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 0];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 1] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 1];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 2] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 2];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 3] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 3];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 4] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 4];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 5] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 5];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 6] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 6];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 7] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 9];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 8] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 10];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 9] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 11];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 10] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 12];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 11] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 7];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 12] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 8];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 13] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 13];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 14] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 14];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 15] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 15];
        zl_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 16] = opt_laptime_saved.optimization_data.zl[(decltype(car)::number_of_inputs + 3) * i_element + 16];

        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 0] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 0];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 1] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 1];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 2] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 2];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 3] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 3];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 4] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 4];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 5] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 5];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 6] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 6];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 7] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 9];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 8] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 10];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 9] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 11];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 10] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 12];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 11] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 7];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 12] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 8];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 13] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 13];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 14] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 14];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 15] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 15];
        zu_flipped[(decltype(car)::number_of_inputs + 3) * i_element + 16] = opt_laptime_saved.optimization_data.zu[(decltype(car)::number_of_inputs + 3) * i_element + 16];
    }

    // Provisionally, flip sign to the roll moment equation, as it was the convention used in the previous version
    std::vector<scalar> lambda_flipped(opt_laptime_saved.optimization_data.lambda);

    for (size_t i_element = 0; i_element < s.size(); ++i_element)
    {
        lambda_flipped[21 * i_element + 10] *= -1.0;
    }

    Optimal_laptime<decltype(car)> opt_laptime(s, true, false, car, opt_laptime_saved.inputs, 
        opt_laptime_saved.controls, 
        zl_flipped, zu_flipped, lambda_flipped, opts);

    opt_laptime.xml();

    EXPECT_EQ(opt_laptime.iter_count, 0);


    // Check the results with a saved simulation
    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, imola_adapted_hypermesh)
{
    Xml_document database = {"./database/vehicles/f1/redbull-2022-imola-wet.xml", true};

    Xml_document imola_xml("./database/tracks/imola/imola.xml",true);
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
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_hypermesh(std::vector<scalar>{0.0, 990.0, 1526.0, 1925.0, 2589.0, 3024.0, 3554.0},
                                                           std::vector<scalar>(7,ss.controls[decltype(car)::Chassis_type::control_names::brake_bias])); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};

    if ( is_valgrind )
    {
        opts.maximum_iterations = 1;
        opts.throw_if_fail = false;
    }

    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);
    opt_laptime.xml();

    if ( is_valgrind ) return;

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_imola_adapted.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Catalunya_engine_energy_limited)
{

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Engine energy limits require a higher value of the smooth throttle coefficient
    car.set_parameter("vehicle/rear-axle/smooth_throttle_coeff", 1.0e-2);
    car.set_parameter("vehicle/rear-axle/boost/maximum-power", 0.0);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = catalunya_pproc.s;

    const size_t n = s.size();

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};

    // Run something on valgrind but not the big thing
    if ( is_valgrind ) 
    {
        opts.maximum_iterations = 1;
        opts.throw_if_fail = false;
    }

    opts.integral_quantities = { {"engine-energy", 0.0, 24.0} };
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);
    opt_laptime.xml();

    if ( is_valgrind )
        return;

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_engine_energy_limited.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);

    EXPECT_NEAR(opt_laptime.integral_quantities.front().value, 24.0, 1.0e-4);
}


TEST_F(F1_optimal_laptime_test, Catalunya_tire_energy_limited)
{

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya.xml",true);
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
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 20.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};

    // Run something on valgrind but not the big thing
    if ( is_valgrind ) 
    {
        opts.maximum_iterations = 1;
        opts.throw_if_fail = false;
    }

    opts.integral_quantities = { {"tire-fl-energy", 0.0, 0.8}, {"tire-fr-energy", 0.0, 0.8}, {"tire-rl-energy", 0.0, 0.8}, {"tire-rr-energy", 0.0, 0.8} };
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);
    opt_laptime.xml();

    if ( is_valgrind )
        return;

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_tire_energy_limited.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);

    for (size_t i = 1; i < 5; ++i)
        EXPECT_TRUE(opt_laptime.integral_quantities[i].value < 0.8 + 1.0e-6);
}


TEST_F(F1_optimal_laptime_test, Catalunya_boost)
{
    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya.xml",true);
    Circuit_preprocessor catalunya_pproc(catalunya_xml);
    Track_by_polynomial catalunya(catalunya_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    car.set_parameter("vehicle/front-axle/inertia", 1.00);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = catalunya_pproc.s;

    const size_t n = s.size();

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 8.0e-4); 

    // boost: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::rear_axle_type::control_names::boost]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,0.0), 0.0); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};

    // Run something on valgrind but not the big thing
    if ( is_valgrind ) 
    {
        opts.maximum_iterations = 1;
        opts.throw_if_fail = false;
    }

    opts.integral_quantities = { {"boost-time", 0.0, 10.0} };
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);
    opt_laptime.xml();

    if ( is_valgrind )
        return;

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_boost.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);

    // boost
    auto boost_saved = opt_saved.get_element("optimal_laptime/control_variables/rear-axle.boost/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
    {
        EXPECT_NEAR(opt_laptime.controls[limebeer2014f1<scalar>::Chassis_t::rear_axle_type::control_names::boost].controls[i], boost_saved[i], 1.0e-6);
    }

    EXPECT_NEAR(opt_laptime.integral_quantities.back().value, 10.0, 1.0e-3);
}


TEST_F(F1_optimal_laptime_test, Catalunya_2022_3d)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/tracks/catalunya_2022/catalunya_2022_3d.xml",true);
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
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 50.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.print_level = 0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_2022_3d.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);

}

TEST_F(F1_optimal_laptime_test, Catalunya_3d)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya_3d.xml",true);

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
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 50.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.print_level = 0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_3d.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
}

TEST_F(F1_optimal_laptime_test, laguna_seca_3d)
{
    if ( is_valgrind ) GTEST_SKIP();

    Xml_document laguna_xml("./database/tracks/laguna_seca/laguna_seca_3d.xml",true);

    Circuit_preprocessor laguna_pproc(laguna_xml);
    Track_by_polynomial laguna(laguna_pproc);
    
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(laguna);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const auto& s = laguna_pproc.s;
    const auto& n = s.size();
    
    EXPECT_EQ(n, 500);

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::template Control_variables_type<>{};

    // steering wheel: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 50.0e0); 

    // throttle: optimize in the full mesh
    control_variables[decltype(car)::Chassis_type::control_names::throttle]
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::control_names::throttle]), 50.0*8.0e-4); 

    // brake bias: don't optimize
    control_variables[decltype(car)::Chassis_type::control_names::brake_bias]
        = Optimal_laptime<decltype(car)>::create_dont_optimize(); 

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.print_level = 0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);
    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_laguna_seca_3d.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n, 1.0e-6);
}

