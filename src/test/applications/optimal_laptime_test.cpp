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
    Xml_document database = {"./database/vehicles/kart/roberto-lot-kart-2016.xml", true};
    Xml_document results  = {"./data/optimal_laptime.xml", true};
    lot2016kart<CppAD::AD<scalar>>::cartesian car_cartesian = { database };
    lot2016kart<scalar>::cartesian car_cartesian_scalar = { database };
};


template<typename Dynamic_model_t>
static void check_optimal_laptime(const Optimal_laptime<Dynamic_model_t>& opt_laptime, Xml_document& opt_saved, const size_t n)
{

    Optimal_laptime<Dynamic_model_t> opt_laptime_saved(opt_saved);

//  const auto new_reference_file = std::regex_replace(opt_saved.get_file_name(), std::regex("data\\/"), "new-references/");
//  std::ifstream fin( new_reference_file );
 
//  if( fin.fail() )
//  {
//      opt_laptime.xml()->save(new_reference_file);
//  }

    EXPECT_NEAR(opt_laptime.laptime, opt_saved.get_element("optimal_laptime/laptime").get_value(scalar()), 5.0e-5);

    // omega axle
    auto omega_axle_saved = opt_saved.get_element("optimal_laptime/rear-axle.omega").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::Rear_axle_t::input_names::OMEGA_AXLE], omega_axle_saved[i], 5.0e-5);

    // u
    auto u_saved = opt_saved.get_element("optimal_laptime/chassis.velocity.x").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::Chassis_t::input_names::velocity_x_mps], u_saved[i], 1.0e-6);

    // v
    auto v_saved = opt_saved.get_element("optimal_laptime/chassis.velocity.y").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::Chassis_t::input_names::velocity_y_mps], v_saved[i], 1.0e-6);

    // omega
    auto omega_saved = opt_saved.get_element("optimal_laptime/chassis.omega.z").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::Chassis_t::input_names::yaw_rate_radps], omega_saved[i], 1.0e-6);

    // z
    auto z_saved = opt_saved.get_element("optimal_laptime/chassis.position.z").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::Chassis_t::input_names::Z], z_saved[i], 1.0e-6);

    // phi
    auto phi_saved = opt_saved.get_element("optimal_laptime/chassis.attitude.phi").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::Chassis_t::input_names::PHI], phi_saved[i], 1.0e-6);

    // mu
    auto mu_saved = opt_saved.get_element("optimal_laptime/chassis.attitude.mu").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::Chassis_t::input_names::MU], mu_saved[i], 1.0e-6);

    // dzdt
    auto dzdt_saved = opt_saved.get_element("optimal_laptime/chassis.velocity.z").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::Chassis_t::input_names::DZDT], dzdt_saved[i], 1.0e-6);

    // dphidt
    auto dphidt_saved = opt_saved.get_element("optimal_laptime/chassis.omega.x").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::Chassis_t::input_names::DPHIDT], dphidt_saved[i], 1.0e-6);

    // dmudt
    auto dmudt_saved = opt_saved.get_element("optimal_laptime/chassis.omega.y").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::Chassis_t::input_names::DMUDT], dmudt_saved[i], 1.0e-6);

    // time
    auto time_saved = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::curvilinear_a::Road_t::input_names::time], time_saved[i], 1.0e-6);

    // n
    auto n_saved = opt_saved.get_element("optimal_laptime/road.lateral-displacement").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::curvilinear_a::Road_t::input_names::lateral_displacement], n_saved[i], 1.0e-6);

    // alpha
    auto alpha_saved = opt_saved.get_element("optimal_laptime/road.track-heading-angle").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.inputs[i][lot2016kart<scalar>::curvilinear_a::Road_t::input_names::track_heading_angle], alpha_saved[i], 1.0e-6);

    // delta
    auto delta_saved = opt_saved.get_element("optimal_laptime/control_variables/front-axle.steering-angle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.controls[lot2016kart<scalar>::Front_axle_t::control_names::STEERING].controls[i], delta_saved[i], 1.0e-6);

    // torque
    auto torque_saved = opt_saved.get_element("optimal_laptime/control_variables/rear-axle.throttle/values").get_value(std::vector<scalar>());
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.controls[lot2016kart<scalar>::Rear_axle_t::control_names::TORQUE].controls[i], torque_saved[i], 5.0e-5);

}


TEST_F(Optimal_laptime_test, maximum_acceleration)
{


    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document straight_xml("./data/straight.xml",true);
    Track_by_arcs straight(straight_xml,false);

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, {straight});
    car.get_chassis().get_rear_axle().enable_direct_torque();

    constexpr const size_t n = 30;
    auto s = linspace(0.0, straight.get_total_length(), n+1);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dstates_dt[0]*0.2;
   
    ss.controls[1] = T;
    
    ss.dstates_dt = car_cartesian_scalar(ss.inputs, ss.controls, 0.0).dstates_dt;

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 0.0e0);

    control_variables[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.controls[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE]), 4.0e-5);

    // Compute optimal laptime 
    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.controls_lb[1] = -200.0;
    opts.controls_ub[1] =  200.0;
    Optimal_laptime<decltype(car)> opt_laptime(s, false, true, car, {n+1,ss.inputs}, control_variables, opts);

    // Perform tests ------------------------------------:-

    // Check that the car accelerates
    constexpr const size_t IU = lot2016kart<scalar>::curvilinear<Track_by_arcs>::Chassis_type::input_names::velocity_x_mps;
    for (size_t i = 1; i < n; ++i)
    {
        EXPECT_TRUE(opt_laptime.inputs.at(i).at(IU) > opt_laptime.inputs.at(i-1).at(IU) );
    }

    // Check that kappa is between [0.090,0.096] from the third onwards
    for (size_t i = 4; i < n; ++i)
    {
        car_cartesian_scalar(opt_laptime.inputs.at(i), opt_laptime.controls.control_array_at_s(car,i,s[i]), s[i]);
        const auto& kappa_left  = car_cartesian_scalar.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = car_cartesian_scalar.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        EXPECT_TRUE( (kappa_left > 0.090) && (kappa_left < 0.096) ) << ", with i = " << i;
        EXPECT_TRUE( (kappa_right > 0.090) && (kappa_right < 0.096) ) << ", with i = " << i;
    }

    // Check that steering is zero
    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(Value(opt_laptime.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING].controls.at(i)), 0.0, 1.0e-14);

    std::vector<scalar> T_saved = results.get_root_element().get_child("maximum_acceleration/T").get_value(std::vector<scalar>());

    EXPECT_EQ(opt_laptime.controls[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE].controls.size(), T_saved.size());

    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(opt_laptime.controls[1].controls[i], T_saved[i],1.0e-2);
}


TEST_F(Optimal_laptime_test, Ovaltrack_open)
{


    if ( is_valgrind ) GTEST_SKIP();

    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document ovaltrack_xml("./data/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,0.2,true);
    
    constexpr const size_t n = 400;
    auto s = linspace(0.0, ovaltrack.get_total_length(), n+1);

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dstates_dt[0]*0.2;
   
    ss.controls[1] = T;
    
    ss.dstates_dt = car_cartesian_scalar(ss.inputs, ss.controls, 0.0).dstates_dt;

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 1.0e2);

    control_variables[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n+1,ss.controls[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE]), 2.0e-7);

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.controls_lb[1] = -200.0;
    opts.controls_ub[1] =  200.0;
    Optimal_laptime<decltype(car)> opt_laptime(s, false, true, car, {n+1,ss.inputs}, control_variables, opts);

    opt_laptime.xml()->save("data/kart_ovaltrack_open.xml");
    Xml_document opt_saved("data/kart_ovaltrack_open.xml", true);
    check_optimal_laptime(opt_laptime, opt_saved, n);
}


TEST_F(Optimal_laptime_test, Ovaltrack_closed)
{


    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document ovaltrack_xml("./data/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,0.2,true);
    
    constexpr const size_t n = 100;
    auto s = linspace(0.0, ovaltrack.get_total_length(), n+1);
    s.pop_back();

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dstates_dt[0]*0.2;
   
    ss.controls[1] = T;
    
    ss.dstates_dt = car_cartesian_scalar(ss.inputs, ss.controls, 0.0).dstates_dt;

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 1.0e2);

    control_variables[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE]), 2.0e-7);

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.controls_lb[1] = -200.0;
    opts.controls_ub[1] =  200.0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);

    opt_laptime.xml()->save("data/kart_ovaltrack_closed.xml");
    Xml_document opt_saved("data/kart_ovaltrack_closed.xml", true);
    check_optimal_laptime(opt_laptime, opt_saved, n);
}


TEST_F(Optimal_laptime_test, Ovaltrack_derivative)
{


    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document ovaltrack_xml("./data/ovaltrack.xml",true);
    Track_by_arcs ovaltrack(ovaltrack_xml,0.2,true);
    
    constexpr const size_t n = 100;
    auto s = linspace(0.0, ovaltrack.get_total_length(), n+1);
    s.pop_back();

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(ovaltrack);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dstates_dt[0]*0.2;
   
    ss.controls[1] = T;
    
    ss.dstates_dt = car_cartesian_scalar(ss.inputs, ss.controls, 0.0).dstates_dt;

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-3);

    control_variables[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE]), 
                                                           std::vector<scalar>(n,0.0),
                                                           2.0e-9);

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.controls_lb[1] = -200.0;
    opts.controls_ub[1] =  200.0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, false, car, {n,ss.inputs}, control_variables, opts);

    // Check the results with a saved simulation
    opt_laptime.xml()->save("data/ovaltrack_derivative.xml");
    Xml_document opt_saved("data/ovaltrack_derivative.xml", true);
    check_optimal_laptime(opt_laptime, opt_saved, n);
}


TEST_F(Optimal_laptime_test, Catalunya_direct)
{


    if ( is_valgrind ) GTEST_SKIP();

    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document catalunya_xml("./data/catalunya.xml",true);
    Track_by_arcs catalunya(catalunya_xml,0.2,true);
    
    constexpr const size_t n = 500;
    auto s = linspace(0.0, catalunya.get_total_length(), n+1);
    s.pop_back();

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(catalunya);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dstates_dt[0]*0.2;
   
    ss.controls[1] = T;

    ss.dstates_dt = car_cartesian_scalar(ss.inputs, ss.controls, 0.0).dstates_dt;

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 
                                                           5.0e0);

    control_variables[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE]), 
                                                           8.0e-6);


    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.controls_lb[1] = -200.0;
    opts.controls_ub[1] =  200.0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, true, car, {n,ss.inputs}, control_variables, opts);

    std::vector<scalar> delta_saved = results.get_root_element().get_child("catalunya/delta").get_value(std::vector<scalar>());
    std::vector<scalar> T_saved = results.get_root_element().get_child("catalunya/T").get_value(std::vector<scalar>());

    EXPECT_EQ(opt_laptime.controls[0].controls.size(), delta_saved.size());
    EXPECT_EQ(opt_laptime.controls[1].controls.size(), T_saved.size());

    opt_laptime.xml();

    Xml_document opt_saved("data/optimal_laptime_catalunya_discrete.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n);

}


TEST_F(Optimal_laptime_test, Catalunya_derivative)
{


    #ifndef NDEBUG
        GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document catalunya_xml("./data/catalunya.xml",true);
    Track_by_arcs catalunya(catalunya_xml,0.2,true);
    
    constexpr const size_t n = 500;
    auto s = linspace(0.0, catalunya.get_total_length(), n+1);
    s.pop_back();

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(catalunya);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);
    car.get_chassis().get_rear_axle().enable_direct_torque();

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = -ss.dstates_dt[0]*0.2;
   
    ss.controls[1] = T;

    ss.dstates_dt = car_cartesian_scalar(ss.inputs, ss.controls, 0.0).dstates_dt;

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-2);

    control_variables[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE]), 
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-10);

    auto opts = Optimal_laptime<decltype(car)>::Options{};
    opts.controls_lb[1] = -200.0;
    opts.controls_ub[1] =  200.0;
    Optimal_laptime<decltype(car)> opt_laptime(s, true, false, car, {n,ss.inputs}, control_variables, opts);

    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/catalunya_derivative.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n);
}

TEST_F(Optimal_laptime_test, Catalunya_derivative_throttle)
{


    #ifndef NDEBUG
        GTEST_SKIP();
    #endif

    if ( is_valgrind ) GTEST_SKIP();

    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document catalunya_xml("./data/catalunya.xml",true);
    Track_by_arcs catalunya(catalunya_xml,0.2,true);
    
    constexpr const size_t n = 500;
    auto s = linspace(0.0, catalunya.get_total_length(), n+1);
    s.pop_back();

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs>::Road_t road(catalunya);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_arcs> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 50.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = 0.0;
   
    ss.controls[1] = T;

    ss.dstates_dt = car_cartesian_scalar(ss.inputs, ss.controls, 0.0).dstates_dt;

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-2);

    control_variables[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE]), 
                                                           std::vector<scalar>(n,0.0),
                                                           200*200.0*1.0e-10);

    Optimal_laptime<decltype(car)> opt_laptime(s, true, false, car, {n,ss.inputs}, control_variables, {});

    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/catalunya_derivative_throttle.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n);
}


TEST_F(Optimal_laptime_test, Vendrell)
{


    if ( is_valgrind ) GTEST_SKIP();

    Xml_document database = {"./database/vehicles/kart/rental-kart.xml", true};
    lot2016kart<CppAD::AD<scalar>>::cartesian car_cartesian = { database };
    lot2016kart<scalar>::cartesian car_cartesian_scalar = { database };

    car_cartesian.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_cartesian_scalar.get_chassis().get_rear_axle().enable_direct_torque();

    Xml_document vendrell_xml("./database/tracks/vendrell/vendrell.xml",true);
    Circuit_preprocessor vendrell_pproc(vendrell_xml);
    Track_by_polynomial vendrell(vendrell_pproc);
    
    constexpr const size_t n = 500;
    auto s = linspace(0.0, vendrell.get_total_length(), n+1);
    s.pop_back();

    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(vendrell);
    lot2016kart<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    // Start from the steady-state values at 50km/h-0g    
    const scalar v = 30.0*KMH;
    auto ss = Steady_state(car_cartesian).solve(v,0.0,0.0); 

    const scalar T = 0.0;
   
    ss.controls[1] = T;

    ss.dstates_dt = car_cartesian_scalar(ss.inputs, ss.controls, 0.0).dstates_dt;

    // Construct control variables
    auto control_variables = Optimal_laptime<decltype(car)>::Control_variables_type<>{};

    control_variables[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING]), 
                                                           std::vector<scalar>(n,0.0),
                                                           1.0e-2);

    control_variables[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE] 
        = Optimal_laptime<decltype(car)>::create_full_mesh(std::vector<scalar>(n,ss.controls[decltype(car)::Chassis_type::rear_axle_type::control_names::TORQUE]), 
                                                           std::vector<scalar>(n,0.0),
                                                           200*200.0*1.0e-10);


    Optimal_laptime<decltype(car)> opt_laptime(s, true, false, car, {n,ss.inputs}, control_variables, {});

    opt_laptime.xml();

    // Check the results with a saved simulation
    Xml_document opt_saved("data/vendrell_optimal.xml", true);

    check_optimal_laptime(opt_laptime, opt_saved, n);
}
