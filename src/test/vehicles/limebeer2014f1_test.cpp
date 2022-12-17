#include "gtest/gtest.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "src/core/applications/optimal_laptime.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "src/core/applications/steady_state.h"
#include "lion/propagators/crank_nicolson.h"
#include <unordered_map>
#include "src/main/c/fastestlapc.h"
#include "src/test/chassis/helper_functions.h"

// Define convenient aliases
using Front_left_tire_type  = limebeer2014f1<scalar>::Front_left_tire_type;
using Front_right_tire_type = limebeer2014f1<scalar>::Front_left_tire_type;
using Rear_left_tire_type   = limebeer2014f1<scalar>::Rear_left_tire_type;
using Rear_right_tire_type  = limebeer2014f1<scalar>::Rear_right_tire_type;

using Front_axle_t          = limebeer2014f1<scalar>::Front_axle_t;
using Rear_axle_t           = limebeer2014f1<scalar>::Rear_axle_t;

using Chassis_t             = limebeer2014f1<scalar>::Chassis_t;

using Road_t                = limebeer2014f1<scalar>::Road_cartesian_t;

class limebeer2014f1_test : public testing::Test
{
 protected:
    Xml_document database   = {"./database/vehicles/f1/limebeer-2014-f1.xml", true};
};


// Check expected indexes for state variables
static_assert(Front_axle_t::input_names::KAPPA_LEFT  == 0);
static_assert(Front_axle_t::input_names::KAPPA_RIGHT == 1);
static_assert(Rear_axle_t::input_names::KAPPA_LEFT   == 2);
static_assert(Rear_axle_t::input_names::KAPPA_RIGHT  == 3);
static_assert(Chassis_t::input_names::velocity_x_mps == 4);
static_assert(Chassis_t::input_names::velocity_y_mps == 5);
static_assert(Chassis_t::input_names::yaw_rate_radps == 6);
static_assert(Chassis_t::input_names::force_z_fl_g   == 7);
static_assert(Chassis_t::input_names::force_z_fr_g   == 8);
static_assert(Chassis_t::input_names::force_z_rl_g   == 9);
static_assert(Chassis_t::input_names::force_z_rr_g   == 10);
static_assert(Road_t::input_names::X                 == 11);
static_assert(Road_t::input_names::Y                 == 12);
static_assert(Road_t::input_names::PSI               == 13);
static_assert(limebeer2014f1<scalar>::cartesian::number_of_inputs == 14);

static_assert(Front_axle_t::state_names::angular_momentum_left  == 0);
static_assert(Front_axle_t::state_names::angular_momentum_right == 1);
static_assert(Rear_axle_t::state_names::angular_momentum_left   == 2);
static_assert(Rear_axle_t::state_names::angular_momentum_right  == 3);
static_assert(Chassis_t::state_names::com_velocity_x_mps  == 4);
static_assert(Chassis_t::state_names::com_velocity_y_mps  == 5);
static_assert(Chassis_t::state_names::yaw_rate_radps      == 6);
static_assert(Chassis_t::state_names::com_velocity_z_mps  == 7);
static_assert(Chassis_t::state_names::roll_angular_momentum_Nms == 8);
static_assert(Chassis_t::state_names::pitch_angular_momentum_Nms == 9);
static_assert(Chassis_t::state_names::roll_balance_equation_g == 10);
static_assert(Road_t::state_names::X                 == 11);
static_assert(Road_t::state_names::Y                 == 12);
static_assert(Road_t::state_names::PSI               == 13);
static_assert(limebeer2014f1<scalar>::cartesian::number_of_states == 14);


// Check expected indexes for control variables
static_assert(Front_axle_t::control_names::STEERING == 0);
static_assert(Rear_axle_t::control_names::boost     == 1);
static_assert(Chassis_t::control_names::throttle    == 2);
static_assert(Chassis_t::control_names::brake_bias  == 3);
static_assert(limebeer2014f1<scalar>::cartesian::number_of_controls == 4);


TEST_F(limebeer2014f1_test, indexes)
{
    EXPECT_EQ(Front_axle_t::input_names::KAPPA_LEFT  ,  0);
    EXPECT_EQ(Front_axle_t::input_names::KAPPA_RIGHT ,  1);
    EXPECT_EQ(Rear_axle_t::input_names::KAPPA_LEFT   ,  2);
    EXPECT_EQ(Rear_axle_t::input_names::KAPPA_RIGHT  ,  3);
    EXPECT_EQ(Chassis_t::input_names::velocity_x_mps ,  4);
    EXPECT_EQ(Chassis_t::input_names::velocity_y_mps ,  5);
    EXPECT_EQ(Chassis_t::input_names::yaw_rate_radps ,  6);
    EXPECT_EQ(Chassis_t::input_names::force_z_fl_g   ,  7);
    EXPECT_EQ(Chassis_t::input_names::force_z_fr_g   ,  8);
    EXPECT_EQ(Chassis_t::input_names::force_z_rl_g   ,  9);
    EXPECT_EQ(Chassis_t::input_names::force_z_rr_g   ,  10);
    EXPECT_EQ(Road_t::input_names::X                 ,  11);
    EXPECT_EQ(Road_t::input_names::Y                 ,  12);
    EXPECT_EQ(Road_t::input_names::PSI               ,  13);
    EXPECT_EQ(limebeer2014f1<scalar>::cartesian::number_of_inputs ,  14);

    EXPECT_EQ(Front_axle_t::state_names::angular_momentum_left  ,  0);
    EXPECT_EQ(Front_axle_t::state_names::angular_momentum_right ,  1);
    EXPECT_EQ(Rear_axle_t::state_names::angular_momentum_left   ,  2);
    EXPECT_EQ(Rear_axle_t::state_names::angular_momentum_right  ,  3);
    EXPECT_EQ(Chassis_t::state_names::com_velocity_x_mps  ,  4);
    EXPECT_EQ(Chassis_t::state_names::com_velocity_y_mps  ,  5);
    EXPECT_EQ(Chassis_t::state_names::yaw_rate_radps      ,  6);
    EXPECT_EQ(Chassis_t::state_names::com_velocity_z_mps  ,  7);
    EXPECT_EQ(Chassis_t::state_names::roll_angular_momentum_Nms ,  8);
    EXPECT_EQ(Chassis_t::state_names::pitch_angular_momentum_Nms ,  9);
    EXPECT_EQ(Chassis_t::state_names::roll_balance_equation_g ,  10);
    EXPECT_EQ(Road_t::state_names::X                 ,  11);
    EXPECT_EQ(Road_t::state_names::Y                 ,  12);
    EXPECT_EQ(Road_t::state_names::PSI               ,  13);
    EXPECT_EQ(limebeer2014f1<scalar>::cartesian::number_of_states ,  14);

    // Check expected indexes for control variables
    EXPECT_EQ(Front_axle_t::control_names::STEERING ,  0);
    EXPECT_EQ(Rear_axle_t::control_names::boost     ,  1);
    EXPECT_EQ(Chassis_t::control_names::throttle    ,  2);
    EXPECT_EQ(Chassis_t::control_names::brake_bias  ,  3);
    EXPECT_EQ(limebeer2014f1<scalar>::cartesian::number_of_controls ,  4);
}

TEST_F(limebeer2014f1_test, vehicle_from_xml_variable_names)
{
    limebeer2014f1<double>::curvilinear_p car_sc(database);
    auto [s_name, q_names, u_names] = car_sc.get_state_and_control_names();

    EXPECT_EQ(s_name, "road.arclength");
    EXPECT_EQ(q_names[Front_axle_t::input_names::KAPPA_LEFT]   , "front-axle.left-tire.kappa");
    EXPECT_EQ(q_names[Front_axle_t::input_names::KAPPA_RIGHT]  , "front-axle.right-tire.kappa");
    EXPECT_EQ(q_names[Rear_axle_t::input_names::KAPPA_LEFT]    , "rear-axle.left-tire.kappa");
    EXPECT_EQ(q_names[Rear_axle_t::input_names::KAPPA_RIGHT]   , "rear-axle.right-tire.kappa");
    EXPECT_EQ(q_names[Chassis_t::input_names::velocity_x_mps]  , "chassis.velocity.x");
    EXPECT_EQ(q_names[Chassis_t::input_names::velocity_y_mps]  , "chassis.velocity.y");
    EXPECT_EQ(q_names[Chassis_t::input_names::yaw_rate_radps]  , "chassis.omega.z");
    EXPECT_EQ(q_names[Chassis_t::input_names::force_z_fl_g]    , "chassis.force_z_fl_g");
    EXPECT_EQ(q_names[Chassis_t::input_names::force_z_fr_g]    , "chassis.force_z_fr_g");
    EXPECT_EQ(q_names[Chassis_t::input_names::force_z_rl_g]    , "chassis.force_z_rl_g");
    EXPECT_EQ(q_names[Chassis_t::input_names::force_z_rr_g]    , "chassis.force_z_rr_g");
    EXPECT_EQ(q_names[limebeer2014f1<double>::curvilinear_p::Road_type::input_names::time]  , "time");
    EXPECT_EQ(q_names[limebeer2014f1<double>::curvilinear_p::Road_type::input_names::lateral_displacement]     , "road.lateral-displacement");
    EXPECT_EQ(q_names[limebeer2014f1<double>::curvilinear_p::Road_type::input_names::track_heading_angle] , "road.track-heading-angle");

    EXPECT_EQ(u_names[Chassis_t::control_names::throttle]   , "chassis.throttle");
    EXPECT_EQ(u_names[Chassis_t::control_names::brake_bias] , "chassis.brake-bias");
    EXPECT_EQ(u_names[Front_axle_t::control_names::STEERING], "front-axle.steering-angle");
    EXPECT_EQ(u_names[Rear_axle_t::control_names::boost]    , "rear-axle.boost");
}

TEST_F(limebeer2014f1_test, vehicle_empty_variable_names)
{
    auto car = limebeer2014f1<double>::curvilinear_p{};

    auto [s_name, q_names, u_names] = car.get_state_and_control_names();

    EXPECT_EQ(s_name, "road.arclength");
    EXPECT_EQ(q_names[Front_axle_t::input_names::KAPPA_LEFT]   , "front-axle.left-tire.kappa");
    EXPECT_EQ(q_names[Front_axle_t::input_names::KAPPA_RIGHT]  , "front-axle.right-tire.kappa");
    EXPECT_EQ(q_names[Rear_axle_t::input_names::KAPPA_LEFT]    , "rear-axle.left-tire.kappa");
    EXPECT_EQ(q_names[Rear_axle_t::input_names::KAPPA_RIGHT]   , "rear-axle.right-tire.kappa");
    EXPECT_EQ(q_names[Chassis_t::input_names::velocity_x_mps]  , "chassis.velocity.x");
    EXPECT_EQ(q_names[Chassis_t::input_names::velocity_y_mps]  , "chassis.velocity.y");
    EXPECT_EQ(q_names[Chassis_t::input_names::yaw_rate_radps]  , "chassis.omega.z");
    EXPECT_EQ(q_names[Chassis_t::input_names::force_z_fl_g]    , "chassis.force_z_fl_g");
    EXPECT_EQ(q_names[Chassis_t::input_names::force_z_fr_g]    , "chassis.force_z_fr_g");
    EXPECT_EQ(q_names[Chassis_t::input_names::force_z_rl_g]    , "chassis.force_z_rl_g");
    EXPECT_EQ(q_names[Chassis_t::input_names::force_z_rr_g]    , "chassis.force_z_rr_g");
    EXPECT_EQ(q_names[limebeer2014f1<double>::curvilinear_p::Road_type::input_names::time]  , "time");
    EXPECT_EQ(q_names[limebeer2014f1<double>::curvilinear_p::Road_type::input_names::lateral_displacement]     , "road.lateral-displacement");
    EXPECT_EQ(q_names[limebeer2014f1<double>::curvilinear_p::Road_type::input_names::track_heading_angle] , "road.track-heading-angle");

    EXPECT_EQ(u_names[Chassis_t::control_names::throttle]   , "chassis.throttle");
    EXPECT_EQ(u_names[Chassis_t::control_names::brake_bias] , "chassis.brake-bias");
    EXPECT_EQ(u_names[Front_axle_t::control_names::STEERING], "front-axle.steering-angle");
    EXPECT_EQ(u_names[Rear_axle_t::control_names::boost]    , "rear-axle.boost");
}

TEST_F(limebeer2014f1_test, is_ready)
{
    limebeer2014f1<double>::curvilinear_p car_sc(database);
    EXPECT_TRUE(car_sc.is_ready());
}

TEST_F(limebeer2014f1_test, equations_classification)
{
    limebeer2014f1<double>::curvilinear_p car_sc(database);

    const auto [time_derivative_equations, algebraic_equations] = car_sc.classify_equations();

    EXPECT_EQ(time_derivative_equations.size(), 9);
    EXPECT_EQ(algebraic_equations.size(), 4);

    EXPECT_EQ(Front_axle_t::state_names::angular_momentum_left  ,  time_derivative_equations[0]);
    EXPECT_EQ(Front_axle_t::state_names::angular_momentum_right ,  time_derivative_equations[1]);
    EXPECT_EQ(Rear_axle_t::state_names::angular_momentum_left   ,  time_derivative_equations[2]);
    EXPECT_EQ(Rear_axle_t::state_names::angular_momentum_right  ,  time_derivative_equations[3]);
    EXPECT_EQ(Chassis_t::state_names::com_velocity_x_mps  ,        time_derivative_equations[4]);
    EXPECT_EQ(Chassis_t::state_names::com_velocity_y_mps  ,        time_derivative_equations[5]);
    EXPECT_EQ(Chassis_t::state_names::yaw_rate_radps      ,        time_derivative_equations[6]);
    EXPECT_EQ(Chassis_t::state_names::com_velocity_z_mps  ,        algebraic_equations[0]);
    EXPECT_EQ(Chassis_t::state_names::roll_angular_momentum_Nms ,  algebraic_equations[1]);
    EXPECT_EQ(Chassis_t::state_names::pitch_angular_momentum_Nms , algebraic_equations[2]);
    EXPECT_EQ(Chassis_t::state_names::roll_balance_equation_g,     algebraic_equations[3]);
    EXPECT_EQ(decltype(car_sc)::Road_type::state_names::lateral_displacement, time_derivative_equations[7]);
    EXPECT_EQ(decltype(car_sc)::Road_type::state_names::track_heading_angle, time_derivative_equations[8]);
}


TEST_F(limebeer2014f1_test, is_not_ready)
{
    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial> car;

    car.set_parameter("vehicle/front-axle/track", 1.46);
    car.set_parameter("vehicle/front-axle/inertia", 1.0);
//  car.set_parameter("vehicle/front-axle/smooth_throttle_coeff", 1.0e-5);  Let's take this one out, arbitrarily
    car.set_parameter("vehicle/front-axle/brakes/max_torque", 5000.0);

    car.set_parameter("vehicle/rear-axle/track", 1.46);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);
    car.set_parameter("vehicle/rear-axle/smooth_throttle_coeff", 1.0e-5);
    car.set_parameter("vehicle/rear-axle/differential_stiffness", 10.47);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque", 5000.0);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power", 735.499);

    car.set_parameter("vehicle/chassis/mass", 660.0);
    car.set_parameter("vehicle/chassis/inertia/Ixx", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Ixy", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Ixz", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Iyx", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Iyy", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Iyz", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Izx", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Izy", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Izz", 450.0);
    car.set_parameter("vehicle/chassis/aerodynamics/rho", 1.2);
    car.set_parameter("vehicle/chassis/aerodynamics/area", 1.5);
    car.set_parameter("vehicle/chassis/aerodynamics/cd", 0.9);
    car.set_parameter("vehicle/chassis/aerodynamics/cl", 3.0);
    car.set_parameter("vehicle/chassis/com/x", 0.0 );
    car.set_parameter("vehicle/chassis/com/y", 0.0 );
    car.set_parameter("vehicle/chassis/com/z", -0.3 );
    car.set_parameter("vehicle/chassis/front_axle/x",1.8);
    car.set_parameter("vehicle/chassis/front_axle/y",0.0);
    car.set_parameter("vehicle/chassis/front_axle/z",-0.33);
    car.set_parameter("vehicle/chassis/rear_axle/x", -1.6);
    car.set_parameter("vehicle/chassis/rear_axle/y",  0.0);
    car.set_parameter("vehicle/chassis/rear_axle/z", -0.33);
    car.set_parameter("vehicle/chassis/pressure_center/x", -0.1);
    car.set_parameter("vehicle/chassis/pressure_center/y",  0.0);
    car.set_parameter("vehicle/chassis/pressure_center/z", -0.3);
    car.set_parameter("vehicle/chassis/brake_bias", 0.6);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", 0.5);
    car.set_parameter("vehicle/chassis/Fz_max_ref2", 1.0);
    
    car.set_parameter("vehicle/front-tire/radius",0.330); 
    car.set_parameter("vehicle/front-tire/radial-stiffness",0.0);
    car.set_parameter("vehicle/front-tire/radial-damping",0.0);
    car.set_parameter("vehicle/front-tire/Fz-max-ref2", 1.0 );
    car.set_parameter("vehicle/front-tire/reference-load-1", 2000.0 ); 
    car.set_parameter("vehicle/front-tire/reference-load-2", 6000.0 ); 
    car.set_parameter("vehicle/front-tire/mu-x-max-1", 1.75 );
    car.set_parameter("vehicle/front-tire/mu-x-max-2", 1.40 );
    car.set_parameter("vehicle/front-tire/kappa-max-1", 0.11 );
    car.set_parameter("vehicle/front-tire/kappa-max-2", 0.10 );
    car.set_parameter("vehicle/front-tire/mu-y-max-1", 1.80 );
    car.set_parameter("vehicle/front-tire/mu-y-max-2", 1.45 );
    car.set_parameter("vehicle/front-tire/lambda-max-1", 9.0 );
    car.set_parameter("vehicle/front-tire/lambda-max-2", 8.0 );
    car.set_parameter("vehicle/front-tire/Qx", 1.9 );
    car.set_parameter("vehicle/front-tire/Qy", 1.9 );

    car.set_parameter("vehicle/rear-tire/radius",0.330); 
    car.set_parameter("vehicle/rear-tire/radial-stiffness",0.0);
    car.set_parameter("vehicle/rear-tire/radial-damping",0.0);
    car.set_parameter("vehicle/rear-tire/Fz-max-ref2", 1.0 );
    car.set_parameter("vehicle/rear-tire/reference-load-1", 2000.0 ); 
    car.set_parameter("vehicle/rear-tire/reference-load-2", 6000.0 ); 
    car.set_parameter("vehicle/rear-tire/mu-x-max-1", 1.75 );
    car.set_parameter("vehicle/rear-tire/mu-x-max-2", 1.40 );
    car.set_parameter("vehicle/rear-tire/kappa-max-1", 0.11 );
    car.set_parameter("vehicle/rear-tire/kappa-max-2", 0.10 );
    car.set_parameter("vehicle/rear-tire/mu-y-max-1", 1.80 );
    car.set_parameter("vehicle/rear-tire/mu-y-max-2", 1.45 );
    car.set_parameter("vehicle/rear-tire/lambda-max-1", 9.0);
    car.set_parameter("vehicle/rear-tire/lambda-max-2", 8.0);
    car.set_parameter("vehicle/rear-tire/Qx", 1.9 );
    car.set_parameter("vehicle/rear-tire/Qy", 1.9 );

    EXPECT_FALSE(car.is_ready());
}


TEST_F(limebeer2014f1_test, jacobian_autodiff)
{
    constexpr const auto& number_of_inputs           = limebeer2014f1<double>::cartesian::number_of_inputs;
    constexpr const auto& number_of_states           = limebeer2014f1<double>::cartesian::number_of_states;
    constexpr const auto& number_of_controls         = limebeer2014f1<double>::cartesian::number_of_controls;

    limebeer2014f1<CppAD::AD<double>>::cartesian car_ad(database);
    limebeer2014f1<double>::cartesian            car_sc(database);

    // Fill the inputs to the operator() of the vehicle. These inputs belong to a 0g trim at 300km/h
    std::array<double,14> inputs = {0.0, 0.0, 0.111971, 0.111971, 83.3333, 0.0, 0.0, -0.874103, -0.874103, -1.07386, -1.07386, 0.0, 0.0, 0.0};
    auto controls = car_ad.get_state_and_control_upper_lower_and_default_values().controls_def;
    controls[decltype(car_ad)::Chassis_type::control_names::throttle] = 0.644468;

    // Call operator()
    auto solution = car_ad.equations(inputs,controls,0.0);

    // Compute the numerical Jacobian
    std::array<std::array<double,number_of_states>, number_of_inputs + number_of_controls> numerical_jacobian_states;
    std::array<std::array<double,number_of_states>, number_of_inputs + number_of_controls> numerical_jacobian_dstates_dt;

    // derivatives w.r.t input_states
    for (size_t i = 0; i < number_of_inputs; ++i)
    {
        // Freeze u0 and add a perturbation on input_states 
        const double eps = std::max(1.0,fabs(inputs[i]))*1.0e-7;
        inputs[i] += eps;

        auto [states_eps, dstates_dt_eps] = car_sc(inputs,controls,0.0);

        inputs[i] -= 2*eps;

        auto [states_meps, dstates_dt_meps] = car_sc(inputs,controls,0.0);

        numerical_jacobian_states[i]              = (states_eps - states_meps)*(0.5/eps);
        numerical_jacobian_dstates_dt[i]          = (dstates_dt_eps - dstates_dt_meps)*(0.5/eps);

        inputs[i] += eps;
    }

    // derivatives w.r.t u
    for (size_t i = 0; i < number_of_controls; ++i)
    {
        // Freeze controls and add a perturbation on input_states 
        const double eps = std::max(1.0,fabs(controls[i]))*1.0e-7;
        controls[i] += eps;

        auto [states_eps, dstates_dt_eps] = car_sc(inputs,controls,0.0);

        controls[i] -= 2*eps;

        auto [states_meps, dstates_dt_meps] = car_sc(inputs,controls,0.0);

        numerical_jacobian_states[i+number_of_inputs]              = (states_eps - states_meps)*(0.5/eps);
        numerical_jacobian_dstates_dt[i+number_of_inputs]          = (dstates_dt_eps - dstates_dt_meps)*(0.5/eps);

        controls[i] += eps;
    }

    for (size_t i = 0; i < number_of_inputs + number_of_controls; ++i)
        for (size_t j = 0; j < number_of_states; ++j)
        {
            EXPECT_NEAR(numerical_jacobian_states[i][j], 
                        solution.jacobian_states[j][i], 2.0e-6*std::max(fabs(solution.jacobian_states[j][i]),1.0)) << "with i = " << i << " and j = " << j ;
        }


    for (size_t i = 0; i < number_of_inputs + number_of_controls; ++i)
        for (size_t j = 0; j < number_of_states; ++j)
        {
            EXPECT_NEAR(numerical_jacobian_dstates_dt[i][j], 
                        solution.jacobian_dstates_dt[j][i], 2.0e-6*std::max(fabs(solution.jacobian_dstates_dt[j][i]),1.0)) << "with i = " << i << " and j = " << j ;
        }
}

TEST_F(limebeer2014f1_test, jacobian_autodiff_random)
{
    constexpr const auto& number_of_inputs           = limebeer2014f1<double>::cartesian::number_of_inputs;
    constexpr const auto& number_of_states           = limebeer2014f1<double>::cartesian::number_of_states;
    constexpr const auto& number_of_controls         = limebeer2014f1<double>::cartesian::number_of_controls;

    limebeer2014f1<CppAD::AD<double>>::cartesian car_ad(database);
    limebeer2014f1<double>::cartesian            car_sc(database);

    // Fill the inputs to the operator() of the vehicle. These are made up inputs
    std::array<double,14> inputs = {-0.5, -0.8, 0.200000, 0.800000, 50.0000, -5.0, 0.4, -0.674103, -0.474103, -0.80386, -0.70386, 0.0, 0.0, 5.0*DEG};
    auto controls = car_ad.get_state_and_control_upper_lower_and_default_values().controls_def;
    controls[decltype(car_ad)::Chassis_type::front_axle_type::control_names::STEERING] = -2.0*DEG;
    controls[decltype(car_ad)::Chassis_type::control_names::throttle] = 0.100000;

    // Call operator()
    auto solution = car_ad.equations(inputs,controls,0.0);

    // Compute the numerical Jacobian
    std::array<std::array<double,number_of_states>, number_of_inputs + number_of_controls> numerical_jacobian_states;
    std::array<std::array<double,number_of_states>, number_of_inputs + number_of_controls> numerical_jacobian_dstates_dt;

    // derivatives w.r.t input_states
    for (size_t i = 0; i < number_of_inputs; ++i)
    {
        // Freeze u0 and add a perturbation on input_states 
        const double eps = std::max(1.0,fabs(inputs[i]))*1.0e-7;
        inputs[i] += eps;

        auto [states_eps, dstates_dt_eps] = car_sc(inputs,controls,0.0);

        inputs[i] -= 2*eps;

        auto [states_meps, dstates_dt_meps] = car_sc(inputs,controls,0.0);

        numerical_jacobian_states[i]              = (states_eps - states_meps)*(0.5/eps);
        numerical_jacobian_dstates_dt[i]          = (dstates_dt_eps - dstates_dt_meps)*(0.5/eps);

        inputs[i] += eps;
    }

    // derivatives w.r.t u
    for (size_t i = 0; i < number_of_controls; ++i)
    {
        // Freeze controls and add a perturbation on input_states 
        const double eps = std::max(1.0,fabs(controls[i]))*1.0e-7;
        controls[i] += eps;

        auto [states_eps, dstates_dt_eps] = car_sc(inputs,controls,0.0);

        controls[i] -= 2*eps;

        auto [states_meps, dstates_dt_meps] = car_sc(inputs,controls,0.0);

        numerical_jacobian_states[i+number_of_inputs]              = (states_eps - states_meps)*(0.5/eps);
        numerical_jacobian_dstates_dt[i+number_of_inputs]          = (dstates_dt_eps - dstates_dt_meps)*(0.5/eps);

        controls[i] += eps;
    }

    for (size_t i = 0; i < number_of_inputs + number_of_controls; ++i)
        for (size_t j = 0; j < number_of_states; ++j)
        {
            EXPECT_NEAR(numerical_jacobian_states[i][j], 
                        solution.jacobian_states[j][i], 2.0e-6*std::max(fabs(solution.jacobian_states[j][i]),1.0)) << "with i = " << i << " and j = " << j ;
        }


    for (size_t i = 0; i < number_of_inputs + number_of_controls; ++i)
        for (size_t j = 0; j < number_of_states; ++j)
        {
            EXPECT_NEAR(numerical_jacobian_dstates_dt[i][j], 
                        solution.jacobian_dstates_dt[j][i], 2.0e-6*std::max(fabs(solution.jacobian_dstates_dt[j][i]),1.0)) << "with i = " << i << " and j = " << j ;
        }
}



TEST_F(limebeer2014f1_test, set_parameter)
{
    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya.xml",true);
    Track_by_polynomial catalunya(catalunya_xml);
    

    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial> car(road);
    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial> car_correct(database, road);

    car.set_parameter("vehicle/front-axle/track", 1.46);
    car.set_parameter("vehicle/front-axle/inertia", 0.0);
    car.set_parameter("vehicle/front-axle/smooth_throttle_coeff", 1.0e-5);
    car.set_parameter("vehicle/front-axle/brakes/max_torque", 5000.0);

    car.set_parameter("vehicle/rear-axle/track", 1.46);
    car.set_parameter("vehicle/rear-axle/inertia", 0.0);
    car.set_parameter("vehicle/rear-axle/smooth_throttle_coeff", 1.0e-5);
    car.set_parameter("vehicle/rear-axle/differential_stiffness", 10.47);
    car.set_parameter("vehicle/rear-axle/brakes/max_torque", 5000.0);
    car.set_parameter("vehicle/rear-axle/engine/maximum-power", 735.499);
    car.set_parameter("vehicle/rear-axle/boost/maximum-power", 120.000);

    car.set_parameter("vehicle/chassis/mass", 660.0);
    car.set_parameter("vehicle/chassis/inertia/Ixx", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Ixy", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Ixz", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Iyx", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Iyy", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Iyz", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Izx", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Izy", 0.0);
    car.set_parameter("vehicle/chassis/inertia/Izz", 450.0);
    car.set_parameter("vehicle/chassis/aerodynamics/rho", 1.2);
    car.set_parameter("vehicle/chassis/aerodynamics/area", 1.5);
    car.set_parameter("vehicle/chassis/aerodynamics/cd", 0.9);
    car.set_parameter("vehicle/chassis/aerodynamics/cl", 3.0);
    car.set_parameter("vehicle/chassis/com/x", 0.0 );
    car.set_parameter("vehicle/chassis/com/y", 0.0 );
    car.set_parameter("vehicle/chassis/com/z", -0.3 );
    car.set_parameter("vehicle/chassis/front_axle/x",1.8);
    car.set_parameter("vehicle/chassis/front_axle/y",0.0);
    car.set_parameter("vehicle/chassis/front_axle/z",-0.33);
    car.set_parameter("vehicle/chassis/rear_axle/x", -1.6);
    car.set_parameter("vehicle/chassis/rear_axle/y",  0.0);
    car.set_parameter("vehicle/chassis/rear_axle/z", -0.33);
    car.set_parameter("vehicle/chassis/pressure_center/x", -0.1);
    car.set_parameter("vehicle/chassis/pressure_center/y",  0.0);
    car.set_parameter("vehicle/chassis/pressure_center/z", -0.3);
    car.set_parameter("vehicle/chassis/brake_bias", 0.6);
    car.set_parameter("vehicle/chassis/roll_balance_coefficient", 0.5);
    car.set_parameter("vehicle/chassis/Fz_max_ref2", 1.0);
    
    car.set_parameter("vehicle/front-tire/radius",0.330); 
    car.set_parameter("vehicle/front-tire/radial-stiffness",0.0);
    car.set_parameter("vehicle/front-tire/radial-damping",0.0);
    car.set_parameter("vehicle/front-tire/Fz-max-ref2", 1.0 );
    car.set_parameter("vehicle/front-tire/reference-load-1", 2000.0 ); 
    car.set_parameter("vehicle/front-tire/reference-load-2", 6000.0 ); 
    car.set_parameter("vehicle/front-tire/mu-x-max-1", 1.75 );
    car.set_parameter("vehicle/front-tire/mu-x-max-2", 1.40 );
    car.set_parameter("vehicle/front-tire/kappa-max-1", 0.11 );
    car.set_parameter("vehicle/front-tire/kappa-max-2", 0.10 );
    car.set_parameter("vehicle/front-tire/mu-y-max-1", 1.80 );
    car.set_parameter("vehicle/front-tire/mu-y-max-2", 1.45 );
    car.set_parameter("vehicle/front-tire/lambda-max-1", 9.0 );
    car.set_parameter("vehicle/front-tire/lambda-max-2", 8.0 );
    car.set_parameter("vehicle/front-tire/Qx", 1.9 );
    car.set_parameter("vehicle/front-tire/Qy", 1.9 );

    car.set_parameter("vehicle/rear-tire/radius",0.330); 
    car.set_parameter("vehicle/rear-tire/radial-stiffness",0.0);
    car.set_parameter("vehicle/rear-tire/radial-damping",0.0);
    car.set_parameter("vehicle/rear-tire/Fz-max-ref2", 1.0 );
    car.set_parameter("vehicle/rear-tire/reference-load-1", 2000.0 ); 
    car.set_parameter("vehicle/rear-tire/reference-load-2", 6000.0 ); 
    car.set_parameter("vehicle/rear-tire/mu-x-max-1", 1.75 );
    car.set_parameter("vehicle/rear-tire/mu-x-max-2", 1.40 );
    car.set_parameter("vehicle/rear-tire/kappa-max-1", 0.11 );
    car.set_parameter("vehicle/rear-tire/kappa-max-2", 0.10 );
    car.set_parameter("vehicle/rear-tire/mu-y-max-1", 1.80 );
    car.set_parameter("vehicle/rear-tire/mu-y-max-2", 1.45 );
    car.set_parameter("vehicle/rear-tire/lambda-max-1", 9.0);
    car.set_parameter("vehicle/rear-tire/lambda-max-2", 8.0);
    car.set_parameter("vehicle/rear-tire/Qx", 1.9 );
    car.set_parameter("vehicle/rear-tire/Qy", 1.9 );

    std::array<double,14> q0;
    auto u0 = car.get_state_and_control_upper_lower_and_default_values().controls_def;
    
    constexpr const size_t n = 500;

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_discrete.xml", true);

    auto arclength_saved = opt_saved.get_element("optimal_laptime/road.arclength").get_value(std::vector<scalar>());
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/front-axle.left-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/front-axle.right-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/rear-axle.left-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/rear-axle.right-tire.kappa").get_value(std::vector<scalar>());
    auto u_saved        = opt_saved.get_element("optimal_laptime/chassis.velocity.x").get_value(std::vector<scalar>());
    auto v_saved        = opt_saved.get_element("optimal_laptime/chassis.velocity.y").get_value(std::vector<scalar>());
    auto omega_saved    = opt_saved.get_element("optimal_laptime/chassis.omega.z").get_value(std::vector<scalar>());
    auto time_saved     = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    auto n_saved        = opt_saved.get_element("optimal_laptime/road.lateral-displacement").get_value(std::vector<scalar>());
    auto alpha_saved    = opt_saved.get_element("optimal_laptime/road.track-heading-angle").get_value(std::vector<scalar>());
    auto delta_saved    = opt_saved.get_element("optimal_laptime/control_variables/front-axle.steering-angle/values").get_value(std::vector<scalar>());
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/chassis.throttle/values").get_value(std::vector<scalar>());
    auto Fz_fl_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_fl_g").get_value(std::vector<scalar>());
    auto Fz_fr_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_fr_g").get_value(std::vector<scalar>());
    auto Fz_rl_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_rl_g").get_value(std::vector<scalar>());
    auto Fz_rr_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_rr_g").get_value(std::vector<scalar>());

    for (size_t i = 0; i < n; ++i)
    {
        q0 = {kappa_fl_saved[i], kappa_fr_saved[i], kappa_rl_saved[i], kappa_rr_saved[i], u_saved[i], v_saved[i], omega_saved[i],
              Fz_fl_saved[i], Fz_fr_saved[i], Fz_rl_saved[i], Fz_rr_saved[i],
              time_saved[i], n_saved[i], alpha_saved[i]};

        u0[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] = delta_saved[i];
        u0[decltype(car)::Chassis_type::control_names::throttle] = throttle_saved[i];

        auto [states, dqdt] = car(q0,u0,arclength_saved[i]);
        auto [states_c, dqdt_c] = car_correct(q0,u0,arclength_saved[i]);

        for (size_t j = 0; j < states.size(); ++j)
            EXPECT_NEAR(states[j], states_c[j], 2.0e-15);

        for (size_t j = 0; j < dqdt.size(); ++j)
            EXPECT_NEAR(dqdt[j], dqdt_c[j], 2.0e-15);
    }

    EXPECT_TRUE(car.is_ready());
}

#ifdef TEST_LIBFASTESTLAPC
std::unordered_map<std::string,limebeer2014f1_all>& get_table_f1_3dof();
std::unordered_map<std::string,Track_by_polynomial>& get_table_track();

TEST_F(limebeer2014f1_test, set_parameter_c_api)
{
    set_print_level(0);
    create_vehicle_empty("vehicle_test", "f1-3dof");
    create_track_from_xml("track_test", "./database/tracks/catalunya/catalunya.xml");
    vehicle_change_track("vehicle_test", "track_test");
    
    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya.xml",true);
    Track_by_polynomial catalunya(catalunya_xml);
    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial> car_correct(database, road);

    vehicle_set_parameter("vehicle_test", "vehicle/front-axle/track", 1.46);
    vehicle_set_parameter("vehicle_test", "vehicle/front-axle/inertia", 0.0);
    vehicle_set_parameter("vehicle_test", "vehicle/front-axle/smooth_throttle_coeff", 1.0e-5);
    vehicle_set_parameter("vehicle_test", "vehicle/front-axle/brakes/max_torque", 5000.0);

    vehicle_set_parameter("vehicle_test", "vehicle/rear-axle/track", 1.46);
    vehicle_set_parameter("vehicle_test", "vehicle/rear-axle/inertia", 0.0);
    vehicle_set_parameter("vehicle_test", "vehicle/rear-axle/smooth_throttle_coeff", 1.0e-5);
    vehicle_set_parameter("vehicle_test", "vehicle/rear-axle/differential_stiffness", 10.47);
    vehicle_set_parameter("vehicle_test", "vehicle/rear-axle/brakes/max_torque", 5000.0);
    vehicle_set_parameter("vehicle_test", "vehicle/rear-axle/engine/maximum-power", 735.499);
    vehicle_set_parameter("vehicle_test", "vehicle/rear-axle/boost/maximum-power", 120.000);

    vehicle_set_parameter("vehicle_test", "vehicle/chassis/mass", 660.0);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/inertia/Izz", 450.0);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/aerodynamics/rho", 1.2);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/aerodynamics/area", 1.5);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/aerodynamics/cd", 0.9);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/aerodynamics/cl", 3.0);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/com/x", 0.0 );
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/com/y", 0.0 );
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/com/z", -0.3 );
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/front_axle/x",1.8);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/front_axle/y",0.0);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/front_axle/z",-0.33);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/rear_axle/x", -1.6);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/rear_axle/y",  0.0);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/rear_axle/z", -0.33);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/pressure_center/x", -0.1);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/pressure_center/y",  0.0);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/pressure_center/z", -0.3);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/brake_bias", 0.6);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/roll_balance_coefficient", 0.5);
    vehicle_set_parameter("vehicle_test", "vehicle/chassis/Fz_max_ref2", 1.0);
    
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/radius",0.330); 
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/radial-stiffness",0.0);
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/radial-damping",0.0);
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/Fz-max-ref2", 1.0 );
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/reference-load-1", 2000.0 ); 
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/reference-load-2", 6000.0 ); 
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/mu-x-max-1", 1.75 );
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/mu-x-max-2", 1.40 );
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/kappa-max-1", 0.11 );
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/kappa-max-2", 0.10 );
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/mu-y-max-1", 1.80 );
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/mu-y-max-2", 1.45 );
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/lambda-max-1", 9.0 );
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/lambda-max-2", 8.0 );
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/Qx", 1.9 );
    vehicle_set_parameter("vehicle_test", "vehicle/front-tire/Qy", 1.9 );

    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/radius",0.330); 
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/radial-stiffness",0.0);
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/radial-damping",0.0);
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/Fz-max-ref2", 1.0 );
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/reference-load-1", 2000.0 ); 
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/reference-load-2", 6000.0 ); 
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/mu-x-max-1", 1.75 );
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/mu-x-max-2", 1.40 );
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/kappa-max-1", 0.11 );
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/kappa-max-2", 0.10 );
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/mu-y-max-1", 1.80 );
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/mu-y-max-2", 1.45 );
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/lambda-max-1", 9.0);
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/lambda-max-2", 8.0);
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/Qx", 1.9 );
    vehicle_set_parameter("vehicle_test", "vehicle/rear-tire/Qy", 1.9 );

    EXPECT_EQ(get_table_f1_3dof().count("vehicle_test"), 1);

    auto& car = get_table_f1_3dof().at("vehicle_test").get_curvilinear_scalar_car();

    std::array<double,14> q0;
    auto u0 = car.get_state_and_control_upper_lower_and_default_values().controls_def;
    
    constexpr const size_t n = 500;

    // Check the results with a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_discrete.xml", true);

    auto arclength_saved = opt_saved.get_element("optimal_laptime/road.arclength").get_value(std::vector<scalar>());
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/front-axle.left-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/front-axle.right-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/rear-axle.left-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/rear-axle.right-tire.kappa").get_value(std::vector<scalar>());
    auto u_saved        = opt_saved.get_element("optimal_laptime/chassis.velocity.x").get_value(std::vector<scalar>());
    auto v_saved        = opt_saved.get_element("optimal_laptime/chassis.velocity.y").get_value(std::vector<scalar>());
    auto omega_saved    = opt_saved.get_element("optimal_laptime/chassis.omega.z").get_value(std::vector<scalar>());
    auto time_saved     = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    auto n_saved        = opt_saved.get_element("optimal_laptime/road.lateral-displacement").get_value(std::vector<scalar>());
    auto alpha_saved    = opt_saved.get_element("optimal_laptime/road.track-heading-angle").get_value(std::vector<scalar>());
    auto delta_saved    = opt_saved.get_element("optimal_laptime/control_variables/front-axle.steering-angle/values").get_value(std::vector<scalar>());
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/chassis.throttle/values").get_value(std::vector<scalar>());
    auto Fz_fl_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_fl_g").get_value(std::vector<scalar>());
    auto Fz_fr_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_fr_g").get_value(std::vector<scalar>());
    auto Fz_rl_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_rl_g").get_value(std::vector<scalar>());
    auto Fz_rr_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_rr_g").get_value(std::vector<scalar>());

    for (size_t i = 0; i < n; ++i)
    {
        q0 = {kappa_fl_saved[i], kappa_fr_saved[i], kappa_rl_saved[i], kappa_rr_saved[i], u_saved[i], v_saved[i], omega_saved[i],
              Fz_fl_saved[i], Fz_fr_saved[i], Fz_rl_saved[i], Fz_rr_saved[i],
              time_saved[i], n_saved[i], alpha_saved[i]};

        u0[decltype(car_correct)::Chassis_type::front_axle_type::control_names::STEERING] = delta_saved[i];
        u0[decltype(car_correct)::Chassis_type::control_names::throttle] = throttle_saved[i];

        auto [states, dqdt] = car(q0,u0,arclength_saved[i]);
        auto [states_c, dqdt_c] = car_correct(q0,u0,arclength_saved[i]);

        for (size_t j = 0; j < states.size(); ++j)
            EXPECT_NEAR(states[j], states_c[j], 2.0e-15);

        for (size_t j = 0; j < dqdt.size(); ++j)
            EXPECT_NEAR(dqdt[j], dqdt_c[j], 2.0e-15);
    }

    delete_variable("vehicle_test");
    delete_variable("track_test");
    EXPECT_EQ(get_table_f1_3dof().count("vehicle_test"), 0);
    EXPECT_EQ(get_table_track().count("track_test"), 0);
}

#endif


TEST_F(limebeer2014f1_test,propagation_crank_nicolson)
{
    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya.xml",true);
    Track_by_polynomial catalunya(catalunya_xml);

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial>::Road_t road_sc(catalunya);
    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial> car_sc(database, road_sc);

    car_sc.set_parameter("vehicle/front-axle/inertia", 1.00);
    car_sc.set_parameter("vehicle/rear-axle/inertia", 1.55);

    car.set_parameter("vehicle/front-axle/inertia", 1.00);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);

    // Get the results from a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_discrete_with_wheel_inertia.xml", true);

    auto arclength_saved = opt_saved.get_element("optimal_laptime/road.arclength").get_value(std::vector<scalar>());
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/front-axle.left-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/front-axle.right-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/rear-axle.left-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/rear-axle.right-tire.kappa").get_value(std::vector<scalar>());
    auto u_saved        = opt_saved.get_element("optimal_laptime/chassis.velocity.x").get_value(std::vector<scalar>());
    auto v_saved        = opt_saved.get_element("optimal_laptime/chassis.velocity.y").get_value(std::vector<scalar>());
    auto omega_saved    = opt_saved.get_element("optimal_laptime/chassis.omega.z").get_value(std::vector<scalar>());
    auto time_saved     = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    auto n_saved        = opt_saved.get_element("optimal_laptime/road.lateral-displacement").get_value(std::vector<scalar>());
    auto alpha_saved    = opt_saved.get_element("optimal_laptime/road.track-heading-angle").get_value(std::vector<scalar>());
    auto delta_saved    = opt_saved.get_element("optimal_laptime/control_variables/front-axle.steering-angle/values").get_value(std::vector<scalar>());
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/chassis.throttle/values").get_value(std::vector<scalar>());
    auto Fz_fl_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_fl_g").get_value(std::vector<scalar>());
    auto Fz_fr_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_fr_g").get_value(std::vector<scalar>());
    auto Fz_rl_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_rl_g").get_value(std::vector<scalar>());
    auto Fz_rr_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_rr_g").get_value(std::vector<scalar>());

    // Take a Crank-Nicolson step on i = 112
    const size_t i_start = 112;
    std::array<scalar,limebeer2014f1<scalar>::curvilinear_p::number_of_inputs> q = 
        {kappa_fl_saved[i_start], kappa_fr_saved[i_start], kappa_rl_saved[i_start], kappa_rr_saved[i_start],
         u_saved[i_start], v_saved[i_start], omega_saved[i_start], 
         Fz_fl_saved[i_start], Fz_fr_saved[i_start], Fz_rl_saved[i_start], Fz_rr_saved[i_start],
         time_saved[i_start], n_saved[i_start], alpha_saved[i_start]};

    auto u = car.get_state_and_control_upper_lower_and_default_values().controls_def;

    u[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] = delta_saved[i_start];
    u[decltype(car)::Chassis_type::control_names::throttle] = throttle_saved[i_start];

    std::array<scalar,limebeer2014f1<scalar>::curvilinear_p::number_of_inputs> q_next = 
        {kappa_fl_saved[i_start + 1], kappa_fr_saved[i_start + 1], kappa_rl_saved[i_start + 1], kappa_rr_saved[i_start + 1],
         u_saved[i_start + 1], v_saved[i_start + 1], omega_saved[i_start + 1], 
         Fz_fl_saved[i_start + 1], Fz_fr_saved[i_start + 1], Fz_rl_saved[i_start + 1], Fz_rr_saved[i_start + 1],
         time_saved[i_start + 1], n_saved[i_start + 1], alpha_saved[i_start + 1]};

    auto u_next = car.get_state_and_control_upper_lower_and_default_values().controls_def;

    u_next[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] = delta_saved[i_start+1];
    u_next[decltype(car)::Chassis_type::control_names::throttle] = throttle_saved[i_start+1];

    scalar s = arclength_saved[i_start];
    scalar s_next = arclength_saved[i_start+1];

    // Check the scheme on the original point
    auto [states_ini, dqdt_ini] = car_sc(q,u,s);
    auto [states_fin, dqdt_fin] = car_sc(q_next,u_next,s_next);

    for (size_t i = 0; i < 13; ++i)
        EXPECT_NEAR(states_fin[i], states_ini[i] + 0.5*(s_next-s)*(dqdt_ini[i] + dqdt_fin[i]),1.0e-8) << " with i = " << i;

    Crank_nicolson<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::take_step(car, u, u_next, q, s, s_next-s, {});

    for (size_t i = 0; i < 14; ++i)
        EXPECT_NEAR(q[i],q_next[i],1.0e-10) << ", with i = " << i;
}


TEST_F(limebeer2014f1_test,propagation_crank_nicolson_corner_exit)
{
    Xml_document catalunya_xml("./database/tracks/catalunya/catalunya.xml",true);
    Track_by_polynomial catalunya(catalunya_xml);

    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial>::Road_t road(catalunya);
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear<Track_by_polynomial> car(database, road);

    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial>::Road_t road_sc(catalunya);
    limebeer2014f1<scalar>::curvilinear<Track_by_polynomial> car_sc(database, road_sc);

    car_sc.set_parameter("vehicle/front-axle/inertia", 1.00);
    car_sc.set_parameter("vehicle/rear-axle/inertia", 1.55);

    car.set_parameter("vehicle/front-axle/inertia", 1.00);
    car.set_parameter("vehicle/rear-axle/inertia", 1.55);

    // Get the results from a saved simulation
    Xml_document opt_saved("data/f1_optimal_laptime_catalunya_discrete_with_wheel_inertia.xml", true);

    auto arclength_saved = opt_saved.get_element("optimal_laptime/road.arclength").get_value(std::vector<scalar>());
    auto kappa_fl_saved = opt_saved.get_element("optimal_laptime/front-axle.left-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_fr_saved = opt_saved.get_element("optimal_laptime/front-axle.right-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_rl_saved = opt_saved.get_element("optimal_laptime/rear-axle.left-tire.kappa").get_value(std::vector<scalar>());
    auto kappa_rr_saved = opt_saved.get_element("optimal_laptime/rear-axle.right-tire.kappa").get_value(std::vector<scalar>());
    auto u_saved        = opt_saved.get_element("optimal_laptime/chassis.velocity.x").get_value(std::vector<scalar>());
    auto v_saved        = opt_saved.get_element("optimal_laptime/chassis.velocity.y").get_value(std::vector<scalar>());
    auto omega_saved    = opt_saved.get_element("optimal_laptime/chassis.omega.z").get_value(std::vector<scalar>());
    auto time_saved     = opt_saved.get_element("optimal_laptime/time").get_value(std::vector<scalar>());
    auto n_saved        = opt_saved.get_element("optimal_laptime/road.lateral-displacement").get_value(std::vector<scalar>());
    auto alpha_saved    = opt_saved.get_element("optimal_laptime/road.track-heading-angle").get_value(std::vector<scalar>());
    auto delta_saved    = opt_saved.get_element("optimal_laptime/control_variables/front-axle.steering-angle/values").get_value(std::vector<scalar>());
    auto throttle_saved = opt_saved.get_element("optimal_laptime/control_variables/chassis.throttle/values").get_value(std::vector<scalar>());
    auto Fz_fl_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_fl_g").get_value(std::vector<scalar>());
    auto Fz_fr_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_fr_g").get_value(std::vector<scalar>());
    auto Fz_rl_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_rl_g").get_value(std::vector<scalar>());
    auto Fz_rr_saved    = opt_saved.get_element("optimal_laptime/chassis.force_z_rr_g").get_value(std::vector<scalar>());

    // Take a Crank-Nicolson step on i = 325
    const size_t i_start = 101;
    std::array<scalar,limebeer2014f1<scalar>::curvilinear_p::number_of_inputs> q = 
        {kappa_fl_saved[i_start], kappa_fr_saved[i_start], kappa_rl_saved[i_start], kappa_rr_saved[i_start],
         u_saved[i_start], v_saved[i_start], omega_saved[i_start], 
         Fz_fl_saved[i_start], Fz_fr_saved[i_start], Fz_rl_saved[i_start], Fz_rr_saved[i_start],
         time_saved[i_start], n_saved[i_start], alpha_saved[i_start]};

    auto u = car.get_state_and_control_upper_lower_and_default_values().controls_def;

    u[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] = delta_saved[i_start];
    u[decltype(car)::Chassis_type::control_names::throttle] = throttle_saved[i_start];

    std::array<scalar,limebeer2014f1<scalar>::curvilinear_p::number_of_inputs> q_next = 
        {kappa_fl_saved[i_start + 1], kappa_fr_saved[i_start + 1], kappa_rl_saved[i_start + 1], kappa_rr_saved[i_start + 1],
         u_saved[i_start + 1], v_saved[i_start + 1], omega_saved[i_start + 1], 
         Fz_fl_saved[i_start + 1], Fz_fr_saved[i_start + 1], Fz_rl_saved[i_start + 1], Fz_rr_saved[i_start + 1],
         time_saved[i_start + 1], n_saved[i_start + 1], alpha_saved[i_start + 1]};

    auto u_next = car.get_state_and_control_upper_lower_and_default_values().controls_def;

    u_next[decltype(car)::Chassis_type::front_axle_type::control_names::STEERING] = delta_saved[i_start+1];
    u_next[decltype(car)::Chassis_type::control_names::throttle] = throttle_saved[i_start+1];

    scalar s = arclength_saved[i_start];
    scalar s_next = arclength_saved[i_start+1];

    // Check the scheme on the original point
    auto [states_ini, dqdt_ini] = car_sc(q,u,s);
    auto [states_fin, dqdt_fin] = car_sc(q_next,u_next,s_next);

    for (size_t i = 0; i < 13; ++i)
        EXPECT_NEAR(states_fin[i], states_ini[i] + 0.5*(s_next-s)*(dqdt_ini[i] + dqdt_fin[i]),1.0e-8) << " with i = " << i;

    Crank_nicolson<limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p>::take_step(car, u, u_next, q, s, s_next-s, {});

    for (size_t i = 0; i < 14; ++i)
        EXPECT_NEAR(q[i],q_next[i],1.0e-10) << ", with i = " << i;
}


TEST_F(limebeer2014f1_test, get_vehicle_number_of_outputs_c_api)
{
#ifdef TEST_LIBFASTESTLAPC
    create_vehicle_from_xml("test", "./database/vehicles/f1/limebeer-2014-f1.xml");
    limebeer2014f1<double>::curvilinear_p car_cpp(database);

    int n_inputs, n_control, n_outputs;
    vehicle_type_get_sizes(&n_inputs, &n_control, &n_outputs, "f1-3dof");
    EXPECT_EQ(car_cpp.get_outputs_map().size(), static_cast<size_t>(n_outputs));
    
    delete_variable("test");
#else
GTEST_SKIP();
#endif
}

TEST_F(limebeer2014f1_test, get_vehicle_output_variable_names_c_api)
{
#ifdef TEST_LIBFASTESTLAPC
    create_vehicle_from_xml("test", "./database/vehicles/f1/limebeer-2014-f1.xml");
    limebeer2014f1<double>::curvilinear_p car_cpp(database);

    const size_t n_outputs = car_cpp.get_outputs_map().size();
    int n_input, n_control, n_outputs_c;
    vehicle_type_get_sizes(&n_input, &n_control, &n_outputs_c, "f1-3dof");
    EXPECT_EQ(car_cpp.get_outputs_map().size(), static_cast<size_t>(n_outputs_c));

    constexpr const size_t str_max_len = 60;

    char* key_name = new char[str_max_len];
    char** input_names = new char*[n_input];
    for (size_t i = 0; i < static_cast<size_t>(n_input); ++i)
        input_names[i] = new char[str_max_len];

    char** control_names = new char*[n_control];
    for (size_t i = 0; i < static_cast<size_t>(n_control); ++i)
        control_names[i] = new char[str_max_len];
    
    char** output_names = new char*[n_outputs];
    for (size_t i = 0; i < static_cast<size_t>(n_outputs); ++i)
        output_names[i] = new char[str_max_len];

    vehicle_type_get_names(key_name, input_names, control_names, output_names, str_max_len, "f1-3dof");

    size_t i = 0;
    for (const auto& [key,val] : car_cpp.get_outputs_map())
        EXPECT_EQ(std::string(output_names[i++]), key);

    for (size_t i = 0; i < static_cast<size_t>(n_input); ++i)
        delete[] input_names[i];

    for (size_t i = 0; i < static_cast<size_t>(n_control); ++i)
        delete[] control_names[i];

    for (size_t i = 0; i < static_cast<size_t>(n_outputs); ++i)
        delete[] output_names[i];

    delete[] key_name;
    delete[] input_names;
    delete[] control_names;
    delete[] output_names;
    
    delete_variable("test");
#else
GTEST_SKIP();
#endif
}


TEST_F(limebeer2014f1_test, three_dimensional_track)
{
    // Define a fictional three dimensional road with arbitrary inputs
    const std::vector<scalar> s = { 0.0, 1.0, 2.0, 3.0 };

    const std::vector<sVector3d> r_road_data{ {1.0, -5.0, -10.0}, {2.0, -100.0, -50.0}, {-4.0, 0.1, -100.0}, {50.0,1000.0,0.1} };
    const std::vector<scalar> yaw_road_data{ 45.0 * DEG, 180.0 * DEG, -25.0 * DEG, 50.0 * DEG};
    const std::vector<scalar> pitch_road_data{ 30.0 * DEG, -20.0 * DEG, 15.0 * DEG, 23.0 * DEG };
    const std::vector<scalar> roll_road_data{ 1.0 * DEG, -20.0 * DEG, 30.0 * DEG, -0.01 * DEG };

    const std::vector<scalar> dyawds_road_data{ 2.0, -2.1, 3.4, 5.6 };
    const std::vector<scalar> dpitchds_road_data{ 0.01, -0.05, 0.84, -0.9 };
    const std::vector<scalar> drollds_road_data{ 0.2, -0.15, 0.6, -0.8 };

    const vPolynomial r_road = { s,r_road_data,1,false };
    const sPolynomial yaw_road = { s, yaw_road_data, 1, false };
    const sPolynomial pitch_road = { s, pitch_road_data, 1, false };
    const sPolynomial roll_road = { s, roll_road_data, 1, false };

    const sPolynomial dyawds_road = { s, dyawds_road_data, 1, false };
    const sPolynomial dpitchds_road = { s, dpitchds_road_data, 1, false };
    const sPolynomial drollds_road = { s, drollds_road_data, 1, false };

    const sPolynomial normal_dist = { {s.front(), s.back()}, {0.0, 0.0}, 1, false };

    // Define states to evaluate the car dynamics
    const std::vector<scalar> kappa_fl_data = { 0.8147, 0.9058, 0.1270, 0.9134 };
    const std::vector<scalar> kappa_fr_data = { 0.632359, 0.097540, 0.278498, 0.546882 };
    const std::vector<scalar> kappa_rl_data = { 0.957507, 0.964889, 0.157613, 0.970593 };
    const std::vector<scalar> kappa_rr_data = { 0.914334, -0.029249, 0.600561, -0.716227 };
    const std::vector<scalar> u_data = { 51.596397, 55.985563, 55.174026, 35.679279 };
    const std::vector<scalar> v_data = { 1.820772, 0.475519, 1.961239, 0.088425 };
    const std::vector<scalar> omega_z_data = { 0.769231, 0.128254, 0.269811, 2.287383 };
    const std::vector<scalar> force_z_fl_data = { -0.694829, -0.317099, -0.950222, -0.034446 };
    const std::vector<scalar> force_z_fr_data = { -0.438744, -0.381558, -0.765517, -0.795200 };
    const std::vector<scalar> force_z_rl_data = { -0.186873, -0.489764, -0.445586, -0.646313 };
    const std::vector<scalar> force_z_rr_data = { -0.709365, -0.754687, -0.276025, -0.679703 };
    const std::vector<scalar> time_data = { 0.0, 0.0, 0.0, 0.0 };
    const std::vector<scalar> lateral_displacement_data = { 1.0, -0.4, 1.0, -0.1 };
    const std::vector<scalar> track_heading_angle_data = { 1.0 * DEG, 20.0 * DEG, -12.0 * DEG, 24.0 * DEG };

    const std::vector<scalar> steering_data = { 10.0 * DEG, -4.0 * DEG, 5.0 * DEG, -1.0 * DEG };
    const std::vector<scalar> throttle_data = { 1.0, 0.2, -0.5, -1.0 };

    const Track_by_polynomial track(r_road, yaw_road, pitch_road, roll_road, dyawds_road, dpitchds_road, drollds_road, normal_dist, normal_dist);
    limebeer2014f1<scalar>::curvilinear_p car_sc(database, { track });
    car_sc.set_parameter("vehicle/front-axle/inertia", 2.0);
    car_sc.set_parameter("vehicle/rear-axle/inertia", 3.0);

    EXPECT_TRUE(track.has_elevation());

    const auto [time_derivative_equations, algebraic_equations] = car_sc.classify_equations();

    const std::vector<size_t> time_derivative_equations_expected = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13 };

    EXPECT_EQ(time_derivative_equations.size(), 13);
    EXPECT_EQ(algebraic_equations.size(), 0);

    for (size_t i = 0; i < 13; ++i)
        EXPECT_EQ(time_derivative_equations.at(i), time_derivative_equations_expected.at(i));

    const auto& chassis = car_sc.get_chassis();
    const auto& road = car_sc.get_road();
    const auto& tire_fl = chassis.get_front_axle().template get_tire<0>();
    const auto& tire_fr = chassis.get_front_axle().template get_tire<1>();
    const auto& tire_rl = chassis.get_rear_axle().template get_tire<0>();
    const auto& tire_rr = chassis.get_rear_axle().template get_tire<1>();

    for (size_t i_point = 0; i_point < s.size(); ++i_point)
    {
        const auto& kappa_fl_adim = kappa_fl_data[i_point];
        const auto& kappa_fr_adim = kappa_fr_data[i_point];
        const auto& kappa_rl_adim = kappa_rl_data[i_point];
        const auto& kappa_rr_adim = kappa_rr_data[i_point];
        const auto& u = u_data[i_point];
        const auto& v = v_data[i_point];
        const auto& omega_z = omega_z_data[i_point];
        const auto& force_z_fl_g = force_z_fl_data[i_point];
        const auto& force_z_fr_g = force_z_fr_data[i_point];
        const auto& force_z_rl_g = force_z_rl_data[i_point];
        const auto& force_z_rr_g = force_z_rr_data[i_point];
        const auto& time = time_data[i_point];
        const auto& lateral_displacement = lateral_displacement_data[i_point];
        const auto& track_heading_angle = track_heading_angle_data[i_point];
        const auto& steering = steering_data[i_point];
        const auto& throttle = throttle_data[i_point];

        const auto& roll = roll_road_data[i_point];
        const auto& pitch = pitch_road_data[i_point];
        const auto& yaw = yaw_road_data[i_point];
        const auto& droll_ds = drollds_road_data[i_point];
        const auto& dpitch_ds = dpitchds_road_data[i_point];
        const auto& dyaw_ds = dyawds_road_data[i_point];


        const std::array<scalar, decltype(car_sc)::number_of_inputs> inputs = { kappa_fl_adim, kappa_fr_adim, kappa_rl_adim, kappa_rr_adim, u, v, omega_z, force_z_fl_g, force_z_fr_g, force_z_rl_g, force_z_rr_g, time, lateral_displacement, track_heading_angle };
        const std::array<scalar, decltype(car_sc)::number_of_controls> controls = { steering, 0.0, throttle, 0.5 };

        const auto [states, dstates_dt] = car_sc(inputs, controls, s[i_point]);

        // Check track
        EXPECT_NEAR(road.get_euler_angles().yaw(), yaw, 1.0e-15);
        EXPECT_NEAR(road.get_euler_angles().pitch(), pitch, 1.0e-15);
        EXPECT_NEAR(road.get_euler_angles().roll(), roll, 1.0e-15);

        const sVector3d curvature = { droll_ds - sin(pitch) * dyaw_ds,
            cos(roll) * dpitch_ds + cos(pitch) * sin(roll) * dyaw_ds,
            -sin(roll) * dpitch_ds + cos(pitch) * cos(roll) * dyaw_ds };

        EXPECT_NEAR(road.get_curvature().x(), curvature.x(), 1.0e-15);
        EXPECT_NEAR(road.get_curvature().y(), curvature.y(), 1.0e-15);
        EXPECT_NEAR(road.get_curvature().z(), curvature.z(), 1.0e-15);

        const scalar dsdtime = (u * cos(track_heading_angle) - v * sin(track_heading_angle)) / (1.0 - lateral_displacement * curvature.z());

        EXPECT_NEAR(road.get_euler_angles_dot().yaw(), dyaw_ds * dsdtime, 1.0e-14 * max(1.0, std::abs(dyaw_ds * dsdtime)));
        EXPECT_NEAR(road.get_euler_angles_dot().pitch(), dpitch_ds * dsdtime, 1.0e-14 * max(1.0, std::abs(dpitch_ds * dsdtime)));
        EXPECT_NEAR(road.get_euler_angles_dot().roll(), droll_ds * dsdtime, 1.0e-14 * max(1.0, std::abs(droll_ds * dsdtime)));

        EXPECT_DOUBLE_EQ(road.get_lateral_displacement(), lateral_displacement);
        EXPECT_DOUBLE_EQ(road.get_track_heading_angle(), track_heading_angle);
        EXPECT_NEAR(road.get_track_heading_angle_dot(), omega_z - curvature.z() * dsdtime, 2.0e-15 * max(1.0, std::abs(road.get_track_heading_angle_dot())));

        EXPECT_NEAR(road.get_tangent_vector().x(), cos(yaw) * cos(pitch), 1.0e-15);
        EXPECT_NEAR(road.get_tangent_vector().y(), sin(yaw) * cos(pitch), 1.0e-15);
        EXPECT_NEAR(road.get_tangent_vector().z(), -sin(pitch), 1.0e-15);

        EXPECT_NEAR(road.get_normal_vector().x(), cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll), 1.0e-15);
        EXPECT_NEAR(road.get_normal_vector().y(), sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll), 1.0e-15);
        EXPECT_NEAR(road.get_normal_vector().z(), cos(pitch) * sin(roll), 1.0e-15);

        EXPECT_NEAR(road.get_binormal_vector().x(), cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll), 1.0e-15);
        EXPECT_NEAR(road.get_binormal_vector().y(), sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll), 1.0e-15);
        EXPECT_NEAR(road.get_binormal_vector().z(), cos(pitch) * cos(roll), 1.0e-15);


        // Check car -----------------------------------------------------------------------------------------

        // Check that the road frame was successfully updated
        EXPECT_NEAR(chassis.get_road_frame().get_absolute_position().x(), r_road_data[i_point].x() + lateral_displacement * road.get_normal_vector().x(), 7.0e-15);
        EXPECT_NEAR(chassis.get_road_frame().get_absolute_position().y(), r_road_data[i_point].y() + lateral_displacement * road.get_normal_vector().y(), 7.0e-15);
        EXPECT_NEAR(chassis.get_road_frame().get_absolute_position().z(), r_road_data[i_point].z() + lateral_displacement * road.get_normal_vector().z(), 7.0e-15);

        EXPECT_NEAR(chassis.get_road_frame().get_absolute_velocity_in_body().x(), u, 1.0e-14 * max(1.0, std::abs(u)));
        EXPECT_NEAR(chassis.get_road_frame().get_absolute_velocity_in_body().y(), v, 1.0e-14 * max(1.0, std::abs(v)));
        EXPECT_NEAR(chassis.get_road_frame().get_absolute_velocity_in_body().z(), lateral_displacement * curvature.x() * dsdtime, 1.0e-13 * max(1.0, std::abs(lateral_displacement)));

        const sVector3d omega_body = { (curvature.x() * cos(track_heading_angle) + curvature.y() * sin(track_heading_angle)) * dsdtime,
         (curvature.y() * cos(track_heading_angle) - curvature.x() * sin(track_heading_angle)) * dsdtime, omega_z };

        EXPECT_NEAR(chassis.get_road_frame().get_omega_absolute_in_body().x(), (curvature.x() * cos(track_heading_angle) + curvature.y() * sin(track_heading_angle)) * dsdtime, 5.0e-13);
        EXPECT_NEAR(chassis.get_road_frame().get_omega_absolute_in_body().y(), (curvature.y() * cos(track_heading_angle) - curvature.x() * sin(track_heading_angle)) * dsdtime, 5.0e-13);
        EXPECT_NEAR(chassis.get_road_frame().get_omega_absolute_in_body().z(), omega_z, 5.0e-13);

        EXPECT_NEAR(chassis.get_road_frame().get_rotation_angles().at(0), yaw, 1.0e-15);
        EXPECT_NEAR(chassis.get_road_frame().get_rotation_angles().at(1), pitch, 1.0e-15);
        EXPECT_NEAR(chassis.get_road_frame().get_rotation_angles().at(2), roll, 1.0e-15);
        EXPECT_NEAR(chassis.get_road_frame().get_rotation_angles().at(3), track_heading_angle, 1.0e-15);

        EXPECT_NEAR(chassis.get_road_frame().get_rotation_angles_derivative().at(0), dyaw_ds * dsdtime, 1.0e-12);
        EXPECT_NEAR(chassis.get_road_frame().get_rotation_angles_derivative().at(1), dpitch_ds * dsdtime, 1.0e-12);
        EXPECT_NEAR(chassis.get_road_frame().get_rotation_angles_derivative().at(2), droll_ds * dsdtime, 1.0e-12);
        EXPECT_NEAR(chassis.get_road_frame().get_rotation_angles_derivative().at(3), omega_z - curvature.z() * dsdtime, 1.0e-12);

        EXPECT_EQ(chassis.get_road_frame().get_rotation_axis().at(0), Z);
        EXPECT_EQ(chassis.get_road_frame().get_rotation_axis().at(1), Y);
        EXPECT_EQ(chassis.get_road_frame().get_rotation_axis().at(2), X);
        EXPECT_EQ(chassis.get_road_frame().get_rotation_axis().at(3), Z);

        // Check states
        const auto Fz_fl = force_z_fl_g * 660.0 * 9.81;
        const auto Fz_fr = force_z_fr_g * 660.0 * 9.81;
        const auto Fz_rl = force_z_rl_g * 660.0 * 9.81;
        const auto Fz_rr = force_z_rr_g * 660.0 * 9.81;
        const auto neg_Fz_fl = 0.5 * (Fz_fl - sqrt(Fz_fl * Fz_fl + 1.0));
        const auto neg_Fz_fr = 0.5 * (Fz_fr - sqrt(Fz_fr * Fz_fr + 1.0));
        const auto neg_Fz_rl = 0.5 * (Fz_rl - sqrt(Fz_rl * Fz_rl + 1.0));
        const auto neg_Fz_rr = 0.5 * (Fz_rr - sqrt(Fz_rr * Fz_rr + 1.0));

        const auto kappa_fl = kappa_fl_adim * tire_fl.get_model().maximum_kappa(-neg_Fz_fl);
        const auto kappa_fr = kappa_fr_adim * tire_fr.get_model().maximum_kappa(-neg_Fz_fr);
        const auto kappa_rl = kappa_rl_adim * tire_rl.get_model().maximum_kappa(-neg_Fz_rl);
        const auto kappa_rr = kappa_rr_adim * tire_rr.get_model().maximum_kappa(-neg_Fz_rr);

        EXPECT_NEAR(kappa_fl, tire_fl.get_kappa(), 1.0e-14);
        EXPECT_NEAR(kappa_fr, tire_fr.get_kappa(), 1.0e-14);
        EXPECT_NEAR(kappa_rl, tire_rl.get_kappa(), 1.0e-14);
        EXPECT_NEAR(kappa_rr, tire_rr.get_kappa(), 1.0e-14);

        const auto omega_fl = (cos(steering) * (u + omega_z * 0.73) + sin(steering) * (omega_z * 1.8 + v)) * (1.0 + kappa_fl) / 0.33;
        const auto omega_fr = (cos(steering) * (u - omega_z * 0.73) + sin(steering) * (omega_z * 1.8 + v)) * (1.0 + kappa_fr) / 0.33;
        const auto omega_rl = (u + omega_z * 0.73) * (1.0 + kappa_rl) / 0.33;
        const auto omega_rr = (u - omega_z * 0.73) * (1.0 + kappa_rr) / 0.33;

        EXPECT_NEAR(tire_fl.get_omega(), omega_fl, 2.0e-13);
        EXPECT_NEAR(tire_fr.get_omega(), omega_fr, 2.0e-13);
        EXPECT_NEAR(tire_rl.get_omega(), omega_rl, 2.0e-13);
        EXPECT_NEAR(tire_rr.get_omega(), omega_rr, 2.0e-13);

        EXPECT_NEAR(states[decltype(car_sc)::Chassis_type::Front_axle_type::state_names::angular_momentum_left], 2.0 * omega_fl, 3.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Chassis_type::Front_axle_type::state_names::angular_momentum_right], 2.0 * omega_fr, 3.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Chassis_type::Rear_axle_type::state_names::angular_momentum_left], 3.0 * omega_rl, 3.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Chassis_type::Rear_axle_type::state_names::angular_momentum_right], 3.0 * omega_rr, 3.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Chassis_type::state_names::com_velocity_x_mps], u - 0.3 * omega_body.y(), 1.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Chassis_type::state_names::com_velocity_y_mps], v + 0.3 * omega_body.x(), 1.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Chassis_type::state_names::yaw_rate_radps], omega_z, 1.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Chassis_type::state_names::pitch_angular_momentum_Nms], 425.0 * omega_body.y() / (660.0 * 9.81), 1.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Chassis_type::state_names::roll_angular_momentum_Nms], 112.5 * omega_body.x() / (660.0 * 9.81), 1.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Chassis_type::state_names::com_velocity_z_mps], lateral_displacement * curvature.x() * dsdtime / 9.81, 1.0e-13);
        EXPECT_DOUBLE_EQ(states[decltype(car_sc)::Chassis_type::state_names::roll_balance_equation_g], 0.0);
        EXPECT_NEAR(states[decltype(car_sc)::Road_type::state_names::time], time, 2.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Road_type::state_names::lateral_displacement], lateral_displacement, 2.0e-13);
        EXPECT_NEAR(states[decltype(car_sc)::Road_type::state_names::track_heading_angle], track_heading_angle, 2.0e-13);

        // Check state derivatives
        EXPECT_NEAR(chassis.get_roll_balance_equation_N(), Fz_fr - Fz_fl - 0.5*(Fz_fr + Fz_rr - Fz_fl - Fz_rl), 2.0e-11);

        const scalar lambda_fl = -(cos(steering)*(omega_z*1.8+v)-sin(steering)*(omega_z*0.73+u))/(cos(steering)*(omega_z*0.73+u)+sin(steering)*(omega_z*1.8+v));
        const scalar lambda_fr = -(cos(steering)*(omega_z*1.8+v)+sin(steering)*(omega_z*0.73-u))/(cos(steering)*(u-omega_z*0.73)+sin(steering)*(omega_z*1.8+v));
        const scalar lambda_rl = -(v-omega_z*1.6)/(u+omega_z*0.73);
        const scalar lambda_rr = -(v-omega_z*1.6)/(u-omega_z*0.73);
        EXPECT_NEAR(tire_fl.get_lambda(), lambda_fl, 1.0e-12);
        EXPECT_NEAR(tire_fr.get_lambda(), lambda_fr, 1.0e-12);
        EXPECT_NEAR(tire_rl.get_lambda(), lambda_rl, 1.0e-12);
        EXPECT_NEAR(tire_rr.get_lambda(), lambda_rr, 1.0e-12);


        const auto F_fl = get_tire_forces(kappa_fl, lambda_fl, -neg_Fz_fl);
        const auto F_fr = get_tire_forces(kappa_fr, lambda_fr, -neg_Fz_fr);
        const auto F_rl = get_tire_forces(kappa_rl, lambda_rl, -neg_Fz_rl);
        const auto F_rr = get_tire_forces(kappa_rr, lambda_rr, -neg_Fz_rr);
    
        // Compute aerodynamic forces
        const auto w = lateral_displacement * curvature.x() * dsdtime;
        const auto F_lift = 0.5*1.2*u*u*1.5*3.0;
        const auto Fx_drag = -0.5*1.2*sqrt(u*u+v*v)*u*1.5*0.9;
        const auto Fy_drag = -0.5*1.2*sqrt(u*u+v*v)*v*1.5*0.9;
        const auto Fz_drag = -0.5*1.2*sqrt(u*u+v*v)*w*1.5*0.9;
    
        // Compute total forces
        const auto Fx = cos(steering)*(F_fr.x()+F_fl.x())-sin(steering)*(F_fr.y()+F_fl.y()) + F_rr.x() + F_rl.x() + Fx_drag;
        const auto Fy = cos(steering)*(F_fr.y()+F_fl.y())+sin(steering)*(F_fr.x()+F_fl.x()) + F_rr.y() + F_rl.y() + Fy_drag;
    
        const auto gravity = 660.0 * 9.81 * sVector3d{ sin(track_heading_angle) * sin(roll) * cos(pitch) - cos(track_heading_angle) * sin(pitch),
            sin(track_heading_angle) * sin(pitch) + cos(track_heading_angle) * sin(roll) * cos(pitch),
            cos(roll) * cos(pitch) };

        const scalar du = omega_z*(v + 0.3*omega_body.x()) - w*omega_body.y() + gravity.x()/660.0 + Fx/660.0;
        const scalar dv = w*omega_body.x() - omega_z*(u - 0.3*omega_body.y()) + gravity.y()/660.0 + Fy/660.0;
        const scalar dw = F_fl.z()+F_fr.z()+F_rl.z()+F_rr.z() + gravity.z() + Fz_drag + F_lift + 660.0*(u-0.3*omega_body.y())*omega_body.y() - 660.0*(v + 0.3*omega_body.x())*omega_body.x();

        EXPECT_NEAR(chassis.get_com_velocity_x_mps(), u - 0.3 * omega_body.y(), 2.0e-11);
        EXPECT_NEAR(chassis.get_com_velocity_y_mps(), v + 0.3 * omega_body.x(), 2.0e-11);
        EXPECT_NEAR(chassis.get_com_velocity_z_mps(), w, 2.0e-11);

        EXPECT_NEAR(chassis.get_com_velocity_x_dot_mps2(), du, 2.0e-13*max(1.0, std::abs(du)));
        EXPECT_NEAR(chassis.get_com_velocity_y_dot_mps2(), dv, 2.0e-13*max(1.0, std::abs(dv)));
        EXPECT_NEAR(chassis.get_com_velocity_z_dot_mps2()*660.0, dw, 2.0e-13*max(1.0, std::abs(dw)));

        const scalar domega = (-0.1*Fy_drag + 1.8*(cos(steering)*(F_fr.y()+F_fl.y())+sin(steering)*(F_fr.x()+F_fl.x())) + 0.73*(sin(steering)*F_fr.y()-cos(steering)*F_fr.x()) - 0.73*F_rr.x() + 0.73*(cos(steering)*F_fl.x() - sin(steering)*F_fl.y()) + 0.73*F_rl.x() - 1.6*(F_rr.y()+F_rl.y()) + (112.5-425.0)*omega_body.x()*omega_body.y())/450.0;

        EXPECT_NEAR(chassis.get_roll_angular_momentum_dot_Nm(), 0.73*(neg_Fz_rr-neg_Fz_rl)+0.73*(neg_Fz_fr-neg_Fz_fl) - 0.3*Fy + (425.0 - 450.0)*omega_z*omega_body.y(), 4.0e-11);
        EXPECT_NEAR(chassis.get_pitch_angular_momentum_dot_Nm(), 1.6*(neg_Fz_rr+neg_Fz_rl)-1.8*(neg_Fz_fr+neg_Fz_fl)+ 0.3*Fx + 0.1*(F_lift + Fz_drag) + (450.0 - 112.5)*omega_z*omega_body.x(), 4.0e-11);
        EXPECT_NEAR(chassis.get_yaw_rate_dot_radps2(), domega, 4.0e-11);

        EXPECT_NEAR(dstates_dt[decltype(car_sc)::Chassis_type::state_names::com_velocity_x_mps], du/dsdtime, 2.0e-11);
        EXPECT_NEAR(dstates_dt[decltype(car_sc)::Chassis_type::state_names::com_velocity_y_mps], dv/dsdtime, 2.0e-11);
        EXPECT_NEAR(dstates_dt[decltype(car_sc)::Chassis_type::state_names::com_velocity_z_mps], dw/(660.0*9.81*dsdtime), 2.0e-11);
        EXPECT_NEAR(dstates_dt[decltype(car_sc)::Chassis_type::state_names::yaw_rate_radps], domega/(dsdtime), 2.0e-11);
        EXPECT_NEAR(dstates_dt[decltype(car_sc)::Chassis_type::state_names::pitch_angular_momentum_Nms], (1.6*(neg_Fz_rr+neg_Fz_rl)-1.8*(neg_Fz_fr+neg_Fz_fl)+ 0.3*Fx + 0.1*(F_lift + Fz_drag) + (450.0 - 112.5)*omega_z*omega_body.x())/(660.0*9.81*dsdtime), 2.0e-11);
        EXPECT_NEAR(dstates_dt[decltype(car_sc)::Chassis_type::state_names::roll_angular_momentum_Nms],  (0.73*(neg_Fz_rr-neg_Fz_rl)+0.73*(neg_Fz_fr-neg_Fz_fl) - 0.3*Fy + (425.0 - 450.0)*omega_z*omega_body.y())/(660.0*9.81*dsdtime), 2.0e-11);
        EXPECT_NEAR(dstates_dt[decltype(car_sc)::Chassis_type::state_names::roll_balance_equation_g], (Fz_fr - Fz_fl - 0.5*(Fz_fr + Fz_rr - Fz_fl - Fz_rl))/(660.0*9.81*dsdtime), 2.0e-11);
        EXPECT_NEAR(dstates_dt[decltype(car_sc)::Road_type::state_names::time], 1.0/dsdtime, 2.0e-13);
        EXPECT_NEAR(dstates_dt[decltype(car_sc)::Road_type::state_names::lateral_displacement], (u*sin(track_heading_angle) + v*cos(track_heading_angle))/dsdtime, 2.0e-13);
        EXPECT_NEAR(dstates_dt[decltype(car_sc)::Road_type::state_names::track_heading_angle], omega_body.z()/dsdtime - curvature.z(), 2.0e-13);
    }
}


TEST_F(limebeer2014f1_test, parameters_all_used_test)
{
    EXPECT_NO_THROW(limebeer2014f1<double>::curvilinear_p car_sc(database));
    database.add_element("vehicle/test_element");
    EXPECT_THROW(limebeer2014f1<double>::curvilinear_p car_sc(database), fastest_lap_exception);
}
