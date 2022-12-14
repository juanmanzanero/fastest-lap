#include "gtest/gtest.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "src/core/applications/optimal_laptime.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "src/core/applications/steady_state.h"
#include "lion/propagators/crank_nicolson.h"
#include <unordered_map>
#include "src/main/c/fastestlapc.h"

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


TEST_F(limebeer2014f1_test, parameters_all_used_test)
{
    EXPECT_NO_THROW(limebeer2014f1<double>::curvilinear_p car_sc(database));
    database.add_element("vehicle/test_element");
    EXPECT_THROW(limebeer2014f1<double>::curvilinear_p car_sc(database), fastest_lap_exception);
}
