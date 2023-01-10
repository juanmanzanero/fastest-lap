#include "gtest/gtest.h"

#include "lion/thirdparty/include/cppad/cppad.hpp"

#include "src/core/vehicles/lot2016kart.h"
#include "lion/propagators/rk4.h"
#include "lion/thirdparty/include/logger.hpp"

static inline vPolynomial construct_ninety_degrees_bend(double L_end = 20.0)
{
    // Part 1: 20m straight
    scalar L1 = 20.0;
    std::vector<sVector3d> y1 = { {0.0,0.0,0.0}, {10.0, 0.0, 0.0}, {20.0,0.0,0.0} };
 
    // Part 2: 90 degrees bend with 10m radius
    scalar L2 = 10*pi/2.0;
    std::vector<sVector3d> y2;
    const size_t N2 = 10;
 
    const auto xi2 = std::get<0>(gauss_legendre_lobatto_nodes_and_weights(N2));

    for (size_t i = 0; i <= N2; ++i)
        y2.push_back(sVector3d(20.0+10.0*sin(pi/4.0*(xi2[i]+1.0)),10.0-10.0*cos(pi/4.0*(xi2[i]+1.0)),0.0));

    // Part 3: 20m straight
    scalar L3 = L_end;
    std::vector<sVector3d> y3 = { y2.back(), {30.0, 10.0+L_end*0.5, 0.0} , {30.0,10.0+L_end,0.0} };
 
    return {0.0, {L1,L2,L3}, {y1,y2,y3}};
}

using Front_left_tire_type  = lot2016kart<scalar>::Front_left_tire_type;
using Front_right_tire_type = lot2016kart<scalar>::Front_left_tire_type;
using Rear_left_tire_type   = lot2016kart<scalar>::Rear_left_tire_type;
using Rear_right_tire_type  = lot2016kart<scalar>::Rear_right_tire_type;

using Front_axle_t          = lot2016kart<scalar>::Front_axle_t;
using Rear_axle_t           = lot2016kart<scalar>::Rear_axle_t;
using Chassis_t             = lot2016kart<scalar>::Chassis_t;

using Road_t                = lot2016kart<scalar>::curvilinear_p::Road_t;
using Dynamic_model_t       = lot2016kart<scalar>::curvilinear_p;

// Make sure that the enumerators are correct
// Controls
static_assert(Front_axle_t::Axle_type::control_names::STEERING==0);
static_assert(Rear_axle_t::Axle_type::control_names::TORQUE==1);

// State
static_assert(Rear_axle_t::input_names::OMEGA_AXLE==0);
static_assert(Chassis_t::input_names::velocity_x_mps==1);
static_assert(Chassis_t::input_names::velocity_y_mps==2);
static_assert(Chassis_t::input_names::yaw_rate_radps==3);
static_assert(Chassis_t::input_names::Z==4);
static_assert(Chassis_t::input_names::PHI==5);
static_assert(Chassis_t::input_names::MU==6);
static_assert(Chassis_t::input_names::DZDT==7);
static_assert(Chassis_t::input_names::DPHIDT==8);
static_assert(Chassis_t::input_names::DMUDT==9);
static_assert(Road_t::input_names::time==10);
static_assert(Road_t::input_names::lateral_displacement==11);
static_assert(Road_t::input_names::track_heading_angle==12);

static_assert(Rear_axle_t::state_names::OMEGA_AXLE==0);
static_assert(Chassis_t::state_names::com_velocity_x_mps==1);
static_assert(Chassis_t::state_names::com_velocity_y_mps==2);
static_assert(Chassis_t::state_names::yaw_rate_radps==3);
static_assert(Chassis_t::state_names::Z==4);
static_assert(Chassis_t::state_names::PHI==5);
static_assert(Chassis_t::state_names::MU==6);
static_assert(Chassis_t::state_names::DZDT==7);
static_assert(Chassis_t::state_names::DPHIDT==8);
static_assert(Chassis_t::state_names::DMUDT==9);
static_assert(Road_t::state_names::time==10);
static_assert(Road_t::state_names::lateral_displacement==11);
static_assert(Road_t::state_names::track_heading_angle==12);

class Control
{
 public:
    Control(const Road_t& road) : _roadPtr(&road) {};

    std::array<scalar,2> operator()(const std::array<scalar,13>& q, const scalar t) const { 
         return { std::min(40.0,std::max(-40.0,-07.0*DEG*Value(_roadPtr->get_lateral_displacement()))), 0.0 };
    }

    const Road_t* _roadPtr;
};


class Car_road_curvilinear_test : public ::testing::Test
{
 protected:
    Car_road_curvilinear_test()
            {
                // Left here on purpose to test the operator=

                // set the 'smooth' max parameter to 0
                database.get_element("vehicle/front-tire/Fz-max-ref2").set_value("0.0");
                database.get_element("vehicle/rear-tire/Fz-max-ref2").set_value("0.0");
                database.get_element("vehicle/rear-axle/smooth_throttle_coeff").set_value("0.0");

                _car = {database, _road };

            }  

    const scalar omega = 0.1;

    const scalar u = 0.95;
    const scalar v = 0.1;

    const scalar z    = 0.1;
    const scalar dz   = 0.03;
    const scalar mu   = -5.0*DEG;
    const scalar dmu  = 0.4;
    const scalar phi  = 4.0*DEG;
    const scalar dphi = 0.2;

    const scalar delta = -16.0*DEG;

    const scalar T = 0.1*17.6;    // Maximum torque at 10.250rpm
    const scalar omega_axle = 10.0;

    Xml_document database = {"database/vehicles/kart/roberto-lot-kart-2016.xml", true};
    Road_t _road = {{construct_ninety_degrees_bend(), sPolynomial({0.0,1000.0},{5.0,5.0},1,false), sPolynomial({0.0,1000.0},{5.0,5.0},1,false)}};
    Dynamic_model_t _car = {database, _road};
};


TEST_F(Car_road_curvilinear_test, state_vector_sizes)
{
    static_assert( Dynamic_model_t::number_of_inputs == 13 );
    static_assert( Dynamic_model_t::number_of_controls == 2 );
    EXPECT_EQ(Dynamic_model_t::number_of_inputs,13);
    EXPECT_EQ(Dynamic_model_t::number_of_controls,2);

}


TEST_F(Car_road_curvilinear_test, curvilinear_road_test)
{
    const scalar t = 0.5*_road.track_length();
    _road.update_track(t);

    EXPECT_DOUBLE_EQ(_road.track_length(), 20.0 + 10.0*pi/2.0 + 20.0);
    EXPECT_NEAR(_road.get_curvature().x(), 0.0, 7.0e-12);
    EXPECT_NEAR(_road.get_curvature().y(), 0.0, 7.0e-12);
    EXPECT_NEAR(_road.get_curvature().z(), 1.0/10.0, 7.0e-12);
    EXPECT_NEAR(_road.get_heading_angle(), pi/4.0, 7.0e-12);
}


TEST_F(Car_road_curvilinear_test, dqdt_test)
{
    const scalar t = 0.5*_road.track_length();
    scalar n = 3.0;
    scalar alpha = 15.0*DEG;
    std::array<scalar,13> q = {omega_axle,u,v,omega,z,phi,mu,dz,dphi,dmu,0.0,n,alpha};
    std::array<scalar,2> u_con = {delta, T};

    auto dqdt = _car.ode(q,u_con,t);

    scalar dtimedt = (1.0-n*0.1)/(u*cos(alpha) - v*sin(alpha));
    scalar dndt = (u*sin(alpha) + v*cos(alpha))*dtimedt;
    scalar dalphadt = (omega - (u*cos(alpha)-v*sin(alpha))/(1.0 - 0.1*n)*0.1)*dtimedt;

    EXPECT_NEAR(Value(dqdt[Road_t::state_names::time]) , Value(dtimedt) , 5.0e-11);
    EXPECT_NEAR(Value(dqdt[Road_t::state_names::lateral_displacement])    , Value(dndt)    , 5.0e-11);
    EXPECT_NEAR(Value(dqdt[Road_t::state_names::track_heading_angle]), Value(dalphadt), 5.0e-11);
}

  
TEST_F(Car_road_curvilinear_test, corner_simulation)
{
    const std::vector<scalar> q_saved = { 49.079560265452528, 6.3863859462710311, -0.288866824372148, -0.83786554730703189, 0.018774867090066576, 0.011861202220227633, -0.0065017152255987653, -0.00010518089441523141, -0.0012199840201436597, -0.0063030759539832899, 7.2701395434941851, 1.3235850867497201, -0.080712238424611518};
    const size_t n_timesteps = 600;
    const scalar L = _road.track_length();
    const scalar ds = L/n_timesteps;

    Control control(_car.get_road());

    std::array<scalar,13> q = {10.0*0.139,10.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    for (size_t i = 0; i < n_timesteps; ++i)
    {
        RK4<Dynamic_model_t,Control,Dynamic_model_t::number_of_inputs>::take_step(_car, control, q, i*ds, ds);
    }

    for (size_t i = 0; i < q.size(); ++i)
    {
#ifdef CHECK_BINARY_EQUAL
        EXPECT_DOUBLE_EQ(q.at(i), q_saved.at(i));
        out(2) << "Check binary equal: on" << std::endl;
#else
        EXPECT_NEAR(Value(q.at(i)), Value(q_saved.at(i)),2.0e-08) << "with i = " << i;
#endif
    }
}
  
  
