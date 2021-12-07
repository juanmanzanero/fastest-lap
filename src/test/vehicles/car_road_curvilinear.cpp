#include "gtest/gtest.h"

#include "lion/thirdparty/include/cppad/cppad.hpp"

#include "src/core/vehicles/lot2016kart.h"
#include "lion/propagators/rk4.h"
#include "lion/thirdparty/include/logger.hpp"

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
static_assert(Front_axle_t::Axle_type::ISTEERING==0);
static_assert(Rear_axle_t::Axle_type::ITORQUE==1);

// State
static_assert(Rear_axle_t::IOMEGA_AXLE==0);
static_assert(Chassis_t::IU==1);
static_assert(Chassis_t::IV==2);
static_assert(Chassis_t::IOMEGA==3);
static_assert(Chassis_t::IZ==4);
static_assert(Chassis_t::IPHI==5);
static_assert(Chassis_t::IMU==6);
static_assert(Chassis_t::IDZ==7);
static_assert(Chassis_t::IDPHI==8);
static_assert(Chassis_t::IDMU==9);
static_assert(Road_t::ITIME==10);
static_assert(Road_t::IN==11);
static_assert(Road_t::IALPHA==12);

// Stateder
static_assert(Rear_axle_t::IIDOMEGA_AXLE==0);
static_assert(Chassis_t::IIDU==1);
static_assert(Chassis_t::IIDV==2);
static_assert(Chassis_t::IIDOMEGA==3);
static_assert(Chassis_t::IIDZ==4);
static_assert(Chassis_t::IIDPHI==5);
static_assert(Chassis_t::IIDMU==6);
static_assert(Chassis_t::IID2Z==7);
static_assert(Chassis_t::IID2PHI==8);
static_assert(Chassis_t::IID2MU==9);
static_assert(Road_t::IIDTIME==10);
static_assert(Road_t::IIDN==11);
static_assert(Road_t::IIDALPHA==12);


class Control
{
 public:
    Control(const Road_t& road) : _roadPtr(&road) {};

    std::array<scalar,2> operator()(const std::array<scalar,13>& q, const scalar t) const { 
         return { std::min(40.0,std::max(-40.0,-07.0*DEG*Value(_roadPtr->get_n()))), 0.0 };
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

    Xml_document database = {"database/roberto-lot-kart-2016.xml", true};
    Road_t _road = {construct_ninety_degrees_bend(), 5.0};
    Dynamic_model_t _car = {database, _road};
};


TEST_F(Car_road_curvilinear_test, state_vector_sizes)
{
    static_assert( Dynamic_model_t::NSTATE == 13 );
    static_assert( Dynamic_model_t::NCONTROL == 2 );
    EXPECT_EQ(Dynamic_model_t::NSTATE,13);
    EXPECT_EQ(Dynamic_model_t::NCONTROL,2);

}


TEST_F(Car_road_curvilinear_test, curvilinear_road_test)
{
    const scalar t = 0.5*_road.track_length();
    _road.update_track(t);

    EXPECT_DOUBLE_EQ(_road.track_length(), 20.0 + 10.0*pi/2.0 + 20.0);
    EXPECT_NEAR(_road.get_curvature(), 1.0/10.0, 7.0e-12);
    EXPECT_NEAR(_road.get_heading_angle(), pi/4.0, 7.0e-12);
}


TEST_F(Car_road_curvilinear_test, dqdt_test)
{
    const scalar t = 0.5*_road.track_length();
    scalar n = 3.0;
    scalar alpha = 15.0*DEG;
    std::array<scalar,13> q = {omega_axle,u,v,omega,z,phi,mu,dz,dphi,dmu,0.0,n,alpha};
    std::array<scalar,2> u_con = {delta, T};

    auto dqdt = _car(q,u_con,t);

    scalar dtimedt = (1.0-n*0.1)/(u*cos(alpha) - v*sin(alpha));
    scalar dndt = (u*sin(alpha) + v*cos(alpha))*dtimedt;
    scalar dalphadt = (omega - (u*cos(alpha)-v*sin(alpha))/(1.0 - 0.1*n)*0.1)*dtimedt;

    EXPECT_NEAR(Value(dqdt[Road_t::IIDTIME]) , Value(dtimedt) , 5.0e-11);
    EXPECT_NEAR(Value(dqdt[Road_t::IIDN])    , Value(dndt)    , 5.0e-11);
    EXPECT_NEAR(Value(dqdt[Road_t::IIDALPHA]), Value(dalphadt), 5.0e-11);
}

  
TEST_F(Car_road_curvilinear_test, corner_simulation)
{
    const std::vector<scalar> q_saved = {48.300857539581237, 6.7982892067808844, -0.10329911987170565, -0.27071165610742076, 0.018718968090190604, 0.0037984889001121372, -0.005866140125318919, 0.00013622832006761286, 0.00070794803100000911, 0.022490934180667464, 7.1730404334292572, 0.42983556313373483, 0.12554681676280013};
    const size_t n_timesteps = 600;
    const scalar L = _road.track_length();
    const scalar ds = L/n_timesteps;

    Control control(_car.get_road());

    std::array<scalar,13> q = {10.0*0.139,10.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    for (size_t i = 0; i < n_timesteps; ++i)
    {
        RK4<Dynamic_model_t,Control,Dynamic_model_t::NSTATE>::take_step(_car, control, q, i*ds, ds);
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
  
  
