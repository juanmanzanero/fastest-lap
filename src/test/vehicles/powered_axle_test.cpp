/*
#include "gtest/gtest.h"

#include "lion/thirdparty/include/cppad/cppad.hpp"

#include "src/core/tire/tire_pacejka.h"
#include "src/core/chassis/axle_car_6dof.h"
#include "src/core/vehicles/dynamic_model_powered_axle.h"
#include "src/core/chassis/chassis_car_6dof.h"
#include "lion/math/matrix_extensions.h"
#include "lion/propagators/rk4.h"
#include <iomanip>

using Rear_left_tire = Tire_pacejka_std<scalar,0,0>;
using Rear_right_tire = Tire_pacejka_std<scalar,Rear_left_tire::input_state_names::end,Rear_left_tire::control_names::end>;
using Axle_type = Axle_car_6dof<scalar,Rear_left_tire,Rear_right_tire,POWERED_WITHOUT_DIFFERENTIAL,Rear_right_tire::input_state_names::end,Rear_right_tire::control_names::end>;
using Dynamic_model_t = Dynamic_model_powered_axle<scalar,Axle_type,Axle_type::input_state_names::end,Axle_type::control_names::end>;

const static std::map<std::string,scalar> Lot2016kart_rear_axle =
{
     std::make_pair("rear-axle/track"               , 1.200      ), 
     std::make_pair("rear-axle/chassis_stiffness"   , 60.0e3     ), 
     std::make_pair("rear-axle/antiroll_stiffness"  , 0.0        ), 
     std::make_pair("rear-axle/inertia"             , 0.2        ), 
     std::make_pair("chassis/mass"                  , 165.0      ), 
     std::make_pair("chassis/rho_air"               , 1.2        ), 
     std::make_pair("chassis/CdA"                   , 0.7        ), 
     std::make_pair("rear-tire/R0"                  , 0.139      ), 
     std::make_pair("rear-tire/kt"                  , 61.3e3     ), 
     std::make_pair("rear-tire/ct"                  , 1.0e3      ), 
     std::make_pair("rear-tire/Fz0"                 , 560        ), 
     std::make_pair("rear-tire/pCx1"                , 2.3        ), 
     std::make_pair("rear-tire/pDx1"                , 0.9        ), 
     std::make_pair("rear-tire/pEx1"                , 0.95       ), 
     std::make_pair("rear-tire/pKx1"                , 20.0       ), 
     std::make_pair("rear-tire/pKx2"                , 1.0        ), 
     std::make_pair("rear-tire/pKx3"                , -0.5       ), 
     std::make_pair("rear-tire/pCy1"                , 2.3        ), 
     std::make_pair("rear-tire/pDy1"                , 1.5        ), 
     std::make_pair("rear-tire/pEy1"                , 0.9        ), 
     std::make_pair("rear-tire/pKy1"                , 37.6       ), // I have changed its sign
     std::make_pair("rear-tire/pKy2"                , 1.6        ), 
     std::make_pair("rear-tire/pKy4"                , 2.0        ), 
     std::make_pair("rear-tire/rBx1"                , 14.0       ), 
     std::make_pair("rear-tire/rCx1"                , 1.0        ), // This one I am not 100% sure
     std::make_pair("rear-tire/rBy1"                , 12.0       ), 
     std::make_pair("rear-tire/rCy1"                , 0.6        ), 
     std::make_pair("rear-tire/lambdaFz0"           , 1.6        )
};


Xml_document* get_database_axle()
{
    Xml_document* database = new Xml_document("data/roberto-lot-kart-2016-rear-axle.xml", true);
    
    database->get_element("vehicle/rear-tire/Fz-max-ref2").set_value("0.0");

    return database;
}


class Dynamic_model_powered_axle_test : public ::testing::Test
{
 protected:
    Dynamic_model_powered_axle_test() 
    {
        std::array<scalar,Dynamic_model_t::NSTATE> q = {1.0, 0.0, 0.0, 0.0, 0.139};
        std::array<scalar,Dynamic_model_t::NCONTROL> u = {0.0};
        _car(q,u,0.0);
    }

    ~Dynamic_model_powered_axle_test() 
    {
        delete database;
    }

    Xml_document* database = get_database_axle();
    scalar Fz = 560.0; 
    Dynamic_model_t _car = {Fz, *database};
};

// check state vector
static_assert(Axle_type::input_state_names::OMEGA_AXLE == 0);
static_assert(Dynamic_model_t::Road_type::input_state_names::X == 1);
static_assert(Dynamic_model_t::Road_type::input_state_names::Y == 2);
static_assert(Dynamic_model_t::Road_type::input_state_names::PSI == 3);
static_assert(Dynamic_model_t::input_state_names::U == 4);
static_assert(Dynamic_model_t::NSTATE == 5);

static_assert(Axle_type::state_names::OMEGA_AXLE == 0);
static_assert(Dynamic_model_t::Road_type::state_names::X == 1);
static_assert(Dynamic_model_t::Road_type::state_names::Y == 2);
static_assert(Dynamic_model_t::Road_type::state_names::PSI == 3);
static_assert(Dynamic_model_t::state_names::U == 4);

// check control vector
static_assert(Axle_type::control_names::TORQUE == 0);
static_assert(Dynamic_model_t::NCONTROL == 1);


TEST_F(Dynamic_model_powered_axle_test, state_vector_sizes)
{
    EXPECT_EQ(Axle_type::input_state_names::OMEGA_AXLE, 0);
    EXPECT_EQ(Dynamic_model_t::Road_type::input_state_names::X, 1);
    EXPECT_EQ(Dynamic_model_t::Road_type::input_state_names::Y, 2);
    EXPECT_EQ(Dynamic_model_t::Road_type::input_state_names::PSI, 3);
    EXPECT_EQ(Dynamic_model_t::input_state_names::U, 4);

    EXPECT_EQ(Axle_type::state_names::OMEGA_AXLE, 0);
    EXPECT_EQ(Dynamic_model_t::Road_type::state_names::X, 1);
    EXPECT_EQ(Dynamic_model_t::Road_type::state_names::Y, 2);
    EXPECT_EQ(Dynamic_model_t::Road_type::state_names::PSI, 3);
    EXPECT_EQ(Dynamic_model_t::state_names::U, 4);

    EXPECT_EQ(Dynamic_model_t::NSTATE, 5);
}

TEST_F(Dynamic_model_powered_axle_test, control_vector_sizes)
{
    EXPECT_EQ(Axle_type::control_names::TORQUE, 0);
    EXPECT_EQ(Dynamic_model_t::NCONTROL, 1);
}

TEST_F(Dynamic_model_powered_axle_test, tire_x_forces)
{
    EXPECT_DOUBLE_EQ(Value(_car.get_axle().get_tire<Axle_type::LEFT>().get_force()[0]), 0.0);
    EXPECT_DOUBLE_EQ(Value(_car.get_axle().get_tire<Axle_type::RIGHT>().get_force()[0]), 0.0);
}

TEST_F(Dynamic_model_powered_axle_test, tire_y_forces)
{
    EXPECT_DOUBLE_EQ(Value(_car.get_axle().get_tire<Axle_type::LEFT>().get_force()[1]), 0.0);
    EXPECT_DOUBLE_EQ(Value(_car.get_axle().get_tire<Axle_type::RIGHT>().get_force()[1]), 0.0);
}

TEST_F(Dynamic_model_powered_axle_test, tire_z_forces)
{
    EXPECT_DOUBLE_EQ(Value(_car.get_axle().get_tire<Axle_type::LEFT>().get_force()[2]), -Fz);
    EXPECT_DOUBLE_EQ(Value(_car.get_axle().get_tire<Axle_type::RIGHT>().get_force()[2]),-Fz);
}    


TEST_F(Dynamic_model_powered_axle_test, dqdt)
{
    std::array<scalar,Dynamic_model_t::NSTATE> q = {137.9804,24.2145,0.0,0.0,17.3268};
    std::array<scalar,Dynamic_model_t::NCONTROL> u = {148.59};

    std::array<scalar,Dynamic_model_t::NSTATE> dqdt_matlab = {92.934526888399489, 17.326799999999999, 0.0, 0.0, 5.302878134700940};

    const auto dqdt = _car(q,u,0.0);

    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
        EXPECT_NEAR(Value(dqdt[i]), Value(dqdt_matlab[i]),1.0e-12);
}

class Torque_maximum_accel
{
 public:
    Torque_maximum_accel(const std::vector<scalar>& torque_values) : _torque_values(torque_values), _torque(0.0,{10.0},{_torque_values}){}
    std::array<scalar,1> operator()(const std::array<scalar,5>& q, const scalar t) const { 
        return {_torque[t]};
    }

 private:
    std::vector<scalar> _torque_values;
    sPolynomial _torque;
};



TEST_F(Dynamic_model_powered_axle_test, simulation)
{
    // Torques are provided from an optimization for maximum acceleration
    std::vector<scalar> torque_values = {141.4598, 140.2284, 139.8956, 138.8799, 137.6030, 136.0224, 134.8086, 133.9224, 133.0248, 129.2422};

    const size_t n_timesteps = 2000;
    const scalar dt = 0.005;

    std::array<scalar,5> q = {5.0/0.139,0.0,0.0,0.0,5.0};

    Torque_maximum_accel control(torque_values);

    for (size_t i = 0; i < n_timesteps; ++i)
    {
        RK4<Dynamic_model_t,Torque_maximum_accel,Dynamic_model_t::NSTATE>::take_step(_car, control, q, i*dt, dt);
    }
    
    const std::vector<scalar> q_saved = { 332.90195128105444, 281.20101145486979, 0.0, 0.0, 42.805806972214526};

    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
        EXPECT_NEAR(Value(q[i]), Value(q_saved[i]), 1.0e-12);
}
*/
