#include "gtest/gtest.h"

#include "lion/thirdparty/include/cppad/cppad.hpp"

#include "lion/frame/frame.h"
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

using Road_t                = lot2016kart<scalar>::cartesian::Road_t;
using Dynamic_model_t       = lot2016kart<scalar>::cartesian;

// Make sure that the enumerators are correct
// Controls
static_assert(Front_axle_t::ISTEERING==0);
static_assert(Rear_axle_t::ITORQUE==1);

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
static_assert(Road_t::IX==10);
static_assert(Road_t::IY==11);
static_assert(Road_t::IPSI==12);

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
static_assert(Road_t::IIDX==10);
static_assert(Road_t::IIDY==11);
static_assert(Road_t::IIDPSI==12);


class Control_acceleration
{
 public:
    std::array<scalar,2> operator()(const std::array<scalar,13>& q, const scalar t) const { 
        return {0.0,1.0};
    }
};


class Control_brake
{
 public:
    std::array<scalar,2> operator()(const std::array<scalar,13>& q, const scalar t) const { 
        return {0.0,-0.5};
    }
};

Xml_document* get_database()
{
    Xml_document* database = new Xml_document("database/vehicles/kart/roberto-lot-kart-2016.xml", true ); 

    // set the 'smooth' max parameter to 0
    database->get_element("vehicle/front-tire/Fz-max-ref2").set_value("0.0");
    database->get_element("vehicle/rear-tire/Fz-max-ref2").set_value("0.0");

    return database;
}
  
class Car_road_cartesian_test : public ::testing::Test
{
 protected:
    Car_road_cartesian_test() 
    {
        std::array<scalar,13> q = { omega_axle,u,v,omega,z,phi,mu,dz,dphi,dmu,x,y,psi};
        std::array<scalar,2> u = {delta, T};

        car(q,u,0.0);
    }
    
    ~Car_road_cartesian_test()
    {
        delete database;
    }

    const scalar x = 2.0;
    const scalar y = 4.0;

    const scalar psi = pi/4.0;
    const scalar omega = 0.1;

    const scalar u = 0.95;
    const scalar v = 0.1;

    const scalar dx = u*cos(psi) - v*sin(psi);
    const scalar dy = u*sin(psi) + v*cos(psi);

    const scalar z    = 0.1;
    const scalar dz   = 0.03;
    const scalar mu   = -5.0*DEG;
    const scalar dmu  = 0.4;
    const scalar phi  = 4.0*DEG;
    const scalar dphi = 0.2;

    const scalar delta = -16.0*DEG;

    const scalar T = 0.1*17.6;    // Maximum torque at 10.250rpm
    const scalar omega_axle = 10.0;

    const scalar rho = 1.2;
    const scalar CdA = 0.7;

    Xml_document* database = get_database(); 
    Dynamic_model_t car = *database;

    Xml_document results = { "./data/saved_results.xml", true};
};


const static std::map<std::string,scalar> Lot2016kart_parameters =
{
     std::make_pair("front-axle/track"              , 1.055      ), 
     std::make_pair("front-axle/chassis_stiffness"  , 17.7e3     ), 
     std::make_pair("front-axle/antiroll_stiffness" , 0.0        ), 
     std::make_pair("front-axle/beta_steering"      , 0.058      ), 
     std::make_pair("rear-axle/track"               , 1.200      ), 
     std::make_pair("rear-axle/chassis_stiffness"   , 60.0e3     ), 
     std::make_pair("rear-axle/antiroll_stiffness"  , 0.0        ), 
     std::make_pair("rear-axle/inertia"             , 0.2        ), 
     std::make_pair("chassis/cog_height"            , 0.250      ), 
     std::make_pair("chassis/front_axle_x"          , 0.645      ), 
     std::make_pair("chassis/rear_axle_x"           , -0.400     ), 
     std::make_pair("chassis/front_axle_z"          , 0.250-0.139), 
     std::make_pair("chassis/rear_axle_z"           , 0.250-0.139), 
     std::make_pair("chassis/mass"                  , 165.0      ), 
     std::make_pair("chassis/inertia_xx"            , 20.0       ), 
     std::make_pair("chassis/inertia_yy"            , 15.0       ), 
     std::make_pair("chassis/inertia_zz"            , 25.0       ), 
     std::make_pair("chassis/inertia_xz"            , 5.0        ), 
     std::make_pair("chassis/inertia_xy"            , 0.0        ), 
     std::make_pair("chassis/inertia_yz"            , 0.0        ), 
     std::make_pair("chassis/rho_air"               , 1.2        ), 
     std::make_pair("chassis/CdA"                   , 0.7        ), 
     std::make_pair("rear-tire/R0"                  , 0.139      ), 
     std::make_pair("rear-tire/kt"                  , 61.3e3     ), 
     std::make_pair("rear-tire/ct"                  , 1.0e3      ), 
     std::make_pair("front-tire/R0"                 , 0.139      ), 
     std::make_pair("front-tire/kt"                 , 64.5e3     ), 
     std::make_pair("front-tire/ct"                 , 1.0e3      ) 
};


TEST_F(Car_road_cartesian_test, state_vector_sizes)
{
    static_assert( Dynamic_model_t::NSTATE == 13 );
    static_assert( Dynamic_model_t::NCONTROL == 2 );
    EXPECT_EQ(Dynamic_model_t::NSTATE,13);
    EXPECT_EQ(Dynamic_model_t::NCONTROL,2);
}



TEST_F(Car_road_cartesian_test, copy_constructor_frame_trees_sanity)
{
    const Front_axle_t& front_axle = car.get_chassis().get_front_axle();
    const Rear_axle_t& rear_axle   = car.get_chassis().get_rear_axle();

    const Front_left_tire_type& tire_fl = front_axle.get_tire<Front_axle_t::LEFT>();
    const Front_right_tire_type& tire_fr = front_axle.get_tire<Front_axle_t::RIGHT>();

    const Rear_left_tire_type& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const Rear_right_tire_type& tire_rr = rear_axle.get_tire<Rear_axle_t::RIGHT>();

    EXPECT_EQ(&car.get_chassis().get_inertial_frame(), car.get_chassis().get_road_frame().get_parent_ptr());
    EXPECT_EQ(&car.get_chassis().get_road_frame(), car.get_chassis().get_chassis_frame().get_parent_ptr());

    EXPECT_EQ(&car.get_chassis().get_chassis_frame(), front_axle.get_frame().get_parent_ptr());
    EXPECT_EQ(&car.get_chassis().get_chassis_frame(), rear_axle.get_frame().get_parent_ptr());

    EXPECT_EQ(&front_axle.get_frame(), tire_fl.get_frame().get_parent_ptr());
    EXPECT_EQ(&front_axle.get_frame(), tire_fr.get_frame().get_parent_ptr());

    EXPECT_EQ(&rear_axle.get_frame(), tire_rl.get_frame().get_parent_ptr());
    EXPECT_EQ(&rear_axle.get_frame(), tire_rr.get_frame().get_parent_ptr());
}


TEST_F(Car_road_cartesian_test, copy_assignment_frame_trees_sanity)
{
    Dynamic_model_t car_copy;

    car_copy = car;

    const Front_axle_t& front_axle = car_copy.get_chassis().get_front_axle();
    const Rear_axle_t& rear_axle   = car_copy.get_chassis().get_rear_axle();

    const Front_left_tire_type& tire_fl = front_axle.get_tire<Front_axle_t::LEFT>();
    const Front_right_tire_type& tire_fr = front_axle.get_tire<Front_axle_t::RIGHT>();

    const Rear_left_tire_type& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const Rear_right_tire_type& tire_rr = rear_axle.get_tire<Rear_axle_t::RIGHT>();

    EXPECT_EQ(&car_copy.get_chassis().get_inertial_frame(), car_copy.get_chassis().get_road_frame().get_parent_ptr());

    EXPECT_EQ(&car_copy.get_chassis().get_road_frame(), car_copy.get_chassis().get_chassis_frame().get_parent_ptr());

    EXPECT_EQ(&car_copy.get_chassis().get_chassis_frame(), front_axle.get_frame().get_parent_ptr());
    EXPECT_EQ(&car_copy.get_chassis().get_chassis_frame(), rear_axle.get_frame().get_parent_ptr());

    EXPECT_EQ(&front_axle.get_frame(), tire_fl.get_frame().get_parent_ptr());
    EXPECT_EQ(&front_axle.get_frame(), tire_fr.get_frame().get_parent_ptr());

    EXPECT_EQ(&rear_axle.get_frame(), tire_rl.get_frame().get_parent_ptr());
    EXPECT_EQ(&rear_axle.get_frame(), tire_rr.get_frame().get_parent_ptr());
}
  

TEST_F(Car_road_cartesian_test, chassis_position)
{
    const scalar x_chassis = x;
    const scalar y_chassis = y;
    const scalar z_chassis = z-car.get_chassis().get_parameter("cog_height");

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_origin().at(0)), Value(x_chassis));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_origin().at(1)), Value(y_chassis));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_origin().at(2)), 0.0);

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_absolute_position().at(0)), Value(x_chassis));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_absolute_position().at(1)), Value(y_chassis));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_absolute_position().at(2)), 0.0);

    EXPECT_EQ(car.get_chassis().get_road_frame().get_rotation_angles().size(),1);
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_rotation_angles().at(0)), Value(psi));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_rotation_angles_derivative().at(0)), Value(omega));
    EXPECT_EQ(car.get_chassis().get_road_frame().get_rotation_axis().at(0), Z);

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_origin().at(0)), 0.0);
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_origin().at(1)), 0.0);
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_origin().at(2)), Value(z_chassis));

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_absolute_position().at(0)), Value(x_chassis));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_absolute_position().at(1)), Value(y_chassis));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_absolute_position().at(2)), Value(z_chassis));

    EXPECT_EQ(car.get_chassis().get_chassis_frame().get_rotation_angles().size(), 0);
}


TEST_F(Car_road_cartesian_test, chassis_velocity)
{
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_relative_velocity().at(0)), Value(dx));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_relative_velocity().at(1)), Value(dy));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_relative_velocity().at(2)),0.0);

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_absolute_velocity_in_inertial().at(0)), Value(dx));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_absolute_velocity_in_inertial().at(1)), Value(dy));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_absolute_velocity_in_inertial().at(2)),0.0);

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_absolute_velocity_in_body().at(0)), Value(u));
    EXPECT_NEAR     (Value(car.get_chassis().get_road_frame().get_absolute_velocity_in_body().at(1)), Value(v), 1.0e-16);
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_road_frame().get_absolute_velocity_in_body().at(2)),0.0);

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_relative_velocity().at(0)), 0.0);
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_relative_velocity().at(1)), 0.0);
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_relative_velocity().at(2)), Value(dz));

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_absolute_velocity_in_inertial().at(0)), Value(dx));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_absolute_velocity_in_inertial().at(1)), Value(dy));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_absolute_velocity_in_inertial().at(2)), Value(dz));

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_absolute_velocity_in_body().at(0)), Value( u));
    EXPECT_NEAR     (Value(car.get_chassis().get_chassis_frame().get_absolute_velocity_in_body().at(1)), Value(v), 1.0e-16);
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_chassis_frame().get_absolute_velocity_in_body().at(2)), Value(dz));
}


TEST_F(Car_road_cartesian_test, front_axle_position)
{
    const scalar& R0              = 0.139;
    const Front_axle_t& front_axle = car.get_chassis().get_front_axle();

    const scalar& a = car.get_chassis().get_parameter("front_axle_x");
    const scalar& h = car.get_chassis().get_parameter("cog_height");

    const scalar x_axle = x + a*cos(psi);
    const scalar y_axle = y + a*sin(psi);
    const scalar z_axle = z - a*mu - R0;

    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_origin().at(0)), a);
    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_origin().at(1)), 0.0);
    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_origin().at(2)), Value(h-R0-a*mu));
                    
    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_absolute_position().at(0)), Value(x_axle));
    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_absolute_position().at(1)), Value(y_axle));
    EXPECT_NEAR(Value(front_axle.get_frame().get_absolute_position().at(2)), Value(z_axle), 3.0e-17);

    EXPECT_EQ(front_axle.get_frame().get_rotation_angles().size(), 0);
}


TEST_F(Car_road_cartesian_test, rear_axle_position)
{
    const scalar& R0            = 0.139;
    const Rear_axle_t& rear_axle = car.get_chassis().get_rear_axle();

    const scalar& a = car.get_chassis().get_parameter("rear_axle_x");
    const scalar& h = car.get_chassis().get_parameter("cog_height");

    const scalar x_axle = x + a*cos(psi);
    const scalar y_axle = y + a*sin(psi);
    const scalar z_axle = z - a*mu - R0;

    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_origin().at(0)), a);
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_origin().at(1)),  0.0);
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_origin().at(2)), Value(h-R0-a*mu));
                    
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_absolute_position().at(0)), Value(x_axle));
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_absolute_position().at(1)), Value(y_axle));
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_absolute_position().at(2)), Value(z_axle));

    EXPECT_EQ(rear_axle.get_frame().get_rotation_angles().size(), 0);
}


TEST_F(Car_road_cartesian_test, front_axle_velocity)
{
    const Front_axle_t& front_axle = car.get_chassis().get_front_axle();

    const scalar& a = car.get_chassis().get_parameter("front_axle_x");

    const scalar u_axle = u;
    const scalar v_axle = v + a*omega;

    const scalar dx_axle = u_axle*cos(psi) - v_axle*sin(psi);
    const scalar dy_axle = u_axle*sin(psi) + v_axle*cos(psi);

    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_relative_velocity().at(0)),  0.0);
    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_relative_velocity().at(1)),  0.0);
    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_relative_velocity().at(2)), Value(-a*dmu));

    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_absolute_velocity_in_body().at(0)), Value(u));
    EXPECT_NEAR     (Value(front_axle.get_frame().get_absolute_velocity_in_body().at(1)), Value(v+a*omega), 2.0e-16);
    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_absolute_velocity_in_body().at(2)), Value(dz-a*dmu));
                    
    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_absolute_velocity_in_inertial().at(0)), Value(dx_axle));
    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_absolute_velocity_in_inertial().at(1)), Value(dy_axle));
    EXPECT_DOUBLE_EQ(Value(front_axle.get_frame().get_absolute_velocity_in_inertial().at(2)), Value(dz-a*dmu));
}


TEST_F(Car_road_cartesian_test, rear_axle_velocity)
{
    const Rear_axle_t& rear_axle = car.get_chassis().get_rear_axle();

    const scalar& a = car.get_chassis().get_parameter("rear_axle_x");

    const scalar u_axle = u;
    const scalar v_axle = v + a*omega;

    const scalar dx_axle = u_axle*cos(psi) - v_axle*sin(psi);
    const scalar dy_axle = u_axle*sin(psi) + v_axle*cos(psi);

    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_relative_velocity().at(0)),  0.0);
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_relative_velocity().at(1)),  0.0);
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_relative_velocity().at(2)), Value(-a*dmu));

    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_absolute_velocity_in_body().at(0)), Value(u));
    EXPECT_NEAR     (Value(rear_axle.get_frame().get_absolute_velocity_in_body().at(1)), Value(v+a*omega), 2.0e-16);
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_absolute_velocity_in_body().at(2)), Value(dz-a*dmu));
                    
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_absolute_velocity_in_inertial().at(0)), Value(dx_axle));
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_absolute_velocity_in_inertial().at(1)), Value(dy_axle));
    EXPECT_DOUBLE_EQ(Value(rear_axle.get_frame().get_absolute_velocity_in_inertial().at(2)), Value(dz-a*dmu));
}


TEST_F(Car_road_cartesian_test, front_left_tire_position)
{
    const scalar& R0         = 0.139;
    const Front_axle_t& axle  = car.get_chassis().get_front_axle();
    const Front_left_tire_type& tire = axle.get_tire<Front_axle_t::LEFT>();

    const scalar& a = car.get_chassis().get_parameter("front_axle_x");
    const scalar& t = axle.get_parameter("track");
    const scalar& beta = axle.get_parameter("beta_steering");

    const scalar& s = axle.get_chassis_deformation(Front_axle_t::LEFT);

    const scalar x_tire = x + a*cos(psi) + 0.5*t*sin(psi);
    const scalar y_tire = y + a*sin(psi) - 0.5*t*cos(psi);
    const scalar z_tire = z - R0 - a*mu - 0.5*t*phi - beta*delta + s;

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(0)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(1)), -0.5*t);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(2)), Value(-0.5*t*phi - beta*delta + s ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(0)), Value(x_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(1)), Value(y_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(2)), Value(z_tire ));
}


TEST_F(Car_road_cartesian_test, front_right_tire_position)
{
    const scalar& R0         = 0.139;
    const Front_axle_t& axle  = car.get_chassis().get_front_axle();
    const Front_right_tire_type& tire = axle.get_tire<Front_axle_t::RIGHT>();

    const scalar& a = car.get_chassis().get_parameter("front_axle_x");
    const scalar& t = axle.get_parameter("track");
    const scalar& beta = axle.get_parameter("beta_steering");

    const scalar& s = axle.get_chassis_deformation(Front_axle_t::RIGHT);

    const scalar x_tire = x + a*cos(psi) - 0.5*t*sin(psi);
    const scalar y_tire = y + a*sin(psi) + 0.5*t*cos(psi);
    const scalar z_tire = z - R0 - a*mu + 0.5*t*phi + beta*delta + s;

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(0)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(1)), 0.5*t);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(2)), Value(0.5*t*phi + beta*delta + s ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(0)), Value(x_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(1)), Value(y_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(2)), Value(z_tire ));
}


TEST_F(Car_road_cartesian_test, rear_left_tire_position)
{
    const scalar& R0         = 0.139;
    const Rear_axle_t& axle   = car.get_chassis().get_rear_axle();
    const Rear_left_tire_type& tire = axle.get_tire<Rear_axle_t::LEFT>();

    const scalar& a = car.get_chassis().get_parameter("rear_axle_x");
    const scalar& t = axle.get_parameter("track");

    const scalar& s = axle.get_chassis_deformation(Rear_axle_t::LEFT);

    const scalar x_tire = x + a*cos(psi) + 0.5*t*sin(psi);
    const scalar y_tire = y + a*sin(psi) - 0.5*t*cos(psi);
    const scalar z_tire = z - R0 - a*mu - 0.5*t*phi + s;

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(0)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(1)), -0.5*t);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(2)), Value(-0.5*t*phi + s ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(0)), Value(x_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(1)), Value(y_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(2)), Value(z_tire ));
}


TEST_F(Car_road_cartesian_test, rear_right_tire_position)
{
    const scalar& R0         = 0.139;
    const Rear_axle_t& axle   = car.get_chassis().get_rear_axle();
    const Rear_right_tire_type& tire = axle.get_tire<Rear_axle_t::RIGHT>();

    const scalar& a = car.get_chassis().get_parameter("rear_axle_x");
    const scalar& t = axle.get_parameter("track");

    const scalar& s = axle.get_chassis_deformation(Rear_axle_t::RIGHT);

    const scalar x_tire = x + a*cos(psi) - 0.5*t*sin(psi);
    const scalar y_tire = y + a*sin(psi) + 0.5*t*cos(psi);
    const scalar z_tire = z - R0 - a*mu + 0.5*t*phi + s;

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(0)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(1)), 0.5*t);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_origin().at(2)), Value(0.5*t*phi + s ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(0)), Value(x_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(1)), Value(y_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_position().at(2)), Value(z_tire ));
}


TEST_F(Car_road_cartesian_test, front_axle_deformations)
{
    const scalar& R0               = 0.139;
    const Front_axle_t& axle        = car.get_chassis().get_front_axle();
    const Front_left_tire_type& tire_left  = axle.get_tire<Front_axle_t::LEFT>();
    const Front_right_tire_type& tire_right = axle.get_tire<Front_axle_t::RIGHT>();

    const scalar& a = car.get_chassis().get_parameter("front_axle_x");
    const scalar& t = axle.get_parameter("track");
    const scalar& beta = axle.get_parameter("beta_steering");

    const scalar& kchassis  = axle.get_parameter("chassis_stiffness");
    const scalar& kantiroll = axle.get_parameter("antiroll_stiffness");
    const scalar& ktire     = Lot2016kart_parameters.at("front-tire/kt");

    const scalar wl = tire_left.get_frame().get_absolute_position({0.0, 0.0, R0}).at(Z);
    const scalar wr = tire_right.get_frame().get_absolute_position({0.0, 0.0, R0}).at(Z);

    const scalar sl = axle.get_chassis_deformation(Front_axle_t::LEFT);
    const scalar sr = axle.get_chassis_deformation(Front_axle_t::RIGHT);

    EXPECT_NEAR(Value(kchassis*sl + kantiroll*(sl-sr) + ktire*wl), 0.0, 1.0e-16*ktire);
    EXPECT_NEAR(Value(kchassis*sr + kantiroll*(sr-sl) + ktire*wr), 0.0, 1.0e-16*ktire);
    EXPECT_DOUBLE_EQ(Value(wl), Value(z - a*mu - 0.5*t*phi - beta*delta + sl));
    EXPECT_DOUBLE_EQ(Value(wr), Value(z - a*mu + 0.5*t*phi + beta*delta + sr));
    
    EXPECT_DOUBLE_EQ(Value(wl), Value(tire_left.get_vertical_deformation()));
    EXPECT_DOUBLE_EQ(Value(wr), Value(tire_right.get_vertical_deformation()));
}


TEST_F(Car_road_cartesian_test, rear_axle_deformations)
{
    const scalar& R0               = 0.139;
    const Rear_axle_t& axle         = car.get_chassis().get_rear_axle();
    const Rear_left_tire_type& tire_left  = axle.get_tire<Rear_axle_t::LEFT>();
    const Rear_right_tire_type& tire_right = axle.get_tire<Rear_axle_t::RIGHT>();

    const scalar& a = Lot2016kart_parameters.at("chassis/rear_axle_x");
    const scalar& t = Lot2016kart_parameters.at("rear-axle/track");

    const scalar& kchassis  = Lot2016kart_parameters.at("rear-axle/chassis_stiffness");
    const scalar& kantiroll = Lot2016kart_parameters.at("rear-axle/antiroll_stiffness");
    const scalar& ktire     = Lot2016kart_parameters.at("rear-tire/kt");

    const scalar wl = tire_left.get_frame().get_absolute_position({0.0, 0.0, R0}).at(Z);
    const scalar wr = tire_right.get_frame().get_absolute_position({0.0, 0.0, R0}).at(Z);

    const scalar sl = axle.get_chassis_deformation(Rear_axle_t::LEFT);
    const scalar sr = axle.get_chassis_deformation(Rear_axle_t::RIGHT);

    EXPECT_NEAR(Value(kchassis*sl + kantiroll*(sl-sr) + ktire*wl), 0.0, 1.0e-16*ktire);
    EXPECT_NEAR(Value(kchassis*sr + kantiroll*(sr-sl) + ktire*wr), 0.0, 1.0e-16*ktire);
    EXPECT_DOUBLE_EQ(Value(wl), Value(z - a*mu - 0.5*t*phi + sl));
    EXPECT_DOUBLE_EQ(Value(wr), Value(z - a*mu + 0.5*t*phi + sr));

    EXPECT_DOUBLE_EQ(Value(wl), Value(tire_left.get_vertical_deformation()));
    EXPECT_DOUBLE_EQ(Value(wr), Value(tire_right.get_vertical_deformation()));
}


TEST_F(Car_road_cartesian_test, front_axle_deformations_velocity)
{
    const scalar& R0               = 0.139;
    const Front_axle_t& axle        = car.get_chassis().get_front_axle();
    const Front_left_tire_type& tire_left  = axle.get_tire<Front_axle_t::LEFT>();
    const Front_right_tire_type& tire_right = axle.get_tire<Front_axle_t::RIGHT>();

    const scalar& a = Lot2016kart_parameters.at("chassis/front_axle_x");
    const scalar& t = Lot2016kart_parameters.at("front-axle/track");

    const scalar& kchassis  = Lot2016kart_parameters.at("front-axle/chassis_stiffness");
    const scalar& kantiroll = Lot2016kart_parameters.at("front-axle/antiroll_stiffness");
    const scalar& ktire     = 64.5e3;

    const scalar dwl = tire_left.get_frame().get_absolute_velocity_in_body({0.0, 0.0, R0}).at(Z);
    const scalar dwr = tire_right.get_frame().get_absolute_velocity_in_body({0.0, 0.0, R0}).at(Z);

    const scalar dsl = axle.get_chassis_deformation_velocity(Front_axle_t::LEFT);
    const scalar dsr = axle.get_chassis_deformation_velocity(Front_axle_t::RIGHT);

    EXPECT_NEAR(Value(kchassis*dsl + kantiroll*(dsl-dsr) + ktire*dwl), 0.0, 1.0e-16*ktire);
    EXPECT_NEAR(Value(kchassis*dsr + kantiroll*(dsr-dsl) + ktire*dwr), 0.0, 1.0e-16*ktire);
    EXPECT_DOUBLE_EQ(Value(dwl), Value(dz - a*dmu - 0.5*t*dphi + dsl));
    EXPECT_DOUBLE_EQ(Value(dwr), Value(dz - a*dmu + 0.5*t*dphi + dsr));
    
    EXPECT_DOUBLE_EQ(Value(dwl), Value(tire_left.get_vertical_deformation_velocity()));
    EXPECT_DOUBLE_EQ(Value(dwr), Value(tire_right.get_vertical_deformation_velocity()));
}


TEST_F(Car_road_cartesian_test, rear_axle_deformations_velocity)
{
    const scalar& R0               = 0.139;
    const Rear_axle_t& axle         = car.get_chassis().get_rear_axle();
    const Rear_left_tire_type& tire_left  = axle.get_tire<Rear_axle_t::LEFT>();
    const Rear_right_tire_type& tire_right = axle.get_tire<Rear_axle_t::RIGHT>();

    const scalar& a = Lot2016kart_parameters.at("chassis/rear_axle_x");
    const scalar& t = Lot2016kart_parameters.at("rear-axle/track");

    const scalar& kchassis  = Lot2016kart_parameters.at("rear-axle/chassis_stiffness");
    const scalar& kantiroll = Lot2016kart_parameters.at("rear-axle/antiroll_stiffness");
    const scalar& ktire     = 61.3e3;

    const scalar dwl = tire_left.get_frame().get_absolute_velocity_in_body({0.0, 0.0, R0}).at(Z);
    const scalar dwr = tire_right.get_frame().get_absolute_velocity_in_body({0.0, 0.0, R0}).at(Z);

    const scalar dsl = axle.get_chassis_deformation_velocity(Rear_axle_t::LEFT);
    const scalar dsr = axle.get_chassis_deformation_velocity(Rear_axle_t::RIGHT);

    EXPECT_NEAR(Value(kchassis*dsl + kantiroll*(dsl-dsr) + ktire*dwl), 0.0, 1.0e-16*ktire);
    EXPECT_NEAR(Value(kchassis*dsr + kantiroll*(dsr-dsl) + ktire*dwr), 0.0, 1.0e-16*ktire);
    EXPECT_DOUBLE_EQ(Value(dwl), Value(dz - a*dmu - 0.5*t*dphi + dsl));
    EXPECT_DOUBLE_EQ(Value(dwr), Value(dz - a*dmu + 0.5*t*dphi + dsr));
    
    EXPECT_DOUBLE_EQ(Value(dwl), Value(tire_left.get_vertical_deformation_velocity()));
    EXPECT_DOUBLE_EQ(Value(dwr), Value(tire_right.get_vertical_deformation_velocity()));
}


TEST_F(Car_road_cartesian_test, front_left_tire_velocity)
{
    const Front_axle_t& axle  = car.get_chassis().get_front_axle();
    const Front_left_tire_type& tire = axle.get_tire<Front_axle_t::LEFT>();

    const scalar& a = Lot2016kart_parameters.at("chassis/front_axle_x");
    const scalar& t = Lot2016kart_parameters.at("front-axle/track");

    const scalar ds = axle.get_chassis_deformation_velocity(Front_axle_t::LEFT);

    const scalar u_tire = u + 0.5*t*omega;
    const scalar v_tire = v + a*omega;

    const scalar u_body = u_tire*cos(delta) + v_tire*sin(delta);
    const scalar v_body = v_tire*cos(delta) - u_tire*sin(delta);
    

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(0)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(1)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(2)), Value(ds - 0.5*t*dphi ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_parent().at(0)), Value(u_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_parent().at(1)), Value(v_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_parent().at(2)), Value(dz - a*dmu + ds - 0.5*t*dphi ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_body().at(0)), Value(u_body));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_body().at(1)), Value(v_body));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_body().at(2)), Value(dz - a*dmu + ds - 0.5*t*dphi ));

    EXPECT_DOUBLE_EQ(Value(tire.get_lambda()), Value(-v_body/u_body));
    EXPECT_DOUBLE_EQ(Value(tire.get_kappa()),  0.0);
}


TEST_F(Car_road_cartesian_test, front_right_tire_velocity)
{
    const Front_axle_t& axle  = car.get_chassis().get_front_axle();
    const Front_right_tire_type& tire = axle.get_tire<Front_axle_t::RIGHT>();

    const scalar& a = Lot2016kart_parameters.at("chassis/front_axle_x");
    const scalar& t = Lot2016kart_parameters.at("front-axle/track");

    const scalar ds = axle.get_chassis_deformation_velocity(Front_axle_t::RIGHT);

    const scalar u_tire = u - 0.5*t*omega;
    const scalar v_tire = v + a*omega;

    const scalar u_body = u_tire*cos(delta) + v_tire*sin(delta);
    const scalar v_body = v_tire*cos(delta) - u_tire*sin(delta);

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(0)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(1)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(2)), Value(ds + 0.5*t*dphi ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_parent().at(0)), Value(u_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_parent().at(1)), Value(v_tire));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_parent().at(2)), Value(dz - a*dmu + ds + 0.5*t*dphi ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_body().at(0)), Value(u_body));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_body().at(1)), Value(v_body));
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_body().at(2)), Value(dz - a*dmu + ds + 0.5*t*dphi ));

    EXPECT_DOUBLE_EQ(Value(tire.get_lambda()), Value(-v_body/u_body));
    EXPECT_NEAR     (Value(tire.get_kappa()),  0.0, 3.0e-16);
}


TEST_F(Car_road_cartesian_test, rear_left_tire_velocity)
{
    const scalar& R0         = 0.139;
    const Rear_axle_t& axle   = car.get_chassis().get_rear_axle();
    const Rear_left_tire_type& tire = axle.get_tire<Rear_axle_t::LEFT>();

    const scalar& a = Lot2016kart_parameters.at("chassis/rear_axle_x");
    const scalar& t = Lot2016kart_parameters.at("rear-axle/track");

    const scalar ds = axle.get_chassis_deformation_velocity(Rear_axle_t::LEFT);

    const scalar u_tire = u + 0.5*t*omega;
    const scalar v_tire = v + a*omega;

    const scalar u_body = u_tire;
    const scalar v_body = v_tire;
    

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(0)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(1)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(2)), Value(ds - 0.5*t*dphi ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_parent().at(0)), Value(u_tire));
    EXPECT_NEAR     (Value(tire.get_frame().get_absolute_velocity_in_parent().at(1)), Value(v_tire), 1.0e-16);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_parent().at(2)), Value(dz - a*dmu + ds - 0.5*t*dphi ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_body().at(0)), Value(u_body));
    EXPECT_NEAR     (Value(tire.get_frame().get_absolute_velocity_in_body().at(1)), Value(v_body), 1.0e-16);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_body().at(2)), Value(dz - a*dmu + ds - 0.5*t*dphi ));

    EXPECT_NEAR     (Value(tire.get_lambda()), Value(-v_body/u_body),1.0e-16);
    EXPECT_NEAR     (Value(tire.get_kappa()), Value( omega_axle*R0/u_body - 1.0),5.0e-16);
}


TEST_F(Car_road_cartesian_test, rear_right_tire_velocity)
{
    const scalar& R0         = 0.139;
    const Rear_axle_t& axle   = car.get_chassis().get_rear_axle();
    const Rear_right_tire_type& tire = axle.get_tire<Rear_axle_t::RIGHT>();

    const scalar& a = Lot2016kart_parameters.at("chassis/rear_axle_x");
    const scalar& t = Lot2016kart_parameters.at("rear-axle/track");

    const scalar ds = axle.get_chassis_deformation_velocity(Rear_axle_t::RIGHT);

    const scalar u_tire = u - 0.5*t*omega;
    const scalar v_tire = v + a*omega;

    const scalar u_body = u_tire;
    const scalar v_body = v_tire;

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(0)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(1)), 0.0);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_relative_velocity().at(2)), Value(ds + 0.5*t*dphi ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_parent().at(0)), Value(u_tire));
    EXPECT_NEAR     (Value(tire.get_frame().get_absolute_velocity_in_parent().at(1)), Value(v_tire), 1.0e-16);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_parent().at(2)), Value(dz - a*dmu + ds + 0.5*t*dphi ));

    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_body().at(0)), Value(u_body));
    EXPECT_NEAR     (Value(tire.get_frame().get_absolute_velocity_in_body().at(1)), Value(v_body), 1.0e-16);
    EXPECT_DOUBLE_EQ(Value(tire.get_frame().get_absolute_velocity_in_body().at(2)), Value(dz - a*dmu + ds + 0.5*t*dphi ));

    EXPECT_NEAR     (Value(tire.get_lambda()), Value(-v_body/u_body), 1.0e-16);
    EXPECT_DOUBLE_EQ(Value(tire.get_kappa()), Value( omega_axle*R0/u_body - 1.0));
}


TEST_F(Car_road_cartesian_test, total_force)
{
    const Front_axle_t& front_axle = car.get_chassis().get_front_axle();
    const Rear_axle_t& rear_axle   = car.get_chassis().get_rear_axle();

    const Front_left_tire_type& tire_fl = front_axle.get_tire<Front_axle_t::LEFT>();
    const Front_right_tire_type& tire_fr = front_axle.get_tire<Front_axle_t::RIGHT>();
    const Rear_left_tire_type& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const Rear_right_tire_type& tire_rr = rear_axle.get_tire<Rear_axle_t::RIGHT>();
    
    const sVector3d& F_fl = tire_fl.get_force();
    const sVector3d& F_fr = tire_fr.get_force();
    const sVector3d& F_rl = tire_rl.get_force();
    const sVector3d& F_rr = tire_rr.get_force();
    
    // Forces as written in Lot 2016 (except the normal, we use the other sign convention)
    const scalar Fx = F_rl[X] + F_rr[X] - (F_fl[Y]+F_fr[Y])*sin(delta);
    const scalar Fy = F_rl[Y] + F_rr[Y] + (F_fl[Y]+F_fr[Y])*cos(delta);
    const scalar Fz = car.get_chassis().get_mass()*g0 + F_fl[Z] + F_fr[Z] + F_rl[Z] + F_rr[Z];

    const scalar Fx_aero = 0.5*Lot2016kart_parameters.at("chassis/rho_air")*Lot2016kart_parameters.at("chassis/CdA")*u*u;

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_force().at(X)), Value(Fx-Fx_aero));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_force().at(Y)), Value(Fy));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_force().at(Z)), Value(Fz));

    EXPECT_TRUE(Value(F_fl[Z]) <= 0.0);
    EXPECT_TRUE(Value(F_fr[Z]) <= 0.0);
    EXPECT_TRUE(Value(F_rl[Z]) <= 0.0);
    EXPECT_TRUE(Value(F_rr[Z]) <= 0.0);
}


TEST_F(Car_road_cartesian_test, total_torque)
{
    const Front_axle_t& front_axle = car.get_chassis().get_front_axle();
    const Rear_axle_t& rear_axle   = car.get_chassis().get_rear_axle();

    const Front_left_tire_type& tire_fl = front_axle.get_tire<Front_axle_t::LEFT>();
    const Front_right_tire_type& tire_fr = front_axle.get_tire<Front_axle_t::RIGHT>();
    const Rear_left_tire_type& tire_rl = rear_axle.get_tire<Rear_axle_t::LEFT>();
    const Rear_right_tire_type& tire_rr = rear_axle.get_tire<Rear_axle_t::RIGHT>();

    const scalar  tf = 0.5*Lot2016kart_parameters.at("front-axle/track");
    const scalar  tr = 0.5*Lot2016kart_parameters.at("rear-axle/track");

    const scalar& a = Lot2016kart_parameters.at("chassis/front_axle_x");
    const scalar& b = - Lot2016kart_parameters.at("chassis/rear_axle_x");
    const scalar& h       = Lot2016kart_parameters.at("chassis/cog_height");
    
    const sVector3d& F_fl = tire_fl.get_force();
    const sVector3d& F_fr = tire_fr.get_force();
    const sVector3d& F_rl = tire_rl.get_force();
    const sVector3d& F_rr = tire_rr.get_force();
    
    // Torques as written in Lot 2016
    const scalar Tx = - tf*(F_fl[Z] - F_fr[Z]) - tr*(F_rl[Z] - F_rr[Z]);
    const scalar Ty = - a*(F_fr[Z] + F_fl[Z]) + b*(F_rr[Z] + F_rl[Z]) - 0.5*rho*u*u*CdA*(z-h);
    const scalar Tz = a*(F_fl[Y]+F_fr[Y])*cos(delta) - b*(F_rl[Y]+F_rr[Y])
                          +tf*(F_fr[Y]-F_fl[Y])*sin(delta) + tr*(F_rl[X] - F_rr[X]);

    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_torque().at(X)), Value(Tx));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_torque().at(Y)), Value(Ty));
    EXPECT_DOUBLE_EQ(Value(car.get_chassis().get_torque().at(Z)), Value(Tz));

    EXPECT_EQ(F_fl[Z] <= 0.0, true);
    EXPECT_EQ(F_fr[Z] <= 0.0, true);
    EXPECT_EQ(F_rl[Z] <= 0.0, true);
    EXPECT_EQ(F_rr[Z] <= 0.0, true);
}


TEST_F(Car_road_cartesian_test, newton_equations)
{
    const scalar m     = Lot2016kart_parameters.at("chassis/mass");
    const sVector3d dv = car.get_chassis().get_acceleration();

    EXPECT_DOUBLE_EQ(Value(m*(dv[X] - omega*v)), Value(car.get_chassis().get_force().at(X)));
    EXPECT_DOUBLE_EQ(Value(m*(dv[Y] + omega*u)), Value(car.get_chassis().get_force().at(Y)));
    EXPECT_DOUBLE_EQ(Value(m*dv[Z]), Value(car.get_chassis().get_force().at(Z)));
}


TEST_F(Car_road_cartesian_test, euler_equations)
{
    const scalar m        = Lot2016kart_parameters.at("chassis/mass");
    const sVector3d dv    = car.get_chassis().get_acceleration();
    const sVector3d d2phi = car.get_chassis().get_angles_acceleration();
    const scalar& h       = Lot2016kart_parameters.at("chassis/cog_height");

    const scalar Ixx = Lot2016kart_parameters.at("chassis/inertia_xx");
    const scalar Iyy = Lot2016kart_parameters.at("chassis/inertia_yy");
    const scalar Izz = Lot2016kart_parameters.at("chassis/inertia_zz");
    const scalar Ixz = Lot2016kart_parameters.at("chassis/inertia_xz");

    EXPECT_DOUBLE_EQ(Value(m*(dv[Y]*h + (h-z)*omega*u) + Ixx*d2phi[X] - (Iyy-Izz+Ixx)*omega*dmu
                     -(Iyy-Izz)*omega*omega*phi - (Ixx-Izz)*d2phi[Z]*mu - Ixz*d2phi[Z]), Value(car.get_chassis().get_torque().at(X)));

    EXPECT_DOUBLE_EQ(Value(m*((z-h)*dv[X] + h*omega*v) + Iyy*d2phi[Y] + (Iyy-Izz+Ixx)*omega*dphi
                     -(Ixx-Izz)*omega*omega*mu+(Iyy-Izz)*d2phi[Z]*phi - Ixz*omega*omega),Value(car.get_chassis().get_torque().at(Y)));

    EXPECT_DOUBLE_EQ(Value(Izz*d2phi[Z] - Ixz*(d2phi[X]-2.0*mu*d2phi[Z]-2.0*dmu*omega)), Value(car.get_chassis().get_torque().at(Z)));
}


TEST_F(Car_road_cartesian_test, sizes)
{
    EXPECT_EQ(car.NSTATE, 13);
    EXPECT_EQ(car.NCONTROL, 2);

    static_assert(Dynamic_model_t::NSTATE == 13);
}

TEST_F(Car_road_cartesian_test, dqdt)
{
    const sVector3d dv    = car.get_chassis().get_acceleration();
    const sVector3d d2phi = car.get_chassis().get_angles_acceleration();

    const scalar& domega_axle = car.get_chassis().get_rear_axle().get_omega_derivative();

    std::array<scalar,13> q = { omega_axle,u,v,omega,z,phi,mu,dz,dphi,dmu,x,y,psi};
    std::array<scalar,2> u = {delta, T};

    const auto dqdt = car(q,u,0.0);

    EXPECT_EQ(dqdt.size(), 12+1);

    EXPECT_DOUBLE_EQ(Value(dqdt.at(Road_t::IIDX)), Value(dx));
    EXPECT_DOUBLE_EQ(Value(dqdt.at(Road_t::IIDY)), Value(dy));
    EXPECT_DOUBLE_EQ(Value(dqdt.at(Road_t::IIDPSI)), Value(omega));

    EXPECT_DOUBLE_EQ(Value(dqdt.at(Chassis_t::IIDZ)), Value(dz));

    EXPECT_DOUBLE_EQ(Value(dqdt.at(Chassis_t::IIDPHI)), Value(dphi));
    EXPECT_DOUBLE_EQ(Value(dqdt.at(Chassis_t::IIDMU)), Value(dmu));

    EXPECT_DOUBLE_EQ(Value(dqdt.at(Chassis_t::IIDU)), Value(dv[X]));
    EXPECT_DOUBLE_EQ(Value(dqdt.at(Chassis_t::IIDV)), Value(dv[Y]));
    EXPECT_DOUBLE_EQ(Value(dqdt.at(Chassis_t::IID2Z)), Value(dv[Z]));

    EXPECT_DOUBLE_EQ(Value(dqdt.at(Chassis_t::IID2PHI)), Value(d2phi[X]));
    EXPECT_DOUBLE_EQ(Value(dqdt.at(Chassis_t::IID2MU)), Value(d2phi[Y]));
    EXPECT_DOUBLE_EQ(Value(dqdt.at(Chassis_t::IIDOMEGA)), Value(d2phi[Z]));

    EXPECT_DOUBLE_EQ(Value(dqdt.at(Rear_axle_t::IIDOMEGA_AXLE)), Value(domega_axle));
}


TEST_F(Car_road_cartesian_test, acceleration_simulation)
{
    const std::vector<scalar> q_saved = results.get_element("vehicles_test/car_cartesian/acceleration").get_value(std::vector<scalar>());
    const size_t n_timesteps = 5000;
    const scalar dt = 0.02;
    const scalar v0 = 18.0;

    std::array<scalar,13> q = {v0/0.139,v0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    Control_acceleration control;

    for (size_t i = 0; i < n_timesteps; ++i)
    {
        RK4<Dynamic_model_t,Control_acceleration,Dynamic_model_t::NSTATE>::take_step(car, control, q, i*dt, dt);
    }

    for (size_t i = 0; i < q.size(); ++i)
        EXPECT_NEAR(Value(q.at(i)), Value(q_saved.at(i)), 1.0e-12);
}
  
TEST_F(Car_road_cartesian_test, braking_simulation)
{
    const std::vector<scalar> q_saved = results.get_element("vehicles_test/car_cartesian/braking").get_value(std::vector<scalar>());

    const size_t n_timesteps = 500;
    const scalar dt = 0.010;
    const scalar v0 = 25.0;

    std::array<scalar,13> q = {v0/0.139,v0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    Control_brake control;

    for (size_t i = 0; i < n_timesteps; ++i)
    {
        RK4<Dynamic_model_t,Control_brake,Dynamic_model_t::NSTATE>::take_step(car, control, q, i*dt, dt);
    }

    for (size_t i = 0; i < q.size(); ++i)
        EXPECT_NEAR(Value(q.at(i)), Value(q_saved.at(i)),2.0e-08);
}


TEST_F(Car_road_cartesian_test, jacobian_autodiff)
{
    lot2016kart<CppAD::AD<double>>::cartesian car_ad(*database);
    car_ad.get_chassis().get_rear_axle().enable_direct_torque(); 
    // Set the initial condition in a vector of scalar
    std::vector<double> x0_double = {1.0942729386749036e+02, 1.3888864656170503e+01, -2.5944758219334368e-02, 1.4399999999999999e-01, 1.7813769736939513e-02, -2.8474281872469206e-03, 6.8675368270008927e-03, 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00, 1.8680236782072025e-03, 9.4530179243706314e-03, 0.0000000000000000e+00};
  
    std::vector<CppAD::AD<double>> x0(15);

    for (size_t i = 0; i < 15; ++i)
    {
        x0[i] = x0_double[i];
    }

    // Declare the contents of x0 as the independent variables
    CppAD::Independent(x0);

    // Fill the inputs to the operator() of the vehicle 
    std::array<CppAD::AD<double>,13> q0;
    std::array<CppAD::AD<double>,2> u0;
    for (size_t i = 0; i < 13; ++i)
    {
        q0[i] = x0[i];
    }
    for (size_t i = 0; i < 2; ++i)
    {
        u0[i] = x0[i+13];
    }

    // Call operator()
    auto dqdt_out = car_ad(q0,u0,0.0);
    std::vector<CppAD::AD<double>> dqdt_v(dqdt_out.cbegin(), dqdt_out.cend());

    // Create the AD function and stop the recording
    CppAD::ADFun<double> adfun;
    adfun.Dependent(x0,dqdt_v);

    // Evaluate dqdt = f(q0,u0,0)
    auto dqdt0 = adfun.Forward(0, x0_double);
    auto dqdt0_jacobian = adfun.Jacobian(x0_double);

    EXPECT_EQ(dqdt0_jacobian.size(), 15*13);

    // Compute the numerical Jacobian

    std::array<std::array<CppAD::AD<double>,13>,15> numerical_jacobian;

    // derivatives w.r.t q
    for (size_t i = 0; i < 13; ++i)
    {
        // Freeze u0 and add a perturbation on q0 
        const double eps = std::max(1.0,Value(fabs(q0[i])))*1.0e-8;
        q0[i] += eps;

        auto dqdt_eps = car_ad(q0,u0,0.0);

        q0[i] -= 2*eps;

        auto dqdt_meps = car_ad(q0,u0,0.0);

        numerical_jacobian[i] = (dqdt_eps - dqdt_meps)*(0.5/eps);

        q0[i] += eps;
    }

    // derivatives w.r.t u
    for (size_t i = 0; i < 2; ++i)
    {
        // Freeze u0 and add a perturbation on q0 
        const double eps = std::max(1.0,Value(fabs(u0[i])))*1.0e-8;
        u0[i] += eps;

        auto dqdt_eps = car_ad(q0,u0,0.0);

        u0[i] -= 2*eps;

        auto dqdt_meps = car_ad(q0,u0,0.0);

        numerical_jacobian[i+13] = (dqdt_eps - dqdt_meps)*(0.5/eps);

        u0[i] += eps;
    }


    for (size_t i = 0; i < 15; ++i)
        for (size_t j = 0; j < 13; ++j)
        {
            EXPECT_NEAR(Value(numerical_jacobian[i][j]), Value(dqdt0_jacobian[i + 15*j]), 2.0e-6*std::max(Value(fabs(dqdt0_jacobian[i+15*j])),1.0)) << "with i = " << i << " and j = " << j ;
        }
}

TEST_F(Car_road_cartesian_test, jacobian_autodiff_integrated_function)
{
    lot2016kart<CppAD::AD<double>>::cartesian car_ad(*database);
    lot2016kart<double>::cartesian            car_sc(*database);
    car_ad.get_chassis().get_rear_axle().enable_direct_torque(); 
    car_sc.get_chassis().get_rear_axle().enable_direct_torque(); 
    // Set the initial condition in a vector of scalar
    std::vector<double> x0 = {1.0942729386749036e+02, 1.3888864656170503e+01, -2.5944758219334368e-02, 1.4399999999999999e-01, 1.7813769736939513e-02, -2.8474281872469206e-03, 6.8675368270008927e-03, 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00, 0.0000000000000000e+00, 1.8680236782072025e-03, 9.4530179243706314e-03, 0.0000000000000000e+00};
  
    // Fill the inputs to the operator() of the vehicle 
    std::array<double,13> q0;
    std::array<double,2> u0;
    for (size_t i = 0; i < 13; ++i)
    {
        q0[i] = x0[i];
    }
    for (size_t i = 0; i < 2; ++i)
    {
        u0[i] = x0[i+13];
    }

    // Call operator()
    auto solution = car_ad.equations(q0,{},u0,0.0);

    // Compute the numerical Jacobian

    std::array<std::array<double,13>,15> numerical_jacobian;

    // derivatives w.r.t q
    for (size_t i = 0; i < 13; ++i)
    {
        // Freeze u0 and add a perturbation on q0 
        const double eps = std::max(1.0,fabs(q0[i]))*1.0e-8;
        q0[i] += eps;

        auto dqdt_eps = car_sc(q0,u0,0.0);

        q0[i] -= 2*eps;

        auto dqdt_meps = car_sc(q0,u0,0.0);

        numerical_jacobian[i] = (dqdt_eps - dqdt_meps)*(0.5/eps);

        q0[i] += eps;
    }

    // derivatives w.r.t u
    for (size_t i = 0; i < 2; ++i)
    {
        // Freeze u0 and add a perturbation on q0 
        const double eps = std::max(1.0,fabs(u0[i]))*1.0e-8;
        u0[i] += eps;

        auto dqdt_eps = car_sc(q0,u0,0.0);

        u0[i] -= 2*eps;

        auto dqdt_meps = car_sc(q0,u0,0.0);

        numerical_jacobian[i+13] = (dqdt_eps - dqdt_meps)*(0.5/eps);

        u0[i] += eps;
    }


    for (size_t i = 0; i < 15; ++i)
        for (size_t j = 0; j < 13; ++j)
        {
            EXPECT_NEAR(numerical_jacobian[i][j], solution.jac_dqdt[j][i], 2.0e-6*std::max(fabs(solution.jac_dqdt[j][i]),1.0)) << "with i = " << i << " and j = " << j ;
        }
}


TEST_F(Car_road_cartesian_test, autodifftest)
{
    std::vector<CppAD::AD<double>> x = {0.0, 0.0, 0.0};
    CppAD::Independent(x);

    std::vector<CppAD::AD<double>> y = {x[0]+2.0*x[1]+0.5*x[0]*x[0] + x[0]*x[2] ,x[1]+x[1]*x[1]}; 

    CppAD::ADFun<double> adfun;
    adfun.Dependent(x,y);


    std::vector<double> x0 = {0,0,0};

    auto y0 = adfun.Forward(0, x0);
    auto dydx0 = adfun.Jacobian(x0);
    auto d2ydx20 = adfun.Hessian(x0,0);
}
