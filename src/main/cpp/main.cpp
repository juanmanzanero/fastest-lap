#include "lion/propagators/rk4.h"
#include "src/core/chassis/axle_car_6dof.h"
#include "lion/math/optimise.h"
#include "src/core/chassis/chassis_car_6dof.h"
#include "src/core/tire/tire_pacejka.h"
#include "src/core/vehicles/road_curvilinear.h"
#include "src/core/vehicles/dynamic_model_car.h"
#include "src/core/vehicles/track_run.h"
#include "src/core/actuators/engine.h"
#include "src/core/vehicles/track_by_polynomial.h"
#include "lion/math/matrix_extensions.h"

#include "lion/thirdparty/include/coin-or/IpIpoptApplication.hpp"

//#include "lion/thirdparty/include/matplotlibcpp.h"
//namespace plt = matplotlibcpp;


// Tires
using Front_left_tire_type  = Tire_pacejka_std<scalar,0,0>;
using Front_right_tire_type = Tire_pacejka_std<scalar,Front_left_tire_type ::STATE_END, Front_left_tire_type ::CONTROL_END>;
using Rear_left_tire_type   = Tire_pacejka_std<scalar,Front_right_tire_type::STATE_END, Front_right_tire_type::CONTROL_END>;
using Rear_right_tire_type  = Tire_pacejka_std<scalar,Rear_left_tire_type  ::STATE_END, Rear_left_tire_type  ::CONTROL_END>;

// Axles
using Front_axle_type = Axle_car_6dof<scalar,Front_left_tire_type, Front_right_tire_type, STEERING_FREE_ROLL          , Rear_right_tire_type::STATE_END, Rear_right_tire_type::CONTROL_END>;
using Rear_axle_type  = Axle_car_6dof<scalar,Rear_left_tire_type , Rear_right_tire_type , POWERED_WITHOUT_DIFFERENTIAL, Front_axle_type     ::STATE_END, Front_axle_type     ::CONTROL_END>;

// Chassis
using Chassis_t = Chassis_car_6dof<scalar,Front_axle_type,Rear_axle_type,Rear_axle_type::STATE_END,Rear_axle_type::CONTROL_END>;

// Road
using Road_t           = Road_curvilinear<scalar,Track_by_polynomial,Chassis_t::STATE_END, Chassis_t::CONTROL_END>;

// Dynamic model
using Dynamic_model_t  = Dynamic_model_car<scalar,Chassis_t,Road_t,Road_t::STATE_END,Road_t::CONTROL_END>;

static_assert(Front_axle_type::Axle_type::ISTEERING==0);
static_assert(Rear_axle_type::Axle_type::ITORQUE==1);

static_assert(Rear_axle_type::IOMEGA_AXLE==0);
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


static_assert(Rear_axle_type::IIDOMEGA_AXLE==0);
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


  
int main()
{
    // Fitness function: simply f(x,y) = y
    class F
    {
     public:
        using argument_type = std::vector<double>;

        double operator()(const argument_type& x)
        {
            return x[1];
        }
    };
    
    // Equality constraint: y = sin(x)
    class C
    {
     public:
        using argument_type = std::vector<double>;

        std::array<double,1> operator()(const argument_type& x)
        {
            return { sin(x[0]) - x[1] };
        }
    };

    F f;
    C c;

    // Perform optimisation
    auto result = Optimise<F,C>::optimise(2,1,{0.0,0.0},f,c,{-2.0,-2.0},{2.0,2.00},{0.0},{0.0});

//    plt::plot({1,3,2,4});
//    plt::show();

}
