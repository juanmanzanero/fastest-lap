#ifndef __LOT2016KART_H__
#define __LOT2016KART_H__

#include "src/core/tire/tire_pacejka.h"
#include "src/core/chassis/axle_car_6dof.h"
#include "src/core/chassis/chassis_car_6dof.h"
#include "src/core/vehicles/track_by_polynomial.h"
#include "src/core/vehicles/track_by_arcs.h"
#include "src/core/vehicles/road_cartesian.h"
#include "src/core/vehicles/road_curvilinear.h"
#include "src/core/vehicles/dynamic_model_car.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"

template<typename Timeseries_t>
class lot2016kart
{
 public:
    lot2016kart() = delete;

    using Front_left_tire_type  = Tire_pacejka<Timeseries_t,0,0>;
    using Front_right_tire_type = Tire_pacejka<Timeseries_t,Front_left_tire_type::STATE_END,Front_left_tire_type::CONTROL_END>;
    using Rear_left_tire_type   = Tire_pacejka<Timeseries_t,Front_right_tire_type::STATE_END,Front_right_tire_type::CONTROL_END>;
    using Rear_right_tire_type  = Tire_pacejka<Timeseries_t,Rear_left_tire_type::STATE_END,Rear_left_tire_type::CONTROL_END>;

    using Front_axle_t          = Axle_car_6dof<Timeseries_t,Front_left_tire_type,Front_right_tire_type,STEERING_FREE_ROLL,Rear_right_tire_type::STATE_END,Rear_right_tire_type::CONTROL_END>;
    using Rear_axle_t           = Axle_car_6dof<Timeseries_t,Rear_left_tire_type,Rear_right_tire_type,POWERED_WITHOUT_DIFFERENTIAL,Front_axle_t::STATE_END,Front_axle_t::CONTROL_END>;
    using Chassis_t             = Chassis_car_6dof<Timeseries_t,Front_axle_t,Rear_axle_t,Rear_axle_t::STATE_END,Rear_axle_t::CONTROL_END>;

    using Road_cartesian_t   = Road_cartesian<Timeseries_t,Chassis_t::STATE_END,Chassis_t::CONTROL_END>;

    template<typename Track_t>
    using Road_curvilinear_t = Road_curvilinear<Timeseries_t,Track_t,Chassis_t::STATE_END,Chassis_t::CONTROL_END>;
 
 private:
    
    template<typename Road_type>
    class Dynamic_model : public Dynamic_model_car<Timeseries_t,Chassis_t,Road_type,Road_type::STATE_END,Road_type::CONTROL_END>
    {
     public:
        using Road_t          = Road_type;
        using Dynamic_model_t = Dynamic_model_car<Timeseries_t,Chassis_t, Road_type, Road_type::STATE_END, Road_type::CONTROL_END>;

        Dynamic_model() {}
        Dynamic_model(Xml_document& database, const Road_t& road = Road_t()) : Dynamic_model_t(database,road) {}

        static std::pair<std::vector<scalar>,std::vector<scalar>> steady_state_variable_bounds() 
        {
            return { { -0.5, 1.0e-4, -10.0*DEG, -10.0*DEG, -10.0*DEG, -10.0*DEG },
                     {  0.5, 0.04  ,  10.0*DEG,  10.0*DEG,  10.0*DEG,  10.0*DEG } };
        }


        static std::pair<std::vector<scalar>,std::vector<scalar>> steady_state_constraint_bounds() 
        {
            return { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.11, -0.11, -0.09, -0.09, -0.09, -0.09},
                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.11,  0.11,  0.09,  0.09,  0.09,  0.09} };
        }
    };

 public:
    
    using cartesian = Dynamic_model<Road_cartesian_t>;

    template<typename Track_t>
    using curvilinear = Dynamic_model<Road_curvilinear_t<Track_t>>;

    using curvilinear_p = curvilinear<Track_by_polynomial>;
    using curvilinear_a = curvilinear<Track_by_arcs>;
};


struct lot2016kart_all
{
    lot2016kart_all(Xml_document& database_xml)
    : cartesian_scalar(database_xml), 
      curvilinear_scalar(database_xml),
      cartesian_ad(database_xml),
      curvilinear_ad(database_xml) 
    {}

    lot2016kart<scalar>::cartesian                 cartesian_scalar;
    lot2016kart<scalar>::curvilinear_a             curvilinear_scalar;
    lot2016kart<CppAD::AD<scalar>>::cartesian      cartesian_ad;
    lot2016kart<CppAD::AD<scalar>>::curvilinear_a  curvilinear_ad;
};


#endif
