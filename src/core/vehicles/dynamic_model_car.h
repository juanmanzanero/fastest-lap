#ifndef __DYNAMIC_MODEL_CAR_H__
#define __DYNAMIC_MODEL_CAR_H__

#include <map>
#include <vector>
#include "lion/foundation/types.h"
#include "src/core/chassis/chassis_car.h"
#include "src/core/chassis/axle_car.h"
#include "src/core/tire/tire_pacejka.h"

//!      The dynamic model of a Car
//!      --------------------------
//!
//!  This model takes a car chassis and a road, and provides an ODE functor to be used
//! by a propagator of the form dqdt = operator()(q,u,t)
//! @param Chassis_t: type of the chassis
//! @param RoadModel_t: type of the road model
//! @param _NSTATE: number of state variables
//! @param _NCONTROL: number of control variables
template<typename Timeseries_t, typename Chassis_t, typename RoadModel_t, size_t _NSTATE, size_t _NCONTROL>
class Dynamic_model_car
{
 public: 
    using Timeseries_type = Timeseries_t;
    //! The chassis type
    using Chassis_type = Chassis_t;
   
    //! The road model type
    using Road_type = RoadModel_t;

    //! The number of state variables
    constexpr static size_t NSTATE    = _NSTATE;

    //! The number of control variables
    constexpr static size_t NCONTROL  = _NCONTROL; 

    //! Default constructor
    Dynamic_model_car() = default;

    //! Constructor from parameter map and tire types
    //! @param[in] parameters: parameters map
    //! @param[in] front_left_tire_type: type of the front left tire 
    //! @param[in] front_right_tire_type: type of the front right tire 
    //! @param[in] rear_left_tire_type: type of the rear left tire 
    //! @param[in] rear_right_tire_type: type of the rear right tire 
    //! @param[in] road: the road
    Dynamic_model_car(Xml_document& database, const RoadModel_t& road = RoadModel_t()) 
        : _chassis(database), _road(road) {};

    //! The time derivative functor, dqdt = operator()(q,u,t)
    std::array<Timeseries_t,_NSTATE> operator()(const std::array<Timeseries_t,_NSTATE>& q, 
                                                const std::array<Timeseries_t,_NCONTROL>& u,
                                                scalar t);

    std::tuple<std::string,std::array<std::string,_NSTATE>,std::array<std::string,_NCONTROL>> get_state_and_control_names() const;

    //! Return the chassis
    constexpr const Chassis_t& get_chassis() const { return _chassis; }
    constexpr       Chassis_t& get_chassis()       { return _chassis; }

    //! Return the road
    constexpr const RoadModel_t& get_road() const { return _road; }
    constexpr       RoadModel_t& get_road()       { return _road; }


 private:
    Chassis_t _chassis;    //! The chassis
    RoadModel_t _road;     //! The road
};

#include "dynamic_model_car.hpp"

#endif
