#ifndef __DYNAMIC_MODEL_POWERED_AXLE_H__
#define __DYNAMIC_MODEL_POWERED_AXLE_H__

#include "road_cartesian.h"

//!      A dynamic model of a powered car axle
//!      -------------------------------------
template<typename Timeseries_t,typename Axle_t, size_t STATE0, size_t CONTROL0>
class Dynamic_model_powered_axle
{
 public:
    using Timeseries_type = Timeseries_t;
    using Axle_type = Axle_t;
    using Road_type = Road_cartesian<Timeseries_t,STATE0,CONTROL0>;

    struct input_state_names
    {
        enum { U = Road_type::input_state_names::end, end };
    };

    struct state_names : public input_state_names {};

    struct control_names
    {
        enum { end = Road_type::control_names::end };
    };

    constexpr static size_t NSTATE    = input_state_names::end;
    constexpr static size_t NCONTROL  = control_names::end;

    Dynamic_model_powered_axle(const scalar Fz, Xml_document& database);

    std::array<Timeseries_t,NSTATE> operator()(const std::array<Timeseries_t,NSTATE>& input_states, 
                                  const std::array<Timeseries_t,NCONTROL>& controls,
                                  Timeseries_t t);

    const Axle_t& get_axle() const { return _axle; }

    std::array<Timeseries_t, NSTATE> transform_states_to_input_states(const std::array<Timeseries_t, NSTATE>& states, 
                                                                       const std::array<Timeseries_t, NCONTROL>& controls 
                                                                      ) const { return states; }


 private:
    Axle_type _axle;
    Road_type _road;

    Frame<Timeseries_t> _inertial_frame;


    //! The mass [kg]
    scalar _m;

    //! The expected normal force
    scalar _Fz;

    //! The vertical coordinate [m]
    scalar _z;

    //! The forward velocity [m/s]
    Timeseries_t _u;
};

#include "dynamic_model_powered_axle.hpp"

#endif
