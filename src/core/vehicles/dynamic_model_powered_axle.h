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

    enum State    { IU          = Road_type::STATE_END   , STATE_END    } ;
    enum Control  { CONTROL_END = Road_type::CONTROL_END                } ;

    constexpr static size_t IIDU = IU;

    constexpr static size_t NSTATE    = STATE_END;
    constexpr static size_t NCONTROL  = CONTROL_END;

    Dynamic_model_powered_axle(const scalar Fz, Xml_document& database);

    std::array<Timeseries_t,NSTATE> operator()(const std::array<Timeseries_t,NSTATE>& q, 
                                             const std::array<Timeseries_t,NCONTROL>& u,
                                             Timeseries_t t);

    const Axle_t& get_axle() const { return _axle; }


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
