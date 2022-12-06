#ifndef ROAD_H
#define ROAD_H

#include "lion/foundation/types.h"

//!
//! A basic road class with the minimum functionality.
//!
template<typename Timeseries_t, size_t state_start, size_t algebraic_state_start, size_t control_start>
class Road
{
 public:
    using Timeseries_type = Timeseries_t; 

    struct input_names
    {
        enum { end = state_start + algebraic_state_start };
    };

    struct state_names
    {
        enum { end = state_start };
    };

    struct algebraic_state_names
    {
        enum { end = algebraic_state_start };
    };

    struct control_names
    {
        enum { end = state_start };
    };

    //! Return the dtime/ds (delta-time / delta-arclength) derivative.
    constexpr const auto& get_dtimeds() const { return _dtimeds; } 

    constexpr const auto& get_position() const { return _position; }

    constexpr const auto& get_psi() const { return _psi; }

    constexpr const auto& get_ground_vertical_velocity() const { return _ground_vertical_velocity; }

 protected:
    //! Non-const version of get_dtimeds for the children to access
    auto& get_dtimeds() { return _dtimeds; }

    auto& get_position() { return _position; }

    auto& get_psi() { return _psi; }

    auto& get_ground_vertical_velocity() { return _ground_vertical_velocity; }

 private:
    Timeseries_t           _dtimeds = 1.0;
    Vector3d<Timeseries_t> _position;
    Timeseries_t           _psi;
    Timeseries_t           _ground_vertical_velocity = 0.0;
};

template<typename T>
struct road_is_curvilinear
{
    const static inline bool value = false;
};

#endif
