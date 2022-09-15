#ifndef ROAD_CARTESIAN_H
#define ROAD_CARTESIAN_H

#include "road.h"

template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
class Road_cartesian : public Road<Timeseries_t,STATE0,CONTROL0>
{
 public:
    using base_type = Road<Timeseries_t,STATE0,CONTROL0>;

    struct input_state_names
    {
        enum { X = base_type::input_state_names::end, Y, PSI, end };
    };
    
    struct control_names
    {
        enum { end = base_type::control_names::end };
    };

    struct state_names : public input_state_names {};

    void update(const Timeseries_t u, const Timeseries_t v, const Timeseries_t omega);

    template<size_t N>
    void get_state_and_state_derivative(std::array<Timeseries_t, N>& state, std::array<Timeseries_t,N>& dstate_dt) const;

    template<size_t NSTATE, size_t NCONTROL>
    void set_state_and_controls(const Timeseries_t t, const std::array<Timeseries_t,NSTATE>& input_state, const std::array<Timeseries_t,NCONTROL>& controls);

    //! Set the state and controls upper, lower, and default values
    template<size_t NSTATE, size_t NCONTROL>
    static void set_state_and_control_upper_lower_and_default_values(std::array<scalar, NSTATE>& input_states_def,
        std::array<scalar, NSTATE>& input_states_lb,
        std::array<scalar, NSTATE>& input_states_ub,
        std::array<scalar, NCONTROL>& controls_def,
        std::array<scalar, NCONTROL>& controls_lb,
        std::array<scalar, NCONTROL>& controls_ub
    );

    template<size_t NSTATE, size_t NCONTROL>
    static void set_state_and_control_names(std::string& key_name,
        std::array<std::string, NSTATE>& input_state_names,
        std::array<std::string, NCONTROL>& control_names
    );
 
 private:
    Timeseries_t _dx;
    Timeseries_t _dy;
    Timeseries_t _dpsi;
};

#include "road_cartesian.hpp"

#endif
