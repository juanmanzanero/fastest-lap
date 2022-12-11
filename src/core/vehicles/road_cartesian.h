#ifndef ROAD_CARTESIAN_H
#define ROAD_CARTESIAN_H

#include "road.h"

template<typename Timeseries_t,size_t state_start, size_t control_start>
class Road_cartesian : public Road<Timeseries_t,state_start,control_start>
{
 public:
    using base_type = Road<Timeseries_t,state_start,control_start>;

    struct input_names : public base_type::input_names
    {
        enum { X = base_type::input_names::end, Y, PSI, end };
    };

    struct state_names : public base_type::state_names
    {
        enum { X = base_type::state_names::end, Y, PSI, end };
    };

    struct control_names : public base_type::control_names {};

    void update(const Timeseries_t u, const Timeseries_t v, const Timeseries_t omega);

    template<size_t number_of_states>
    void get_state_and_state_derivative(std::array<Timeseries_t, number_of_states>& state, 
                                        std::array<Timeseries_t,number_of_states>& dstate_dt
                                        ) const;

    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_controls(const Timeseries_t t, const std::array<Timeseries_t,number_of_inputs>& input, const std::array<Timeseries_t,number_of_controls>& controls);

    //! Set the state and controls upper, lower, and default values
    template<size_t number_of_inputs, size_t number_of_controls>
    static void set_state_and_control_upper_lower_and_default_values(std::array<scalar, number_of_inputs>& inputs_def,
        std::array<scalar, number_of_inputs>& inputs_lb,
        std::array<scalar, number_of_inputs>& inputs_ub,
        std::array<scalar, number_of_controls>& controls_def,
        std::array<scalar, number_of_controls>& controls_lb,
        std::array<scalar, number_of_controls>& controls_ub
    );

    template<size_t number_of_inputs, size_t number_of_controls>
    static void set_state_and_control_names(std::string& key_name,
        std::array<std::string, number_of_inputs>& input_names,
        std::array<std::string, number_of_controls>& control_names
    );
 
 private:
    Timeseries_t _dx;
    Timeseries_t _dy;
    Timeseries_t _dpsi;
};

#include "road_cartesian.hpp"

#endif
