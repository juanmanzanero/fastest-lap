#ifndef ROAD_CARTESIAN_HPP
#define ROAD_CARTESIAN_HPP

template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
void Road_cartesian<Timeseries_t,STATE0,CONTROL0>::update(const Timeseries_t u, const Timeseries_t v, const Timeseries_t omega)
{
    // dxdt
    _dx = u*cos(base_type::_psi) - v*sin(base_type::_psi);

    // dydt
    _dy = u*sin(base_type::_psi) + v*cos(base_type::_psi);

    // dpsidt
    _dpsi = omega;
}

template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
template<size_t N>
void Road_cartesian<Timeseries_t,STATE0,CONTROL0>::get_state_and_state_derivative
    (std::array<Timeseries_t, N>& state, std::array<Timeseries_t,N>& dstate_dt) const
{
    // dxdt
    state    [input_state_names::X] = base_type::_x;
    dstate_dt[input_state_names::X] = _dx;

    // dydt
    state    [input_state_names::Y] = base_type::_y;
    dstate_dt[input_state_names::Y] = _dy;

    // dpsidt
    state    [input_state_names::PSI] = base_type::_psi;
    dstate_dt[input_state_names::PSI] = _dpsi;
}


template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_cartesian<Timeseries_t,STATE0,CONTROL0>::set_state_and_controls
    (const Timeseries_t t, const std::array<Timeseries_t,NSTATE>& input_states, const std::array<Timeseries_t,NCONTROL>& controls)
{
    // x
    base_type::_x = input_states[input_state_names::X];

    // y
    base_type::_y = input_states[input_state_names::Y];

    // psi
    base_type::_psi = input_states[input_state_names::PSI];
}

template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_cartesian<Timeseries_t, STATE0, CONTROL0>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& input_states_def, std::array<scalar, NSTATE>& input_states_lb,
        std::array<scalar, NSTATE>& input_states_ub, std::array<scalar, NCONTROL>& controls_def,
        std::array<scalar, NCONTROL>& controls_lb, std::array<scalar, NCONTROL>& controls_ub)
{
    // x
    input_states_def[input_state_names::X] = 0.0;
    input_states_lb[input_state_names::X]  = std::numeric_limits<scalar>::lowest();
    input_states_ub[input_state_names::X]  = std::numeric_limits<scalar>::max();

    // y
    input_states_def[input_state_names::Y] = 0.0;
    input_states_lb[input_state_names::Y]  = std::numeric_limits<scalar>::lowest();
    input_states_ub[input_state_names::Y]  = std::numeric_limits<scalar>::max();

    // psi
    input_states_def[input_state_names::PSI] = 0.0;
    input_states_lb[input_state_names::PSI]  = std::numeric_limits<scalar>::lowest();
    input_states_ub[input_state_names::PSI]  = std::numeric_limits<scalar>::max();
}




template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_cartesian<Timeseries_t,STATE0,CONTROL0>::set_state_and_control_names
    (std::string& key_name, std::array<std::string,NSTATE>& input_states, 
     std::array<std::string,NCONTROL>& controls) 
{
    key_name = "time";

    // x
    input_states[input_state_names::X] = "x";

    // y
    input_states[input_state_names::Y] = "y";

    // psi
    input_states[input_state_names::PSI] = "psi";
}

#endif
