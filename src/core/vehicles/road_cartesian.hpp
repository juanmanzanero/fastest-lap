#ifndef ROAD_CARTESIAN_HPP
#define ROAD_CARTESIAN_HPP

template<typename Timeseries_t,size_t state_start, size_t control_start>
void Road_cartesian<Timeseries_t,state_start,control_start>::update(const Timeseries_t u, const Timeseries_t v, const Timeseries_t omega)
{
    // dxdt
    _dx = u*cos(base_type::get_psi()) - v*sin(base_type::get_psi());

    // dydt
    _dy = u*sin(base_type::get_psi()) + v*cos(base_type::get_psi());

    // dpsidt
    _dpsi = omega;
}


template<typename Timeseries_t,size_t state_start, size_t control_start>
template<size_t number_of_states>
void Road_cartesian<Timeseries_t,state_start,control_start>::get_state_and_state_derivative
    (std::array<Timeseries_t, number_of_states>& state, std::array<Timeseries_t,number_of_states>& dstate_dt) const
{
    // dxdt
    state    [state_names::X] = base_type::get_position().x();
    dstate_dt[state_names::X] = _dx;

    // dydt
    state    [state_names::Y] = base_type::get_position().y();
    dstate_dt[state_names::Y] = _dy;

    // dpsidt
    state    [state_names::PSI] = base_type::get_psi();
    dstate_dt[state_names::PSI] = _dpsi;
}


template<typename Timeseries_t,size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Road_cartesian<Timeseries_t,state_start,control_start>::set_state_and_controls
    (const Timeseries_t t, const std::array<Timeseries_t,number_of_inputs>& inputs, const std::array<Timeseries_t,number_of_controls>& controls)
{
    // x
    base_type::get_position().x() = inputs[input_names::X];

    // y
    base_type::get_position().y() = inputs[input_names::Y];

    base_type::get_position().z() = 0.0;

    // psi
    base_type::get_psi() = inputs[input_names::PSI];
}

template<typename Timeseries_t,size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Road_cartesian<Timeseries_t, state_start,control_start>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, number_of_inputs>& inputs_def, std::array<scalar, number_of_inputs>& inputs_lb,
        std::array<scalar, number_of_inputs>& inputs_ub, std::array<scalar, number_of_controls>& controls_def,
        std::array<scalar, number_of_controls>& controls_lb, std::array<scalar, number_of_controls>& controls_ub)
{
    // x
    inputs_def[input_names::X] = 0.0;
    inputs_lb[input_names::X]  = std::numeric_limits<scalar>::lowest();
    inputs_ub[input_names::X]  = std::numeric_limits<scalar>::max();

    // y
    inputs_def[input_names::Y] = 0.0;
    inputs_lb[input_names::Y]  = std::numeric_limits<scalar>::lowest();
    inputs_ub[input_names::Y]  = std::numeric_limits<scalar>::max();

    // psi
    inputs_def[input_names::PSI] = 0.0;
    inputs_lb[input_names::PSI]  = std::numeric_limits<scalar>::lowest();
    inputs_ub[input_names::PSI]  = std::numeric_limits<scalar>::max();
}




template<typename Timeseries_t,size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Road_cartesian<Timeseries_t,state_start,control_start>::set_state_and_control_names
    (std::string& key_name, std::array<std::string,number_of_inputs>& inputs, 
     std::array<std::string,number_of_controls>& controls) 
{
    key_name = "time";

    // x
    inputs[input_names::X] = "x";

    // y
    inputs[input_names::Y] = "y";

    // psi
    inputs[input_names::PSI] = "psi";
}

#endif
