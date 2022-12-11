#ifndef AXLE_HPP
#define AXLE_HPP

#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t, typename Tires_tuple, size_t state_start, size_t control_start>
inline Axle<Timeseries_t,Tires_tuple,state_start,control_start>::Axle(const std::string& name, const Tires_tuple& tires)
: _frame(),
  _name(name),
  _path(),
  _tires(tires),
  _F(Vector3d<Timeseries_t>(0.0)),
  _T(Vector3d<Timeseries_t>(0.0))
{
    static_assert(NTIRES > 0);
    static_assert(NTIRES <= 2);

    // Update the parent frames of the tires to this one
    std::get<0>(_tires).get_frame().set_parent(_frame);

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).get_frame().set_parent(_frame);
}


template<typename Timeseries_t, typename Tires_tuple, size_t state_start, size_t control_start>
inline Axle<Timeseries_t,Tires_tuple,state_start,control_start>::Axle(const Axle& other)
: _frame(other._frame),
  __used_parameters(other.__used_parameters),
  _name(other._name),
  _path(other._path),
  _tires(other._tires),
  _F(other._F),
  _T(other._T) 
{
    static_assert(NTIRES <= 2);

    // Update the parent frames of the tires to this one
    std::get<0>(_tires).get_frame().set_parent(_frame);

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).get_frame().set_parent(_frame);
}


template<typename Timeseries_t, typename Tires_tuple, size_t state_start, size_t control_start>
inline Axle<Timeseries_t,Tires_tuple,state_start,control_start>& Axle<Timeseries_t,Tires_tuple,state_start,control_start>::operator=(const Axle& other)
{
    _name = other._name;
    _path = other._path;
    _frame = other._frame;
    _tires = other._tires;

    static_assert(NTIRES <= 2);

    // Update the parent frames of the tires to this one
    std::get<0>(_tires).get_frame().set_parent(_frame);

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).get_frame().set_parent(_frame);

    __used_parameters = other.__used_parameters;

    return *this;
}

template<typename Timeseries_t, typename Tires_tuple, size_t state_start, size_t control_start>
template<size_t number_of_states>
void Axle<Timeseries_t,Tires_tuple,state_start,control_start>::get_state_and_state_derivative
    (std::array<Timeseries_t,number_of_states>& state, std::array<Timeseries_t,number_of_states>& dstate_dt) const
{
    std::get<0>(_tires).get_state_and_state_derivative(state, dstate_dt);

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).get_state_and_state_derivative(state, dstate_dt);
}


template<typename Timeseries_t, typename Tires_tuple, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Axle<Timeseries_t,Tires_tuple,state_start,control_start>::set_state_and_control_names(std::array<std::string,number_of_inputs>& inputs, std::array<std::string,number_of_controls>& controls) const
{
    std::get<0>(_tires).set_state_and_control_names(inputs, controls);
    
    if constexpr (NTIRES == 2)
        std::get<1>(_tires).set_state_and_control_names(inputs, controls);
}


template<typename Timeseries_t, typename Tires_tuple, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Axle<Timeseries_t,Tires_tuple,state_start,control_start>::set_state_and_controls(const std::array<Timeseries_t,number_of_inputs>& inputs, const std::array<Timeseries_t,number_of_controls>& controls)
{
    std::get<0>(_tires).set_state_and_controls(inputs,controls);

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).set_state_and_controls(inputs,controls);
}


template<typename Timeseries_t, typename Tires_tuple, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Axle<Timeseries_t,Tires_tuple,state_start,control_start>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, number_of_inputs>& inputs_def     , std::array<scalar, number_of_inputs>& inputs_lb     , std::array<scalar, number_of_inputs>& inputs_ub     ,
    std::array<scalar , number_of_controls>& controls_def   , std::array<scalar, number_of_controls>& controls_lb   , std::array<scalar, number_of_controls>& controls_ub) const
{
    std::get<0>(_tires).set_state_and_control_upper_lower_and_default_values(inputs_def, inputs_lb, inputs_ub, controls_def, controls_lb, controls_ub); 

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).set_state_and_control_upper_lower_and_default_values(inputs_def, inputs_lb, inputs_ub, controls_def, controls_lb, controls_ub);
}

template<typename Timeseries_t, typename Tires_tuple, size_t state_start, size_t control_start>
template<typename T>
bool Axle<Timeseries_t,Tires_tuple,state_start,control_start>::set_parameter(const std::string& parameter, const T value)
{
    // This class does not have any parameter, so we just search it in the tires

    bool found = false;

    // Check that the tires have the same path
    if constexpr (NTIRES == 2)
    {
        if ( std::get<0>(_tires).get_path() != std::get<1>(_tires).get_path() )
            throw fastest_lap_exception("Tires shall have the same path in the database");
    }
    
    // Check if the parameter belongs to the tires
    if ( parameter.find(std::get<0>(_tires).get_path()) == 0 )
    {
        std::get<0>(_tires).set_parameter(parameter, value);

        if constexpr (NTIRES == 2)
            std::get<1>(_tires).set_parameter(parameter, value);

        found = true;
    }

    return found;
}

template<typename Timeseries_t, typename Tires_tuple, size_t state_start, size_t control_start>
inline void Axle<Timeseries_t,Tires_tuple,state_start,control_start>::fill_xml(Xml_document& doc) const
{
    // Call the method for the first tire
    std::get<0>(_tires).fill_xml(doc);
}


template<typename Timeseries_t, typename Tires_tuple, size_t state_start, size_t control_start>
inline bool Axle<Timeseries_t,Tires_tuple,state_start,control_start>::is_ready() const
{
    if constexpr (NTIRES == 1)
        return std::get<0>(_tires).is_ready() &&
            std::all_of(__used_parameters.begin(), __used_parameters.end(), [](const auto& v) -> auto { return v; });

    else if constexpr (NTIRES == 2)
    {
        return std::get<0>(_tires).is_ready() && std::get<1>(_tires).is_ready() &&
            std::all_of(__used_parameters.begin(), __used_parameters.end(), [](const auto& v) -> auto { return v; });
    }
}

#endif
