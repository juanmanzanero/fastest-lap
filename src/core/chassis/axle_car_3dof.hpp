#ifndef AXLE_CAR_3DOF_HPP
#define AXLE_CAR_3DOF_HPP

#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
Axle_car_3dof<Timeseries_t, Tire_left_t, Tire_right_t, Axle_mode, state_start,control_start>::Axle_car_3dof(const std::string& name,
    const Tire_left_t& tire_l, const Tire_right_t& tire_r,
    const std::string& path)
: Axle<Timeseries_t, std::tuple<Tire_left_t, Tire_right_t>, state_start,control_start>(name, { tire_l, tire_r }),
  _track(0.0),
  _y_tire({ 0.0,0.0 }),
  _I(0.0),
  _differential_stiffness(0.0),
  _throttle_smooth_pos(0.0),
  _kappa_dimensionless_left(0.0),
  _kappa_dimensionless_right(0.0),
  _dangular_momentum_dt_left(0.0),
  _dangular_momentum_dt_right(0.0),
  _torque_left(0.0),
  _torque_right(0.0),
  _throttle(0.0),
  _boost(0.0),
  _brakes(),
  _engine(),
  _engine_boost(),
  _delta(0.0)
{
    base_type::_path = path;

    _y_tire = {-0.5*_track, 0.5*_track};

    // Construct the brakes
    _brakes = Brake<Timeseries_t>(path + "brakes/");

    // Construct the specific parameters of the axle
    if constexpr ( std::is_same<Axle_mode<0,0>, POWERED<0,0>>::value )
    {
        // Construct engine and brakes
        _engine = Engine<Timeseries_t>(path + "engine/", true);
        _engine_boost = Engine<Timeseries_t>(path + "boost/", true);
    }
    else if constexpr ( std::is_same<Axle_mode<0,0>, STEERING<0,0>>::value )
    {
        // Prepare the tires frames to have one rotation (the steering)
        if ( std::get<LEFT>(Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::_tires).get_frame().get_rotation_angles().size() != 0 )
            throw fastest_lap_exception("Left tire frame must have zero rotations");

        if ( std::get<RIGHT>(Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::_tires).get_frame().get_rotation_angles().size() != 0 )
            throw fastest_lap_exception("Right tire frame must have zero rotations");

        std::get<LEFT>(base_type::_tires).get_frame().add_rotation(0.0, 0.0, Z);
        std::get<RIGHT>(base_type::_tires).get_frame().add_rotation(0.0, 0.0, Z);
    }

    std::get<LEFT>(base_type::_tires).get_frame().set_origin(get_tire_position(LEFT), get_tire_velocity(LEFT), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
    std::get<RIGHT>(base_type::_tires).get_frame().set_origin(get_tire_position(RIGHT), get_tire_velocity(RIGHT), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
Axle_car_3dof<Timeseries_t, Tire_left_t, Tire_right_t, Axle_mode, state_start,control_start>::Axle_car_3dof(const std::string& name,
    const Tire_left_t& tire_l, const Tire_right_t& tire_r,
    Xml_document& database,
    const std::string& path)
: Axle<Timeseries_t, std::tuple<Tire_left_t, Tire_right_t>, state_start,control_start>(name, { tire_l, tire_r }),
  _track(0.0),
  _y_tire({ 0.0,0.0 }),
  _I(0.0),
  _differential_stiffness(0.0),
  _throttle_smooth_pos(0.0),
  _kappa_dimensionless_left(0.0),
  _kappa_dimensionless_right(0.0),
  _dangular_momentum_dt_left(0.0),
  _dangular_momentum_dt_right(0.0),
  _torque_left(0.0),
  _torque_right(0.0),
  _throttle(0.0),
  _boost(0.0),
  _brakes(),
  _engine(),
  _engine_boost(),
  _delta(0.0)
{
    base_type::_path = path;

    read_parameters(database, path, get_parameters(), __used_parameters);
    _y_tire = {-0.5*_track, 0.5*_track};

    // Construct the brakes
    _brakes = Brake<Timeseries_t>(database, path + "brakes/");

    // Construct the specific parameters of the axle
    if constexpr ( std::is_same<Axle_mode<0,0>, POWERED<0,0>>::value )
    {
        // Construct engine and brakes
        _engine = Engine<Timeseries_t>(database, path + "engine/", true);
        _engine_boost = Engine<Timeseries_t>(database, path + "boost/", true);
    }
    else if constexpr ( std::is_same<Axle_mode<0,0>, STEERING<0,0>>::value )
    {
        // Prepare the tires frames to have one rotation (the steering)
        if ( std::get<LEFT>(Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::_tires).get_frame().get_rotation_angles().size() != 0 )
            throw fastest_lap_exception("Left tire frame must have zero rotations");

        if ( std::get<RIGHT>(Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>::_tires).get_frame().get_rotation_angles().size() != 0 )
            throw fastest_lap_exception("Right tire frame must have zero rotations");

        std::get<LEFT>(base_type::_tires).get_frame().add_rotation(0.0, 0.0, Z);
        std::get<RIGHT>(base_type::_tires).get_frame().add_rotation(0.0, 0.0, Z);
    }

    std::get<LEFT>(base_type::_tires).get_frame().set_origin(get_tire_position(LEFT), get_tire_velocity(LEFT), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
    std::get<RIGHT>(base_type::_tires).get_frame().set_origin(get_tire_position(RIGHT), get_tire_velocity(RIGHT), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<typename T>
inline bool Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::set_parameter(const std::string& parameter, const T value)
{
    bool found = false;
    // Check if the parameter goes to this object
    if ( parameter.find(base_type::_path) == 0 )
    {
        // Find the parameter in the database
        found = ::set_parameter(get_parameters(), __used_parameters, parameter, base_type::_path, value); 

        // If found, update the tires frames in case the parameter modified was their position
        if ( found )
        {
            _y_tire = {-0.5*_track, 0.5*_track};
            std::get<LEFT>(base_type::_tires).get_frame().set_origin(get_tire_position(LEFT), get_tire_velocity(LEFT), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
            std::get<RIGHT>(base_type::_tires).get_frame().set_origin(get_tire_position(RIGHT), get_tire_velocity(RIGHT), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
        }

        // If not found, look for the brakes
        if ( !found )
            if ( parameter.find(base_type::_path + "brakes/") == 0 ) 
            {
                _brakes.set_parameter(parameter, value);
                found = true;
            }

        // If not found, look for the engine
        if constexpr ( std::is_same<Axle_mode<0,0>, POWERED<0,0>>::value )
        {
            if ( !found )
                if ( parameter.find(base_type::_path + "engine/") == 0 )
                {
                    _engine.set_parameter(parameter, value);
                    found = true;
                }

            if ( !found )
                if ( parameter.find(base_type::_path + "boost/") == 0 )
                {
                    _engine_boost.set_parameter(parameter, value);
                    found = true;
                }
        }

        // If not found, look for the parameter in the parent class
        if ( !found )
            found = base_type::set_parameter(parameter, value);
    }
    else
    {
        // Look for the parameter in the parent class
        found = base_type::set_parameter(parameter, value);
    }
           
    return found;
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
inline void Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::fill_xml(Xml_document& doc) const
{
    // Write the parameters of the base class
    base_type::fill_xml(doc);

    // Write the parameters of the brake
    _brakes.fill_xml(doc);

    // Write the parameters of the engine
    if constexpr ( std::is_same<Axle_mode<0,0>, POWERED<0,0>>::value )
    {
        _engine.fill_xml(doc);
        _engine_boost.fill_xml(doc);
    }

    // Write the parameters of this class
    ::write_parameters(doc, base_type::_path, get_parameters());
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::transform_states_to_inputs
    (const std::array<Timeseries_t,number_of_inputs>& states, const std::array<Timeseries_t,number_of_controls>& controls, std::array<Timeseries_t,number_of_inputs>& inputs)
{
    // Rotate the tires frame
    if constexpr (std::is_same<Axle_mode<0,0>,STEERING<0,0>>::value)
    {
        // steering angle
        const auto& delta = controls[control_names::STEERING];

        // rotate the tires' frames
        std::get<LEFT>(base_type::_tires).get_frame().set_rotation_angle(0,delta);
        std::get<RIGHT>(base_type::_tires).get_frame().set_rotation_angle(0,delta);
    }

    // Placeholder to compute kappa from omega when ready
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
void Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::update
    (Timeseries_t Fz_left, Timeseries_t Fz_right, Timeseries_t throttle, Timeseries_t brake_bias, const Frame<Timeseries_t>& road_frame)
{
    // Create aliases
    Tire_left_t& tire_l  = std::get<LEFT>(base_type::_tires);
    Tire_right_t& tire_r = std::get<RIGHT>(base_type::_tires);

    // Update the tires
    tire_l.update(-Fz_left, _kappa_dimensionless_left, road_frame);
    tire_r.update(-Fz_right, _kappa_dimensionless_right, road_frame);

    const Timeseries_t& omega_left = tire_l.get_omega();
    const Timeseries_t& omega_right = tire_r.get_omega();

    // Compute brake percentage
    const Timeseries_t brake_percentage     =  smooth_pos(-throttle,_throttle_smooth_pos);

    // Compute the braking torque
    _torque_left  = -smooth_sign(omega_left,1.0)*_brakes(brake_percentage)*brake_bias;
    _torque_right = -smooth_sign(omega_right,1.0)*_brakes(brake_percentage)*brake_bias;

    if constexpr (std::is_same<Axle_mode<0,0>, POWERED<0,0>>::value)
    {
        // Compute throttle percentage 
        const Timeseries_t throttle_percentage  =  smooth_pos( throttle,_throttle_smooth_pos);

        // Compute engine torque as engine_power/mean(omega_l,omega_r)
        // Computed this way, the power balance of the axle is:
        // T_left.omega_left + T_right.omega_right = net_power = engine_power - differential_dissipation <= engine_power
        // with differential_dissipation = differential_stiffness.(omega_left - omega_right)^2
        //
        // Ref: https://eprints.soton.ac.uk/417133/1/GP2manuscriptPURE_002_.pdf
        const Timeseries_t engine_torque = _engine(throttle_percentage, 0.5*(omega_left + omega_right));
        const Timeseries_t differential_torque = _differential_stiffness*(omega_left - omega_right);

        const Timeseries_t boost_torque  = _engine_boost(throttle_percentage*_boost, 0.5*(omega_left + omega_right));
        _torque_left  += 0.5*(engine_torque + boost_torque) - differential_torque;
        _torque_right += 0.5*(engine_torque + boost_torque) + differential_torque;
    }

    // Compute the time derivative of the two kappas
    _dangular_momentum_dt_left = (_torque_left  + tire_l.get_longitudinal_torque_at_wheel_center());
    _dangular_momentum_dt_right = (_torque_right + tire_r.get_longitudinal_torque_at_wheel_center());

    // Get the total force and torque by the tires
    const Vector3d<Timeseries_t> F_left = tire_l.get_force_in_parent(); 
    const Vector3d<Timeseries_t> F_right = tire_r.get_force_in_parent(); 

    const Vector3d<Timeseries_t> T_left = tire_l.get_torque_in_parent(); 
    const Vector3d<Timeseries_t> T_right = tire_r.get_torque_in_parent(); 

    base_type::_F = F_left + F_right;

    base_type::_T =  T_left  + cross(tire_l.get_frame().get_origin(), F_left) 
                   + T_right + cross(tire_r.get_frame().get_origin(), F_right);
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
scalar Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::get_parameter(const std::string& parameter_name) const
{
    if (parameter_name == "track") return _track; 

    throw fastest_lap_exception("Parameter " + parameter_name + " does not exist in Axle_car_3dof");
}


// ------- Handle state vector
template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<size_t number_of_states>
void Axle_car_3dof<Timeseries_t, Tire_left_t, Tire_right_t, Axle_mode, state_start, control_start>::get_state_and_state_derivative
(std::array<Timeseries_t, number_of_states>& state, std::array<Timeseries_t, number_of_states>& dstate_dt, const Timeseries_t& mass_kg) const
{
    base_type::get_state_and_state_derivative(state, dstate_dt);

    const auto scaling_factor = (_I < 1.0e-10 ? 1.0/mass_kg : 1.0);

    // Left tire's kappa
    state    [state_names::angular_momentum_left] = std::get<0>(base_type::_tires).get_omega() * _I * scaling_factor;
    dstate_dt[state_names::angular_momentum_left] = _dangular_momentum_dt_left * scaling_factor;

    // Right tire's omega
    state    [state_names::angular_momentum_right] = std::get<1>(base_type::_tires).get_omega() * _I * scaling_factor;
    dstate_dt[state_names::angular_momentum_right] = _dangular_momentum_dt_right * scaling_factor;
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::set_state_and_control_names
     (std::array<std::string,number_of_inputs>& inputs, std::array<std::string,number_of_controls>& controls) const
{
    base_type::set_state_and_control_names(inputs, controls);

    // kappa left
    inputs[input_names::KAPPA_LEFT] = base_type::_name + ".left-tire.kappa";

    // kappa right
    inputs[input_names::KAPPA_RIGHT] = base_type::_name + ".right-tire.kappa";

    if constexpr (std::is_same<Axle_mode<0,0>,STEERING<0,0>>::value)
    {
        // steering angle
        controls[control_names::STEERING] = base_type::_name + ".steering-angle";
    }

    if constexpr (std::is_same<Axle_mode<0,0>,POWERED<0,0>>::value) 
    {
        // boost
        controls[control_names::boost] = base_type::_name + ".boost";
    }
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::set_state_and_controls
    (const std::array<Timeseries_t,number_of_inputs>& inputs, const std::array<Timeseries_t,number_of_controls>& controls) 
{
    base_type::set_state_and_controls(inputs, controls);

    // kappa left
    _kappa_dimensionless_left  = inputs[input_names::KAPPA_LEFT];

    // kappa right
    _kappa_dimensionless_right  = inputs[input_names::KAPPA_RIGHT];

    if constexpr (std::is_same<Axle_mode<0,0>,STEERING<0,0>>::value)
    {
        // steering angle
        _delta = controls[control_names::STEERING];

        // rotate the tires' frames
        std::get<LEFT>(base_type::_tires).get_frame().set_rotation_angle(0,_delta);
        std::get<RIGHT>(base_type::_tires).get_frame().set_rotation_angle(0,_delta);
    }


    if constexpr (std::is_same<Axle_mode<0,0>,POWERED<0,0>>::value) 
    {
        _boost = controls[control_names::boost];
    }
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, number_of_inputs>& inputs_def     , std::array<scalar, number_of_inputs>& inputs_lb     , std::array<scalar, number_of_inputs>& inputs_ub     ,
    std::array<scalar , number_of_controls>& controls_def   , std::array<scalar, number_of_controls>& controls_lb   , std::array<scalar, number_of_controls>& controls_ub) const
{
    base_type::set_state_and_control_upper_lower_and_default_values(inputs_def,inputs_lb,inputs_ub,controls_def,controls_lb,controls_ub);

    // State ------------
    
    // Kappa left
    inputs_def[input_names::KAPPA_LEFT] = 0.0;
    inputs_lb[input_names::KAPPA_LEFT]  = -1.0;
    inputs_ub[input_names::KAPPA_LEFT]  =  1.0;

    // Kappa right
    inputs_def[input_names::KAPPA_RIGHT] = 0.0;
    inputs_lb[input_names::KAPPA_RIGHT]  = -1.0;
    inputs_ub[input_names::KAPPA_RIGHT]  =  1.0;

    if constexpr (std::is_same_v<Axle_mode<0,0>,STEERING<0,0>>)
    {
        controls_def[control_names::STEERING] = 0.0;
        controls_lb[control_names::STEERING] = -20.0*DEG;
        controls_ub[control_names::STEERING] =  20.0*DEG;
    }

    if constexpr (std::is_same<Axle_mode<0,0>,POWERED<0,0>>::value) 
    {
        controls_def[control_names::boost] = 0.0;
        controls_lb[control_names::boost] = 0.0;
        controls_ub[control_names::boost] = 1.0;
    }
}

#endif
