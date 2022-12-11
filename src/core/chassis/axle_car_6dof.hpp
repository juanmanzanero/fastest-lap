#ifndef AXLE_CAR_6DOF_HPP
#define AXLE_CAR_6DOF_HPP

#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::Axle_car_6dof(const std::string& name,
                           const Tire_left_t& tire_l, const Tire_right_t& tire_r,
                           const std::string& path)
: Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>(name, {tire_l, tire_r}),
  _track(0.0),
  _y_tire({0.0,0.0}),
  _k_chassis(0.0),
  _k_antiroll(0.0),
  _throttle_smooth_pos(0.0),
  _phi(0.0),
  _dphi(0.0),
  _s({0.0,0.0}),
  _ds({0.0,0.0}),
  _omega(0.0),
  _T_ax(0.0),
  _domega(0.0),
  _engine(),
  _brakes(),
  _delta(0.0),
  _beta({0.0,0.0})
{
    base_type::_path = path;

    _y_tire = {-0.5*_track, 0.5*_track};
    // Construct the specific parameters of the axle
    if constexpr ( std::is_same<Axle_mode<0,0>, STEERING_FREE_ROLL<0,0>>::value )
    {
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
Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::Axle_car_6dof(const std::string& name,
                           const Tire_left_t& tire_l, const Tire_right_t& tire_r,
                           Xml_document& database,
                           const std::string& path)
: Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,state_start,control_start>(name, {tire_l, tire_r}),
  _phi(0.0),
  _dphi(0.0),
  _s({0.0,0.0}),
  _ds({0.0,0.0}),
  _omega(0.0),
  _T_ax(0.0),
  _domega(0.0),
  _engine(),
  _brakes(),
  _delta(0.0),
  _beta({0.0,0.0})
{
    base_type::_path = path;

    read_parameters(database, path, get_parameters(), __used_parameters);
    _y_tire = {-0.5*_track, 0.5*_track};
    // Construct the specific parameters of the axle
    if constexpr ( std::is_same<Axle_mode<0,0>, POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value )
    {
        // Construct engine and brakes
        if ( database.has_element(path + "engine/") )
            _engine = Engine<Timeseries_t>(database, path + "engine/", true);

        if ( database.has_element(path + "brakes/" ) )
            _brakes = Brake<Timeseries_t>(database, path + "brakes/");
    }

    if constexpr ( std::is_same<Axle_mode<0,0>, STEERING_FREE_ROLL<0,0>>::value )
    {
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
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::update(const Vector3d<Timeseries_t>& x0, const Vector3d<Timeseries_t>& v0, Timeseries_t phi, Timeseries_t dphi)
{
    base_type::get_frame().set_origin(x0, v0, Frame<Timeseries_t>::Frame_velocity_types::parent_frame);

    update(phi,dphi);
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::update(Timeseries_t phi, Timeseries_t dphi)
{
    // Create aliases
    Tire_left_t& tire_l  = std::get<LEFT>(base_type::_tires);
    Tire_right_t& tire_r = std::get<RIGHT>(base_type::_tires);

    // Set input variables
    _phi = phi;
    _dphi = dphi;

    const scalar& k_tire = tire_l.get_radial_stiffness();
    const scalar& R0     = tire_l.get_radius();

    // The displacement has a symmetric (whole axle) and a assymmetric part (z_assymmetric)

    // Difference between the wheel hub position and the nominal position
    Timeseries_t displ_symmetric  = base_type::get_frame().get_absolute_position().at(Z)+R0;
    Timeseries_t ddispl_symmetric = base_type::get_frame().get_absolute_velocity_in_inertial().at(Z);

    // Wheel hub displacement due to roll
    Timeseries_t displ_assymmetric  = 0.5*_track*phi;
    Timeseries_t ddispl_assymmetric = 0.5*_track*dphi;

    // Wheel hub displacemenet due to steering
    if constexpr (std::is_same<Axle_mode<0,0>, STEERING_FREE_ROLL<0,0>>::value)
        displ_assymmetric += _beta[RIGHT]*_delta;

    // Compute combined stiffness to save operations
    const scalar sym_stiffness = 1.0/(_k_chassis + k_tire);
    const scalar assym_stiffness = 2.0*_k_antiroll*k_tire*sym_stiffness/(2.0*_k_antiroll + _k_chassis + k_tire);

    const Timeseries_t wl = _k_chassis*sym_stiffness*(displ_symmetric - displ_assymmetric) 
                           -displ_assymmetric*assym_stiffness;

    const Timeseries_t wr = _k_chassis*sym_stiffness*(displ_symmetric + displ_assymmetric)  
                          +displ_assymmetric*assym_stiffness;


    const Timeseries_t dwl = _k_chassis*sym_stiffness*(ddispl_symmetric - ddispl_assymmetric)
                           -assym_stiffness*ddispl_assymmetric;

    const Timeseries_t dwr = _k_chassis*sym_stiffness*(ddispl_symmetric + ddispl_assymmetric) 
                          +assym_stiffness*ddispl_assymmetric;

    _s[LEFT]  = wl - displ_symmetric + displ_assymmetric;
    _s[RIGHT] = wr - displ_symmetric - displ_assymmetric;

    _ds[LEFT]  = dwl - ddispl_symmetric + ddispl_assymmetric;
    _ds[RIGHT] = dwr - ddispl_symmetric - ddispl_assymmetric;

    // Update the tires
    tire_l.update(get_tire_position(LEFT), get_tire_velocity(LEFT), _omega);
    tire_r.update(get_tire_position(RIGHT), get_tire_velocity(RIGHT), _omega);

    // Compute the axle acceleration if powered
    if constexpr (std::is_same<Axle_mode<0,0>, POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value)
    {
        _domega = (_T_ax + tire_l.get_longitudinal_torque_at_wheel_center() 
                         + tire_r.get_longitudinal_torque_at_wheel_center()) / _I;
    }

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
template<typename T>
std::enable_if_t<std::is_same<T,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value,void> Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::set_throttle_and_omega(Timeseries_t throttle, Timeseries_t omega)
{
    _omega = omega;
    _throttle = throttle;
    // Compute the torque from the engine curve
    if ( !_engine.direct_torque() )
    {
        // Compute throttle and brake percentage
        const Timeseries_t throttle_percentage  =  smooth_pos( throttle,_throttle_smooth_pos);
        const Timeseries_t brake_percentage     =  smooth_pos(-throttle,_throttle_smooth_pos);

        // Compute engine torque
        _T_ax  =  _engine(throttle_percentage, omega);

        // Compute brake torque
        _T_ax -= smooth_sign(omega,1.0)*_brakes(brake_percentage);
    }
    else
        _T_ax = throttle;

}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<typename T>
std::enable_if_t<std::is_same<T,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value,void> Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::set_torque_and_omega(Timeseries_t T_ax, Timeseries_t omega)
{
    _omega = omega;
    _T_ax  = T_ax;
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<typename T>
std::enable_if_t<std::is_same<T,STEERING_FREE_ROLL<0,0>>::value,void> Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::set_steering_angle(Timeseries_t delta)
{
    _delta = delta;
    std::get<LEFT>(base_type::_tires).get_frame().set_rotation_angle(0,delta);
    std::get<RIGHT>(base_type::_tires).get_frame().set_rotation_angle(0,delta);
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
scalar Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::get_parameter(const std::string& parameter_name) const
{
    if (parameter_name == "track") return _track; 

    if (parameter_name == "beta_steering") return _beta[RIGHT];

    if (parameter_name == "chassis_stiffness") return _k_chassis;

    if (parameter_name == "antiroll_stiffness") return _k_antiroll;
     
    throw fastest_lap_exception("Parameter " + parameter_name + " does not exist in Axle_car_6dof");
}


// ------- Handle state vector
template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<size_t number_of_states>
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::get_state_and_state_derivative
    (std::array<Timeseries_t,number_of_states>& state, std::array<Timeseries_t, number_of_states>& dstate_dt, const Timeseries_t& mass_kg) const
{
    (void)mass_kg;
    base_type::get_state_and_state_derivative(state, dstate_dt);

    if constexpr (std::is_same<Axle_mode<0,0>,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value)
    {
        // domega
        state[state_names::OMEGA_AXLE] = _omega;
        dstate_dt[state_names::OMEGA_AXLE] = _domega;
    }
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::set_state_and_control_names
    (std::array<std::string,number_of_inputs>& inputs, std::array<std::string,number_of_controls>& controls) const
{
    base_type::set_state_and_control_names(inputs,controls);

    if constexpr (std::is_same<Axle_mode<0,0>,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value)
    {
        // omega
        inputs[input_names::OMEGA_AXLE] = base_type::_name + ".omega";

        // axle torque
        controls[control_names::TORQUE] = base_type::_name + ".throttle";
    }

    if constexpr (std::is_same<Axle_mode<0,0>,STEERING_FREE_ROLL<0,0>>::value)
    {
        // steering angle
        controls[control_names::STEERING] = base_type::_name + ".steering-angle";
    }
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::set_state_and_controls
    (const std::array<Timeseries_t,number_of_inputs>& inputs, const std::array<Timeseries_t,number_of_controls>& controls)
{
    base_type::set_state_and_controls(inputs,controls);

    if constexpr (std::is_same<Axle_mode<0,0>,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value)
    {
        // omega
        const Timeseries_t& torque = controls[control_names::TORQUE];
        const Timeseries_t& omega  = inputs[input_names::OMEGA_AXLE];
        set_throttle_and_omega(torque, omega);
    }

    if constexpr (std::is_same<Axle_mode<0,0>,STEERING_FREE_ROLL<0,0>>::value)
    {
        // steering angle
        set_steering_angle(controls[control_names::STEERING]);
    }
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,state_start,control_start>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, number_of_inputs>& inputs_def     , std::array<scalar, number_of_inputs>& inputs_lb     , std::array<scalar, number_of_inputs>& inputs_ub     ,
    std::array<scalar , number_of_controls>& controls_def   , std::array<scalar, number_of_controls>& controls_lb   , std::array<scalar, number_of_controls>& controls_ub) const
{
    base_type::set_state_and_control_upper_lower_and_default_values(inputs_def, inputs_lb, inputs_ub, controls_def, controls_lb, controls_ub); 

    if constexpr (std::is_same_v<Axle_mode<0,0>,POWERED_WITHOUT_DIFFERENTIAL<0,0>>)
    {
        // Set the omega of the axle
        inputs_def[input_names::OMEGA_AXLE] = 10.0*KMH/0.139; 
        inputs_lb[input_names::OMEGA_AXLE]  = 10.0*KMH/0.139;
        inputs_ub[input_names::OMEGA_AXLE]  = 200.0*KMH/0.139;

        controls_def[control_names::TORQUE] = 0.0;
        controls_lb[control_names::TORQUE]  = -1.0;
        controls_ub[control_names::TORQUE]  = 1.0;    
    }
    
    if constexpr (std::is_same_v<Axle_mode<0,0>,STEERING_FREE_ROLL<0,0>>)
    {
        controls_def[control_names::STEERING] = 0.0;
        controls_lb[control_names::STEERING]  = -20.0*DEG;
        controls_ub[control_names::STEERING]  =  20.0*DEG;
    }
}

#endif
