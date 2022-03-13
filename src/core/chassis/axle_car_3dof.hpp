#ifndef __AXLE_CAR_3DOF_HPP__
#define __AXLE_CAR_3DOF_HPP__

template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::Axle_car_3dof(const std::string& name,
                           const Tire_left_t& tire_l, const Tire_right_t& tire_r,
                           const std::string& path)
: Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,STATE0,CONTROL0>(name, {tire_l, tire_r}),
  _track(0.0),
  _y_tire({0.0,0.0}),
  _I(0.0),
  _differential_stiffness(0.0),
  _throttle_smooth_pos(0.0),
  _kappa_left(0.0),
  _kappa_right(0.0),
  _dkappa_left(0.0),
  _dkappa_right(0.0),
  _torque_left(0.0),
  _torque_right(0.0),
  _throttle(0.0),
  _brakes(),
  _engine(),
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
    }
    else if constexpr ( std::is_same<Axle_mode<0,0>, STEERING<0,0>>::value )
    {
        // Prepare the tires frames to have one rotation (the steering)
        if ( std::get<LEFT>(Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,STATE0,CONTROL0>::_tires).get_frame().get_rotation_angles().size() != 0 )
            throw std::runtime_error("Left tire frame must have zero rotations");

        if ( std::get<RIGHT>(Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,STATE0,CONTROL0>::_tires).get_frame().get_rotation_angles().size() != 0 )
            throw std::runtime_error("Right tire frame must have zero rotations");

        std::get<LEFT>(base_type::_tires).get_frame().add_rotation(0.0, 0.0, Z);
        std::get<RIGHT>(base_type::_tires).get_frame().add_rotation(0.0, 0.0, Z);
    }

    std::get<LEFT>(base_type::_tires).get_frame().set_origin(get_tire_position(LEFT), get_tire_velocity(LEFT));
    std::get<RIGHT>(base_type::_tires).get_frame().set_origin(get_tire_position(RIGHT), get_tire_velocity(RIGHT));
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::Axle_car_3dof(const std::string& name,
                           const Tire_left_t& tire_l, const Tire_right_t& tire_r,
                           Xml_document& database,
                           const std::string& path)
: Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,STATE0,CONTROL0>(name, {tire_l, tire_r}),
  _track(0.0),
  _y_tire({0.0,0.0}),
  _I(0.0),
  _differential_stiffness(0.0),
  _throttle_smooth_pos(0.0),
  _kappa_left(0.0),
  _kappa_right(0.0),
  _dkappa_left(0.0),
  _dkappa_right(0.0),
  _torque_left(0.0),
  _torque_right(0.0),
  _throttle(0.0),
  _brakes(),
  _engine(),
  _delta(0.0)
{
    base_type::_path = path;

    read_parameters(database, path, get_parameters());
    _y_tire = {-0.5*_track, 0.5*_track};

    // Construct the brakes
    _brakes = Brake<Timeseries_t>(database, path + "brakes/");

    // Construct the specific parameters of the axle
    if constexpr ( std::is_same<Axle_mode<0,0>, POWERED<0,0>>::value )
    {
        // Construct engine and brakes
        _engine = Engine<Timeseries_t>(database, path + "engine/", true);
    }
    else if constexpr ( std::is_same<Axle_mode<0,0>, STEERING<0,0>>::value )
    {
        // Prepare the tires frames to have one rotation (the steering)
        if ( std::get<LEFT>(Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,STATE0,CONTROL0>::_tires).get_frame().get_rotation_angles().size() != 0 )
            throw std::runtime_error("Left tire frame must have zero rotations");

        if ( std::get<RIGHT>(Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,STATE0,CONTROL0>::_tires).get_frame().get_rotation_angles().size() != 0 )
            throw std::runtime_error("Right tire frame must have zero rotations");

        std::get<LEFT>(base_type::_tires).get_frame().add_rotation(0.0, 0.0, Z);
        std::get<RIGHT>(base_type::_tires).get_frame().add_rotation(0.0, 0.0, Z);
    }

    std::get<LEFT>(base_type::_tires).get_frame().set_origin(get_tire_position(LEFT), get_tire_velocity(LEFT));
    std::get<RIGHT>(base_type::_tires).get_frame().set_origin(get_tire_position(RIGHT), get_tire_velocity(RIGHT));
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<typename T>
inline bool Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::set_parameter(const std::string& parameter, const T value)
{
    bool found = false;
    // Check if the parameter goes to this object
    if ( parameter.find(base_type::_path) == 0 )
    {
        // Find the parameter in the database
        found = ::set_parameter(get_parameters(), parameter, base_type::_path, value); 

        // If found, update the tires frames in case the parameter modified was their position
        if ( found )
        {
            _y_tire = {-0.5*_track, 0.5*_track};
            std::get<LEFT>(base_type::_tires).get_frame().set_origin(get_tire_position(LEFT), get_tire_velocity(LEFT));
            std::get<RIGHT>(base_type::_tires).get_frame().set_origin(get_tire_position(RIGHT), get_tire_velocity(RIGHT));
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
            if ( !found )
                if ( parameter.find(base_type::_path + "engine/") == 0 )
                {
                    _engine.set_parameter(parameter, value);
                    found = true;
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

template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
void Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::update
    (Timeseries_t Fz_left, Timeseries_t Fz_right, Timeseries_t throttle, Timeseries_t brake_bias)
{
    // Create aliases
    Tire_left_t& tire_l  = std::get<LEFT>(base_type::_tires);
    Tire_right_t& tire_r = std::get<RIGHT>(base_type::_tires);

    // Update the tires
    tire_l.update(-Fz_left, _kappa_left);
    tire_r.update(-Fz_right, _kappa_right);

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

        const Timeseries_t engine_torque = _engine(throttle_percentage, 0.5*(omega_left + omega_right));
        const Timeseries_t differential_torque = _differential_stiffness*(omega_left - omega_right);
        _torque_left  += 0.5*engine_torque - differential_torque;
        _torque_right += 0.5*engine_torque + differential_torque;
    }

    // Compute the time derivative of the two kappas
    _dkappa_left  = tire_l.get_dkappadomega()*(_torque_left  + tire_l.get_longitudinal_torque_at_wheel_center()) / _I;
    _dkappa_right = tire_r.get_dkappadomega()*(_torque_right + tire_r.get_longitudinal_torque_at_wheel_center()) / _I;

    // Get the total force and torque by the tires
    const Vector3d<Timeseries_t> F_left = tire_l.get_force_in_parent(); 
    const Vector3d<Timeseries_t> F_right = tire_r.get_force_in_parent(); 

    const Vector3d<Timeseries_t> T_left = tire_l.get_torque_in_parent(); 
    const Vector3d<Timeseries_t> T_right = tire_r.get_torque_in_parent(); 

    base_type::_F = F_left + F_right;

    base_type::_T =  T_left  + cross(tire_l.get_frame().get_origin(), F_left) 
                   + T_right + cross(tire_r.get_frame().get_origin(), F_right);
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
scalar Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::get_parameter(const std::string& parameter_name) const
{
    if (parameter_name == "track") return _track; 

    throw std::runtime_error("Parameter " + parameter_name + " does not exist in Axle_car_3dof");
}


// ------- Handle state vector
template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<size_t N>
void Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::get_state_derivative(std::array<Timeseries_t,N>& dqdt) const
{
    base_type::get_state_derivative(dqdt);

    // Left tire's kappa
    dqdt[Axle_type::IIDKAPPA_LEFT] = _dkappa_left;

    // Right tire's kappa
    dqdt[Axle_type::IIDKAPPA_RIGHT] = _dkappa_right;
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::set_state_and_control_names(std::array<std::string,NSTATE>& q, std::array<std::string,NCONTROL>& u)
{
    base_type::set_state_and_control_names(q,u);

    if constexpr (std::is_same<Axle_mode<0,0>,POWERED<0,0>>::value)
    {
        // kappa left
        q[Axle_type::IKAPPA_LEFT] = "powered-kappa-left";

        // kappa right
        q[Axle_type::IKAPPA_RIGHT] = "powered-kappa-right";
    }
    else if constexpr (std::is_same<Axle_mode<0,0>,STEERING<0,0>>::value)
    {
        // kappa left
        q[Axle_type::IKAPPA_LEFT] = "steering-kappa-left";

        // kappa right
        q[Axle_type::IKAPPA_RIGHT] = "steering-kappa-right";

        // steering angle
        u[Axle_type::ISTEERING] = "delta";
    }
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Axle_car_3dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::set_state_and_controls(const std::array<Timeseries_t,NSTATE>& q, const std::array<Timeseries_t,NCONTROL>& u)
{
    base_type::set_state_and_controls(q,u);

    // kappa left
    _kappa_left  = q[Axle_type::IKAPPA_LEFT];

    // kappa right
    _kappa_right  = q[Axle_type::IKAPPA_RIGHT];

    if constexpr (std::is_same<Axle_mode<0,0>,STEERING<0,0>>::value)
    {
        // steering angle
        _delta = u[Axle_type::ISTEERING];

        // rotate the tires' frames
        std::get<LEFT>(base_type::_tires).get_frame().set_rotation_angle(0,_delta);
        std::get<RIGHT>(base_type::_tires).get_frame().set_rotation_angle(0,_delta);
    }
}

#endif
