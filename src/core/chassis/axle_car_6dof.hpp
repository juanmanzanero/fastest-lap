#ifndef __AXLE_CAR_6DOF_HPP__
#define __AXLE_CAR_6DOF_HPP__

template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::Axle_car_6dof(const std::string& name,
                           const Tire_left_t& tire_l, const Tire_right_t& tire_r,
                           Xml_document& database,
                           const std::string& path)
: Axle<Timeseries_t,std::tuple<Tire_left_t,Tire_right_t>,STATE0,CONTROL0>(name, {tire_l, tire_r}),
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

    read_parameters(database, path, get_parameters());
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
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::update(const Vector3d<Timeseries_t>& x0, const Vector3d<Timeseries_t>& v0, Timeseries_t phi, Timeseries_t dphi)
{
    base_type::get_frame().set_origin(x0, v0);

    update(phi,dphi);
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::update(Timeseries_t phi, Timeseries_t dphi)
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


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<typename T>
std::enable_if_t<std::is_same<T,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value,void> Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::set_throttle_and_omega(Timeseries_t throttle, Timeseries_t omega)
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


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<typename T>
std::enable_if_t<std::is_same<T,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value,void> Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::set_torque_and_omega(Timeseries_t T_ax, Timeseries_t omega)
{
    _omega = omega;
    _T_ax  = T_ax;
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<typename T>
std::enable_if_t<std::is_same<T,STEERING_FREE_ROLL<0,0>>::value,void> Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::set_steering_angle(Timeseries_t delta)
{
    _delta = delta;
    std::get<LEFT>(base_type::_tires).get_frame().set_rotation_angle(0,delta);
    std::get<RIGHT>(base_type::_tires).get_frame().set_rotation_angle(0,delta);
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
scalar Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::get_parameter(const std::string& parameter_name) const
{
    if (parameter_name == "track") return _track; 

    if (parameter_name == "beta_steering") return _beta[RIGHT];

    if (parameter_name == "chassis_stiffness") return _k_chassis;

    if (parameter_name == "antiroll_stiffness") return _k_antiroll;
     
    throw std::runtime_error("Parameter " + parameter_name + " does not exist in Axle_car_6dof");
}


// ------- Handle state vector
template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<size_t N>
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::get_state_derivative(std::array<Timeseries_t,N>& dqdt) const
{
    base_type::get_state_derivative(dqdt);

    if constexpr (std::is_same<Axle_mode<0,0>,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value)
    {
        // domega
        dqdt[Axle_type::IIDOMEGA_AXLE] = _domega;
    }
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::set_state_and_control_names(std::array<std::string,NSTATE>& q, std::array<std::string,NCONTROL>& u) 
{
    base_type::set_state_and_control_names(q,u);

    if constexpr (std::is_same<Axle_mode<0,0>,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value)
    {
        // omega
        q[Axle_type::IOMEGA_AXLE] = "axle-omega";

        // axle torque
        u[Axle_type::ITORQUE] = "torque";
    }

    if constexpr (std::is_same<Axle_mode<0,0>,STEERING_FREE_ROLL<0,0>>::value)
    {
        // steering angle
        u[Axle_type::ISTEERING] = "delta";
    }
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::set_state_and_controls(const std::array<Timeseries_t,NSTATE>& q, const std::array<Timeseries_t,NCONTROL>& u)
{
    base_type::set_state_and_controls(q,u);

    if constexpr (std::is_same<Axle_mode<0,0>,POWERED_WITHOUT_DIFFERENTIAL<0,0>>::value)
    {
        // omega
        const Timeseries_t& torque = u[Axle_type::ITORQUE];
        const Timeseries_t& omega  = q[Axle_type::IOMEGA_AXLE];
        set_throttle_and_omega(torque, omega);
    }

    if constexpr (std::is_same<Axle_mode<0,0>,STEERING_FREE_ROLL<0,0>>::value)
    {
        // steering angle
        set_steering_angle(u[Axle_type::ISTEERING]);
    }
}


template<typename Timeseries_t, typename Tire_left_t, typename Tire_right_t, template<size_t,size_t> typename Axle_mode, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Axle_car_6dof<Timeseries_t,Tire_left_t,Tire_right_t,Axle_mode,STATE0,CONTROL0>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& q_def     , std::array<scalar, NSTATE>& q_lb     , std::array<scalar, NSTATE>& q_ub     ,
    std::array<scalar , NCONTROL>& u_def   , std::array<scalar, NCONTROL>& u_lb   , std::array<scalar, NCONTROL>& u_ub) const
{
    base_type::set_state_and_control_upper_lower_and_default_values(q_def, q_lb, q_ub, u_def, u_lb, u_ub); 

    if constexpr (std::is_same_v<Axle_mode<0,0>,POWERED_WITHOUT_DIFFERENTIAL<0,0>>)
    {
        // Set the omega of the axle
        q_def[Axle_type::IOMEGA_AXLE] = 10.0*KMH/0.139; 
        q_lb[Axle_type::IOMEGA_AXLE]  = 10.0*KMH/0.139;
        q_ub[Axle_type::IOMEGA_AXLE]  = 200.0*KMH/0.139;

        if ( !_engine.direct_torque() )
        {
            u_def[Axle_type::ITORQUE] = 0.0;
            u_lb[Axle_type::ITORQUE]  = -1.0;
            u_ub[Axle_type::ITORQUE]  = 1.0;    
        }
        else
        {
            u_def[Axle_type::ITORQUE] = 0.0;
            u_lb[Axle_type::ITORQUE]  = -200.0;
            u_ub[Axle_type::ITORQUE]  =  200.0;
        }

    }
    
    if constexpr (std::is_same_v<Axle_mode<0,0>,STEERING_FREE_ROLL<0,0>>)
    {
        u_def[Axle_type::ISTEERING] = 0.0;
        u_lb[Axle_type::ISTEERING]  = -20.0*DEG;
        u_ub[Axle_type::ISTEERING]  =  20.0*DEG;
    }
}

#endif
