#ifndef CHASSIS_CAR_3DOF_HPP
#define CHASSIS_CAR_3DOF_HPP

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Chassis_car_3dof()
: base_type(FrontAxle_t("front-axle",
            typename FrontAxle_t::Tire_left_type("front-axle.left-tire", "vehicle/front-tire/"),
            typename FrontAxle_t::Tire_right_type("front-axle.right-tire", "vehicle/front-tire/"),
            "vehicle/front-axle/"),
            RearAxle_t("rear-axle", 
            typename RearAxle_t::Tire_left_type("rear-axle.left-tire", "vehicle/rear-tire/"),
            typename RearAxle_t::Tire_right_type("rear-axle.right-tire", "vehicle/rear-tire/"),
            "vehicle/rear-axle/"),
            "vehicle/chassis/") 
{
    // Chassis frame must have zero rotations
    if ( base_type::get_chassis_frame().get_rotation_angles().size() != 0 )
        throw fastest_lap_exception("Chassis frame must have cero rotations for Chassis_car_3dof");

    // Set axles frame position and velocity
    base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
    base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Chassis_car_3dof(const FrontAxle_t& front_axle, 
                                                         const RearAxle_t& rear_axle,
                                                         Xml_document& database,
                                                         const std::string& path)
: base_type(front_axle, rear_axle, database, path) 
{
    read_parameters(database, path, get_parameters(), __used_parameters);
    _brake_bias = _brake_bias_0;

    // Chassis frame must have zero rotations
    if ( base_type::get_chassis_frame().get_rotation_angles().size() != 0 )
        throw fastest_lap_exception("Chassis frame must have cero rotations for Chassis_car_3dof");

    // Set axles frame position and velocity
    base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
    base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::Chassis_car_3dof(Xml_document& database)
: Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>(
           FrontAxle_t("front-axle",
                       typename FrontAxle_t::Tire_left_type("front-axle.left-tire", database, "vehicle/front-tire/"),
                       typename FrontAxle_t::Tire_right_type("front-axle.right-tire", database, "vehicle/front-tire/"),
                       database, "vehicle/front-axle/"),
           RearAxle_t("rear-axle", 
                      typename RearAxle_t::Tire_left_type("rear-axle.left-tire", database, "vehicle/rear-tire/"),
                      typename RearAxle_t::Tire_right_type("rear-axle.right-tire", database, "vehicle/rear-tire/"),
                      database, "vehicle/rear-axle/"),
           database, "vehicle/chassis/") 
{}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
inline void Chassis_car_3dof<Timeseries_t, FrontAxle_t, RearAxle_t, state_start, control_start>::transform_states_to_inputs
(const std::array<Timeseries_t, number_of_inputs>& states, const std::array<Timeseries_t, number_of_controls>& controls, std::array<Timeseries_t, number_of_inputs>& inputs)
{
    throw fastest_lap_exception("[ERROR] Chassis_car_3dof cannot transform states to inputs because it contains algebraic equations.");
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<typename T>
inline void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_parameter(const std::string& parameter, const T value)
{
    // Check if the parameter goes to this object
    if ( parameter.find("vehicle/chassis/") == 0 )
    {
        // Find the parameter in the database
        const auto found = ::set_parameter(get_parameters(), __used_parameters, parameter, "vehicle/chassis/", value); 

        // If found, update the axles frames in case the parameter modified was their position
        if ( found )
        {
            base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
            base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
        }

        // If not found, look for the parameter in the parent class
        if ( !found )
            base_type::set_parameter(parameter, value);
    }
    else
    {
        // Look for the parameter in the parent class
        base_type::set_parameter(parameter, value);
    }
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::fill_xml(Xml_document& doc) const
{
    // Call the fill_xml of the parent
    base_type::fill_xml(doc);

    // Write the parameters of this class
    ::write_parameters(doc, "vehicle/chassis/", get_parameters());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state
 (const Timeseries_t& u, const Timeseries_t& v, const Timeseries_t& omega)
{
    base_type::set_state(u,v,omega);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::update(const Vector3d<Timeseries_t>& ground_position_vector_m, 
    const Euler_angles<scalar>& road_euler_angles_rad, const Timeseries_t& track_heading_angle_rad, 
    const Euler_angles<Timeseries_t>& road_euler_angles_dot_radps, const Timeseries_t& track_heading_angle_dot_radps,
    const Timeseries_t& ground_velocity_z_body_mps)
{
    // Get aliases
    auto& front_axle = base_type::get_front_axle();
    auto& rear_axle  = base_type::get_rear_axle();
    auto& road_frame = base_type::get_road_frame();
    
    // Update base_type
    base_type::update(ground_position_vector_m, road_euler_angles_rad, track_heading_angle_rad, road_euler_angles_dot_radps, track_heading_angle_dot_radps, ground_velocity_z_body_mps);

    // Smoothly restrict to negative forces
    _neg_force_z_fl_N = -smooth_pos(-_force_z_fl_N, _force_z_max_ref2);
    _neg_force_z_fr_N = -smooth_pos(-_force_z_fr_N, _force_z_max_ref2);
    _neg_force_z_rl_N = -smooth_pos(-_force_z_rl_N, _force_z_max_ref2);
    _neg_force_z_rr_N = -smooth_pos(-_force_z_rr_N, _force_z_max_ref2);

    // Update axles
    base_type::get_front_axle().update(_neg_force_z_fl_N, _neg_force_z_fr_N, _throttle, _brake_bias, base_type::get_road_frame());
    base_type::get_rear_axle().update(_neg_force_z_rl_N, _neg_force_z_rr_N, _throttle, 1.0-_brake_bias, base_type::get_road_frame());

    // Compute aerodynamic forces
    const auto aerodynamic_forces = base_type::get_aerodynamic_force();
    const Vector3d<Timeseries_t> F_aero = aerodynamic_forces.lift + aerodynamic_forces.drag;

    // Get the forces from the axles
    const Vector3d<Timeseries_t> x_front = std::get<0>(front_axle.get_frame().get_position_and_velocity_in_target(road_frame));
    const Vector3d<Timeseries_t> x_rear  = std::get<0>(rear_axle.get_frame().get_position_and_velocity_in_target(road_frame));

    const Vector3d<Timeseries_t> F_front = front_axle.get_force();
    const Vector3d<Timeseries_t> F_rear  = rear_axle.get_force();

    const Vector3d<Timeseries_t> T_front = front_axle.get_torque();
    const Vector3d<Timeseries_t> T_rear  = rear_axle.get_torque();

    // Compute the sum of all the forces that are not applied at the CoM
    base_type::_total_force_N = F_front + F_rear + F_aero;

    base_type::_total_torque_Nm =  T_front + cross(x_front, F_front) + T_rear + cross(x_rear, F_rear)
                     + cross((_x_aero-_x_com), F_aero);

    // In this model, the torque is computed around the road projection, hence inertia forces have to be considered
    base_type::_total_torque_Nm += cross(_x_com, -base_type::_total_force_N);

    // Add the gravity the last one, after the torque computation
    base_type::_total_force_N += base_type::get_gravity_force();

    // Write the 3DOF equations
    const auto absolute_omega_body_radps = base_type::get_road_frame().get_omega_absolute_in_body();
    const auto& mass    = base_type::get_mass();
    const auto& Fx      = base_type::_total_force_N.x();
    const auto& Fy      = base_type::_total_force_N.y();
    const auto& Fz      = base_type::_total_force_N.z();
    const auto& Tx      = base_type::_total_torque_Nm.x();
    const auto& Ty      = base_type::_total_torque_Nm.y();
    const auto& Tz      = base_type::_total_torque_Nm.z();
    const auto& u       = base_type::get_u();
    const auto& v       = base_type::get_v();
    const auto& omega_x = absolute_omega_body_radps.x();
    const auto& omega_y = absolute_omega_body_radps.y();
    const auto& omega_z = base_type::get_yaw_rate_radps();
    const auto& Ixx     = base_type::get_inertia().xx();
    const auto& Iyy     = base_type::get_inertia().yy();
    const auto& Izz     = base_type::get_inertia().zz();
    const auto h        = -_x_com.z();

    // Momentum equations
    base_type::_com_velocity_x_dot_mps2   = Fx/mass + (v + h * omega_x) * omega_z - ground_velocity_z_body_mps * omega_y;
    base_type::_com_velocity_y_dot_mps2   = Fy/mass - (u - h * omega_y) * omega_z + ground_velocity_z_body_mps * omega_x;
    _com_velocity_z_dot_mps2              = Fz/mass + (u - h * omega_y) * omega_y - (v + h * omega_x) * omega_x;

    base_type::_com_velocity_x_mps = u - h * omega_y;
    base_type::_com_velocity_y_mps = v + h * omega_x;
    _com_velocity_z_mps            = ground_velocity_z_body_mps;

    // Angular momentum equations
    _roll_angular_momentum_dot_Nm   = Tx + (Iyy - Izz) * omega_z * omega_y;
    _pitch_angular_momentum_dot_Nm  = Ty + (Izz - Ixx) * omega_z * omega_x;
    base_type::_yaw_rate_dot_radps2 = (Tz + (Ixx - Iyy) * omega_x * omega_y) / Izz;

    _roll_angular_momentum_Nms  = Ixx * omega_x;
    _pitch_angular_momentum_Nms = Iyy * omega_y;

    // Extra compliance equation: roll balance equation
    _roll_balance_equation_N = (_force_z_fr_N-_force_z_fl_N)*(1.0-_roll_balance_coeff) + (_force_z_rl_N - _force_z_rr_N)*_roll_balance_coeff;
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
scalar Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::get_parameter(const std::string& parameter_name) const
{
     if (parameter_name == "cog_height") return -_x_com.z();

     if (parameter_name == "front_axle_x") return _x_front_axle.x();

     if (parameter_name == "rear_axle_x") return _x_rear_axle.x();

     throw fastest_lap_exception("parameter " + parameter_name + " does not exist in Chassis_Car");
}


// ------- Handle state vector
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_states>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::get_state_and_state_derivative
    (std::array<Timeseries_t,number_of_states>& state, std::array<Timeseries_t, number_of_states>& dstate_dt) const
{
    // (1) Call the parent setter
    base_type::get_state_and_state_derivative(state, dstate_dt);

    // (2) Set states
    state[state_names::com_velocity_z_mps]          = _com_velocity_z_mps/g0;
    state[state_names::roll_angular_momentum_Nms]   = _roll_angular_momentum_Nms/(g0*base_type::get_mass());
    state[state_names::pitch_angular_momentum_Nms]  = _pitch_angular_momentum_Nms/(g0*base_type::get_mass());
    state[state_names::roll_balance_equation_g]     = 0.0;

    // (3) Set state time derivatives
    dstate_dt[state_names::com_velocity_z_mps]         = _com_velocity_z_dot_mps2/g0;
    dstate_dt[state_names::roll_angular_momentum_Nms]  = _roll_angular_momentum_dot_Nm/(g0*base_type::get_mass());
    dstate_dt[state_names::pitch_angular_momentum_Nms] = _pitch_angular_momentum_dot_Nm/(g0*base_type::get_mass());
    dstate_dt[state_names::roll_balance_equation_g]    = _roll_balance_equation_N/(g0*base_type::get_mass());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_control_names
    (std::array<std::string,number_of_inputs>& inputs, std::array<std::string,number_of_controls>& controls) const
{
    base_type::set_state_and_control_names(inputs, controls);

    // State ---
    // (empty)

    // Algebraic states ---
    inputs[input_names::force_z_fl_g] = base_type::get_name() + ".force_z_fl_g";

    inputs[input_names::force_z_fr_g] = base_type::get_name() + ".force_z_fr_g";

    inputs[input_names::force_z_rl_g] = base_type::get_name() + ".force_z_rl_g";

    inputs[input_names::force_z_rr_g] = base_type::get_name() + ".force_z_rr_g";

    // Controls ---

    // throttle
    controls[control_names::throttle] = base_type::get_name() + ".throttle";

    // brake bias
    controls[control_names::brake_bias] = base_type::get_name() + ".brake-bias";
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_controls
    (const std::array<Timeseries_t,number_of_inputs>& inputs, const std::array<Timeseries_t,number_of_controls>& controls)
{
    base_type::set_state_and_controls(inputs,controls);

    // State ---
    // (empty)

    // Controls ---

    // throttle
    _throttle = controls[control_names::throttle];

    // brake bias
    _brake_bias = controls[control_names::brake_bias];

    // Algebraic ---

    // Fz_fl
    _force_z_fl_N = inputs[input_names::force_z_fl_g]*g0*base_type::get_mass();

    // Fz_fr
    _force_z_fr_N = inputs[input_names::force_z_fr_g]*g0*base_type::get_mass();

    // Fz_rl
    _force_z_rl_N = inputs[input_names::force_z_rl_g]*g0*base_type::get_mass();

    // Fz_rr
    _force_z_rr_N = inputs[input_names::force_z_rr_g]*g0*base_type::get_mass();
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, number_of_inputs>& inputs_def, std::array<scalar, number_of_inputs>& inputs_lb,
     std::array<scalar, number_of_inputs>& inputs_ub,  
     std::array<scalar, number_of_controls>& controls_def, std::array<scalar, number_of_controls>& controls_lb,
     std::array<scalar, number_of_controls>& controls_ub) const
{
    // Call function for the parent class
    base_type::set_state_and_control_upper_lower_and_default_values
        (inputs_def, inputs_lb, inputs_ub, 
         controls_def, controls_lb, controls_ub, 
         50.0*KMH, 380.0*KMH, 50.0*KMH, 10.0);

    // Inputs ---

    // Fz_fl
    inputs_def[input_names::force_z_fl_g] = 0.25;
    inputs_lb[input_names::force_z_fl_g] = -3.0;
    inputs_ub[input_names::force_z_fl_g] = -0.01;

    // Fz_fr
    inputs_def[input_names::force_z_fr_g] = 0.25;
    inputs_lb[input_names::force_z_fr_g] = -3.0;
    inputs_ub[input_names::force_z_fr_g] = -0.01;

    // Fz_rl
    inputs_def[input_names::force_z_rl_g] = 0.25;
    inputs_lb[input_names::force_z_rl_g] = -3.0;
    inputs_ub[input_names::force_z_rl_g] = -0.01;

    // Fz_rr
    inputs_def[input_names::force_z_rr_g] = 0.25;
    inputs_lb[input_names::force_z_rr_g] = -3.0;
    inputs_ub[input_names::force_z_rr_g] = -0.01;


    // Controls ---

    // throttle
    controls_def[control_names::throttle] = 0.0;
    controls_lb[control_names::throttle]  = -1.0;
    controls_ub[control_names::throttle]  =  1.0;

    // brake bias
    controls_def[control_names::brake_bias] = Value(_brake_bias_0);
    controls_lb[control_names::brake_bias]  = 0.0;
    controls_ub[control_names::brake_bias]  = 1.0;
}


#endif

