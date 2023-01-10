#ifndef ROAD_CURVILINEAR_HPP
#define ROAD_CURVILINEAR_HPP

template<typename Timeseries_t,typename Track_t,size_t state_start, size_t control_start>
void Road_curvilinear<Timeseries_t,Track_t,state_start,control_start>::update(const Timeseries_t& velocity_x_body, const Timeseries_t& velocity_y_body, const Timeseries_t& yaw_rate)
{
    auto& dtime_ds = base_type::get_dtimeds();
    // Compute dtime/ds
    dtime_ds = (1.0 - _lateral_displacement_mps * _curvature.z()) / (velocity_x_body * cos(_track_heading_angle_rad) - velocity_y_body * sin(_track_heading_angle_rad));

    // Compute lateral displacement time derivative
    _lateral_displacement_dot_mps = velocity_x_body*sin(_track_heading_angle_rad) + velocity_y_body*cos(_track_heading_angle_rad);

    // Compute track heading angle time derivative
    _track_heading_angle_dot_radps = yaw_rate - _curvature.z() / dtime_ds;

    // Assign the position, heading angle, and ground vertical velocity to the base type
    base_type::get_position()                 = _position + _lateral_displacement_mps * _normal_vector;
    base_type::get_psi()                      = _track_heading_angle_rad + _theta;
    base_type::get_ground_vertical_velocity() = _lateral_displacement_mps * _curvature.x() / dtime_ds;
}


template<typename Timeseries_t,typename Track_t,size_t state_start, size_t control_start>
template<size_t number_of_states>
void Road_curvilinear<Timeseries_t,Track_t,state_start,control_start>::get_state_and_state_derivative
    (std::array<Timeseries_t, number_of_states>& states, std::array<Timeseries_t,number_of_states>& dstates_dt) const
{
    // dtimedt
    states    [state_names::time] = _time;
    dstates_dt[state_names::time] = 1.0;        // Time derivative of t -> 1

    // dndtime
    states    [state_names::lateral_displacement] = _lateral_displacement_mps;
    dstates_dt[state_names::lateral_displacement] = _lateral_displacement_dot_mps;

    // dalphadtime
    states    [state_names::track_heading_angle] = _track_heading_angle_rad;
    dstates_dt[state_names::track_heading_angle] = _track_heading_angle_dot_radps;
}


template<typename Timeseries_t,typename Track_t,size_t state_start, size_t control_start>
template<size_t NSTATE, size_t NCONTROL>
void Road_curvilinear<Timeseries_t,Track_t,state_start,control_start>::set_state_and_controls
    (const scalar t, const std::array<Timeseries_t,NSTATE>& inputs, const std::array<Timeseries_t,NCONTROL>& controls)
{
    update_track(t);

    // time
    _time = inputs[input_names::time];

    // n 
    _lateral_displacement_mps = inputs[input_names::lateral_displacement];

    // alpha
    _track_heading_angle_rad = inputs[input_names::track_heading_angle];

}

template<typename Timeseries_t,typename Track_t,size_t state_start, size_t control_start>
template<size_t NSTATE, size_t NCONTROL>
void Road_curvilinear<Timeseries_t,Track_t,state_start,control_start>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& inputs_def, std::array<scalar, NSTATE>& inputs_lb, 
     std::array<scalar, NSTATE>& inputs_ub, std::array<scalar , NCONTROL>& controls_def, 
     std::array<scalar, NCONTROL>& controls_lb, std::array<scalar, NCONTROL>& controls_ub) const
{
    // time
    inputs_def[input_names::time] = 0.0;
    inputs_lb [input_names::time] = 0.0;
    inputs_ub [input_names::time] = std::numeric_limits<scalar>::max();

    // n
    inputs_def[input_names::lateral_displacement] = 0.0;
    inputs_lb [input_names::lateral_displacement] = std::numeric_limits<scalar>::lowest();
    inputs_ub [input_names::lateral_displacement] = std::numeric_limits<scalar>::max();

    // alpha
    inputs_def[input_names::track_heading_angle] = 0.0;
    inputs_lb [input_names::track_heading_angle] = -45.0*DEG;
    inputs_ub [input_names::track_heading_angle] =  45.0*DEG;
}


template<typename Timeseries_t,typename Track_t,size_t state_start, size_t control_start>
template<size_t NSTATE, size_t NCONTROL>
void Road_curvilinear<Timeseries_t,Track_t,state_start,control_start>::set_state_and_control_names
    (std::string& key_name, std::array<std::string,NSTATE>& inputs, std::array<std::string,NCONTROL>& controls) 
{
    key_name = "road.arclength";

    // time
    inputs[input_names::time]  = "time";

    // n
    inputs[input_names::lateral_displacement]     = "road.lateral-displacement";

    // alpha
    inputs[input_names::track_heading_angle] = "road.track-heading-angle";
}


template<typename Timeseries_t,typename Track_t,size_t state_start, size_t control_start>
inline void Road_curvilinear<Timeseries_t,Track_t,state_start,control_start>::update_track(const scalar t) 
{
    // Get the Frenet frame at the desired point
    const auto frenet_frame = _track(t);

    // Get position
    _position = frenet_frame.position;

    // Get Euler angles
    _theta = frenet_frame.euler_angles.yaw();
    _mu = frenet_frame.euler_angles.pitch();
    _phi = frenet_frame.euler_angles.roll();

    // Get Euler angles derivative
    _dtheta_ds = frenet_frame.deuler_angles_ds.yaw();
    _dmu_ds    = frenet_frame.deuler_angles_ds.pitch();
    _dphi_ds   = frenet_frame.deuler_angles_ds.roll();

    // Get Frenet frame
    _tangent_vector = Vector3d{ cos(_theta) * cos(_mu), sin(_theta) * cos(_mu), -sin(_mu) };

    _normal_vector = Vector3d{ cos(_theta) * sin(_mu) * sin(_phi) - sin(_theta) * cos(_phi),
                       sin(_theta) * sin(_mu) * sin(_phi) + cos(_theta) * cos(_phi),
                       cos(_mu) * sin(_phi) };

    _binormal_vector = Vector3d{ cos(_theta) * sin(_mu) * cos(_phi) + sin(_theta) * sin(_phi),
                        sin(_theta) * sin(_mu) * cos(_phi) - cos(_theta) * sin(_phi),
                        cos(_mu) * cos(_phi) };

    // Get curvature
    _curvature = curvature(frenet_frame.euler_angles, frenet_frame.deuler_angles_ds);
}
#endif
