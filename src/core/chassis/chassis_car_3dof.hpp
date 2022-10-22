#ifndef CHASSIS_CAR_3DOF_HPP
#define CHASSIS_CAR_3DOF_HPP

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis_car_3dof()
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

    // Set chassis frame position and velocity (it does not matter before calling update())
    base_type::get_chassis_frame().set_origin({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}); 

    // Set axles frame position and velocity
    base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity());
    base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity());
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis_car_3dof(const FrontAxle_t& front_axle, 
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

    // Set chassis frame position and velocity (it does not matter before calling update())
    base_type::get_chassis_frame().set_origin({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}); 

    // Set axles frame position and velocity
    base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity());
    base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis_car_3dof(Xml_document& database)
: Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>(
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

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<typename T>
inline void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_parameter(const std::string& parameter, const T value)
{
    // Check if the parameter goes to this object
    if ( parameter.find("vehicle/chassis/") == 0 )
    {
        // Find the parameter in the database
        const auto found = ::set_parameter(get_parameters(), __used_parameters, parameter, "vehicle/chassis/", value); 

        // If found, update the axles frames in case the parameter modified was their position
        if ( found )
        {
            base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity());
            base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity());
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

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::fill_xml(Xml_document& doc) const
{
    // Call the fill_xml of the parent
    base_type::fill_xml(doc);

    // Write the parameters of this class
    ::write_parameters(doc, "vehicle/chassis/", get_parameters());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
inline void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::transform_states_to_input_states
    (const std::array<Timeseries_t,NSTATE>& states, const std::array<Timeseries_t,NCONTROL>& controls, std::array<Timeseries_t,NSTATE>& input_states)
{
    base_type::transform_states_to_input_states(states, controls, input_states);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state
 (Timeseries_t u, Timeseries_t v, Timeseries_t omega)
{
    base_type::set_state(u,v,omega);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::update
    (Timeseries_t x, Timeseries_t y, Timeseries_t psi)
{
    // Update base_type
    base_type::update(x,y,psi);

    FrontAxle_t& front_axle = base_type::get_front_axle();
    RearAxle_t& rear_axle = base_type::get_rear_axle();
    
    Frame<Timeseries_t>& road_frame = base_type::get_road_frame();

    const Vector3d<Timeseries_t> F_aero = base_type::get_aerodynamic_force();

    // Compute vertical loads
    /*
       The equations that govern the vertical loads are:
        (Fz):           Fz_fl + Fz_fr + Fz_rl + Fz_rr          = -m.g - Fz_a
        (Mx):           wr.(Fz_rl - Fz_rr) + wf(Fz_fl - Fz_fr) = -h.Fy
        (My):           b.(Fz_rr + Fz_rl) - a.(Fz_fr + Fz_fl)  = -h.Fz - (a_A - a).Fz_a
        (compliance):   Fz_fr.(1-D) - Fz_fl.(1-D)              = D.Fz_rr - D.Fz_rl
      
       Written as matrix-vector product: A.F = b
      
            / 1     1    1    1 \/Fz_fl\   /-m.g - Fz_a            \
            | wf   -wf  wr  -wr ||Fz_fr|   |-h.Fy                  |
        A = | -a   -a    b    b ||Fz_rl| = |-h.Fz - (a_A - a).Fz_a |
            \ D-1  1-D  -D    D /\Fz_rr/   \ 0                     /
      
       Whose inverse matrix is: Lambda = 2.(D.wf + (D-1).wr)
      
                / b/2(a+b)    D/Lambda      -1/2(a+b)   wr/Lambda \
                | b/2(a+b)   -D/Lambda      -1/2(a+b)  -wr/Lambda |
       A^(-1) = | a/2(a+b)   (D-1)/Lambda    1/2(a+b)  -wf/Lambda |
                \ a/2(a+b)   (1-D)/Lambda    1/2(a+b)   wf/Lambda /
    */ 

    // Smoothly restrict to negative forces
    _neg_Fz_fl = -smooth_pos(-_Fz_fl, _Fz_max_ref2);
    _neg_Fz_fr = -smooth_pos(-_Fz_fr, _Fz_max_ref2);
    _neg_Fz_rl = -smooth_pos(-_Fz_rl, _Fz_max_ref2);
    _neg_Fz_rr = -smooth_pos(-_Fz_rr, _Fz_max_ref2);

    // Update axles
    base_type::get_front_axle().update(_neg_Fz_fl, _neg_Fz_fr, _throttle, _brake_bias);
    base_type::get_rear_axle().update(_neg_Fz_rl, _neg_Fz_rr, _throttle, 1.0-_brake_bias);

    // Get the forces from the axles
    const Vector3d<Timeseries_t> x_front = std::get<0>(front_axle.get_frame().get_position_and_velocity_in_target(road_frame));
    const Vector3d<Timeseries_t> x_rear  = std::get<0>(rear_axle.get_frame().get_position_and_velocity_in_target(road_frame));

    const Vector3d<Timeseries_t> F_front = front_axle.get_force();
    const Vector3d<Timeseries_t> F_rear  = rear_axle.get_force();

    const Vector3d<Timeseries_t> T_front = front_axle.get_torque();
    const Vector3d<Timeseries_t> T_rear  = rear_axle.get_torque();

    base_type::_F = F_front + F_rear + F_aero;
    base_type::_F[Z] += base_type::get_mass()*g0;

    base_type::_T =  T_front + cross(x_front, F_front) + T_rear + cross(x_rear, F_rear)
                     + cross((_x_aero-_x_com), F_aero);

    // In this model, the torque is computed around the road projection, hence inertia forces have to be considered
    base_type::_T += cross(_x_com, -base_type::_F);

    // Write the 3DOF equations
    base_type::_du     = base_type::_F[X]/base_type::get_mass() + base_type::get_v()*base_type::get_omega();
    base_type::_dv     = base_type::_F[Y]/base_type::get_mass() - base_type::get_u()*base_type::get_omega();
    base_type::_dOmega = base_type::_T[Z]/base_type::get_inertia().zz();

    // Write the algebric equations
    _Fz_eq           = base_type::_F[Z];
    _Mx_eq           = -base_type::_T[X];   // -1 just to be consistent with Limebeer et al.
    _My_eq           = base_type::_T[Y];
    _roll_balance_eq = (_Fz_fr-_Fz_fl)*(1.0-_roll_balance_coeff) + (_Fz_rl - _Fz_rr)*_roll_balance_coeff;

}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
scalar Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_parameter(const std::string& parameter_name) const
{
     if (parameter_name == "cog_height") return -_x_com[2];

     if (parameter_name == "front_axle_x") return _x_front_axle[0];

     if (parameter_name == "rear_axle_x") return _x_rear_axle[0];

     throw fastest_lap_exception("parameter " + parameter_name + " does not exist in Chassis_Car");
}


// ------- Handle state vector
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t N>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_state_and_state_derivative
    (std::array<Timeseries_t,N>& state, std::array<Timeseries_t, N>& dstate_dt) const
{
    base_type::get_state_and_state_derivative(state, dstate_dt);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NALGEBRAIC_>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_algebraic_constraints
    (std::array<Timeseries_t,NALGEBRAIC_>& algebraic_equations) const
{
    static_assert(NALGEBRAIC_ == NALGEBRAIC);

    algebraic_equations[0] = _Fz_eq/(g0*base_type::get_mass());
    algebraic_equations[1] = _Mx_eq/(g0*base_type::get_mass());
    algebraic_equations[2] = _My_eq/(g0*base_type::get_mass());
    algebraic_equations[3] = _roll_balance_eq/(g0*base_type::get_mass());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_names
    (std::array<std::string,NSTATE>& input_states, std::array<std::string,NALGEBRAIC>& algebraic_states, 
     std::array<std::string,NCONTROL>& controls) const
{
    base_type::set_state_and_control_names(input_states, controls);

    // State ---
    // (empty)

    // Algebraic states ---
    algebraic_states[algebraic_state_names::FZFL] = base_type::get_name() + ".Fz_fl";

    algebraic_states[algebraic_state_names::FZFR] = base_type::get_name() + ".Fz_fr";

    algebraic_states[algebraic_state_names::FZRL] = base_type::get_name() + ".Fz_rl";

    algebraic_states[algebraic_state_names::FZRR] = base_type::get_name() + ".Fz_rr";

    // Controls ---

    // throttle
    controls[control_names::THROTTLE] = base_type::get_name() + ".throttle";

    // brake bias
    controls[control_names::BRAKE_BIAS] = base_type::get_name() + ".brake-bias";
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_controls
    (const std::array<Timeseries_t,NSTATE>& input_states,
     const std::array<Timeseries_t,NALGEBRAIC>& algebraic_states, 
     const std::array<Timeseries_t,NCONTROL>& controls)
{
    base_type::set_state_and_controls(input_states,controls);

    // State ---
    // (empty)

    // Controls ---

    // throttle
    _throttle = controls[control_names::THROTTLE];

    // brake bias
    _brake_bias = controls[control_names::BRAKE_BIAS];

    // Algebraic ---

    // Fz_fl
    _Fz_fl = algebraic_states[algebraic_state_names::FZFL]*g0*base_type::get_mass();

    // Fz_fr
    _Fz_fr = algebraic_states[algebraic_state_names::FZFR]*g0*base_type::get_mass();

    // Fz_rl
    _Fz_rl = algebraic_states[algebraic_state_names::FZRL]*g0*base_type::get_mass();

    // Fz_rr
    _Fz_rr = algebraic_states[algebraic_state_names::FZRR]*g0*base_type::get_mass();
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& input_states_def, std::array<scalar, NSTATE>& input_states_lb,
     std::array<scalar, NSTATE>& input_states_ub, std::array<scalar,NALGEBRAIC>& algebraic_states_def,
     std::array<scalar, NALGEBRAIC>& algebraic_states_lb, std::array<scalar,NALGEBRAIC>& algebraic_states_ub,
     std::array<scalar, NCONTROL>& controls_def, std::array<scalar, NCONTROL>& controls_lb,
     std::array<scalar, NCONTROL>& controls_ub) const
{
    // Call function for the parent class
    base_type::set_state_and_control_upper_lower_and_default_values
        (input_states_def, input_states_lb, input_states_ub, 
         controls_def, controls_lb, controls_ub, 
         50.0*KMH, 380.0*KMH, 50.0*KMH, 10.0);

    // State ---
    // (empty)

    // Controls ---

    // throttle
    controls_def[control_names::THROTTLE] = 0.0;
    controls_lb[control_names::THROTTLE]  = -1.0;
    controls_ub[control_names::THROTTLE]  =  1.0;

    // brake bias
    controls_def[control_names::BRAKE_BIAS] = Value(_brake_bias_0);
    controls_lb[control_names::BRAKE_BIAS]  = 0.0;
    controls_ub[control_names::BRAKE_BIAS]  = 1.0;

    // Algebraic ---

    // Fz_fl
    algebraic_states_def[algebraic_state_names::FZFL] = 0.25;
    algebraic_states_lb[algebraic_state_names::FZFL] = -3.0;
    algebraic_states_ub[algebraic_state_names::FZFL] = -0.01;

    // Fz_fr
    algebraic_states_def[algebraic_state_names::FZFR] = 0.25;
    algebraic_states_lb[algebraic_state_names::FZFR] = -3.0;
    algebraic_states_ub[algebraic_state_names::FZFR] = -0.01;

    // Fz_rl
    algebraic_states_def[algebraic_state_names::FZRL] = 0.25;
    algebraic_states_lb[algebraic_state_names::FZRL] = -3.0;
    algebraic_states_ub[algebraic_state_names::FZRL] = -0.01;

    // Fz_rr
    algebraic_states_def[algebraic_state_names::FZRR] = 0.25;
    algebraic_states_lb[algebraic_state_names::FZRR] = -3.0;
    algebraic_states_ub[algebraic_state_names::FZRR] = -0.01;
}


#endif

