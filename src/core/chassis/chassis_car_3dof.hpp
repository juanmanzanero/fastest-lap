#ifndef __CHASSIS_CAR_3DOF_HPP__
#define __CHASSIS_CAR_3DOF_HPP__

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis_car_3dof()
: base_type(FrontAxle_t("front axle",
            typename FrontAxle_t::Tire_left_type("front left tire", "vehicle/front-tire/"),
            typename FrontAxle_t::Tire_right_type("front right tire", "vehicle/front-tire/"),
            "vehicle/front-axle/"),
            RearAxle_t("rear axle", 
            typename RearAxle_t::Tire_left_type("rear left tire", "vehicle/rear-tire/"),
            typename RearAxle_t::Tire_right_type("rear right tire", "vehicle/rear-tire/"),
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
           FrontAxle_t("front axle",
                       typename FrontAxle_t::Tire_left_type("front left tire", database, "vehicle/front-tire/"),
                       typename FrontAxle_t::Tire_right_type("front right tire", database, "vehicle/front-tire/"),
                       database, "vehicle/front-axle/"),
           RearAxle_t("rear axle", 
                      typename RearAxle_t::Tire_left_type("rear left tire", database, "vehicle/rear-tire/"),
                      typename RearAxle_t::Tire_right_type("rear right tire", database, "vehicle/rear-tire/"),
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
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_state_derivative(std::array<Timeseries_t,N>& dqdt) const
{
    base_type::get_state_derivative(dqdt);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NALGEBRAIC_>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_algebraic_constraints(std::array<Timeseries_t,NALGEBRAIC_>& dqa) const
{
    static_assert(NALGEBRAIC_ == NALGEBRAIC);

    dqa[0] = _Fz_eq/(g0*base_type::get_mass());
    dqa[1] = _Mx_eq/(g0*base_type::get_mass());
    dqa[2] = _My_eq/(g0*base_type::get_mass());
    dqa[3] = _roll_balance_eq/(g0*base_type::get_mass());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_names(std::array<std::string,NSTATE>& q, std::array<std::string,NALGEBRAIC>& qa, std::array<std::string,NCONTROL>& u) 
{
    base_type::set_state_and_control_names(q,u);

    // State ---
    // (empty)

    // Algebraic states ---
    qa[IFZFL] = "Fz_fl";

    qa[IFZFR] = "Fz_fr";

    qa[IFZRL] = "Fz_rl";

    qa[IFZRR] = "Fz_rr";

    // Controls ---

    // throttle
    u[ITHROTTLE] = "throttle";

    // brake bias
    u[IBRAKE_BIAS] = "brake-bias";
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_controls(const std::array<Timeseries_t,NSTATE>& q,
    const std::array<Timeseries_t,NALGEBRAIC>& qa, const std::array<Timeseries_t,NCONTROL>& u)
{
    base_type::set_state_and_controls(q,u);

    // State ---
    // (empty)

    // Controls ---

    // throttle
    _throttle = u[ITHROTTLE];

    // brake bias
    _brake_bias = u[IBRAKE_BIAS];

    // Algebraic ---

    // Fz_fl
    _Fz_fl = qa[IFZFL]*g0*base_type::get_mass();

    // Fz_fr
    _Fz_fr = qa[IFZFR]*g0*base_type::get_mass();

    // Fz_rl
    _Fz_rl = qa[IFZRL]*g0*base_type::get_mass();

    // Fz_rr
    _Fz_rr = qa[IFZRR]*g0*base_type::get_mass();
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_3dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& q_def     , std::array<scalar, NSTATE>& q_lb     , std::array<scalar, NSTATE>& q_ub     ,
    std::array<scalar , NALGEBRAIC>& qa_def, std::array<scalar, NALGEBRAIC>& qa_lb, std::array<scalar, NALGEBRAIC>& qa_ub,
    std::array<scalar , NCONTROL>& u_def   , std::array<scalar, NCONTROL>& u_lb   , std::array<scalar, NCONTROL>& u_ub) const
{
    // Call function for the parent class
    base_type::set_state_and_control_upper_lower_and_default_values(q_def, q_lb, q_ub, u_def, u_lb, u_ub, 50.0*KMH, 380.0*KMH, 50.0*KMH, 10.0);

    // State ---
    // (empty)

    // Controls ---

    // throttle
    u_def[ITHROTTLE] = 0.0;
    u_lb[ITHROTTLE]  = -1.0;
    u_ub[ITHROTTLE]  = _maximum_throttle;

    // brake bias
    u_def[IBRAKE_BIAS] = Value(_brake_bias_0);
    u_lb[IBRAKE_BIAS]  = 0.0;
    u_ub[IBRAKE_BIAS]  = 1.0;

    // Algebraic ---

    // Fz_fl
    qa_def[IFZFL] = 0.25;
    qa_lb[IFZFL] = -3.0;
    qa_ub[IFZFL] = -0.01;

    // Fz_fr
    qa_def[IFZFR] = 0.25;
    qa_lb[IFZFR] = -3.0;
    qa_ub[IFZFR] = -0.01;

    // Fz_rl
    qa_def[IFZRL] = 0.25;
    qa_lb[IFZRL] = -3.0;
    qa_ub[IFZRL] = -0.01;

    // Fz_rr
    qa_def[IFZRR] = 0.25;
    qa_lb[IFZRR] = -3.0;
    qa_ub[IFZRR] = -0.01;
}


#endif

