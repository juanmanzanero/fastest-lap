#ifndef __CHASSIS_CAR_6DOF_HPP__
#define __CHASSIS_CAR_6DOF_HPP__

#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis_car_6dof()
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
        throw fastest_lap_exception("Chassis frame must have cero rotations for Chassis_car_6dof");

    // Set chassis frame position and velocity (it does not matter before calling update())
    base_type::get_chassis_frame().set_origin(get_com_position(), get_com_velocity()); 

    // Set axles frame position and velocity
    base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity());
    base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis_car_6dof(const FrontAxle_t& front_axle, 
                                                         const RearAxle_t& rear_axle,
                                                         Xml_document& database,
                                                         const std::string& path)
: base_type(front_axle, rear_axle, database, path),
_z(0.0), _dz(0.0), _d2z(0.0), 
_mu(0.0), _dmu(0.0), _d2mu(0.0), 
_phi(0.0), _dphi(0.0), _d2phi(0.0)
{
    read_parameters(database, path, get_parameters(), __used_parameters);
    // Chassis frame must have zero rotations
    if ( base_type::get_chassis_frame().get_rotation_angles().size() != 0 )
        throw fastest_lap_exception("Chassis frame must have cero rotations for Chassis_car_6dof");

    // Set chassis frame position and velocity (it does not matter before calling update())
    base_type::get_chassis_frame().set_origin(get_com_position(), get_com_velocity()); 

    // Set axles frame position and velocity
    base_type::get_front_axle().get_frame().set_origin(get_front_axle_position(), get_front_axle_velocity());
    base_type::get_rear_axle().get_frame().set_origin(get_rear_axle_position(), get_rear_axle_velocity());
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis_car_6dof(Xml_document& database)
: Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>(
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
inline void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state
 (Timeseries_t u, Timeseries_t v, Timeseries_t omega, Timeseries_t z, Timeseries_t dz, Timeseries_t mu, Timeseries_t dmu, Timeseries_t phi, Timeseries_t dphi)
{
    base_type::set_state(u,v,omega);

    // Update CoG position
    _z    = z;
    _dz   = dz;

    // Update pitch attitude
    _mu   = mu;
    _dmu  = dmu;
    
    // Update roll attitude
    _phi  = phi;
    _dphi = dphi;
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::update
    (Timeseries_t x, Timeseries_t y, Timeseries_t psi)
{
    // Update base_type
    base_type::update(x,y,psi);

    FrontAxle_t& front_axle = base_type::get_front_axle();
    RearAxle_t& rear_axle = base_type::get_rear_axle();
    
    Frame<Timeseries_t>& road_frame = base_type::get_road_frame();

    const Timeseries_t& m = base_type::get_mass();

    // Update frame
    base_type::get_chassis_frame().set_origin(_x_com + Vector3d<Timeseries_t>(0.0, 0.0, _z), {0.0, 0.0, _dz});

    // Update axles
    base_type::get_front_axle().update(get_front_axle_position(),get_front_axle_velocity(), _phi, _dphi);
    base_type::get_rear_axle().update(get_rear_axle_position(), get_rear_axle_velocity(), _phi, _dphi);

    // Get the forces from the axles
    const Matrix3x3<Timeseries_t> Q_front = front_axle.get_frame().get_rotation_matrix(road_frame);
    const Matrix3x3<Timeseries_t> Q_rear = rear_axle.get_frame().get_rotation_matrix(road_frame);
    
    const Vector3d<Timeseries_t> x_front = std::get<0>(front_axle.get_frame().get_position_and_velocity_in_target(road_frame));
    const Vector3d<Timeseries_t> x_rear  = std::get<0>(rear_axle.get_frame().get_position_and_velocity_in_target(road_frame));

    const Vector3d<Timeseries_t> F_front = Q_front*front_axle.get_force();
    const Vector3d<Timeseries_t> F_rear  = Q_rear*rear_axle.get_force();

    const Vector3d<Timeseries_t> T_front = Q_front*front_axle.get_torque();
    const Vector3d<Timeseries_t> T_rear  = Q_rear* rear_axle.get_torque();

    base_type::_F = F_front + F_rear;

    base_type::_F[Z] += base_type::get_mass()*g0;

    base_type::_T =   T_front + cross(x_front, F_front)
                    + T_rear + cross(x_rear, F_rear);

    // Aerodynamic drag
    const Vector3d<Timeseries_t> F_aero = base_type::get_aerodynamic_force();
    base_type::_F += F_aero; 
    base_type::_T += cross(base_type::get_chassis_frame().get_origin(), F_aero);

    // Add the external forces
    base_type::_F += base_type::_Fext;

    base_type::_T += base_type::_Text;

    // Newton equations
    const Vector3d<Timeseries_t> dvdt = -Newton_lhs() + base_type::_F/m;

    base_type::_du = dvdt[X];
    base_type::_dv = dvdt[Y];
    _d2z = dvdt[Z];

    // Euler equations on road contact point
    const Vector3d<Timeseries_t> d2phi = linsolve(Euler_m(), -Euler_lhs() + base_type::_T);

    _d2phi = d2phi[X];
    _d2mu  = d2phi[Y];
    base_type::_dOmega = d2phi[Z];
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Vector3d<Timeseries_t> Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Newton_lhs() const
{
    const Frame<Timeseries_t>& road_frame = base_type::get_road_frame();
    const Vector3d<Timeseries_t> omega = road_frame.get_omega_absolute_in_body();
    const Vector3d<Timeseries_t> vel = road_frame.get_absolute_velocity_in_body(); 

    return cross(omega,vel);
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Vector3d<Timeseries_t> Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Euler_lhs() const
{
    const Frame<Timeseries_t>& road_frame = base_type::get_road_frame();
    const Timeseries_t& omega = road_frame.get_rotation_angles_derivative()[0];
    const Vector3d<Timeseries_t> vel = road_frame.get_absolute_velocity_in_body(); 

    const Timeseries_t& u = vel[X];
    const Timeseries_t& v = vel[Y];

    const Timeseries_t& m = base_type::get_mass();
    const sMatrix3x3& I = base_type::get_inertia();
    const scalar h = -_x_com[2];

    const Timeseries_t& du = base_type::_du;
    const Timeseries_t& dv = base_type::_dv;
  
    return { m*(dv*h + (h - _z)*omega*u) - (I.yy()-I.zz()+I.xx())*omega*_dmu 
             - (I.yy() - I.zz())*omega*omega*_phi , 
             m*((_z - h)*du + h*omega*v) + (I.yy()-I.zz()+I.xx())*omega*_dphi
             - ((I.xx() - I.zz())*_mu + I.xz())*omega*omega,
             2.0*I.zx()*_dmu*omega
           };
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Matrix3x3<Timeseries_t> Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Euler_m() const
{
    const sMatrix3x3& I = base_type::get_inertia();

    return { I.xx(), 0.0, -(I.xx() - I.zz())*_mu - I.xz(),
             0.0, I.yy(), (I.yy() - I.zz())*_phi,
            -I.xz(), 0.0, I.zz() + 2.0*I.xz()*_mu };
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
scalar Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_parameter(const std::string& parameter_name) const
{
     if (parameter_name == "cog_height") return -_x_com[2];

     if (parameter_name == "front_axle_x") return _x_front_axle[0];

     if (parameter_name == "rear_axle_x") return _x_rear_axle[0];

     throw fastest_lap_exception("parameter " + parameter_name + " does not exist in Chassis_Car");
}


// ------- Handle state vector
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t N>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_state_derivative(std::array<Timeseries_t,N>& dqdt) const
{
    base_type::get_state_derivative(dqdt);
    // 1st order

    // dzdt
    dqdt[IIDZ] = _dz;

    // dphidt
    dqdt[IIDPHI] = _dphi;

    // dmudt
    dqdt[IIDMU] = _dmu;

    // 2nd order
    
    // d2zdt2
    dqdt[IID2Z] = _d2z;

    // d2phidt2
    dqdt[IID2PHI] = _d2phi;

    // d2mudt2
    dqdt[IID2MU] = _d2mu;
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_names(std::array<std::string,NSTATE>& q, std::array<std::string,NALGEBRAIC>& qa, std::array<std::string,NCONTROL>& u) const
{
    base_type::set_state_and_control_names(q,u);

    // z
    q[IZ] = "chassis.position.z";

    // phi
    q[IPHI] = "chassis.attitude.phi";

    // mu
    q[IMU] = "chassis.attitude.mu";

    // 2nd order

    // dzdt
    q[IDZ] = "chassis.velocity.z";

    // dphidt
    q[IDPHI] = "chassis.omega.x";

    // dmudt
    q[IDMU] = "chassis.omega.y";
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_controls(const std::array<Timeseries_t,NSTATE>& q, const std::array<Timeseries_t,NALGEBRAIC>& qa, const std::array<Timeseries_t,NCONTROL>& u)
{
    base_type::set_state_and_controls(q,u);

    // State ---

    // 1st order

    // z
    _z = q[IZ];

    // phi
    _phi = q[IPHI];

    // mu
    _mu = q[IMU];

    // 2nd order

    // dz
    _dz = q[IDZ];

    // dphidt
    _dphi = q[IDPHI];

    // dmudt
    _dmu = q[IDMU];
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& q_def     , std::array<scalar, NSTATE>& q_lb     , std::array<scalar, NSTATE>& q_ub     ,
    std::array<scalar , NALGEBRAIC>& qa_def, std::array<scalar, NALGEBRAIC>& qa_lb, std::array<scalar, NALGEBRAIC>& qa_ub,
    std::array<scalar , NCONTROL>& u_def   , std::array<scalar, NCONTROL>& u_lb   , std::array<scalar, NCONTROL>& u_ub) const
{
    // Call function for the parent class
    base_type::set_state_and_control_upper_lower_and_default_values(q_def, q_lb, q_ub, u_def, u_lb, u_ub, 10.0*KMH, 200.0*KMH, 10.0*KMH, 10.0);

    // State ---

    // 1st order

    // z
    q_def[IZ] = 1.0e-5;
    q_lb[IZ]  = 1.0e-5;
    q_ub[IZ]  = 0.139;

    // phi
    q_def[IPHI] = 0.0;
    q_lb[IPHI]  = -30.0*DEG;
    q_ub[IPHI]  =  30.0*DEG;

    // mu
    q_def[IMU] = 0.0;
    q_lb[IMU]  = -30.0*DEG;
    q_ub[IMU]  =  30.0*DEG;

    // 2nd order

    // dz
    q_def[IDZ] = 0.0;
    q_lb[IDZ]  = -10.0;
    q_ub[IDZ]  =  10.0;

    // dphidt
    q_def[IDPHI] = 0.0;
    q_lb[IDPHI]  = -10.0; 
    q_ub[IDPHI]  =  10.0;

    // dmudt
    q_def[IDMU] = 0.0;
    q_lb[IDMU]  = -10.0;
    q_ub[IDMU]  =  10.0;
}

#endif

