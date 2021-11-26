#ifndef __CHASSIS_HPP__
#define __CHASSIS_HPP__

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis(
    const FrontAxle_t& front_axle, const RearAxle_t& rear_axle,
    Xml_document& database, const std::string& path)
: _inertial_frame(),
  _road_frame(Frame<Timeseries_t>(Vector3d<Timeseries_t>(0.0), Vector3d<Timeseries_t>(0.0), {0.0}, {0.0}, {Z}, _inertial_frame)),
  _chassis_frame(Frame<Timeseries_t>(Vector3d<Timeseries_t>(0.0), Vector3d<Timeseries_t>(0.0), {}, {}, {}, _road_frame)),
  _front_axle(front_axle),
  _rear_axle(rear_axle),
  _F(Vector3d<Timeseries_t>(0.0)),
  _T(Vector3d<Timeseries_t>(0.0))
{
    read_parameters(database, path, get_parameters());

    // Set the front and rear axles new frames parents
    _front_axle.get_frame().set_parent(_chassis_frame); 
    _rear_axle.get_frame().set_parent(_chassis_frame); 
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::Chassis(const Chassis& other)
: _inertial_frame(),
  _road_frame(other._road_frame),
  _chassis_frame(other._chassis_frame),
  _m(other._m),
  _I(other._I),
  _rho(other._rho),
  _CdA(other._CdA),
  _front_axle(other._front_axle),
  _rear_axle(other._rear_axle),
  _F(other._F),
  _T(other._T)
{
    _road_frame.set_parent(_inertial_frame);

    // Update parent of chassis frame to new road frame
    _chassis_frame.set_parent(_road_frame);

    // Set the front and rear axles new frames parents
    _front_axle.get_frame().set_parent(_chassis_frame); 
    _rear_axle.get_frame().set_parent(_chassis_frame); 
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>& Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::operator=(const Chassis& other)
{
    _inertial_frame = Frame<Timeseries_t>();
    _road_frame    = other._road_frame;
    _chassis_frame = other._chassis_frame;
    _front_axle    = other._front_axle;
    _rear_axle     = other._rear_axle;
    _m             = other._m;
    _I             = other._I;
    _rho           = other._rho;
    _CdA           = other._CdA;

    _road_frame.set_parent(_inertial_frame);

    // Update parent of chassis frame to new road frame
    _chassis_frame.set_parent(_road_frame);

    // Set the front and rear axles new frames parents
    _front_axle.get_frame().set_parent(_chassis_frame); 
    _rear_axle.get_frame().set_parent(_chassis_frame); 

    return *this;
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state(Timeseries_t u, Timeseries_t v, Timeseries_t omega)
{
    _u = u;
    _v = v;
    _omega = omega;
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
inline Vector3d<Timeseries_t> Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_aerodynamic_force() const
{
    const Vector3d<Timeseries_t> vel = _road_frame.get_absolute_velocity_in_body();

    return { -0.5*_rho*_CdA*vel[X]*vel[X], 0.0, 0.0 };
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::update(Timeseries_t x, Timeseries_t y, Timeseries_t psi)
{
    const Timeseries_t dx = _u*cos(psi) - _v*sin(psi);
    const Timeseries_t dy = _u*sin(psi) + _v*cos(psi);

    _road_frame.set_origin({x,y,0.0},{dx,dy,0.0},false);
    _road_frame.set_rotation_angle(0,psi,_omega);
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::set_state_and_controls(const std::array<Timeseries_t,NSTATE>& q, const std::array<Timeseries_t,NCONTROL>& u)
{
    get_front_axle().set_state_and_controls(q,u);
    get_rear_axle().set_state_and_controls(q,u);

    // u
    _u = q[IU];

    // v
    _v = q[IV];

    // oemga
    _omega = q[IOMEGA];    
}


template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t STATE0, size_t CONTROL0>
template<size_t N>
void Chassis<Timeseries_t,FrontAxle_t,RearAxle_t,STATE0,CONTROL0>::get_state_derivative(std::array<Timeseries_t,N>& dqdt) const
{
    get_front_axle().get_state_derivative(dqdt);
    get_rear_axle().get_state_derivative(dqdt);

    // dudt
    dqdt[IIDU] = _du;

    // dvdt
    dqdt[IIDV] = _dv;

    // domega
    dqdt[IIDOMEGA] = _dOmega;
}



#endif
