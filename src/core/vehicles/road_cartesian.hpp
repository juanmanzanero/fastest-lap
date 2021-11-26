#ifndef __ROAD_CARTESIAN_HPP__
#define __ROAD_CARTESIAN_HPP__

template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
void Road_cartesian<Timeseries_t,STATE0,CONTROL0>::update(const Timeseries_t u, const Timeseries_t v, const Timeseries_t omega)
{
    // dxdt
    _dx = u*cos(base_type::_psi) - v*sin(base_type::_psi);

    // dydt
    _dy = u*sin(base_type::_psi) + v*cos(base_type::_psi);

    // dpsidt
    _dpsi = omega;
}

template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
template<size_t N>
void Road_cartesian<Timeseries_t,STATE0,CONTROL0>::get_state_derivative(std::array<Timeseries_t,N>& dqdt) const
{
    // dxdt
    dqdt[IIDX] = _dx;

    // dydt
    dqdt[IIDY] = _dy;

    // dpsidt
    dqdt[IIDPSI] = _dpsi;
}


template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_cartesian<Timeseries_t,STATE0,CONTROL0>::set_state_and_controls(const Timeseries_t t, const std::array<Timeseries_t,NSTATE>& q, const std::array<Timeseries_t,NCONTROL>& u)
{
    // x
    base_type::_x = q[IX];

    // y
    base_type::_y = q[IY];

    // psi
    base_type::_psi = q[IPSI];
}


template<typename Timeseries_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_cartesian<Timeseries_t,STATE0,CONTROL0>::set_state_and_control_names(std::array<std::string,NSTATE>& q, std::array<std::string,NCONTROL>& u) const
{
    // x
    q[IX] = "x";

    // y
    q[IY] = "y";

    // psi
    q[IPSI] = "psi";
}


#endif
