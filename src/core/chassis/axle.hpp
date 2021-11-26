#ifndef __AXLE_HPP__
#define __AXLE_HPP__

template<typename Timeseries_t, typename Tires_tuple, size_t STATE0, size_t CONTROL0>
inline Axle<Timeseries_t,Tires_tuple,STATE0,CONTROL0>::Axle(const std::string& name, const Tires_tuple& tires)
: _frame(),
  _name(name),
  _tires(tires),
  _F(Vector3d<Timeseries_t>(0.0)),
  _T(Vector3d<Timeseries_t>(0.0))
{
    static_assert(NTIRES > 0);
    static_assert(NTIRES <= 2);

    // Update the parent frames of the tires to this one
    std::get<0>(_tires).get_frame().set_parent(_frame);

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).get_frame().set_parent(_frame);
}


template<typename Timeseries_t, typename Tires_tuple, size_t STATE0, size_t CONTROL0>
inline Axle<Timeseries_t,Tires_tuple,STATE0,CONTROL0>::Axle(const Axle& other)
: _frame(other._frame),
  _name(other._name),
  _tires(other._tires),
  _F(other._F),
  _T(other._T)
{
    static_assert(NTIRES <= 2);

    // Update the parent frames of the tires to this one
    std::get<0>(_tires).get_frame().set_parent(_frame);

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).get_frame().set_parent(_frame);

}


template<typename Timeseries_t, typename Tires_tuple, size_t STATE0, size_t CONTROL0>
inline Axle<Timeseries_t,Tires_tuple,STATE0,CONTROL0>& Axle<Timeseries_t,Tires_tuple,STATE0,CONTROL0>::operator=(const Axle& other)
{
    _name = other._name;
    _frame = other._frame;
    _tires = other._tires;

    static_assert(NTIRES <= 2);

    // Update the parent frames of the tires to this one
    std::get<0>(_tires).get_frame().set_parent(_frame);

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).get_frame().set_parent(_frame);

    return *this;
}

template<typename Timeseries_t, typename Tires_tuple, size_t STATE0, size_t CONTROL0>
template<size_t N>
void Axle<Timeseries_t,Tires_tuple,STATE0,CONTROL0>::get_state_derivative(std::array<Timeseries_t,N>& dqdt) const
{
    std::get<0>(_tires).get_state_derivative(dqdt);

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).get_state_derivative(dqdt);
}


template<typename Timeseries_t, typename Tires_tuple, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Axle<Timeseries_t,Tires_tuple,STATE0,CONTROL0>::set_state_and_control_names(std::array<std::string,NSTATE>& q, std::array<std::string,NCONTROL>& u) const
{
    std::get<0>(_tires).set_state_and_control_names(q,u);
    
    if constexpr (NTIRES == 2)
        std::get<1>(_tires).set_state_and_control_names(q,u);
}


template<typename Timeseries_t, typename Tires_tuple, size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Axle<Timeseries_t,Tires_tuple,STATE0,CONTROL0>::set_state_and_controls(const std::array<Timeseries_t,NSTATE>& q, const std::array<Timeseries_t,NCONTROL>& u)
{
    std::get<0>(_tires).set_state_and_controls(q,u);

    if constexpr (NTIRES == 2)
        std::get<1>(_tires).set_state_and_controls(q,u);
}



#endif
