#ifndef ROAD_CURVILINEAR_HPP
#define ROAD_CURVILINEAR_HPP

template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
inline Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::Road_curvilinear(const Track_t& track)
: _track(track),
  _n(0.0)
{}


template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
void Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::update(const Timeseries_t u, const Timeseries_t v, const Timeseries_t omega)
{
    const Timeseries_t dtimeds = (1.0 - _n*_k)/(u*cos(_alpha) - v*sin(_alpha));

    base_type::_dtimedt = dtimeds*_drnorm;

    // dtimedtime
    _dtime = 1.0;

    // dndtime
    _dn = u*sin(_alpha) + v*cos(_alpha);

    // dalphadtime
    _dalpha = omega - _k/dtimeds;
}


template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
template<size_t N>
void Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::get_state_and_state_derivative
    (std::array<Timeseries_t, N>& states, std::array<Timeseries_t,N>& dstates_dt) const
{
    // dtimedt
    states    [state_names::TIME] = _time;
    dstates_dt[state_names::TIME] = _dtime;

    // dndtime
    states    [state_names::N] = _n;
    dstates_dt[state_names::N] = _dn;

    // dalphadtime
    states   [state_names::ALPHA] = _alpha;
    dstates_dt[state_names::ALPHA] = _dalpha;
}


template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::set_state_and_controls
    (const scalar t, const std::array<Timeseries_t,NSTATE>& input_states, const std::array<Timeseries_t,NCONTROL>& controls)
{
    update_track(t);

    // time
    _time = input_states[input_state_names::TIME];

    // n 
    _n = input_states[input_state_names::N];

    // alpha
    _alpha = input_states[input_state_names::ALPHA];

    // Compute x,y and psi from the track
    
    // Frenet frame (tan,nor,bi)
    base_type::_x   = _r[X] + _n*_nor[X];
    base_type::_y   = _r[Y] + _n*_nor[Y];
    base_type::_psi = _alpha + _theta;
}

template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& input_states_def, std::array<scalar, NSTATE>& input_states_lb, 
     std::array<scalar, NSTATE>& input_states_ub, std::array<scalar , NCONTROL>& controls_def, 
     std::array<scalar, NCONTROL>& controls_lb, std::array<scalar, NCONTROL>& controls_ub) const
{
    // time
    input_states_def[input_state_names::TIME] = 0.0;
    input_states_lb [input_state_names::TIME] = 0.0;
    input_states_ub [input_state_names::TIME] = std::numeric_limits<scalar>::max();

    // n
    input_states_def[input_state_names::N] = 0.0;
    input_states_lb [input_state_names::N] = std::numeric_limits<scalar>::lowest();
    input_states_ub [input_state_names::N] = std::numeric_limits<scalar>::max();

    // alpha
    input_states_def[input_state_names::ALPHA] = 0.0;
    input_states_lb [input_state_names::ALPHA] = -45.0*DEG;
    input_states_ub [input_state_names::ALPHA] =  45.0*DEG;
}


template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::set_state_and_control_names
    (std::string& key_name, std::array<std::string,NSTATE>& input_states, std::array<std::string,NCONTROL>& controls) 
{
    key_name = "road.arclength";

    // time
    input_states[input_state_names::TIME]  = "time";

    // n
    input_states[input_state_names::N]     = "road.lateral-displacement";

    // alpha
    input_states[input_state_names::ALPHA] = "road.track-heading-angle";
}


template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
inline void Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::update_track(const scalar t) 
{
    // Position and two derivatives
    std::tie(_r,_dr,_d2r) = _track(t);

    // Norm of position and derivatives
    _rnorm = _r.norm();
    _drnorm = _dr.norm();

    // Frenet frame
    _tan = _dr/_drnorm;
    _nor = sVector3d(-_tan[Y],_tan[X],0.0);
    _bi  = cross(_tan,_nor);

    // Road heading angle
    _theta = atan2(_tan[Y],_tan[X]);

    // Curvature
    _k = curvature(_dr,_d2r,_drnorm);
}
#endif
