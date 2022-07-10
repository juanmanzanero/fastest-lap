#ifndef __ROAD_CURVILINEAR_HPP__
#define __ROAD_CURVILINEAR_HPP__

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
void Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::get_state_derivative(std::array<Timeseries_t,N>& dqdt) const
{
    // dtimedt
    dqdt[IIDTIME] = _dtime;

    // dndtime
    dqdt[IIDN] = _dn;

    // dalphadtime
    dqdt[IIDALPHA] = _dalpha;
}


template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::set_state_and_controls(const scalar t, const std::array<Timeseries_t,NSTATE>& q, const std::array<Timeseries_t,NCONTROL>& u)
{
    update_track(t);

    // time
    _time = q[ITIME];

    // n 
    _n = q[IN];

    // alpha
    _alpha = q[IALPHA];

    // Compute x,y and psi from the track
    
    // Frenet frame (tan,nor,bi)
    base_type::_x   = _r[X] + _n*_nor[X];
    base_type::_y   = _r[Y] + _n*_nor[Y];
    base_type::_psi = _alpha + _theta;
}

template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::set_state_and_control_upper_lower_and_default_values
    (std::array<scalar, NSTATE>& q_def     , std::array<scalar, NSTATE>& q_lb     , std::array<scalar, NSTATE>& q_ub     ,
    std::array<scalar , NCONTROL>& u_def   , std::array<scalar, NCONTROL>& u_lb   , std::array<scalar, NCONTROL>& u_ub) const
{
    // time
    q_def[ITIME] = 0.0;
    q_lb[ITIME]  = 0.0;
    q_ub[ITIME]  = std::numeric_limits<scalar>::max();

    // n
    q_def[IN] = 0.0;
    q_lb[IN]  = std::numeric_limits<scalar>::lowest();
    q_ub[IN]  = std::numeric_limits<scalar>::max();

    // alpha
    q_def[IALPHA] = 0.0;
    q_lb[IALPHA]  = -30.0*DEG;
    q_ub[IALPHA]  =  30.0*DEG;
}


template<typename Timeseries_t,typename Track_t,size_t STATE0, size_t CONTROL0>
template<size_t NSTATE, size_t NCONTROL>
void Road_curvilinear<Timeseries_t,Track_t,STATE0,CONTROL0>::set_state_and_control_names(std::string& key_name, std::array<std::string,NSTATE>& q, std::array<std::string,NCONTROL>& u) 
{
    key_name = "road.arclength";

    // time
    q[ITIME]  = "time";

    // n
    q[IN]     = "road.lateral-displacement";

    // alpha
    q[IALPHA] = "road.track-heading-angle";
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
