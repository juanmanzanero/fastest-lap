#ifndef __TRACK_RUN_HPP__
#define __TRACK_RUN_HPP__

#include "lion/math/matrix_extensions.h"
#include "lion/propagators/explicit_euler.h"
#include <math.h>
#include "src/core/foundation/fastest_lap_exception.h"

template<class DynamicModel_t>
inline Track_run<DynamicModel_t>::Track_run(const DynamicModel_t& vehicle, const std::array<typename DynamicModel_t::Timeseries_type,DynamicModel_t::NSTATE>& q0, 
        size_t n_control_variables)
: _vehicle(vehicle), 
  _q0(q0),
  _controls(),
  _x(),
  _n_control_variables(n_control_variables), 
  _n_blocks(_n_control_variables,0),
  _lengths(_n_control_variables),
  _u0(_n_control_variables),
  _max_u(_n_control_variables,0.0),
  _min_u(_n_control_variables,0.0)
{}


template<class DynamicModel_t>
inline std::pair<std::vector<typename DynamicModel_t::Timeseries_type>,std::vector<std::array<typename DynamicModel_t::Timeseries_type,DynamicModel_t::NSTATE>>> Track_run<DynamicModel_t>::simulate_and_return(const std::vector<typename DynamicModel_t::Timeseries_type>& x, const double t_start, const double t_final, const double dt, bool write)
{
    _x = x;

    set_control_points(_x);
    construct_controls(t_start, t_final);

    ODE45<DynamicModel_t,Polynomial_array<scalar,DynamicModel_t::NCONTROL>,DynamicModel_t::NSTATE>::set("max h",0.02);
    ODE45<DynamicModel_t,Polynomial_array<scalar,DynamicModel_t::NCONTROL>,DynamicModel_t::NSTATE>::set("relative error",1.0e-8);
    ODE45<DynamicModel_t,Polynomial_array<scalar,DynamicModel_t::NCONTROL>,DynamicModel_t::NSTATE>::set("absolute error",1.0e-6);
    scalar t = t_start;

    bool t_end_reached = false;
    double dti = 1.0;

    std::vector<std::array<Timeseries_t,DynamicModel_t::NSTATE>> qout = {_q0};
    std::vector<Timeseries_t> tout = {t};
    std::array<Timeseries_t,DynamicModel_t::NSTATE> q = _q0;

    while (!t_end_reached)
    {
        ODE45<DynamicModel_t,Polynomial_array<scalar,DynamicModel_t::NCONTROL>,DynamicModel_t::NSTATE>::take_step(_vehicle, _controls, q, t, dti, t_final, t_end_reached);
        qout.push_back(q);
        tout.push_back(t);
    }

    return {tout,qout};
}



template<class DynamicModel_t>
inline typename DynamicModel_t::Timeseries_type Track_run<DynamicModel_t>::operator()(const std::vector<typename DynamicModel_t::Timeseries_type>& x, const double t_start, const double t_final, const double dt, bool write)
{
    _x = x;

    set_control_points(_x);
    construct_controls(t_start, t_final);

    return simulate(t_start, t_final, dt, write);
}


template<class DynamicModel_t>
inline typename DynamicModel_t::Timeseries_type Track_run<DynamicModel_t>::simulate(const double t_start, const double t_final, const double dt, bool write)
{
    ODE45<DynamicModel_t,Polynomial_array<scalar,DynamicModel_t::NCONTROL>,DynamicModel_t::NSTATE>::set("max h",0.01);
    ODE45<DynamicModel_t,Polynomial_array<scalar,DynamicModel_t::NCONTROL>,DynamicModel_t::NSTATE>::set("relative error",1.0e-8);
    ODE45<DynamicModel_t,Polynomial_array<scalar,DynamicModel_t::NCONTROL>,DynamicModel_t::NSTATE>::set("absolute error",1.0e-6);
    std::array<Timeseries_t,DynamicModel_t::NSTATE> q(_q0);
    scalar t = t_start;

    std::ofstream file;
    if ( write ) 
        file.open("best_simulation.dat");

    bool t_end_reached = false;
//  double dti = ODE45<DynamicModel_t,Polynomial_array<scalar,DynamicModel_t::NCONTROL>,DynamicModel_t::NSTATE>::initial_dt_estimation(_vehicle,_controls, q, t_start, t_final);
    double dti = 1.0;

    while (!t_end_reached)
        ODE45<DynamicModel_t,Polynomial_array<scalar,DynamicModel_t::NCONTROL>,DynamicModel_t::NSTATE>::take_step(_vehicle, _controls, q, t, dti, t_final, t_end_reached);

    return q[DynamicModel_t::Road_type::IX] - std::pow(_x[10]-_x[11],2)/1.0e1;
}


template<class DynamicModel_t>
constexpr void Track_run<DynamicModel_t>::set_number_of_blocks(size_t i, size_t n_blocks) 
{ 
    _n_blocks.at(i) = n_blocks; 
    _lengths.at(i) = std::vector<Timeseries_t>(n_blocks,1.0);
    _u0.at(i) = std::vector<std::vector<Timeseries_t>>(n_blocks, std::vector<Timeseries_t>(_p+1,0.0));
} 


template<class DynamicModel_t>
constexpr void Track_run<DynamicModel_t>::set_polynomial_order(size_t p) 
{ 
    _p = p; 
    for (size_t i = 0; i < _n_control_variables; ++i)
        _u0.at(i) = std::vector<std::vector<Timeseries_t>>(_n_blocks.at(i), std::vector<Timeseries_t>(_p+1,0.0));
} 

template<class DynamicModel_t>
void Track_run<DynamicModel_t>::set_control_points(const std::vector<typename DynamicModel_t::Timeseries_type>& x)
{
    if ( x.size() != number_total_variables() ) 
        throw fastest_lap_exception("Incorrect size of input vector. Input = " + std::to_string(x.size()) + " | Expected = " + std::to_string(number_total_variables()));

    auto ix = x.cbegin();

    for (size_t i = 0; i < _n_control_variables; ++i)
    {
        if ( _n_blocks[i] == 0 ) break; 

//      for (auto il = _lengths.at(i).begin(); il != _lengths.at(i).end(); ++il)
//      {
//          *il = *ix;
//          ++ix;
//      }

        for (size_t n = 0; n < _n_blocks.at(i); ++n)
        {
            for (auto iu0 = _u0.at(i).at(n).begin(); iu0 != _u0.at(i).at(n).end(); ++iu0)
            {
                *iu0 = *ix;
                ++ix;
            }
            // One step back such that we have continuity
//          --ix;
        }
//      ++ix;
    }

    if (ix != x.cend())
        throw fastest_lap_exception("Something went wrong");
}


template<class DynamicModel_t>
void inline Track_run<DynamicModel_t>::construct_controls(const double t_start, const double t_end)
{
    for (size_t i = 0; i < _n_control_variables; ++i)
    {
//      L must be scaled such that it sums the track_length
        const scalar L = std::accumulate(_lengths.at(i).cbegin(), _lengths.at(i).cend(), 0.0);
        const scalar factor = (t_end - t_start) / L;

        for (auto il = _lengths.at(i).begin(); il != _lengths.at(i).end(); ++il)
            *il *= factor;

        _controls.at(i) = sPolynomial(t_start, _lengths.at(i), _u0.at(i));
    }
}


template<class DynamicModel_t>
constexpr inline size_t Track_run<DynamicModel_t>::number_total_variables() const 
{
    size_t n_vars = 0;

    for (size_t i = 0; i < _n_control_variables; ++i)
    {
        if ( _n_blocks[i] == 0 ) break; 

        // First the block length are stored
        //n_vars += _n_blocks[i];

        // Then the polynomial data (p points instead of p+1. The last one enforces continuity accross blocks)
        n_vars += (_p+1)*_n_blocks[i];
    }

    return n_vars;
}


template<class DynamicModel_t>
inline std::vector<scalar> Track_run<DynamicModel_t>::get_maximum_x() const
{
    std::vector<scalar> result;
    for (size_t i = 0; i < _n_control_variables; ++i)
    {
        if ( _n_blocks[i] == 0 ) break;

        //std::vector<scalar> Lmax(_n_blocks[i], 1.0);
        std::vector<scalar> umax(_p*_n_blocks[i]+1, _max_u[i]); 

        //result.insert(result.end(), Lmax.begin(), Lmax.end());
        result.insert(result.end(), umax.begin(), umax.end());
    }

    return result;
}


template<class DynamicModel_t>
inline std::vector<scalar> Track_run<DynamicModel_t>::get_minimum_x() const
{
    std::vector<scalar> result;
    for (size_t i = 0; i < _n_control_variables; ++i)
    {
        if ( _n_blocks[i] == 0 ) break;

//      std::vector<scalar> Lmin(_n_blocks[i], 0.1);
        std::vector<scalar> umin(_p*_n_blocks[i]+1, _min_u[i]); 

//      result.insert(result.end(), Lmin.begin(), Lmin.end());
        result.insert(result.end(), umin.begin(), umin.end());
    }

    return result;
}




#endif
