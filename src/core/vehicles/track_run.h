#ifndef __TRACK_RUN_H__
#define __TRACK_RUN_H__

#include <fstream>
#include "lion/math/polynomial.h"
#include "lion/propagators/ode45.h"

template<class DynamicModel_t>
class Track_run
{
 public:
    using Timeseries_t = typename DynamicModel_t::Timeseries_type;
    Track_run() {};

    Track_run(const DynamicModel_t& vehicle, const std::array<Timeseries_t,DynamicModel_t::NSTATE>& q0, size_t n_control_variables);

    Timeseries_t operator()(const std::vector<Timeseries_t>& x, const double t_start, const double t_final, const double dt, bool write=false);

    std::pair<std::vector<Timeseries_t>,std::vector<std::array<Timeseries_t,DynamicModel_t::NSTATE>>> simulate_and_return(const std::vector<Timeseries_t>& x, const double t_start, const double t_final, const double dt, bool write = false);

    constexpr void set_number_of_blocks(size_t i, size_t n_blocks);

    constexpr void set_polynomial_order(size_t p);

    constexpr size_t number_total_variables() const ;

    std::vector<scalar> get_minimum_x() const;
    
    std::vector<scalar> get_maximum_x() const;

    constexpr std::vector<scalar>& get_max_controls() { return _max_u; } 

    constexpr std::vector<scalar>& get_min_controls() { return _min_u; } 

    DynamicModel_t& get_vehicle() { return _vehicle; }

 private:
    void set_control_points(const std::vector<Timeseries_t>& x);

    void construct_controls(const double t_start, const double t_end); 

    Timeseries_t simulate(const double t_start, const double t_final, const double dt, bool write);

    DynamicModel_t _vehicle;                             //! Vehicle to run
    std::array<Timeseries_t,DynamicModel_t::NSTATE> _q0;   //! Initial condition

    Polynomial_array<scalar,DynamicModel_t::NCONTROL> _controls;  //! Polynomials representing the control variables
    std::vector<Timeseries_t> _x;                              //! Save a copy of the raw inputs. Raw inputs are
                                                             //! constructed as: [L0,L1,...,Ln,y0,y1,...,yPN]
                                                             //! one for each control variable.

    size_t _n_control_variables;   //! Number of control variables
    std::vector<size_t> _n_blocks; //! Number of blocks for each control polynomial
    size_t _p = 10;                //! Polynomial order of each block

    std::vector<std::vector<Timeseries_t>> _lengths;               //! Length of the polynomial blocks
    std::vector<std::vector<std::vector<Timeseries_t>>> _u0; //! Control values at LGL points

    std::vector<scalar> _max_u;
    std::vector<scalar> _min_u;
};

#include "track_run.hpp"

#endif
