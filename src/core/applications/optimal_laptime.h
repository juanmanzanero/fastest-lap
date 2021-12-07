#ifndef __OPTIMAL_LAPTIME_H__
#define __OPTIMAL_LAPTIME_H__

#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "lion/foundation/types.h"
#include "src/core/vehicles/track_by_arcs.h"
#include "src/core/vehicles/road_curvilinear.h"

template<typename Dynamic_model_t>
class Optimal_laptime
{
 public:
    //! This class will only support vehicles with automatic differentiation
    using Timeseries_t = typename Dynamic_model_t::Timeseries_type;
    static_assert(std::is_same<Timeseries_t,CppAD::AD<scalar>>::value == true);

    //! Constructor
    //! @param[in] n:     number of discretization points
    //! @param[in] car:   vehicle
    //! @param[in] q0:    initial condition (state at the first point)
    Optimal_laptime(const size_t n, 
                    const bool is_closed,
                    const bool is_direct,
                    const Dynamic_model_t& car, 
                    const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
                    const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, 
                    const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations);

    template<bool is_closed>
    void compute_direct(const size_t n, 
            const Dynamic_model_t& car, 
            const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
            const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, 
            const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations);

    template<bool is_closed>
    void compute_derivative(const size_t n, 
            const Dynamic_model_t& car, 
            const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
            const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, 
            const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations);


    // Outputs
    std::vector<scalar> s;                                           //! Arclengths
    std::vector<std::array<scalar,Dynamic_model_t::NSTATE>> q;       //! All state vectors
    std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>> u;     //! All control vectors
    double laptime;

 private:

    //! Auxiliary class to hold data structures to compute the fitness function and constraints
    class FG
    {
     public:
        using ADvector = std::vector<CppAD::AD<scalar>>;

     protected:
        FG(const size_t n, 
           const size_t n_variables,
           const size_t n_constraints,
           const size_t n_points,
           const Dynamic_model_t& car, 
           const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
           const std::array<scalar,Dynamic_model_t::NCONTROL>& u0,
           const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations
          ) : _n(n), _car(car), _q0(q0), _u0(u0), _dissipations(dissipations), _n_variables(n_variables),
              _n_constraints(n_constraints), _q(n_points,{0.0}), _u(n_points,{0.0}), _dqdt(n_points,{0.0}) {}

     public:
        const size_t& get_n_variables() const { return _n_variables; }

        const size_t& get_n_constraints() const { return _n_constraints; }

        const std::vector<std::array<Timeseries_t,Dynamic_model_t::NSTATE>>& get_states() const { return _q; }

        const std::vector<std::array<Timeseries_t,Dynamic_model_t::NCONTROL>>& get_controls() const { return _u; }

        const std::array<Timeseries_t,Dynamic_model_t::NSTATE>& get_state(const size_t i) const { return _q[i]; }

        const std::array<Timeseries_t,Dynamic_model_t::NCONTROL>& get_control(const size_t i) const { return _u[i]; }

        Dynamic_model_t& get_car() { return _car; }

     protected:
        size_t _n;                          //! [c] Number of discretization points
        Dynamic_model_t _car;               //! Vehicle
        std::array<scalar,Dynamic_model_t::NSTATE> _q0;   //! [c] State vector for the initial node
        std::array<scalar,Dynamic_model_t::NCONTROL> _u0; //! [c] Control vector for the initial node
        std::array<scalar,Dynamic_model_t::NCONTROL> _dissipations;

        size_t _n_variables;        //! [c] Number of total variables (NSTATE+NCONTROL-1).(n-1)
        size_t _n_constraints;      //! [c] Number of total constraints (NSTATE-1).(n-1)
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NSTATE>> _q;       //! All state vectors
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NCONTROL>> _u;     //! All control vectors
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NSTATE>> _dqdt;    //! All state derivative vectors
    };


    //! Auxiliary class to compute the fitness function and constraints with direct controls
    template<bool is_closed>
    class FG_direct : public FG
    {
     public:
        using ADvector = typename FG::ADvector;

        FG_direct(const size_t n, 
           const Dynamic_model_t& car, 
           const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
           const std::array<scalar,Dynamic_model_t::NCONTROL>& u0,
           const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations
          ) : FG(n,n*(Dynamic_model_t::NCONTROL + Dynamic_model_t::NSTATE-1),n*(Dynamic_model_t::NSTATE-1+6), 
                 (is_closed ? n : n+1), car, q0, u0, dissipations) {}

        void operator()(ADvector& fg, const ADvector& x);
    };


    //! Auxiliary class to compute the fitness function and constraints using derivative controls
    template<bool is_closed>
    class FG_derivative : public FG
    {
     public:
        using ADvector = typename FG::ADvector;

        FG_derivative(const size_t n, 
           const Dynamic_model_t& car, 
           const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
           const std::array<scalar,Dynamic_model_t::NCONTROL>& u0,
           const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations
          ) : FG(n,n*(Dynamic_model_t::NSTATE-1+2*Dynamic_model_t::NCONTROL),n*(Dynamic_model_t::NSTATE-1+6)+n*Dynamic_model_t::NCONTROL, 
                 (is_closed ? n : n+1), car, q0, u0, dissipations), _dudt((is_closed ? n : n+1),{0.0}) {}

        void operator()(ADvector& fg, const ADvector& x);
     private:
        
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NCONTROL>> _dudt;
    };

};

#include "optimal_laptime.hpp"

#endif
