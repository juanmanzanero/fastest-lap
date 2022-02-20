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

    //! Constructor with equally spaced mesh of n points and uniform initial condition (q0,qa0,u0)
    //! @param[in] n:     number of discretization points
    //! @param[in] is_closed: compute closed or open track simulations
    //! @param[in] is_direct: to use direct or derivative controls
    //! @param[in] car:   vehicle
    //! @param[in] q0:    initial condition (+state at the first point)
    //! @param[in] qa0:   initial algebraic condition
    //! @param[in] u0:    initial control variables
    Optimal_laptime(const size_t n, 
                    const bool is_closed,
                    const bool is_direct,
                    const Dynamic_model_t& car, 
                    const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
                    const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& qa0,
                    const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, 
                    const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations);

    //! Constructor with given arclength distribution. 
    //! If closed simulation, s[0] shall be 0, and s[end] shall be < track_length
    //! The number of points, n, will be infered from the length of s
    //! @param[in] s: vector of arclength where the points will be computed
    //! @param[in] is_closed: compute closed or open track simulations
    //! @param[in] is_direct: to use direct or derivative controls
    //! @param[in] car:   vehicle
    //! @param[in] q0:    initial condition (+state at the first point)
    //! @param[in] qa0:   initial algebraic condition
    //! @param[in] u0:    initial control variables
    Optimal_laptime(const std::vector<scalar>& s_,  
                    const bool is_closed,
                    const bool is_direct,
                    const Dynamic_model_t& car, 
                    const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
                    const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& qa0,
                    const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, 
                    const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations);

    //! Constructor with given start and finish arclength, only for open track simulations
    //! @param[in] s_start: initial arclength
    //! @param[in] s_finish: final arclength
    //! @param[in] n: number of elements
    //! @param[in] is_closed: compute closed or open track simulations
    //! @param[in] is_direct: to use direct or derivative controls
    //! @param[in] car:   vehicle
    //! @param[in] q0:    initial condition (+state at the first point)
    //! @param[in] qa0:   initial algebraic condition
    //! @param[in] u0:    initial control variables
    Optimal_laptime(const scalar s_start,
                    const scalar s_finish,
                    const size_t n, 
                    const bool is_direct,
                    const Dynamic_model_t& car, 
                    const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
                    const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& qa0,
                    const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, 
                    const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations);

    //! Constructor with distribution of arclength and initial conditions
    //! If closed simulation, s[0] shall be 0, and s[end] shall be < track_length
    //! @param[in] s: vector of arclengths
    //! @param[in] is_closed: compute closed or open track simulations
    //! @param[in] is_direct: to use direct or derivative controls
    //! @param[in] car:   vehicle
    //! @param[in] q0:    vector of initial conditions 
    //! @param[in] qa0:   vector of algebraic initial conditions
    //! @param[in] u0:    vector of control variables
    Optimal_laptime(const std::vector<scalar>& s,
                    const bool is_closed,
                    const bool is_direct,
                    const Dynamic_model_t& car, 
                    const std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>& q0, 
                    const std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>& qa0,
                    const std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>>& u0, 
                    const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations);

    void compute(const bool is_closed, const bool is_direct, const Dynamic_model_t& car, const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations);

    template<bool is_closed>
    void compute_direct(const Dynamic_model_t& car, 
                        const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations);

    template<bool is_closed>
    void compute_derivative(const Dynamic_model_t& car, 
                            const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations);

    //! Export to XML
    std::unique_ptr<Xml_document> xml() const;

    // Outputs
    size_t n_elements;
    size_t n_points;
    std::vector<scalar> s;                                           //! Arclengths
    std::vector<std::array<scalar,Dynamic_model_t::NSTATE>> q;       //! All state vectors
    std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>> qa;  //! All algebraic variables vectors
    std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>> u;     //! All control vectors

    std::array<std::string,Dynamic_model_t::NSTATE> q_names;
    std::array<std::string,Dynamic_model_t::NCONTROL> u_names;
    double laptime;

 private:

    //! Auxiliary class to hold data structures to compute the fitness function and constraints
    class FG
    {
     public:
        using ADvector = std::vector<CppAD::AD<scalar>>;

     protected:
        FG(const size_t n_elements, 
           const size_t n_points,
           const size_t n_variables,
           const size_t n_constraints,
           const Dynamic_model_t& car, 
           const std::vector<scalar>& s,
           const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
           const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& qa0, 
           const std::array<scalar,Dynamic_model_t::NCONTROL>& u0,
           const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations
          ) : _n_elements(n_elements), _n_points(n_points), _car(car), _s(s), _q0(q0), 
              _qa0(qa0), _u0(u0), _dissipations(dissipations), _n_variables(n_variables),
              _n_constraints(n_constraints), _q(n_points,{0.0}), _qa(n_points), _u(n_points,{0.0}), _dqdt(n_points,{0.0}), _dqa(n_points) {}

     public:
        const size_t& get_n_variables() const { return _n_variables; }

        const size_t& get_n_constraints() const { return _n_constraints; }

        const std::vector<std::array<Timeseries_t,Dynamic_model_t::NSTATE>>& get_states() const { return _q; }

        const std::vector<std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC>>& get_algebraic_states() const { return _qa; }

        const std::vector<std::array<Timeseries_t,Dynamic_model_t::NCONTROL>>& get_controls() const { return _u; }

        const std::array<Timeseries_t,Dynamic_model_t::NSTATE>& get_state(const size_t i) const { return _q[i]; }

        const std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC>& get_algebraic_state(const size_t i) const { return _qa[i]; }

        const std::array<Timeseries_t,Dynamic_model_t::NCONTROL>& get_control(const size_t i) const { return _u[i]; }

        Dynamic_model_t& get_car() { return _car; }

     protected:
        size_t _n_elements;                 //! [c] Number of discretization elements
        size_t _n_points;                   //! [c] Number of discretization points
        Dynamic_model_t _car;               //! Vehicle

        std::vector<scalar> _s;                                                 //! [c] Vector of arclengths
        std::array<scalar,Dynamic_model_t::NSTATE> _q0;        //! [c] State vector for the initial node
        std::array<scalar,Dynamic_model_t::NALGEBRAIC> _qa0;   //! [c] Algebraic state vector for the initial node
        std::array<scalar,Dynamic_model_t::NCONTROL> _u0;      //! [c] Control vector for the initial node
        std::array<scalar,Dynamic_model_t::NCONTROL> _dissipations;

        size_t _n_variables;                                                    //! [c] Number of total variables (NSTATE+NCONTROL-1).(n-1)
        size_t _n_constraints;                                                  //! [c] Number of total constraints (NSTATE-1).(n-1)
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NSTATE>> _q;       //! All state vectors
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC>> _qa;  //! All algebraic state vectors
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NCONTROL>> _u;     //! All control vectors
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NSTATE>> _dqdt;    //! All state derivative vectors
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC>> _dqa; //! All algebraic state derivative vectors
    };


    //! Auxiliary class to compute the fitness function and constraints with direct controls
    template<bool is_closed>
    class FG_direct : public FG
    {
     public:
        using ADvector = typename FG::ADvector;

        FG_direct(const size_t n_elements, 
                  const size_t n_points,
                  const Dynamic_model_t& car, 
                  const std::vector<scalar>& s,
                  const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
                  const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& qa0, 
                  const std::array<scalar,Dynamic_model_t::NCONTROL>& u0,
                  const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations
          ) : FG(n_elements, 
                 n_points,
                 n_elements*(Dynamic_model_t::NCONTROL + Dynamic_model_t::NSTATE + Dynamic_model_t::NALGEBRAIC - 1),
                 n_elements*(Dynamic_model_t::NSTATE + Dynamic_model_t::NALGEBRAIC - 1 + Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS), 
                 car, s, q0, qa0, u0, dissipations) {}

        void operator()(ADvector& fg, const ADvector& x);
    };


    //! Auxiliary class to compute the fitness function and constraints using derivative controls
    template<bool is_closed>
    class FG_derivative : public FG
    {
     public:
        using ADvector = typename FG::ADvector;

        FG_derivative(const size_t n_elements, 
                      const size_t n_points,
                      const Dynamic_model_t& car, 
                      const std::vector<scalar>& s,
                      const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
                      const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& qa0, 
                      const std::array<scalar,Dynamic_model_t::NCONTROL>& u0,
                      const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations
          ) : FG(n_elements, 
                 n_points,
                 n_elements*(Dynamic_model_t::NSTATE+Dynamic_model_t::NALGEBRAIC-1+2*Dynamic_model_t::NCONTROL),
                 n_elements*(Dynamic_model_t::NSTATE+Dynamic_model_t::NALGEBRAIC-1+Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS)
                    +n_elements*Dynamic_model_t::NCONTROL, 
                 car, s, q0, qa0, u0, dissipations), _dudt(n_points,{0.0}) {}

        void operator()(ADvector& fg, const ADvector& x);
     private:
        
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NCONTROL>> _dudt;
    };

};

#include "optimal_laptime.hpp"

#endif
