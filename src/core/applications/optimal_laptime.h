#ifndef OPTIMAL_LAPTIME_H
#define OPTIMAL_LAPTIME_H

#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "lion/foundation/types.h"
#include "src/core/vehicles/track_by_arcs.h"
#include "src/core/vehicles/road_curvilinear.h"
#include "src/core/foundation/fastest_lap_exception.h"

template<typename Dynamic_model_t>
class Optimal_laptime
{
 public:

    //! This class will only support vehicles with automatic differentiation
    using Timeseries_t = typename Dynamic_model_t::Timeseries_type;
    static_assert(std::is_same<Timeseries_t,CppAD::AD<scalar>>::value == true);

    struct Options
    {
        struct Integral_quantity_conf
        {
            std::string name;
            scalar lower_bound;
            scalar upper_bound;
        };

        size_t print_level                = 0;
        scalar sigma                      = 0.5;     // 0: explicit euler, 0.5: crank-nicolson, 1.0: implicit euler
        size_t maximum_iterations         = 3000;
        bool   throw_if_fail              = true;
        bool   check_optimality           = false;
        bool   retape                     = false;
        scalar nlp_tolerance              = 1.0e-10;
        scalar constraints_viol_tolerance = 1.0e-10;
        scalar acceptable_tolerance       = 1.0e-8;
        std::vector<Integral_quantity_conf> integral_quantities = {};
    };

    //! Helper classes to encapsulate control variables ---------------------------------------------:-

    //! Types of optimization for the control variables:
    //!     (1) Do not optimize: keep constant from its value given initially
    //!     (2) Constant: optimize using a fixed value along the mesh
    //!     (3) Hypermesh: optimize the control variable using an alternative mesh
    //!     (4) Full mesh: optimize the control variable along the full mesh
    enum Optimal_control_type { DONT_OPTIMIZE, CONSTANT, HYPERMESH, FULL_MESH };

    //! Control variable class: contains the definition of the control variables and its type of optimization
    //! [param] T: scalar for ctor and return values (well, for any interface), CppAD::AD<scalar> for internal values
    template<typename T = scalar>
    struct Control_variable
    {
        Optimal_control_type optimal_control_type; //! Type of the optimization
        std::vector<scalar> s_hypermesh;           //! Provide here the hypermesh. Only non-empty for hypermesh optimization
        std::vector<T>  controls;                  //! Control value(s)
        std::vector<T>  dcontrols_dt;              //! Control values time derivative: only non-empty for full_mesh optimization
        scalar          dissipation;               //! The dissipation associated to this control variable

        //! Transform the set of control variables to cppad
        template<typename U = T>
        std::enable_if_t<std::is_same_v<U,scalar>,Control_variable<CppAD::AD<scalar>>> to_CppAD() const;

        static size_t get_hypermesh_position_for_s(const std::vector<scalar>& s_hypermesh, scalar s);

        const T& get_hypermesh_control_value_for_s(const scalar s) const;

        std::pair<const T&,const T&> get_hypermesh_control_value_and_derivative_for_s(const scalar s) const;

        //! Set to zero the values for optimizable control variables
        Control_variable& clear();
    };


    //! Control variables class: provide a container for the control variables + its statistics and 
    template<typename T = scalar>
    struct Control_variables : public std::array<Control_variable<T>,Dynamic_model_t::NCONTROL>
    {
        //! Store the base_type
        using base_type = std::array<Control_variable<T>,Dynamic_model_t::NCONTROL>;

        //! Transform the set of control variables to cppad
        template<typename U = T>
        std::enable_if_t<std::is_same_v<U,scalar>,Control_variables<CppAD::AD<scalar>>> to_CppAD() const;

        //! Clear
        Control_variables& clear();

        std::array<T,Dynamic_model_t::NCONTROL> control_array_at_s(const Dynamic_model_t& car, const size_t i_fullmesh, const scalar s) const;

        std::pair<std::array<T,Dynamic_model_t::NCONTROL>,std::array<T,Dynamic_model_t::NCONTROL>>
            control_array_and_derivative_at_s(const Dynamic_model_t& car, const size_t i_fullmesh, const scalar s) const;

        Control_variables& check();

        //! Data members -------------------------------------:-
        size_t number_of_constant_optimizations;            //! Number of control variables with constant optimization
        size_t number_of_hypermesh_optimization_points;     //! Total number of points used in hypermesh optimizations
        size_t number_of_full_optimizations;                //! Total number of full optimizations

     private:
    
        //! Checks the consistency of the sizes of the input vectors depending on the optimization case
        void check_inputs();
    
        //! Computes the integer data members
        void compute_statistics();
    };

    template<bool is_direct, typename T>
    constexpr static const size_t n_variables_per_point(const Control_variables<T>& control_variables)
    {
        return Dynamic_model_t::NSTATE - 1 + Dynamic_model_t::NALGEBRAIC + (is_direct ? 1 : 2)*control_variables.number_of_full_optimizations;
    }

    template<bool is_direct, typename T>
    constexpr static const size_t n_constraints_per_element(const Control_variables<T>& control_variables)
    {
        return  Dynamic_model_t::NSTATE - 1 + Dynamic_model_t::NALGEBRAIC + Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS
            + (is_direct ? 0 : control_variables.number_of_full_optimizations);
    }

    //! A """""factory""""" for control variables
    static Control_variable<> create_dont_optimize() 
    { return Control_variable<>{ .optimal_control_type{DONT_OPTIMIZE}, .s_hypermesh{}, .controls{}, .dcontrols_dt{}, .dissipation{} }; }

    static Control_variable<> create_constant(const scalar controls)
    { return Control_variable<>{ .optimal_control_type{CONSTANT}, .s_hypermesh{}, .controls{controls}, .dcontrols_dt{}, .dissipation{} }; }

    static Control_variable<> create_hypermesh(const std::vector<scalar>& s_hypermesh, const std::vector<scalar>& controls)
    { return Control_variable<>{ .optimal_control_type{HYPERMESH}, .s_hypermesh{s_hypermesh}, .controls{controls}, .dcontrols_dt{}, .dissipation{} }; }

    static Control_variable<> create_full_mesh(const std::vector<scalar>& controls, const scalar dissipation)
    { return Control_variable<>{ .optimal_control_type{FULL_MESH}, .s_hypermesh{}, .controls{controls}, .dcontrols_dt{}, .dissipation{dissipation} }; }

    static Control_variable<> create_full_mesh(const std::vector<scalar>& controls, const std::vector<scalar>& dcontrols_dt, const scalar dissipation)
    { return Control_variable<>{ .optimal_control_type{FULL_MESH}, .s_hypermesh{}, .controls{controls}, .dcontrols_dt{dcontrols_dt}, .dissipation{dissipation} }; }

    //! Default constructor
    Optimal_laptime() = default;

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
                    const std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>& input_states_start, 
                    const std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>& algebraic_states_start,
                    Control_variables<> controls_start, 
                    const Options opts);

    //! Warm-start constructor
    Optimal_laptime(const std::vector<scalar>& s,
                    const bool is_closed,
                    const bool is_direct,
                    const Dynamic_model_t& car, 
                    const std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>& input_states_start, 
                    const std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>& algebraic_states_start,
                    Control_variables<> controls_start, 
                    const std::vector<scalar>& zl,
                    const std::vector<scalar>& zu,
                    const std::vector<scalar>& lambda,
                    const Options opts);

    Optimal_laptime(Xml_document& doc);

    void compute(const Dynamic_model_t& car);

    template<bool isClosed>
    void compute_direct(const Dynamic_model_t& car);

    template<bool isClosed>
    void compute_derivative(const Dynamic_model_t& car);

    //! Export to XML
    std::unique_ptr<Xml_document> xml() const;

    Options options;
    
    struct Integral_quantity
    {
        std::string name;
        scalar      value;
        bool        restrict;
        scalar      lower_bound;
        scalar      upper_bound;
    };
    
    struct Integral_quantities : public std::array<Integral_quantity,Dynamic_model_t::Integral_quantities::N_INTEGRAL_QUANTITIES>
    {
        using base_type = std::array<Integral_quantity,Dynamic_model_t::Integral_quantities::N_INTEGRAL_QUANTITIES>;
        static constexpr const size_t N = Dynamic_model_t::Integral_quantities::N_INTEGRAL_QUANTITIES;
    
        size_t get_n_restricted() const { return std::count_if(base_type::cbegin(), base_type::cend(), [](const auto& t) -> auto { return t.restrict; }); }

        template<typename Timeseries_t>
        std::vector<Timeseries_t> get_restricted_quantities(
            const std::array<Timeseries_t,Dynamic_model_t::Integral_quantities::N_INTEGRAL_QUANTITIES>& full_values) const
        {
            std::vector<Timeseries_t> values(get_n_restricted());
    
            std::copy_if(full_values.cbegin(), full_values.cend(), values.begin(), 
                         [this,i=0] (const auto& v) mutable -> auto { return (*this)[i++].restrict; });

            return values;
        }
    } integral_quantities;

    // Outputs
    bool success;
    bool is_closed;
    bool is_direct;
    bool warm_start;
    size_t n_elements;
    size_t n_points;
    std::vector<scalar> s;                                                         //! Arclengths
    std::vector<std::array<scalar,Dynamic_model_t::NSTATE>> input_states;          //! All state vectors
    std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>> algebraic_states;  //! All algebraic variables vectors
    Control_variables<> controls;                                                  //! All control variables pre and post optimization
    std::vector<scalar> x_coord;                                                   //! x-coordinate of the trajectory
    std::vector<scalar> y_coord;                                                   //! y-coordinate of the trajectory
    std::vector<scalar> psi;                                                       //! heading angle

    size_t iter_count;  //! Number of iterations spent in IPOPT

    std::vector<std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>>     dinput_states_dp;
    std::vector<std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>> dalgebraic_states_dp;
    std::vector<Control_variables<>>                                         dcontrols_dp;
    std::vector<std::vector<scalar>>                                         dx_dp;
    std::vector<scalar>                                                      dlaptime_dp;

    struct 
    {
        std::vector<scalar> x;
        std::vector<scalar> x_lb;
        std::vector<scalar> x_ub;
        std::vector<scalar> c_lb;
        std::vector<scalar> c_ub;
        std::vector<scalar> zl;     //! Lagrange multipliers of the variable lower bounds
        std::vector<scalar> zu;     //! Lagrange multipliers of the variable upper bounds
        std::vector<scalar> lambda; //! Lagrange multipliers of the optimization constraints
        std::vector<scalar> s;
        std::vector<scalar> vl;
        std::vector<scalar> vu;
    } optimization_data;            //! Auxiliary class to store the optimization data

    scalar laptime;

 private:
    
    void check_inputs(const Dynamic_model_t& car);

    struct Export_solution
    {
        std::vector<std::array<scalar,Dynamic_model_t::NSTATE>> input_states;
        std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>> algebraic_states;
        Control_variables<> controls;
        Integral_quantities integral_quantities;
    };

    template<typename FG_t>
    Export_solution export_solution(FG_t& fg, const std::vector<scalar>& x) const;
    

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
           const std::array<scalar,Dynamic_model_t::NSTATE>& input_states_open_initial_point, 
           const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& algebraic_states_open_initial_point, 
           const std::array<scalar,Dynamic_model_t::NCONTROL>& controls_open_initial_point, 
           const Control_variables<>& controls,
           const Integral_quantities& integral_quantities,
           const scalar sigma 
          ) : _n_elements(n_elements), _n_points(n_points), _car(car), _s(s), _input_states_open_initial_point(input_states_open_initial_point), 
              _algebraic_states_open_initial_point(algebraic_states_open_initial_point), _controls_open_initial_point(controls_open_initial_point), 
              _integral_quantities(integral_quantities), _sigma(sigma), _n_variables(n_variables),
              _n_constraints(n_constraints), _input_states(n_points,{0.0}), _algebraic_states(n_points), 
              _controls(controls.to_CppAD().clear()), 
              _states(n_points,{0.0}), _dstates_dt(n_points,{0.0}), _algebraic_equations(n_points), _integral_quantities_integrands(n_points), 
              _integral_quantities_values() {}

     public:
        const size_t& get_n_variables() const { return _n_variables; }

        const size_t& get_n_constraints() const { return _n_constraints; }

        const std::vector<std::array<Timeseries_t,Dynamic_model_t::NSTATE>>& get_input_states() const { return _input_states; }

        const std::vector<std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC>>& get_algebraic_states() const { return _algebraic_states; }

        const Control_variables<Timeseries_t>& get_controls() const { return _controls; }

        const std::array<Timeseries_t,Integral_quantities::N>& get_integral_quantities_values() const { return _integral_quantities_values; }

        Dynamic_model_t& get_car() { return _car; }

     protected:
        size_t _n_elements;                 //! [c] Number of discretization elements
        size_t _n_points;                   //! [c] Number of discretization points
        Dynamic_model_t _car;               //! Vehicle

        std::vector<scalar> _s;                                                              //! [c] Vector of arclengths
        std::array<scalar,Dynamic_model_t::NSTATE> _input_states_open_initial_point;         //! [c] State vector for the initial node
        std::array<scalar,Dynamic_model_t::NALGEBRAIC> _algebraic_states_open_initial_point; //! [c] Algebraic state vector for the initial node
        std::array<scalar,Dynamic_model_t::NCONTROL> _controls_open_initial_point;           //! [c] Control vector for the initial node
        Integral_quantities _integral_quantities;
        
        scalar _sigma;

        size_t _n_variables;                                                                 //! [c] Number of total variables (NSTATE+NCONTROL-1).(n-1)
        size_t _n_constraints;                                                               //! [c] Number of total constraints (NSTATE-1).(n-1)
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NSTATE>> _input_states;         //! All input state vectors
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC>> _algebraic_states; //! All algebraic state vectors
        Control_variables<Timeseries_t> _controls;            

        std::vector<std::array<Timeseries_t,Dynamic_model_t::NSTATE>> _states;                  //! All state vectors
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NSTATE>> _dstates_dt;              //! All state derivative vectors
        std::vector<std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC>> _algebraic_equations; //! All algebraic equations vectors

        std::vector<std::array<Timeseries_t,Integral_quantities::N>> _integral_quantities_integrands;
        std::array<Timeseries_t,Integral_quantities::N>              _integral_quantities_values;
    };


    //! Auxiliary class to compute the fitness function and constraints with direct controls
    template<bool is_closed>
    class FG_direct : public FG
    {
     public:
        using base_type = FG;
        using ADvector = typename FG::ADvector;

        FG_direct(const size_t n_elements, 
                  const size_t n_points,
                  const Dynamic_model_t& car, 
                  const std::vector<scalar>& s,
                  const std::array<scalar,Dynamic_model_t::NSTATE>& input_states_open_initial_point, 
                  const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& algebraic_states_open_initial_point, 
                  const std::array<scalar,Dynamic_model_t::NCONTROL>& controls_open_initial_point,
                  const Control_variables<>& controls,
                  const Integral_quantities& integral_quantities,
                  const scalar sigma
          ) : FG(n_elements, 
                 n_points,
                 n_elements*n_variables_per_point<true>(controls) 
                    + controls.number_of_constant_optimizations 
                    + controls.number_of_hypermesh_optimization_points,
                 n_elements*n_constraints_per_element<true>(controls)  
                    + integral_quantities.get_n_restricted(),
                 car, s, input_states_open_initial_point, algebraic_states_open_initial_point, controls_open_initial_point, 
                 controls, integral_quantities, sigma) {}

        void operator()(ADvector& fg, const ADvector& x, const ADvector& p) 
        { 
            FG::_car.set_all_parameters(p);

            (*this)(fg, x);
        }
    
        void operator()(ADvector& fg, const ADvector& x)
        {
            if ( base_type::_integral_quantities.get_n_restricted() > 0 )
                compute<true>(fg, x);
            else
                compute<false>(fg, x);
        }

        template<bool compute_integrated_quantities>
        void compute(ADvector& fg, const ADvector& x);
    };


    //! Auxiliary class to compute the fitness function and constraints using derivative controls
    template<bool is_closed>
    class FG_derivative : public FG
    {
     public:
        using base_type = FG;
        using ADvector = typename FG::ADvector;

        FG_derivative(const size_t n_elements, 
                      const size_t n_points,
                      const Dynamic_model_t& car, 
                      const std::vector<scalar>& s,
                      const std::array<scalar,Dynamic_model_t::NSTATE>& input_states_open_initial_point, 
                      const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& algebraic_states_open_initial_point, 
                      const std::array<scalar,Dynamic_model_t::NCONTROL>& controls_open_initial_point,
                      const std::array<scalar,Dynamic_model_t::NCONTROL>& dcontrols_dt_open_initial_point,
                      const Control_variables<>& controls,
                      const Integral_quantities& integral_quantities,
                      const scalar sigma
          ) : FG(n_elements, 
                 n_points,
                 n_elements*n_variables_per_point<false>(controls) 
                    + controls.number_of_constant_optimizations 
                    + controls.number_of_hypermesh_optimization_points,
                 n_elements*n_constraints_per_element<false>(controls) 
                    + integral_quantities.get_n_restricted(),
                 car, s, input_states_open_initial_point,algebraic_states_open_initial_point, controls_open_initial_point, 
                 controls, integral_quantities, sigma), _dcontrols_dt_open_initial_point(dcontrols_dt_open_initial_point) {}

        void operator()(ADvector& fg, const ADvector& x, const ADvector& p) 
        { 
            FG::_car.set_all_parameters(p);

            (*this)(fg, x);
        }

        void operator()(ADvector& fg, const ADvector& x)
        {
            if ( base_type::_integral_quantities.get_n_restricted() > 0 )
                compute<true>(fg, x);
            else
                compute<false>(fg, x);
        }

        template<bool compute_integrated_quantities>
        void compute(ADvector& fg, const ADvector& x);

     private:
        const std::array<scalar,Dynamic_model_t::NCONTROL> _dcontrols_dt_open_initial_point;
    };

};

#include "optimal_laptime.hpp"

#endif
