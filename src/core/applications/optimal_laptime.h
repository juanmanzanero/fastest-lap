#ifndef OPTIMAL_LAPTIME_H
#define OPTIMAL_LAPTIME_H

#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "lion/foundation/types.h"
#include "src/core/vehicles/track_by_arcs.h"
#include "src/core/vehicles/road_curvilinear.h"
#include "src/core/foundation/fastest_lap_exception.h"
#include "src/core/foundation/model_parameter.h"
#include "src/core/applications/detail/optimal_laptime_data.h"
#include "src/core/applications/detail/optimal_laptime_control_variable.h"
#include "src/core/applications/detail/optimal_laptime_control_variables.h"

template<typename Dynamic_model_t>
class Optimal_laptime : public fastest_lap::optimal_laptime::detail::Optimal_laptime_data<Dynamic_model_t::number_of_inputs, 
                                                                                          Dynamic_model_t::number_of_controls>
{
 public:

    using base_type = fastest_lap::optimal_laptime::detail::Optimal_laptime_data<Dynamic_model_t::number_of_inputs, 
                                                                                 Dynamic_model_t::number_of_controls>;

    //! This class will only support vehicles with automatic differentiation
    using Timeseries_t = typename Dynamic_model_t::Timeseries_type;
    static_assert(std::is_same<Timeseries_t,CppAD::AD<scalar>>::value == true);

    template<typename T = scalar>
    using Control_variables_type = fastest_lap::optimal_laptime::detail::Control_variables<Dynamic_model_t::number_of_controls, T>;

    template<typename T = scalar>
    using Control_variable_type = fastest_lap::optimal_laptime::detail::Control_variable<T>;

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
        std::array<scalar,Dynamic_model_t::number_of_inputs> inputs_lb     = Dynamic_model_t{}.get_state_and_control_upper_lower_and_default_values().inputs_lb;
        std::array<scalar,Dynamic_model_t::number_of_inputs> inputs_ub     = Dynamic_model_t{}.get_state_and_control_upper_lower_and_default_values().inputs_ub;
        std::array<scalar,Dynamic_model_t::number_of_controls> controls_lb = Dynamic_model_t{}.get_state_and_control_upper_lower_and_default_values().controls_lb;
        std::array<scalar,Dynamic_model_t::number_of_controls> controls_ub = Dynamic_model_t{}.get_state_and_control_upper_lower_and_default_values().controls_ub;

        Model_parameters<Timeseries_t> optimization_parameters = {};
    };


    //! A """""factory""""" for control variables
    static Control_variable_type<> create_dont_optimize() 
    { return Control_variable_type<>{ .optimal_control_type = Optimal_control_type::DONT_OPTIMIZE, 
                                      .s_hypermesh{}, .controls{}, .dcontrols_dt{}, .dissipation{} }; 
    }

    static Control_variable_type<> create_constant(const scalar controls)
    { return Control_variable_type<>{ .optimal_control_type = Optimal_control_type::CONSTANT, 
                                      .s_hypermesh{}, .controls = {controls}, .dcontrols_dt{}, .dissipation{} }; 
    }

    static Control_variable_type<> create_hypermesh(const std::vector<scalar>& s_hypermesh, const std::vector<scalar>& controls)
    { return Control_variable_type<>{ .optimal_control_type = Optimal_control_type::HYPERMESH, 
                                      .s_hypermesh = s_hypermesh, .controls = controls, .dcontrols_dt{}, .dissipation{} }; 
    }

    static Control_variable_type<> create_full_mesh(const std::vector<scalar>& controls, const scalar dissipation)
    { return Control_variable_type<>{ .optimal_control_type = Optimal_control_type::FULL_MESH, 
                                      .s_hypermesh{}, .controls = controls, .dcontrols_dt{}, .dissipation = dissipation }; 
    }

    static Control_variable_type<> create_full_mesh(const std::vector<scalar>& controls, const std::vector<scalar>& dcontrols_dt, const scalar dissipation)
    { return Control_variable_type<>{ .optimal_control_type = Optimal_control_type::FULL_MESH, 
                                      .s_hypermesh{}, .controls = controls, .dcontrols_dt = dcontrols_dt, .dissipation = dissipation }; 
    }

    //! Default constructor
    Optimal_laptime() = default;

    //! Constructor with distribution of arclength and initial conditions
    //! If closed simulation, s[0] shall be 0, and s[end] shall be < track_length
    //! @param[in] s: vector of arclengths
    //! @param[in] is_closed: compute closed or open track simulations
    //! @param[in] is_direct: to use direct or derivative controls
    //! @param[in] car:   vehicle
    //! @param[in] q0:    vector of initial conditions 
    //! @param[in] u0:    vector of control variables
    Optimal_laptime(const std::vector<scalar>& s,
                    const bool is_closed,
                    const bool is_direct,
                    const Dynamic_model_t& car, 
                    const std::vector<std::array<scalar,Dynamic_model_t::number_of_inputs>>& inputs_start, 
                    Control_variables_type<> controls_start, 
                    const Options opts);

    //! Warm-start constructor
    Optimal_laptime(const std::vector<scalar>& s,
                    const bool is_closed,
                    const bool is_direct,
                    const Dynamic_model_t& car, 
                    const std::vector<std::array<scalar,Dynamic_model_t::number_of_inputs>>& inputs_start, 
                    Control_variables_type<> controls_start, 
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
    std::unique_ptr<Xml_document> xml() const { return base_type::template xml<Dynamic_model_t>(); }

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


    std::vector<std::vector<std::array<scalar,Dynamic_model_t::number_of_inputs>>>  dinputs_dp;
    std::vector<Control_variables_type<>>                                                dcontrols_dp;
    std::vector<std::vector<scalar>>                                                dx_dp;
    std::vector<scalar>                                                             dlaptime_dp;

 private:
    
    void check_inputs(const Dynamic_model_t& car);

    struct Export_solution
    {
        std::vector<std::array<scalar,Dynamic_model_t::number_of_inputs>> inputs;
        Control_variables_type<> controls;
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
           const std::array<scalar,Dynamic_model_t::number_of_inputs>& inputs_open_initial_point, 
           const std::array<scalar,Dynamic_model_t::number_of_controls>& controls_open_initial_point, 
           const Control_variables_type<>& controls,
           const Integral_quantities& integral_quantities,
           const scalar sigma 
          ) : _n_elements(n_elements), _n_points(n_points), _car(car), _s(s), _inputs_open_initial_point(inputs_open_initial_point), 
              _controls_open_initial_point(controls_open_initial_point), 
              _integral_quantities(integral_quantities), _sigma(sigma), _n_variables(n_variables),
              _n_constraints(n_constraints), _inputs(n_points,{0.0}), 
              _controls(controls.to_CppAD().clear()), 
              _states(n_points,{0.0}), _dstates_dt(n_points,{0.0}), _integral_quantities_integrands(n_points), 
              _integral_quantities_values() {}

     public:
        const size_t& get_n_variables() const { return _n_variables; }

        const size_t& get_n_constraints() const { return _n_constraints; }

        const std::vector<std::array<Timeseries_t,Dynamic_model_t::number_of_inputs>>& get_inputs() const { return _inputs; }

        const Control_variables_type<Timeseries_t>& get_controls() const { return _controls; }

        const std::array<Timeseries_t,Integral_quantities::N>& get_integral_quantities_values() const { return _integral_quantities_values; }

        Dynamic_model_t& get_car() { return _car; }

     protected:
        size_t _n_elements;                 //! [c] Number of discretization elements
        size_t _n_points;                   //! [c] Number of discretization points
        Dynamic_model_t _car;               //! Vehicle

        std::vector<scalar> _s;                                                              //! [c] Vector of arclengths
        std::array<scalar,Dynamic_model_t::number_of_inputs> _inputs_open_initial_point;         //! [c] State vector for the initial node
        std::array<scalar,Dynamic_model_t::number_of_controls> _controls_open_initial_point;           //! [c] Control vector for the initial node
        Integral_quantities _integral_quantities;
        
        scalar _sigma;

        size_t _n_variables;                                                                 //! [c] Number of total variables (number_of_inputs+number_of_controls-1).(n-1)
        size_t _n_constraints;                                                               //! [c] Number of total constraints (number_of_inputs-1).(n-1)
        std::vector<std::array<Timeseries_t,Dynamic_model_t::number_of_inputs>> _inputs;         //! All input state vectors
        Control_variables_type<Timeseries_t> _controls;            

        std::vector<std::array<Timeseries_t,Dynamic_model_t::number_of_states>> _states;                  //! All state vectors
        std::vector<std::array<Timeseries_t,Dynamic_model_t::number_of_states>> _dstates_dt;              //! All state derivative vectors

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
                  const std::array<scalar,Dynamic_model_t::number_of_inputs>& inputs_open_initial_point, 
                  const std::array<scalar,Dynamic_model_t::number_of_controls>& controls_open_initial_point,
                  const Control_variables_type<>& controls,
                  const Integral_quantities& integral_quantities,
                  const scalar sigma
          ) : FG(n_elements, 
                 n_points,
                 n_elements*Optimal_laptime::base_type::template n_variables_per_point<true>(controls) 
                    + controls.number_of_constant_optimizations 
                    + controls.number_of_hypermesh_optimization_points,
                 n_elements*Optimal_laptime::base_type::template n_constraints_per_element<true>(controls, Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS)  
                    + integral_quantities.get_n_restricted(),
                 car, s, inputs_open_initial_point, controls_open_initial_point, 
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
                      const std::array<scalar,Dynamic_model_t::number_of_inputs>& inputs_open_initial_point, 
                      const std::array<scalar,Dynamic_model_t::number_of_controls>& controls_open_initial_point,
                      const std::array<scalar,Dynamic_model_t::number_of_controls>& dcontrols_dt_open_initial_point,
                      const Control_variables_type<>& controls,
                      const Integral_quantities& integral_quantities,
                      const scalar sigma
          ) : FG(n_elements, 
                 n_points,
                 n_elements*Optimal_laptime::base_type::template n_variables_per_point<false>(controls) 
                    + controls.number_of_constant_optimizations 
                    + controls.number_of_hypermesh_optimization_points,
                 n_elements*Optimal_laptime::base_type::template n_constraints_per_element<false>(controls, Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS) 
                    + integral_quantities.get_n_restricted(),
                 car, s, inputs_open_initial_point,controls_open_initial_point, 
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
        const std::array<scalar,Dynamic_model_t::number_of_controls> _dcontrols_dt_open_initial_point;
    };

};

#include "optimal_laptime.hpp"

#endif
