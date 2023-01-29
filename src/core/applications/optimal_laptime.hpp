#include "lion/thirdparty/include/cppad/ipopt/solve.hpp"
#include "lion/math/ipopt_cppad_handler.hpp"
#include "lion/math/sensitivity_analysis.h"
#include "lion/thirdparty/include/logger.hpp"


template<typename Dynamic_model_t>
inline Optimal_laptime<Dynamic_model_t>::Optimal_laptime(const std::vector<scalar>& s_, const bool is_closed_, const bool is_direct_,
    const Dynamic_model_t& car, const std::vector<std::array<scalar,Dynamic_model_t::number_of_inputs>>& inputs_start, 
    Control_variables_type<> controls_start, 
    const Options opts)
: options(opts), integral_quantities()
{
    // (1) Initialize optimal laptime data members
    base_type::is_closed  = is_closed_;
    base_type::is_direct  = is_direct_;
    base_type::warm_start = false;
    base_type::s          = s_;
    base_type::inputs     = inputs_start;
    base_type::controls   = controls_start.check();

    // (2) Check inputs
    check_inputs(car);

    // (3) Compute
    compute(car);
}


template<typename Dynamic_model_t>
inline Optimal_laptime<Dynamic_model_t>::Optimal_laptime(const std::vector<scalar>& s_, const bool is_closed_, const bool is_direct_,
    const Dynamic_model_t& car,
    const std::vector<std::array<scalar,Dynamic_model_t::number_of_inputs>>& inputs_start, 
    Control_variables_type<> controls_start, 
    const std::vector<scalar>& zl,
    const std::vector<scalar>& zu,
    const std::vector<scalar>& lambda,
    const Options opts)
: options(opts), integral_quantities()
{
    // (1) Initialize optimal laptime data
    base_type::is_closed         = is_closed_;
    base_type::is_direct         = is_direct_;
    base_type::warm_start        = true;
    base_type::s                 = s_;
    base_type::inputs            = inputs_start;
    base_type::controls          = controls_start.check();
    base_type::optimization_data = {.zl{zl}, .zu{zu}, .lambda{lambda}};

    // (2) Check inputs
    check_inputs(car);
      
    // (3) Compute
    compute(car);
}


template<typename Dynamic_model_t>
inline void Optimal_laptime<Dynamic_model_t>::check_inputs(const Dynamic_model_t& car)
{
    auto& s          = base_type::s;
    auto& n_points   = base_type::n_points;
    auto& n_elements = base_type::n_elements;
    auto& is_closed  = base_type::is_closed;
    auto& is_direct  = base_type::is_direct;
    auto& inputs     = base_type::inputs;
    auto& controls   = base_type::controls;

    if ( s.size() <= 1 )
        throw fastest_lap_exception("Provide at least two values of arclength");

    // (1) Compute number of elements and points
    n_points = s.size();
    n_elements = (is_closed ? n_points : n_points - 1);

    // (2) Verify the vector of arclength
    const scalar& L = car.get_road().track_length();

    if (is_closed)
    {
        if (std::abs(s.front()) > 1.0e-12)
            throw fastest_lap_exception("In closed circuits, s[0] should be 0.0");

        if (s.back() > L - 1.0e-10)
            throw fastest_lap_exception("In closed circuits, s[end] should be < track_length");
    }
    else
    {
        if (s[0] < -1.0e-12)
            throw fastest_lap_exception("s[0] must be >= 0");

        if (s.back() > L)
            throw fastest_lap_exception("s[end] must be <= L");
    }

    // If closed, replace the initial arclength by 0
    if (is_closed)
    {   
        s.front() = 0.0;
    }

    // (3) Set the initial condition

    // (3.1) State
    if ( inputs.size() != n_points )
        throw fastest_lap_exception("q must have size of n_points");

    // (3.3) Controls: check size only for full-mesh controls
    for (const auto& control_variable : controls )
    {
        if ( control_variable.optimal_control_type == Optimal_control_type::FULL_MESH )
        {
            if ( control_variable.controls.size() != n_points )
                throw fastest_lap_exception("controls.controls must have the size of n_points");            

            if ( is_direct )
            {
                if ( control_variable.dcontrols_dt.size() != 0 )
                    throw fastest_lap_exception("In direct simulations, controls.dcontrols_dt must be empty");            
            }
            else
            {   
                if ( control_variable.dcontrols_dt.size() != n_points )
                    throw fastest_lap_exception("controls.dcontrols_dt must have the size of n_points");            
            }
        }
    }
        
    // (4) Set integral constraints

    // (4.1) Initialization
    for (size_t i = 0; i < Integral_quantities::N; ++i)
    {
        integral_quantities[i].name        = Dynamic_model_t::Integral_quantities::names[i];
        integral_quantities[i].restrict    = false;
        integral_quantities[i].value       = 0.0;
        integral_quantities[i].lower_bound = std::numeric_limits<scalar>::lowest();
        integral_quantities[i].upper_bound = std::numeric_limits<scalar>::max();
    }

    // (4.2) Set active constraints
    for (size_t i = 0; i < options.integral_quantities.size(); ++i)
    {
        const auto it = std::find(Dynamic_model_t::Integral_quantities::names.cbegin(), 
                                  Dynamic_model_t::Integral_quantities::names.cend(),
                                  options.integral_quantities[i].name);

        if (it == Dynamic_model_t::Integral_quantities::names.cend())
        {
            std::ostringstream s_out;
            s_out << "[ERROR] Requested integral constraint was not found." << std::endl;
            s_out << "[ERROR] Available options are: " << Dynamic_model_t::Integral_quantities::names;
            throw fastest_lap_exception(s_out.str()); 
        }

        // (4.2) Fill the integral constraint information
        const size_t index = std::distance(Dynamic_model_t::Integral_quantities::names.cbegin(),it);
        integral_quantities[index].restrict    = true;
        integral_quantities[index].lower_bound = options.integral_quantities[i].lower_bound;
        integral_quantities[index].upper_bound = options.integral_quantities[i].upper_bound;
    }
}


template<typename Dynamic_model_t>
inline Optimal_laptime<Dynamic_model_t>::Optimal_laptime(Xml_document& doc)
{
    base_type::template construct_from_xml<Dynamic_model_t>(doc);
}


template<typename Dynamic_model_t>
template<typename FG_t>
inline typename Optimal_laptime<Dynamic_model_t>::Export_solution 
    Optimal_laptime<Dynamic_model_t>::export_solution(FG_t& fg, const std::vector<scalar>& x) const
{
    auto& n_points          = base_type::n_points;
    auto& controls          = base_type::controls;

    // (1) Create output
    Export_solution output;

    // (1.1) Copy the structure of the control variables
    output.controls = controls;
    output.controls.clear();

    // (1.2) Copy the structure of the integral quantities
    output.integral_quantities = integral_quantities;

    // (2) Transform x to CppAD
    std::vector<CppAD::AD<scalar>> x_cppad(x.size(),0.0),fg_eval(fg.get_n_constraints()+1,0.0);
    std::copy(x.cbegin(), x.cend(), x_cppad.begin());
    fg.template compute<true>(fg_eval,x_cppad);
  
    assert(fg.get_inputs().size() == n_points);

    // (3) Export states
    output.inputs = std::vector<std::array<scalar,Dynamic_model_t::number_of_inputs>>(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        for (size_t j = 0; j < Dynamic_model_t::number_of_inputs; ++j)
            output.inputs[i][j] = Value(fg.get_inputs()[i][j]);

    // (5) Export control variables
    for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
    {
        switch (output.controls[j].optimal_control_type) 
        {
         case (Optimal_control_type::CONSTANT):
            std::transform(fg.get_controls()[j].controls.cbegin(), fg.get_controls()[j].controls.cend(), output.controls[j].controls.begin(), 
                [](const auto& u_cppad) -> auto { return Value(u_cppad); });
            break;
        
         case (Optimal_control_type::HYPERMESH):
            std::transform(fg.get_controls()[j].controls.cbegin(), fg.get_controls()[j].controls.cend(), output.controls[j].controls.begin(), 
                [](const auto& u_cppad) -> auto { return Value(u_cppad); });
            break;

         case (Optimal_control_type::FULL_MESH):
            std::transform(fg.get_controls()[j].controls.cbegin(), fg.get_controls()[j].controls.cend(), output.controls[j].controls.begin(), 
                [](const auto& u_cppad) -> auto { return Value(u_cppad); });

            std::transform(fg.get_controls()[j].dcontrols_dt.cbegin(), fg.get_controls()[j].dcontrols_dt.cend(), output.controls[j].dcontrols_dt.begin(), 
                [](const auto& u_cppad) -> auto { return Value(u_cppad); });
            break;

         default:
            // do nothing
            break;
        }
    }

    // (6) Export integral quantities
    for (size_t i = 0; i < Integral_quantities::N; ++i)
        output.integral_quantities[i].value = Value(fg.get_integral_quantities_values()[i]);

    return output;
}


template<typename Dynamic_model_t>
inline void Optimal_laptime<Dynamic_model_t>::compute(const Dynamic_model_t& car)
{
    auto& is_closed         = base_type::is_closed;
    auto& is_direct         = base_type::is_direct;

    // (1) Check that the car is ready
    if ( !car.is_ready() )
    {
        std::ostringstream s_out;
        s_out << "[ERROR] The car model is not ready because not all the parameters have been set" << std::endl;
        car.xml()->print(s_out);
        throw fastest_lap_exception(s_out.str());
    }

    // (2) Compute
    if ( is_direct )
    {
        if ( is_closed )
            compute_direct<true>(car);
        else
            compute_direct<false>(car);
    }
    else
    {
        if ( is_closed )
            compute_derivative<true>(car);
        else
            compute_derivative<false>(car);
    }
}


template<typename Dynamic_model_t>
template<bool isClosed>
inline void Optimal_laptime<Dynamic_model_t>::compute_direct(const Dynamic_model_t& car) 
{
    class Counter
    {
     public:
        Counter() : k(0) {}

        Counter& increment() { ++k; return *this;}
    
        operator size_t() const { return k; }

     private:
        size_t k;
    };

    auto& s                 = base_type::s;
    auto& n_points          = base_type::n_points;
    auto& n_elements        = base_type::n_elements;
    auto& inputs            = base_type::inputs;
    auto& controls          = base_type::controls;
    auto& laptime           = base_type::laptime;
    auto& x_coord           = base_type::x_coord;
    auto& y_coord           = base_type::y_coord;
    auto& psi               = base_type::psi;
    auto& optimization_data = base_type::optimization_data;
    auto& warm_start        = base_type::warm_start;
    auto& success           = base_type::success;
    auto& iter_count        = base_type::iter_count;

    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> start" << std::endl;

    // (1) Get variable bounds and default control variables
    const auto& inputs_lb     = options.inputs_lb;
    const auto& inputs_ub     = options.inputs_ub;
    const auto& controls_lb         = options.controls_lb;
    const auto& controls_ub         = options.controls_ub;

    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> input states lower bound: " << inputs_lb << std::endl;
    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> input states upper bound: " << inputs_ub << std::endl;
    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> controls lower bound: " << controls_lb << std::endl;
    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> controls upper bound: " << controls_ub << std::endl;

    // (1) Construct starting vector of control variables
    auto controls_start = controls.control_array_at_s(car, 0, s.front());

    // (2) Construct fitness function functor
    FG_direct<isClosed> fg(n_elements, n_points, car, s, inputs.front(), controls_start, 
                               controls, integral_quantities, options.sigma);

    // (3) Construct vectors of initial optimization point, and variable upper/lower bounds
    std::vector<scalar> x_start(fg.get_n_variables(),0.0);
    std::vector<scalar> x_lb(fg.get_n_variables(), std::numeric_limits<scalar>::lowest());
    std::vector<scalar> x_ub(fg.get_n_variables(), std::numeric_limits<scalar>::max());

    Counter k;
    constexpr const size_t offset = isClosed ? 0 : 1;

    for (size_t i = offset; i < n_points; ++i)
    {
        // (3.1) Set state and initial condition from start to ITIME
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_names::time; ++j)    
        {
            x_start[k] = inputs[i][j];
            x_lb[k]    = inputs_lb[j];
            x_ub[k]    = inputs_ub[j];
            k.increment();
        }

        // (3.2) Set state to IN. Assert that ITIME = IN - 1
        static_assert(static_cast<size_t>(Dynamic_model_t::Road_type::input_names::time) 
                          == static_cast<size_t>(( Dynamic_model_t::Road_type::input_names::lateral_displacement - 1 )));
        static_assert(static_cast<size_t>(Dynamic_model_t::Road_type::state_names::time) 
                          == static_cast<size_t>(( Dynamic_model_t::Road_type::state_names::lateral_displacement - 1 )));

        x_start[k] = inputs[i][Dynamic_model_t::Road_type::input_names::lateral_displacement];
        x_lb[k]    = -car.get_road().get_left_track_limit(s[i]);
        x_ub[k]    = car.get_road().get_right_track_limit(s[i]);
        k.increment();

        // (3.3) Set state after IN
        for (size_t j = Dynamic_model_t::Road_type::input_names::lateral_displacement+1; j < Dynamic_model_t::number_of_inputs; ++j)    
        {
            x_start[k] = inputs[i][j];
            x_lb[k]    = inputs_lb[j];
            x_ub[k]    = inputs_ub[j];
            k.increment();
        }

        // (3.5) Set full mesh control variables
        for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
        {
            if ( controls[j].optimal_control_type == Optimal_control_type::FULL_MESH )
            {
                x_start[k] = controls[j].controls[i];
                x_lb[k]    = controls_lb[j];
                x_ub[k]    = controls_ub[j];
                k.increment();
            }
        }
    }

    // (3.6) Add the control variables corresponding to constant optimizations and hypermesh optimizations
    for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
    {
        switch (controls[j].optimal_control_type)
        {
         case (Optimal_control_type::CONSTANT):
            for (size_t i = 0; i < controls[j].controls.size(); ++i)
            {
                x_start[k] = controls[j].controls[i];
                x_lb[k]    = controls_lb[j];
                x_ub[k]    = controls_ub[j];
                k.increment();
            } 
            break;

         case (Optimal_control_type::HYPERMESH):
            for (size_t i = 0; i < controls[j].controls.size(); ++i)
            {
                x_start[k] = controls[j].controls[i];
                x_lb[k]    = controls_lb[j];
                x_ub[k]    = controls_ub[j];
                k.increment();
            }    
            break;
         default:
            break;
        }
    }

    // (4) Construct vector of upper/lower bounds for constraints
    std::vector<scalar> c_lb(fg.get_n_constraints(),0.0);
    std::vector<scalar> c_ub(fg.get_n_constraints(),0.0);

    const auto [time_derivative_equations, algebraic_equations] = car.classify_equations();

    Counter kc;
    for (size_t i = 1; i < n_points; ++i)
    {
        // Equality constraints: --------------- 

        // (4.1) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] 
        for (size_t i_equation = 0; i_equation < time_derivative_equations.size(); ++i_equation)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (4.3) Algebraic constraints: dqa^{i} = 0.0
        for (size_t i_equation = 0; i_equation < algebraic_equations.size(); ++i_equation)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }
        
        // (4.4) Extra constraints per point
        auto [c_extra_lb, c_extra_ub] = car.optimal_laptime_extra_constraints_bounds(s[i]); 
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
        {
            c_lb[kc] = c_extra_lb[j];
            c_ub[kc] = c_extra_ub[j];
            kc.increment();
        }
    }

    // (5) Add the periodic element if track is closed
    if constexpr (isClosed)
    {
        // Equality constraints: --------------- 

        // (5.1) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}]
        for (size_t i_equation = 0; i_equation < time_derivative_equations.size(); ++i_equation)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (5.3) Algebraic constraints: dqa^{i} = 0.0
        for (size_t i_equation = 0; i_equation < algebraic_equations.size(); ++i_equation)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }
        
        // (5.4) Extra constraints per point
        auto [c_extra_lb, c_extra_ub] = car.optimal_laptime_extra_constraints_bounds(s[0]); 
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
        {
            c_lb[kc] = c_extra_lb[j];
            c_ub[kc] = c_extra_ub[j];
            kc.increment();
        }
    }

    // (6) Add the integral constraints
    for (const auto& integral_quantity : integral_quantities)
    {
        if (integral_quantity.restrict)
        {
            c_lb[kc] = integral_quantity.lower_bound;
            c_ub[kc] = integral_quantity.upper_bound;
            kc.increment();
        }
    }        

    // (6) Check the dimensions of the counter used to fill the vectors
    if ( k != fg.get_n_variables() )
    {
        std::ostringstream s_out;
        s_out << "k(=" << k << ") != fg.get_n_variables()(=" << fg.get_n_variables() << ")" ;
        throw fastest_lap_exception(s_out.str());
    }
    
    if ( kc != fg.get_n_constraints() )
    {
        std::ostringstream s_out;
        s_out << "kc(=" << kc << ") != fg.get_n_constraints()(=" << fg.get_n_constraints() << ")" ;
        throw fastest_lap_exception(s_out.str());
    }

    // (7) Warm-start: simply check the dimensions
    if ( warm_start )
    {
        if ( optimization_data.zl.size() != fg.get_n_variables() )
        {
            std::ostringstream s_out;
            s_out << "size of zl should be " << fg.get_n_variables() << " but it is " << optimization_data.zl.size();
            throw fastest_lap_exception(s_out.str());
        }

        if ( optimization_data.zu.size() != fg.get_n_variables() )
        {
            std::ostringstream s_out;
            s_out << "size of zu should be " << fg.get_n_variables() << " but it is " << optimization_data.zu.size();
            throw fastest_lap_exception(s_out.str());
        }

        if ( optimization_data.lambda.size() != fg.get_n_constraints() )
        {
            std::ostringstream s_out;
            s_out << "size of lambda should be " << fg.get_n_constraints() << " but it is " << optimization_data.lambda.size();
            throw fastest_lap_exception(s_out.str());
        }
    } 

    // (8) Run optimization

    // (8.1) Prepare options
    std::ostringstream ipoptoptions; ipoptoptions << std::setprecision(17);
    ipoptoptions << "Integer print_level " << options.print_level << std::endl;
    ipoptoptions << "Integer max_iter " << options.maximum_iterations << std::endl;
    ipoptoptions << "String  sb           yes\n";
    ipoptoptions << "Sparse true forward\n";

    if ( options.retape )
    {
        ipoptoptions << "Retape true\n";
    }

    ipoptoptions << "Numeric tol "             << options.nlp_tolerance              << std::endl;
    ipoptoptions << "Numeric constr_viol_tol " << options.constraints_viol_tolerance << std::endl;
    ipoptoptions << "Numeric acceptable_tol "  << options.acceptable_tolerance       << std::endl;

    // (8.2) Return object
    CppAD::ipopt_cppad_result<std::vector<scalar>> result;

    // (8.3) Solve the problem
    if ( !warm_start )
        CppAD::ipopt_cppad_solve<std::vector<scalar>, FG_direct<isClosed>>(ipoptoptions.str(), x_start, x_lb, x_ub, c_lb, c_ub, fg, result);
    else
        CppAD::ipopt_cppad_solve<std::vector<scalar>, FG_direct<isClosed>>(ipoptoptions.str(), x_start, x_lb, x_ub, c_lb, c_ub, 
            optimization_data.lambda, optimization_data.zl, optimization_data.zu, fg, result);
    

    // (8.4) Check success flag
    success = result.status == CppAD::ipopt_cppad_result<std::vector<scalar>>::success; 
    iter_count = result.iter_count;

    if ( !success && options.throw_if_fail )
    {
        throw fastest_lap_exception("Optimization did not succeed");
    }

    // (8.5) Check optimality (disabled by default)
    if ( options.check_optimality )
    {
        auto sensitivity_opts = typename Sensitivity_analysis<FG_direct<isClosed>>::Options{};
        sensitivity_opts.ipopt_bound_relax_factor = 1.0e-8;
        auto sensitivity_analysis = Sensitivity_analysis<FG_direct<isClosed>>(fg, car.get_parameters().get_all_parameters_as_scalar(), 
                                                                              result.x, result.s, result.lambda, result.zl, result.zu, 
                                                                              result.vl, result.vu, x_lb, x_ub, c_lb, c_ub, sensitivity_opts);
        const auto& optimality_check = sensitivity_analysis.optimality_check;
    
        if ( !optimality_check.success ) 
        {
            std::ostringstream s_out;
            s_out << "[ERROR] Requested optimality check for optimal laptime problem has failed" << std::endl;
            s_out << "        Big components of the gradient vector are:" << std::endl;
            for (size_t i = 0; i < optimality_check.id_not_ok.size(); ++i)
                s_out << "            " << optimality_check.id_not_ok[i] << ": " << optimality_check.nlp_error[optimality_check.id_not_ok[i]] << std::endl;
            throw fastest_lap_exception(s_out.str());
        }

        dx_dp = sensitivity_analysis.dxdp;

        // dxdp contains the solution but also the slack variables and the lagrange multipliers, discard everything except the solution (goes first)
        for (auto& dx_dp_i : dx_dp)
            dx_dp_i.resize(fg.get_n_variables());
    }

    // (9) Export the solution

    // (9.1) Export states
    const auto solution_exported = export_solution(fg, result.x);
    inputs        = solution_exported.inputs;
    controls            = solution_exported.controls;
    integral_quantities = solution_exported.integral_quantities;

    // (9.2) Export time, x, y, and psi
    const scalar& L = car.get_road().track_length();

    x_coord = std::vector<scalar>(fg.get_inputs().size());
    y_coord = std::vector<scalar>(fg.get_inputs().size());
    psi = std::vector<scalar>(fg.get_inputs().size());

    // (9.2.1) Compute the first point
    auto controls_i = fg.get_controls().control_array_at_s(car, 0, s.front());
    auto dtimeds_first = fg.get_car()(fg.get_inputs().front(),controls_i,s.front())
                            .dstates_dt[Dynamic_model_t::Road_type::state_names::time];
    auto dtimeds_prev = dtimeds_first;

    x_coord.front() = Value(std::as_const(fg.get_car().get_road()).get_position().x());
    y_coord.front() = Value(std::as_const(fg.get_car().get_road()).get_position().y());
    psi.front() = Value(std::as_const(fg.get_car().get_road()).get_psi());

    // (9.2.2) Compute the rest of the points
    for (size_t i = 1; i < fg.get_inputs().size(); ++i)
    {
        controls_i = fg.get_controls().control_array_at_s(car, i, s[i]);
        const auto dtimeds = fg.get_car()(fg.get_inputs()[i],controls_i,s[i])
                                .dstates_dt[Dynamic_model_t::Road_type::state_names::time];
        inputs[i][Dynamic_model_t::Road_type::input_names::time] = inputs[i-1][Dynamic_model_t::Road_type::input_names::time] 
            + Value((s[i]-s[i-1])*(options.sigma*dtimeds + (1.0-options.sigma)*dtimeds_prev));
        dtimeds_prev = dtimeds;

        x_coord[i] = Value(std::as_const(fg.get_car().get_road()).get_position().x());
        y_coord[i] = Value(std::as_const(fg.get_car().get_road()).get_position().y());
        psi[i]     = Value(std::as_const(fg.get_car().get_road()).get_psi());
    }

    // (9.2.4) Compute the laptime
    laptime = inputs.back()[Dynamic_model_t::Road_type::input_names::time];

    if (isClosed)
        laptime += Value((L-s.back())*((1.0-options.sigma)*dtimeds_prev + options.sigma*dtimeds_first));

    // (9.3) Save optimization data
    optimization_data.x      = result.x;
    optimization_data.x_lb   = x_lb;
    optimization_data.x_ub   = x_ub;
    optimization_data.c_lb   = c_lb;
    optimization_data.c_ub   = c_ub;
    optimization_data.zl     = result.zl;
    optimization_data.zu     = result.zu;
    optimization_data.lambda = result.lambda;
    optimization_data.s      = result.s;
    optimization_data.vl     = result.vl;
    optimization_data.vu     = result.vu;

    // (9.4) Save sensitivity analysis data

    if ( options.check_optimality )
    {
        const size_t n_parameters = car.get_parameters().get_all_parameters_as_scalar().size();
        dinputs_dp.resize(n_parameters);
        dcontrols_dp.resize(n_parameters);
        dlaptime_dp.resize(n_parameters);

        for (size_t i = 0; i < n_parameters; ++i)
        {
            const auto solution_exported = export_solution(fg, dx_dp[i]);
            dinputs_dp[i]     = solution_exported.inputs;
            dcontrols_dp[i]         = solution_exported.controls;

            // If open simulation, kill the first derivative
            if ( !isClosed )
            {
                std::fill(dinputs_dp[i].front().begin()         , dinputs_dp[i].front().end()         , 0.0);
                std::fill(dcontrols_dp[i].front().controls.begin()    , dcontrols_dp[i].front().controls.end()    , 0.0);
                std::fill(dcontrols_dp[i].front().dcontrols_dt.begin(), dcontrols_dp[i].front().dcontrols_dt.end(), 0.0);
            }

            // Compute the derivative of the time
            fg.get_car().update_track_at_arclength(s[0]);
            const auto& kappa_i = fg.get_car().get_road().get_curvature().z();
            const auto& n_i = inputs[0][Dynamic_model_t::Road_type::input_names::lateral_displacement];
            const auto& dndp_i = dinputs_dp[i][0][Dynamic_model_t::Road_type::input_names::lateral_displacement];

            const auto& u_i = inputs[0][Dynamic_model_t::Chassis_type::input_names::velocity_x_mps];
            const auto& dudp_i = dinputs_dp[i][0][Dynamic_model_t::Chassis_type::input_names::velocity_x_mps];

            const auto& v_i = inputs[0][Dynamic_model_t::Chassis_type::input_names::velocity_y_mps];
            const auto& dvdp_i = dinputs_dp[i][0][Dynamic_model_t::Chassis_type::input_names::velocity_y_mps];

            const auto& alpha_i = inputs[0][Dynamic_model_t::Road_type::input_names::track_heading_angle];
            const auto& dalphadp_i = dinputs_dp[i][0][Dynamic_model_t::Road_type::input_names::track_heading_angle];
    
            auto d2timedsdp = [](const auto& kappa, const auto& n, const auto& dndp, 
                                 const auto& u, const auto& dudp, const auto& v, const auto& dvdp, 
                                 const auto& alpha, const auto& dalphadp) -> auto
            { 
                const auto u_n = u*cos(alpha) - v*sin(alpha);
                const auto r   = 1.0 - kappa*n;
                return -kappa*dndp/u_n - cos(alpha)*r/(u_n*u_n)*dudp + sin(alpha)*r/(u_n*u_n)*dvdp + r*(v*cos(alpha)+u*sin(alpha))/(u_n*u_n)*dalphadp;
            };

            auto d2timedsdp_first = d2timedsdp(kappa_i, n_i, dndp_i, u_i, dudp_i, v_i, dvdp_i, alpha_i, dalphadp_i);
            auto d2timedsdp_prev = d2timedsdp_first;

            // (9.2.2) Compute the rest of the points
            for (size_t j = 1; j < fg.get_inputs().size(); ++j)
            {
                fg.get_car().update_track_at_arclength(s[j]);
                const auto& kappa_i = fg.get_car().get_road().get_curvature().z();
                const auto& n_i = inputs[j][Dynamic_model_t::Road_type::input_names::lateral_displacement];
                const auto& dndp_i = dinputs_dp[i][j][Dynamic_model_t::Road_type::input_names::lateral_displacement];
    
                const auto& u_i = inputs[j][Dynamic_model_t::Chassis_type::input_names::velocity_x_mps];
                const auto& dudp_i = dinputs_dp[i][j][Dynamic_model_t::Chassis_type::input_names::velocity_x_mps];
    
                const auto& v_i = inputs[j][Dynamic_model_t::Chassis_type::input_names::velocity_y_mps];
                const auto& dvdp_i = dinputs_dp[i][j][Dynamic_model_t::Chassis_type::input_names::velocity_y_mps];
    
                const auto& alpha_i = inputs[j][Dynamic_model_t::Road_type::input_names::track_heading_angle];
                const auto& dalphadp_i = dinputs_dp[i][j][Dynamic_model_t::Road_type::input_names::track_heading_angle];

                auto d2timedsdp_i = d2timedsdp(kappa_i, n_i, dndp_i, u_i, dudp_i, v_i, dvdp_i, alpha_i, dalphadp_i);
                dinputs_dp[i][j][Dynamic_model_t::Road_type::input_names::time] = dinputs_dp[i][j-1][Dynamic_model_t::Road_type::input_names::time] 
                    + (s[j]-s[j-1])*(options.sigma*d2timedsdp_i + (1.0-options.sigma)*d2timedsdp_prev);
                d2timedsdp_prev = d2timedsdp_i;
            }

            // (9.2.4) Compute the laptime
            dlaptime_dp[i] = dinputs_dp[i].back()[Dynamic_model_t::Road_type::input_names::time];

            if (isClosed)
                dlaptime_dp[i] += Value((L-s.back())*((1.0-options.sigma)*d2timedsdp_prev + options.sigma*d2timedsdp_first));

        } 
    }
}

template<typename Dynamic_model_t>
template<bool isClosed>
inline void Optimal_laptime<Dynamic_model_t>::compute_derivative(const Dynamic_model_t& car) 
{
    class Counter
    {
     public:
        Counter() : k(0) {}

        Counter& increment() { ++k; return *this;}
    
        operator size_t() const { return k; }

     private:
        size_t k;
    };

    auto& s                 = base_type::s;
    auto& n_points          = base_type::n_points;
    auto& n_elements        = base_type::n_elements;
    auto& inputs            = base_type::inputs;
    auto& controls          = base_type::controls;
    auto& laptime           = base_type::laptime;
    auto& x_coord           = base_type::x_coord;
    auto& y_coord           = base_type::y_coord;
    auto& psi               = base_type::psi;
    auto& optimization_data = base_type::optimization_data;
    auto& warm_start        = base_type::warm_start;
    auto& success           = base_type::success;
    auto& iter_count        = base_type::iter_count;

    // (1) Get variable bounds and default control variables
    const auto& inputs_lb   = options.inputs_lb;
    const auto& inputs_ub   = options.inputs_ub;
    const auto& controls_lb = options.controls_lb;
    const auto& controls_ub = options.controls_ub;

    const auto [dcontrols_dt_lb, dcontrols_dt_ub] = car.optimal_laptime_derivative_control_bounds();

    // (1) Construct starting vector of control variables
    auto [controls_start, dcontrols_dt_start] = controls.control_array_and_derivative_at_s(car, 0, s.front());

    // (2) Construct fitness function functor
    FG_derivative<isClosed> fg(n_elements,n_points,car,s,inputs.front(),controls_start,dcontrols_dt_start,
                                controls,integral_quantities,options.sigma);

    // (3) Construct vectors of initial optimization point, and variable upper/lower bounds
    std::vector<scalar> x_start(fg.get_n_variables(),0.0);
    std::vector<scalar> x_lb(fg.get_n_variables(), std::numeric_limits<scalar>::lowest());
    std::vector<scalar> x_ub(fg.get_n_variables(), std::numeric_limits<scalar>::max());

    Counter k;
    constexpr const size_t offset = isClosed ? 0 : 1;

    for (size_t i = offset; i < n_points; ++i)
    {
        // (3.1) Set state and initial condition from start to ITIME
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_names::time; ++j)    
        {
            x_start[k] = inputs[i][j];
            x_lb[k]    = inputs_lb[j];
            x_ub[k]    = inputs_ub[j];
            k.increment();
        }

        // (3.2) Set state to IN. Assert that ITIME = IN - 1
        static_assert(static_cast<size_t>(Dynamic_model_t::Road_type::input_names::time) 
                          == static_cast<size_t>(( Dynamic_model_t::Road_type::input_names::lateral_displacement - 1 )));
        static_assert(static_cast<size_t>(Dynamic_model_t::Road_type::state_names::time) 
                          == static_cast<size_t>(( Dynamic_model_t::Road_type::state_names::lateral_displacement - 1 )));

        x_start[k] = inputs[i][Dynamic_model_t::Road_type::input_names::lateral_displacement];
        x_lb[k]    = -car.get_road().get_left_track_limit(s[i]);
        x_ub[k]    = car.get_road().get_right_track_limit(s[i]);
        k.increment();

        // (3.3) Set state after IN
        for (size_t j = Dynamic_model_t::Road_type::input_names::lateral_displacement+1; j < Dynamic_model_t::number_of_inputs; ++j)    
        {
            x_start[k] = inputs[i][j];
            x_lb[k]    = inputs_lb[j];
            x_ub[k]    = inputs_ub[j];
            k.increment();
        }

        // (3.5) Set full mesh control variables
        for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
        {
            if ( controls[j].optimal_control_type == Optimal_control_type::FULL_MESH )
            {
                x_start[k] = controls[j].controls[i];
                x_lb[k]    = controls_lb[j];
                x_ub[k]    = controls_ub[j];
                k.increment();
            }
        }

        // (3.6) Set full mesh control variables derivatives
        for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
        {
            if ( controls[j].optimal_control_type == Optimal_control_type::FULL_MESH )
            {
                x_start[k] = controls[j].dcontrols_dt[i];
                x_lb[k]    = dcontrols_dt_lb[j];
                x_ub[k]    = dcontrols_dt_ub[j];
                k.increment();
            }
        }
    }

    // (3.7) Add the control variables corresponding to constant optimizations and hypermesh optimizations
    for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
    {
        switch (controls[j].optimal_control_type)
        {
         case (Optimal_control_type::CONSTANT):
            for (size_t i = 0; i < controls[j].controls.size(); ++i)
            {
                x_start[k] = controls[j].controls[i];
                x_lb[k]    = controls_lb[j];
                x_ub[k]    = controls_ub[j];
                k.increment();
            } 

         case (Optimal_control_type::HYPERMESH):
            for (size_t i = 0; i < controls[j].controls.size(); ++i)
            {
                x_start[k] = controls[j].controls[i];
                x_lb[k]    = controls_lb[j];
                x_ub[k]    = controls_ub[j];
                k.increment();
            }    
         default:
            break;
        }
    }

    // (4) Construct vector of upper/lower bounds for constraints
    const auto [time_derivative_equations, algebraic_equations] = car.classify_equations();

    std::vector<scalar> c_lb(fg.get_n_constraints(),0.0);
    std::vector<scalar> c_ub(fg.get_n_constraints(),0.0);
    Counter kc;
    for (size_t i = 1; i < n_points; ++i)
    {
        // Equality constraints: --------------- 

        // (4.1) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}]
        for (size_t i_equation = 0; i_equation < time_derivative_equations.size(); ++i_equation)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (4.3) Algebraic constraints: dqa^{i} = 0.0
        for (size_t i_equation = 0; i_equation < algebraic_equations.size(); ++i_equation)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }
        
        // (4.4) Extra constraints per point
        auto [c_extra_lb, c_extra_ub] = car.optimal_laptime_extra_constraints_bounds(s[i]); 
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
        {
            c_lb[kc] = c_extra_lb[j];
            c_ub[kc] = c_extra_ub[j];
            kc.increment();
        }

        // (4.5) Full mesh control derivatives
        for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
        {   
            if ( controls[j].optimal_control_type == Optimal_control_type::FULL_MESH )
            {
                c_lb[kc] = 0.0;
                c_ub[kc] = 0.0;
                kc.increment();
            }
        }
    }

    // (5) Add the periodic element if track is closed
    if constexpr (isClosed)
    {
        // Equality constraints: --------------- 

        // (5.1) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}]
        for (size_t i_equation = 0; i_equation < time_derivative_equations.size(); ++i_equation)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (5.3) Algebraic constraints: dqa^{i} = 0.0
        for (size_t i_equation = 0; i_equation < algebraic_equations.size(); ++i_equation)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }
        
        // (5.4) Extra constraints per point
        auto [c_extra_lb, c_extra_ub] = car.optimal_laptime_extra_constraints_bounds(s[0]); 
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
        {
            c_lb[kc] = c_extra_lb[j];
            c_ub[kc] = c_extra_ub[j];
            kc.increment();
        }

        // (5.5) Full mesh control variables derivatives
        for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
        {   
            if ( controls[j].optimal_control_type == Optimal_control_type::FULL_MESH )
            {
                c_lb[kc] = 0.0;
                c_ub[kc] = 0.0;
                kc.increment();
            }
        }
    }

    // (6) Add the integral constraints
    for (const auto& integral_quantity : integral_quantities)
    {
        if (integral_quantity.restrict)
        {
            c_lb[kc] = integral_quantity.lower_bound;
            c_ub[kc] = integral_quantity.upper_bound;
            kc.increment();
        }
    }        

    // (6) Check the dimensions of the counter used to fill the vectors
    if ( k != fg.get_n_variables() )
    {
        std::ostringstream s_out;
        s_out << "k(=" << k << ") != fg.get_n_variables()(=" << fg.get_n_variables() << ")" ;
        throw fastest_lap_exception(s_out.str());
    }
    
    if ( kc != fg.get_n_constraints() )
    {
        std::ostringstream s_out;
        s_out << "kc(=" << kc << ") != fg.get_n_constraints()(=" << fg.get_n_constraints() << ")" ;
        throw fastest_lap_exception(s_out.str());
    }


    // (7) Warm-start: simply check the dimensions
    if ( warm_start )
    {
        if ( optimization_data.zl.size() != fg.get_n_variables() )
        {
            std::ostringstream s_out;
            s_out << "size of zl should be " << fg.get_n_variables() << " but it is " << optimization_data.zl.size();
            throw fastest_lap_exception(s_out.str());
        }

        if ( optimization_data.zu.size() != fg.get_n_variables() )
        {
            std::ostringstream s_out;
            s_out << "size of zu should be " << fg.get_n_variables() << " but it is " << optimization_data.zu.size();
            throw fastest_lap_exception(s_out.str());
        }

        if ( optimization_data.lambda.size() != fg.get_n_constraints() )
        {
            std::ostringstream s_out;
            s_out << "size of lambda should be " << fg.get_n_constraints() << " but it is " << optimization_data.lambda.size();
            throw fastest_lap_exception(s_out.str());
        }
    } 

    // (8) Run optimization

    // (8.1) Prepare options
    std::ostringstream ipoptoptions; ipoptoptions << std::setprecision(17);
    ipoptoptions << "Integer print_level " << options.print_level        << std::endl;
    ipoptoptions << "Integer max_iter "    << options.maximum_iterations << std::endl;
    ipoptoptions << "String  sb           yes\n";
    ipoptoptions << "Sparse true forward\n";

    if ( options.retape )
    {
        ipoptoptions << "Retape true\n";
    }

    ipoptoptions << "Numeric tol "             << options.nlp_tolerance              << std::endl;
    ipoptoptions << "Numeric constr_viol_tol " << options.constraints_viol_tolerance << std::endl;
    ipoptoptions << "Numeric acceptable_tol "  << options.acceptable_tolerance       << std::endl;

    // (8.2) Return object
    CppAD::ipopt_cppad_result<std::vector<scalar>> result;

    // (8.3) Solve the problem
    if ( !warm_start )
        CppAD::ipopt_cppad_solve<std::vector<scalar>, FG_derivative<isClosed>>(ipoptoptions.str(), x_start, x_lb, x_ub, c_lb, c_ub, fg, result);
    else
        CppAD::ipopt_cppad_solve<std::vector<scalar>, FG_derivative<isClosed>>(ipoptoptions.str(), x_start, x_lb, x_ub, c_lb, c_ub, 
            optimization_data.lambda, optimization_data.zl, optimization_data.zu, fg, result);
 
    // (8.4) Check success flag
    success = result.status == CppAD::ipopt_cppad_result<std::vector<scalar>>::success; 
    iter_count = result.iter_count;

    if ( !success && options.throw_if_fail )
    {
        throw fastest_lap_exception("Optimization did not succeed");
    }

    // (8.5) Check optimality (disabled by default)
    if ( options.check_optimality )
    {
        auto sensitivity_analysis = Sensitivity_analysis<FG_derivative<isClosed>>(fg, result.x, result.s, result.lambda, result.zl, result.zu, result.vl, result.vu, x_lb, x_ub, c_lb, c_ub, {});
        const auto& optimality_check = sensitivity_analysis.optimality_check;
    
        if ( !optimality_check.success ) 
        {
            std::ostringstream s_out;
            s_out << "[ERROR] Requested optimality check for optimal laptime problem has failed" << std::endl;
            s_out << "        Big components of the gradient vector are:" << std::endl;
            for (size_t i = 0; i < optimality_check.id_not_ok.size(); ++i)
                s_out << "            " << optimality_check.id_not_ok[i] << ": " << optimality_check.nlp_error[optimality_check.id_not_ok[i]] << std::endl;
            throw fastest_lap_exception(s_out.str());
        }

        out(2) << "[INFO] Optimal laptime -> requested optimality check has passed" << std::endl;

        dx_dp = sensitivity_analysis.dxdp;

        // dxdp contains the solution but also the slack variables and the lagrange multipliers, discard everything except the solution (goes first)
        for (auto& dx_dp_i : dx_dp)
            dx_dp_i.resize(fg.get_n_variables());
    }

    // (9) Export the solution

    // (9.1) Export states
    const auto solution_exported = export_solution(fg, result.x);
    inputs        = solution_exported.inputs;
    controls            = solution_exported.controls;
    integral_quantities = solution_exported.integral_quantities;


    // (9.2) Load the latest solution vector to the fitness function object
    std::vector<CppAD::AD<scalar>> x_final(result.x.size(),0.0),fg_final(fg.get_n_constraints()+1,0.0);
    std::copy(result.x.cbegin(), result.x.cend(), x_final.begin());
    fg(fg_final,x_final);
  
    assert(fg.get_inputs().size() == n_points);

    // (9.5) Export time, x, y, and psi
    const scalar& L = car.get_road().track_length();

    x_coord = std::vector<scalar>(fg.get_inputs().size());
    y_coord = std::vector<scalar>(fg.get_inputs().size());
    psi = std::vector<scalar>(fg.get_inputs().size());

    // (9.5.1) Compute the first point
    auto controls_i = fg.get_controls().control_array_at_s(car, 0, s.front());
    auto dtimeds_first = fg.get_car()(fg.get_inputs().front(),controls_i,s.front())
                            .dstates_dt[Dynamic_model_t::Road_type::state_names::time];
    auto dtimeds_prev = dtimeds_first;

    x_coord.front() = Value(std::as_const(fg.get_car().get_road()).get_position().x());
    y_coord.front() = Value(std::as_const(fg.get_car().get_road()).get_position().y());
    psi.front() = Value(std::as_const(fg.get_car().get_road()).get_psi());

    // (9.5.2) Compute the rest of the points
    for (size_t i = 1; i < fg.get_inputs().size(); ++i)
    {
        controls_i = fg.get_controls().control_array_at_s(car, i, s[i]);
        const auto dtimeds = fg.get_car()(fg.get_inputs()[i],controls_i,s[i])
                                .dstates_dt[Dynamic_model_t::Road_type::state_names::time];
        inputs[i][Dynamic_model_t::Road_type::input_names::time] = inputs[i-1][Dynamic_model_t::Road_type::input_names::time] 
            + Value((s[i]-s[i-1])*(options.sigma*dtimeds + (1.0-options.sigma)*dtimeds_prev));
        dtimeds_prev = dtimeds;

        x_coord[i] = Value(std::as_const(fg.get_car().get_road()).get_position().x());
        y_coord[i] = Value(std::as_const(fg.get_car().get_road()).get_position().y());
        psi[i]     = Value(std::as_const(fg.get_car().get_road()).get_psi());
    }

    // (9.5.4) Compute the laptime
    laptime = inputs.back()[Dynamic_model_t::Road_type::input_names::time];

    if (isClosed)
        laptime += Value((L-s.back())*((1.0-options.sigma)*dtimeds_prev + options.sigma*dtimeds_first));

    // (9.6) Save optimization data
    optimization_data.x      = result.x;
    optimization_data.x_lb   = x_lb;
    optimization_data.x_ub   = x_ub;
    optimization_data.c_lb   = c_lb;
    optimization_data.c_ub   = c_ub;
    optimization_data.zl     = result.zl;
    optimization_data.zu     = result.zu;
    optimization_data.lambda = result.lambda;
    optimization_data.s      = result.s;
    optimization_data.vl     = result.vl;
    optimization_data.vu     = result.vu;

    // (9.7) Save sensitivity analysis data

    if ( options.check_optimality )
    {
        const size_t n_parameters = car.get_parameters().get_all_parameters_as_scalar().size();
        dinputs_dp.resize(n_parameters);
        dcontrols_dp.resize(n_parameters);
        dlaptime_dp.resize(n_parameters);

        for (size_t i = 0; i < n_parameters; ++i)
        {
            const auto solution_exported = export_solution(fg, dx_dp[i]);
            dinputs_dp[i]     = solution_exported.inputs;
            dcontrols_dp[i]         = solution_exported.controls;

            // If open simulation, kill the first derivative
            if ( !isClosed )
            {
                std::fill(dinputs_dp[i].front().begin()         , dinputs_dp[i].front().end()         , 0.0);
                std::fill(dcontrols_dp[i].front().controls.begin()    , dcontrols_dp[i].front().controls.end()    , 0.0);
                std::fill(dcontrols_dp[i].front().dcontrols_dt.begin(), dcontrols_dp[i].front().dcontrols_dt.end(), 0.0);
            }

            // Compute the derivative of the time
            fg.get_car().update_track_at_arclength(s[0]);
            const auto& kappa_i = fg.get_car().get_road().get_curvature().z();
            const auto& n_i = inputs[0][Dynamic_model_t::Road_type::input_names::lateral_displacement];
            const auto& dndp_i = dinputs_dp[i][0][Dynamic_model_t::Road_type::input_names::lateral_displacement];

            const auto& u_i = inputs[0][Dynamic_model_t::Chassis_type::input_names::velocity_x_mps];
            const auto& dudp_i = dinputs_dp[i][0][Dynamic_model_t::Chassis_type::input_names::velocity_x_mps];

            const auto& v_i = inputs[0][Dynamic_model_t::Chassis_type::input_names::velocity_y_mps];
            const auto& dvdp_i = dinputs_dp[i][0][Dynamic_model_t::Chassis_type::input_names::velocity_y_mps];

            const auto& alpha_i = inputs[0][Dynamic_model_t::Road_type::input_names::track_heading_angle];
            const auto& dalphadp_i = dinputs_dp[i][0][Dynamic_model_t::Road_type::input_names::track_heading_angle];
    
            auto d2timedsdp = [](const auto& kappa, const auto& n, const auto& dndp, 
                                 const auto& u, const auto& dudp, const auto& v, const auto& dvdp, 
                                 const auto& alpha, const auto& dalphadp) -> auto
            { 
                const auto u_n = u*cos(alpha) - v*sin(alpha);
                const auto r   = 1.0 - kappa*n;
                return -kappa*dndp/u_n - cos(alpha)*r/(u_n*u_n)*dudp + sin(alpha)*r/(u_n*u_n)*dvdp + r*(v*cos(alpha)+u*sin(alpha))/(u_n*u_n)*dalphadp;
            };

            auto d2timedsdp_first = d2timedsdp(kappa_i, n_i, dndp_i, u_i, dudp_i, v_i, dvdp_i, alpha_i, dalphadp_i);
            auto d2timedsdp_prev = d2timedsdp_first;

            // (9.2.2) Compute the rest of the points
            for (size_t j = 1; j < fg.get_inputs().size(); ++j)
            {
                fg.get_car().update_track_at_arclength(s[j]);
                const auto& kappa_i = fg.get_car().get_road().get_curvature().z();
                const auto& n_i = inputs[j][Dynamic_model_t::Road_type::input_names::lateral_displacement];
                const auto& dndp_i = dinputs_dp[i][j][Dynamic_model_t::Road_type::input_names::lateral_displacement];
    
                const auto& u_i = inputs[j][Dynamic_model_t::Chassis_type::input_names::velocity_x_mps];
                const auto& dudp_i = dinputs_dp[i][j][Dynamic_model_t::Chassis_type::input_names::velocity_x_mps];
    
                const auto& v_i = inputs[j][Dynamic_model_t::Chassis_type::input_names::velocity_y_mps];
                const auto& dvdp_i = dinputs_dp[i][j][Dynamic_model_t::Chassis_type::input_names::velocity_y_mps];
    
                const auto& alpha_i = inputs[j][Dynamic_model_t::Road_type::input_names::track_heading_angle];
                const auto& dalphadp_i = dinputs_dp[i][j][Dynamic_model_t::Road_type::input_names::track_heading_angle];

                auto d2timedsdp_i = d2timedsdp(kappa_i, n_i, dndp_i, u_i, dudp_i, v_i, dvdp_i, alpha_i, dalphadp_i);
                dinputs_dp[i][j][Dynamic_model_t::Road_type::input_names::time] = dinputs_dp[i][j-1][Dynamic_model_t::Road_type::input_names::time] 
                    + (s[j]-s[j-1])*(options.sigma*d2timedsdp_i + (1.0-options.sigma)*d2timedsdp_prev);
                d2timedsdp_prev = d2timedsdp_i;
            }

            // (9.2.4) Compute the laptime
            dlaptime_dp[i] = dinputs_dp[i].back()[Dynamic_model_t::Road_type::input_names::time];

            if (isClosed)
                dlaptime_dp[i] += Value((L-s.back())*((1.0-options.sigma)*d2timedsdp_prev + options.sigma*d2timedsdp_first));

        } 
    }
}


template<typename Dynamic_model_t>
template<bool isClosed>
template<bool compute_integrated_quantities>
inline void Optimal_laptime<Dynamic_model_t>::FG_direct<isClosed>::compute(FG_direct<isClosed>::ADvector& fg, const Optimal_laptime<Dynamic_model_t>::FG_direct<isClosed>::ADvector& x)
{
    // (1) Useful aliases
    auto& _n_points                            = FG::_n_points;
    auto& _car                                 = FG::_car;
    auto& _s                                   = FG::_s;
    auto& _inputs_open_initial_point           = FG::_inputs_open_initial_point;
    auto& _controls_open_initial_point         = FG::_controls_open_initial_point;
    auto& _inputs                              = FG::_inputs;
    auto& _controls                            = FG::_controls;
    auto& _integral_quantities                 = FG::_integral_quantities;
    auto& _integral_quantities_values          = FG::_integral_quantities_values;
    auto& _integral_quantities_integrands      = FG::_integral_quantities_integrands;
    auto& _states                              = FG::_states;
    auto& _dstates_dt                          = FG::_dstates_dt;
    auto& _sigma                               = FG::_sigma;

    // (2) Check dimensions
    assert(x.size() == FG::_n_variables);
    assert(fg.size() == (1 + FG::_n_constraints));

    // (3) Load the state and control vectors
    size_t k = 0;

    // (3.1) For open simulations, load the stored initial condition
    if constexpr (!isClosed)
    {
        // (3.1.1) State
        for (size_t j = 0; j < Dynamic_model_t::number_of_inputs; ++j)
            _inputs.front()[j] = _inputs_open_initial_point[j];

        // (3.1.3) Control for full mesh calculations
        for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
        {
            if ( _controls[j].optimal_control_type == Optimal_control_type::FULL_MESH )
                _controls[j].controls.front() = _controls_open_initial_point[j];
        }
    }


    // (3.2) Load the rest of the points
    constexpr const size_t offset = (isClosed ? 0 : 1);

    for (size_t i = offset; i < _n_points; ++i)
    {
        // (3.2.1) Load state (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_names::time; ++j)
            _inputs[i][j] = x[k++];

        // (3.2.2) Load state (after time)
        for (size_t j = Dynamic_model_t::Road_type::input_names::time+1; j < Dynamic_model_t::number_of_inputs; ++j)
            _inputs[i][j] = x[k++];

        // (3.2.4) Load full mesh controls
        for (auto& control_variable : _controls)
        {
            if ( control_variable.optimal_control_type == Optimal_control_type::FULL_MESH )
                control_variable.controls[i] = x[k++];
        }
    }

    // (3.3) Load the rest of the controls
    for (auto& control_variable : _controls)
    {
        switch (control_variable.optimal_control_type)
        {
         case (Optimal_control_type::CONSTANT):
            for (size_t i = 0; i < control_variable.controls.size(); ++i)
                control_variable.controls[i] = x[k++];
            break;
         case (Optimal_control_type::HYPERMESH):
            for (size_t i = 0; i < control_variable.controls.size(); ++i)
                control_variable.controls[i] = x[k++];
            break;
         default:
            break;
        }
    }

    // (4) Check that all variables in x were used
    assert(k == FG::_n_variables);

    // (5) Write fitness function and constraints
    fg.front() = 0.0;
    const auto equations_front = _car(_inputs.front(),_controls.control_array_at_s(_car, 0,_s.front()),_s.front());
    _states.front()              = equations_front.states;
    _dstates_dt.front()          = equations_front.dstates_dt;

    if constexpr (compute_integrated_quantities)
    {
        std::fill(_integral_quantities_values.begin(), _integral_quantities_values.end(), 0.0);
        _integral_quantities_integrands.front() = _dstates_dt.front()[Dynamic_model_t::Road_type::state_names::time]*_car.compute_integral_quantities();
    }

    const auto [time_derivative_equations_ids, algebraic_equations_ids] = _car.classify_equations();

    k = 1;  // Reset the counter
    for (size_t i = 1; i < _n_points; ++i)
    {
        const auto equations    = _car(_inputs[i],_controls.control_array_at_s(_car, i, _s[i]),_s[i]);
        _states[i]              = equations.states;
        _dstates_dt[i]          = equations.dstates_dt;

        // (5.1) Fitness function: integral of time
        const auto& dtime_ds_left = _dstates_dt[i-1][Dynamic_model_t::Road_type::state_names::time];
        const auto& dtime_ds_right = _dstates_dt[i][Dynamic_model_t::Road_type::state_names::time];

        fg.front() += (_s[i]-_s[i-1])*((1.0-_sigma) * dtime_ds_left + _sigma * dtime_ds_right);

        // Equality constraints: --------------- 
        // (5.2) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (const auto eq_id : time_derivative_equations_ids)
            fg[k++] = _states[i][eq_id] - _states[i-1][eq_id] - (_s[i]-_s[i-1])*((1.0-_sigma)*_dstates_dt[i-1][eq_id] + _sigma*_dstates_dt[i][eq_id]);

        for (const auto eq_id : algebraic_equations_ids)
            fg[k++] = _dstates_dt[i][eq_id]/dtime_ds_right;

        // (5.5) Problem dependent extra constraints
        auto c_extra = _car.optimal_laptime_extra_constraints();

        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        if constexpr ( compute_integrated_quantities )
        {
            // (5.6) Compute integral quantities
            _integral_quantities_integrands[i] = _dstates_dt[i][Dynamic_model_t::Road_type::state_names::time]*_car.compute_integral_quantities();
            _integral_quantities_values += (_s[i]-_s[i-1])*((1.0-_sigma)*_integral_quantities_integrands[i-1] + _sigma*_integral_quantities_integrands[i]);
        }
    }

    // (5.6) Add a penalisation to the controls
    for (const auto& control_variable : _controls)
    {
        if (control_variable.optimal_control_type == Optimal_control_type::FULL_MESH)
        {
            for (size_t i = 1; i < _n_points; ++i)
            {
                const auto derivative = (control_variable.controls[i]-control_variable.controls[i-1])/(_s[i]-_s[i-1]);
                fg.front() += control_variable.dissipation*(derivative*derivative)*(_s[i]-_s[i-1]);
            }
        }
    }

    // (5.7) Add the periodic element if track is closed
    const scalar& L = _car.get_road().track_length();
    if constexpr (isClosed)
    {
        // (5.7.1) Fitness function: integral of time
        const auto& dtime_ds_left = _dstates_dt.back()[Dynamic_model_t::Road_type::state_names::time];
        const auto& dtime_ds_right = _dstates_dt.front()[Dynamic_model_t::Road_type::state_names::time];

        fg.front() += (L-_s.back())*(_sigma * dtime_ds_right + (1.0-_sigma) * dtime_ds_left);

        // Equality constraints: 

        // (5.7.2) q^{0} = q^{n-1} + 0.5.ds.[dqdt^{0} + dqdt^{n-1}] (before time)
        for (const auto eq_id : time_derivative_equations_ids)
            fg[k++] = _states.front()[eq_id] - _states.back()[eq_id] - (L-_s.back())*((1.0-_sigma)*_dstates_dt.back()[eq_id] + _sigma*_dstates_dt.front()[eq_id]);

        for (const auto eq_id : algebraic_equations_ids)
            fg[k++] = _dstates_dt.front()[eq_id]/dtime_ds_right;

        // (5.7.5) Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        _car(_inputs.front(),_controls.control_array_at_s(_car,0,_s.front()),_s.front());
        const auto c_extra = _car.optimal_laptime_extra_constraints();
    
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        // (5.7.6) Add a penalisation to the controls
        for (const auto& control_variable : _controls)
        {
            if (control_variable.optimal_control_type == Optimal_control_type::FULL_MESH)
            {
                const auto derivative = (control_variable.controls.front()-control_variable.controls.back())/(L-_s.back());
                fg.front() += control_variable.dissipation*(derivative*derivative)*(L-_s.back());
            }
        }

        if constexpr ( compute_integrated_quantities )
        {
            // (5.7.6) Compute integral quantities
            _integral_quantities_values += (L-_s.back())*((1.0-_sigma)*_integral_quantities_integrands.front() + _sigma*_integral_quantities_integrands.back());
        }
    }

    if constexpr ( compute_integrated_quantities )
    {
        for (const auto& restricted_integral_quantity : _integral_quantities.get_restricted_quantities(_integral_quantities_values))
            fg[k++] = restricted_integral_quantity;
    }

    assert(k == FG::_n_constraints+1);
}


template<typename Dynamic_model_t>
template<bool isClosed>
template<bool compute_integrated_quantities>
inline void Optimal_laptime<Dynamic_model_t>::FG_derivative<isClosed>::compute(FG_derivative<isClosed>::ADvector& fg, const Optimal_laptime<Dynamic_model_t>::FG_derivative<isClosed>::ADvector& x)
{
    // (1) Useful aliases
    auto& _n_points                            = FG::_n_points;
    auto& _car                                 = FG::_car;
    auto& _s                                   = FG::_s;
    auto& _inputs_open_initial_point           = FG::_inputs_open_initial_point;
    auto& _controls_open_initial_point         = FG::_controls_open_initial_point;
    auto& _inputs                              = FG::_inputs;
    auto& _controls                            = FG::_controls;
    auto& _integral_quantities                 = FG::_integral_quantities;
    auto& _integral_quantities_values          = FG::_integral_quantities_values;
    auto& _integral_quantities_integrands      = FG::_integral_quantities_integrands;
    auto& _states                              = FG::_states;
    auto& _dstates_dt                          = FG::_dstates_dt;
    auto& _sigma                               = FG::_sigma;

    // (2) Check dimensions
    assert(x.size() == FG::_n_variables);
    assert(fg.size() == (1 + FG::_n_constraints));

    // (3) Load the state and control vectors
    size_t k = 0;

    // (3.1) For open simulations, load the stored initial condition
    if constexpr (!isClosed)
    {
        // (3.1.1) State
        for (size_t j = 0; j < Dynamic_model_t::number_of_inputs; ++j)
            _inputs[0][j] = _inputs_open_initial_point[j];

        // (3.1.3) Control for full mesh calculations
        for (size_t j = 0; j < Dynamic_model_t::number_of_controls; ++j)
        {
            if ( _controls[j].optimal_control_type == Optimal_control_type::FULL_MESH )
            {
                _controls[j].controls.front()     = _controls_open_initial_point[j];
                _controls[j].dcontrols_dt.front() = _dcontrols_dt_open_initial_point[j];
            }
        }
    }

    // (3.2) Load the rest of the points
    constexpr const size_t offset = (isClosed ? 0 : 1);

    for (size_t i = offset; i < _n_points; ++i)
    {
        // (3.2.1) Load state (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_names::time; ++j)
            _inputs[i][j] = x[k++];

        // (3.2.2) Load state (after time)
        for (size_t j = Dynamic_model_t::Road_type::input_names::time+1; j < Dynamic_model_t::number_of_inputs; ++j)
            _inputs[i][j] = x[k++];

        // (3.2.4) Load full mesh controls
        for (auto& control_variable : _controls)
        {
            if ( control_variable.optimal_control_type == Optimal_control_type::FULL_MESH )
                control_variable.controls[i] = x[k++];
        }

        // (3.2.4) Load full mesh control derivatives
        for (auto& control_variable : _controls)
        {
            if ( control_variable.optimal_control_type == Optimal_control_type::FULL_MESH )
                control_variable.dcontrols_dt[i] = x[k++];
        }
    }

    // (3.3) Load the rest of the controls
    for (auto& control_variable : _controls)
    {
        switch (control_variable.optimal_control_type)
        {
         case (Optimal_control_type::CONSTANT):
            for (size_t i = 0; i < control_variable.controls.size(); ++i)
                control_variable.controls[i] = x[k++];
            break;
         case (Optimal_control_type::HYPERMESH):
            for (size_t i = 0; i < control_variable.controls.size(); ++i)
                control_variable.controls[i] = x[k++];
            break;
         default:
            break;
        }
    }


    // (4) Check that all variables in x were used
    assert(k == FG::_n_variables);

    // (5) Write fitness function and constraints
    fg.front() = 0.0;
    const auto equations_front = _car(_inputs.front(),_controls.control_array_at_s(_car, 0,_s.front()),_s.front());
    _states.front()              = equations_front.states;
    _dstates_dt.front()          = equations_front.dstates_dt;

    if constexpr ( compute_integrated_quantities )
    {
        std::fill(_integral_quantities_values.begin(), _integral_quantities_values.end(), 0.0);
        _integral_quantities_integrands.front() = _dstates_dt.front()[Dynamic_model_t::Road_type::state_names::time]*_car.compute_integral_quantities();
    }

    const auto [time_derivative_equations_ids, algebraic_equations_ids] = _car.classify_equations();

    k = 1;  // Reset the counter
    for (size_t i = 1; i < _n_points; ++i)
    {
        const auto equations    = _car(_inputs[i],_controls.control_array_at_s(_car, i, _s[i]),_s[i]);
        _states[i]              = equations.states;
        _dstates_dt[i]          = equations.dstates_dt;

        // (5.1) Fitness function: 
        const auto& dtime_ds_left = _dstates_dt[i-1][Dynamic_model_t::Road_type::state_names::time];
        const auto& dtime_ds_right = _dstates_dt[i][Dynamic_model_t::Road_type::state_names::time];

        // (5.1.1) Integral of time
        fg[0] += (_s[i]-_s[i-1]) * ( (1.0-_sigma)* dtime_ds_left + _sigma * dtime_ds_right ); 

        // (5.1.2) Penalisation of controls
        for (const auto& control_variable : _controls)
        {
            if ( control_variable.optimal_control_type == Optimal_control_type::FULL_MESH )
            {
                const auto& dcontrols_dt = control_variable.dcontrols_dt;
                const auto& dissipation = control_variable.dissipation;
                fg[0] += 0.5*dissipation*(dcontrols_dt[i-i]*dcontrols_dt[i-i]+dcontrols_dt[i]*dcontrols_dt[i])*(_s[i]-_s[i-1]);
            }
        }

        // Equality constraints: 

        // (5.2) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (const auto eq_id : time_derivative_equations_ids)
            fg[k++] = _states[i][eq_id] - _states[i-1][eq_id] - (_s[i]-_s[i-1])*((1.0-_sigma)*_dstates_dt[i-1][eq_id] + _sigma*_dstates_dt[i][eq_id]);

        for (const auto eq_id : algebraic_equations_ids)
            fg[k++] = _dstates_dt[i][eq_id]/dtime_ds_right;

        // (5.5) Problem dependent extra constraints
        auto c_extra = _car.optimal_laptime_extra_constraints();

        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        // (5.6) Time derivative of full mesh controls
        for (const auto& control_variable : _controls)
        {
            if (control_variable.optimal_control_type == Optimal_control_type::FULL_MESH)
            {
                const auto& controls = control_variable.controls;
                const auto& dcontrols_dt = control_variable.dcontrols_dt;
    
                fg[k++] = controls[i] - controls[i-1] - (_s[i]-_s[i-1])
                    *((1.0-_sigma) * dcontrols_dt[i-1] * dtime_ds_left + _sigma * dcontrols_dt[i] * dtime_ds_right);
            }
        }

        if constexpr ( compute_integrated_quantities )
        {
            // (5.7) Compute integral quantities
            _integral_quantities_integrands[i] = _dstates_dt[i][Dynamic_model_t::Road_type::state_names::time]*_car.compute_integral_quantities();
            _integral_quantities_values += (_s[i]-_s[i-1])*((1.0-_sigma)*_integral_quantities_integrands[i-1] + _sigma*_integral_quantities_integrands[i]);
        }
    }

    // (5.7) Add the periodic element if track is closed
    const scalar& L = _car.get_road().track_length();
    if constexpr (isClosed)
    {
        // (5.7.1) Fitness function: 
        const auto& dtime_ds_left = _dstates_dt.back()[Dynamic_model_t::Road_type::state_names::time];
        const auto& dtime_ds_right = _dstates_dt.front()[Dynamic_model_t::Road_type::state_names::time];

        // (5.7.1.1) Integral of time
        fg[0] += (L-_s.back())*((1.0-_sigma) * dtime_ds_left + _sigma * dtime_ds_right);

        // (5.7.1.2) Penalisation to the controls
        for (const auto& control_variable : _controls)
        {
            if ( control_variable.optimal_control_type == Optimal_control_type::FULL_MESH )
            {
                const auto& dcontrols_dt = control_variable.dcontrols_dt;
                const auto& dissipation = control_variable.dissipation;
                fg[0] += 0.5*dissipation*(dcontrols_dt.back()*dcontrols_dt.back() + dcontrols_dt.front()*dcontrols_dt.front())*(L-_s.back());
            }
        }

        // Equality constraints: 

        // (5.7.2) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (const auto eq_id : time_derivative_equations_ids)
            fg[k++] = _states.front()[eq_id] - _states.back()[eq_id] - (L-_s.back())*((1.0-_sigma)*_dstates_dt.back()[eq_id] + _sigma*_dstates_dt.front()[eq_id]);

        for (const auto eq_id : algebraic_equations_ids)
            fg[k++] = _dstates_dt.front()[eq_id]/dtime_ds_right;

        // (5.7.4) Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        _car(_inputs.front(),_controls.control_array_at_s(_car,0,_s.front()),_s.front());
        auto c_extra = _car.optimal_laptime_extra_constraints();

        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        // (5.7.5) Time derivative of the control variables
        for (const auto& control_variable : _controls)
        {
            if (control_variable.optimal_control_type == Optimal_control_type::FULL_MESH)
            {
                const auto& controls = control_variable.controls;
                const auto& dcontrols_dt = control_variable.dcontrols_dt;
                fg[k++] = controls.front() - controls.back() 
                    - (L-_s.back()) * ((1.0-_sigma) * dcontrols_dt.back() * dtime_ds_left + _sigma * dcontrols_dt.front() * dtime_ds_right);
            }
        }

        if constexpr ( compute_integrated_quantities )
        {
            // (5.7.6) Compute integral quantities
            _integral_quantities_values += (L-_s.back())*((1.0-_sigma)*_integral_quantities_integrands.front() + _sigma*_integral_quantities_integrands.back());
        }
    }

    if constexpr ( compute_integrated_quantities )
    {
        for (const auto& restricted_integral_quantity : _integral_quantities.get_restricted_quantities(_integral_quantities_values))
            fg[k++] = restricted_integral_quantity;
    }

    assert(k == FG::_n_constraints+1);
}



