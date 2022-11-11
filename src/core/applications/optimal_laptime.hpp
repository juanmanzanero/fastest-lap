#include "lion/thirdparty/include/cppad/ipopt/solve.hpp"
#include "lion/math/ipopt_cppad_handler.hpp"
#include "lion/math/sensitivity_analysis.h"
#include "lion/thirdparty/include/logger.hpp"


template <typename T,size_t N>
std::ostream& operator<<(std::ostream &os, const std::array<T,N>& v);

template<typename Dynamic_model_t>
inline Optimal_laptime<Dynamic_model_t>::Optimal_laptime(const std::vector<scalar>& s_, const bool is_closed_, const bool is_direct_,
    const Dynamic_model_t& car, const std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>& input_states_start, 
    const std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>& algebraic_states_start,
    Control_variables<> controls_start, 
    const Options opts)
: options(opts), integral_quantities(), is_closed(is_closed_), is_direct(is_direct_), warm_start(false), 
  s(s_), input_states(input_states_start), algebraic_states(algebraic_states_start), 
  controls(controls_start.check()),
  optimization_data()
{
    // (1) Check inputs
    check_inputs(car);

    // (4) Compute
    compute(car);
}


template<typename Dynamic_model_t>
inline Optimal_laptime<Dynamic_model_t>::Optimal_laptime(const std::vector<scalar>& s_, const bool is_closed_, const bool is_direct_,
    const Dynamic_model_t& car,
    const std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>& input_states_start, 
    const std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>& algebraic_states_start,
    Control_variables<> controls_start, 
    const std::vector<scalar>& zl,
    const std::vector<scalar>& zu,
    const std::vector<scalar>& lambda,
    const Options opts)
: options(opts), integral_quantities(), is_closed(is_closed_), is_direct(is_direct_), warm_start(true), 
  s(s_), input_states(input_states_start), algebraic_states(algebraic_states_start), 
  controls(controls_start.check()),
  optimization_data({.zl{zl},.zu{zu},.lambda{lambda}})
{
    // (1) Check inputs
    check_inputs(car);
      
    // (5) Compute
    compute(car);
}


template<typename Dynamic_model_t>
inline void Optimal_laptime<Dynamic_model_t>::check_inputs(const Dynamic_model_t& car)
{
    if ( s.size() <= 1 )
        throw fastest_lap_exception("Provide at least two values of arclength");

    // (1) Compute number of elements and points
    n_points = s.size();
    n_elements = (is_closed ? n_points : n_points - 1);

    // (2) Verify the vector or arclength
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
    if ( input_states.size() != n_points )
        throw fastest_lap_exception("q must have size of n_points");

    // (3.2) Algebraic state
    if ( algebraic_states.size() != n_points )
        throw fastest_lap_exception("algebraic_states must have size of n_points");
    
    // (3.3) Controls: check size only for full-mesh controls
    for (const auto& control_variable : controls )
    {
        if ( control_variable.optimal_control_type == FULL_MESH )
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
    Xml_element root = doc.get_root_element();

    if ( root.get_attribute("type") == "closed" )
        is_closed = true;
    else if ( root.get_attribute("type") == "open" )
        is_closed = false;
    else
        throw fastest_lap_exception("Incorrect track type, should be \"open\" or \"closed\"");

    if ( root.get_attribute("is_direct") == "true" )
        is_direct = true;
    else if ( root.get_attribute("is_direct") == "false" )
        is_direct = false;
    else
        throw fastest_lap_exception("Incorrect \"is_direct\" attribute, should be \"true\" or \"false\"");

    auto children = doc.get_root_element().get_children();

    const auto [key_name, q_names, qa_names, u_names] = Dynamic_model_t{}.get_state_and_control_names();

    // Get the data
    n_points = std::stoi(root.get_attribute("n_points"));
    n_elements = (is_closed ? n_points : n_points - 1);

    laptime = root.get_child("laptime").get_value(scalar());

    s = root.get_child(key_name).get_value(std::vector<scalar>());

    // Get state
    input_states = std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>(n_points);
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        std::vector<scalar> data_in = root.get_child(q_names[i]).get_value(std::vector<scalar>());
        for (size_t j = 0; j < n_points; ++j)
            input_states[j][i] = data_in[j];
    }

    // Get algebraic states
    algebraic_states = std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>(n_points);
    for (size_t i = 0; i < Dynamic_model_t::NALGEBRAIC; ++i)
    {
        std::vector<scalar> data_in = root.get_child(qa_names[i]).get_value(std::vector<scalar>());
        for (size_t j = 0; j < n_points; ++j)
            algebraic_states[j][i] = data_in[j];
    }

    // Get controls
    controls = Control_variables<>{};
    for (size_t i = 0; i < Dynamic_model_t::NCONTROL; ++i)
    {
        auto control_var_element = root.get_child("control_variables/" + u_names[i]);
        
        // Get optimal control type attribute
        auto optimal_control_type_str = control_var_element.get_attribute("optimal_control_type");

        if ( optimal_control_type_str == "dont optimize" )
            controls[i].optimal_control_type = DONT_OPTIMIZE;
        else if ( optimal_control_type_str == "constant" )
            controls[i].optimal_control_type = CONSTANT;
        else if ( optimal_control_type_str == "hypermesh" )
            controls[i].optimal_control_type = HYPERMESH;
        else if ( optimal_control_type_str == "full-mesh" )
            controls[i].optimal_control_type = FULL_MESH;
        else
            throw fastest_lap_exception("optimal_control_type attribute not recognize. Options are: 'dont optimize', 'constant', 'hypermesh', 'full-mesh'");

        // Get value
        controls[i].controls = root.get_child("control_variables/" + u_names[i] + "/values").get_value(std::vector<scalar>());

        // Get hypermesh
        if ( controls[i].optimal_control_type == HYPERMESH )
            controls[i].s_hypermesh = root.get_child("control_variables/" + u_names[i] + "/hypermesh").get_value(std::vector<scalar>());

        // Get derivative value if present
        if ( !is_direct && root.has_child("control_variables/" + u_names[i] + "/derivatives") )
            controls[i].dcontrols_dt = root.get_child("control_variables/" + u_names[i] + "/derivatives").get_value(std::vector<scalar>());

        // Get hypermesh 
        if ( controls[i].optimal_control_type == HYPERMESH )
            controls[i].s_hypermesh = root.get_child("control_variables/" + u_names[i] + "/hypermesh").get_value(std::vector<scalar>());
    }

    // check them and compute its statistics
    controls.check();

    // Get x
    x_coord = root.get_child("x").get_value(std::vector<scalar>());
    y_coord = root.get_child("y").get_value(std::vector<scalar>());
    psi     = root.get_child("psi").get_value(std::vector<scalar>());

    // Get optimization data
    optimization_data.zl     = root.get_child("optimization_data").get_child("zl").get_value(std::vector<scalar>());
    optimization_data.zu     = root.get_child("optimization_data").get_child("zu").get_value(std::vector<scalar>());
    optimization_data.lambda = root.get_child("optimization_data").get_child("lambda").get_value(std::vector<scalar>());
}


template<typename Dynamic_model_t>
template<typename FG_t>
inline typename Optimal_laptime<Dynamic_model_t>::Export_solution 
    Optimal_laptime<Dynamic_model_t>::export_solution(FG_t& fg, const std::vector<scalar>& x) const
{
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
  
    assert(fg.get_input_states().size() == n_points);
    assert(fg.get_algebraic_states().size() == n_points);

    // (3) Export states
    output.input_states = std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
            output.input_states[i][j] = Value(fg.get_input_states()[i][j]);

    // (4) Export algebraic states
    output.algebraic_states = std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            output.algebraic_states[i][j] = Value(fg.get_algebraic_states()[i][j]);

    // (5) Export control variables
    for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
    {
        switch (output.controls[j].optimal_control_type) 
        {
         case (CONSTANT):
            std::transform(fg.get_controls()[j].controls.cbegin(), fg.get_controls()[j].controls.cend(), output.controls[j].controls.begin(), 
                [](const auto& u_cppad) -> auto { return Value(u_cppad); });
            break;
        
         case (HYPERMESH):
            std::transform(fg.get_controls()[j].controls.cbegin(), fg.get_controls()[j].controls.cend(), output.controls[j].controls.begin(), 
                [](const auto& u_cppad) -> auto { return Value(u_cppad); });
            break;

         case (FULL_MESH):
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

    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> start" << std::endl;

    // (1) Get variable bounds and default control variables
    const auto& input_states_lb     = options.input_states_lb;
    const auto& input_states_ub     = options.input_states_ub;
    const auto& algebraic_states_lb = options.algebraic_states_lb;
    const auto& algebraic_states_ub = options.algebraic_states_ub;
    const auto& controls_lb         = options.controls_lb;
    const auto& controls_ub         = options.controls_ub;

    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> input states lower bound: " << input_states_lb << std::endl;
    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> input states upper bound: " << input_states_ub << std::endl;
    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> algebraic states lower bound: " << algebraic_states_lb << std::endl;
    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> algebraic states upper bound: " << algebraic_states_ub << std::endl;
    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> controls lower bound: " << controls_lb << std::endl;
    out(2) << "[" << get_current_date_and_time() << "] Optimal laptime computation -> controls upper bound: " << controls_ub << std::endl;

    // (1) Construct starting vector of control variables
    auto controls_start = controls.control_array_at_s(car, 0, s.front());

    // (2) Construct fitness function functor
    FG_direct<isClosed> fg(n_elements, n_points, car, s, input_states.front(), algebraic_states.front(), controls_start, 
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
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)    
        {
            x_start[k] = input_states[i][j];
            x_lb[k]    = input_states_lb[j];
            x_ub[k]    = input_states_ub[j];
            k.increment();
        }

        // (3.2) Set state to IN. Assert that ITIME = IN - 1
        static_assert(static_cast<size_t>(Dynamic_model_t::Road_type::input_state_names::TIME) 
                          == static_cast<size_t>(( Dynamic_model_t::Road_type::input_state_names::N - 1 )));
        static_assert(static_cast<size_t>(Dynamic_model_t::Road_type::input_state_names::TIME) 
                          == static_cast<size_t>(Dynamic_model_t::Road_type::state_names::TIME));
        static_assert(static_cast<size_t>(Dynamic_model_t::Road_type::input_state_names::N) 
                          == static_cast<size_t>(Dynamic_model_t::Road_type::state_names::N));

        x_start[k] = input_states[i][Dynamic_model_t::Road_type::input_state_names::N];
        x_lb[k]    = -car.get_road().get_left_track_limit(s[i]);
        x_ub[k]    = car.get_road().get_right_track_limit(s[i]);
        k.increment();

        // (3.3) Set state after IN
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::N+1; j < Dynamic_model_t::NSTATE; ++j)    
        {
            x_start[k] = input_states[i][j];
            x_lb[k]    = input_states_lb[j];
            x_ub[k]    = input_states_ub[j];
            k.increment();
        }

        // (3.4) Set algebraic variables
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
        {
            x_start[k] = algebraic_states[i][j];
            x_lb[k]    = algebraic_states_lb[j];
            x_ub[k]    = algebraic_states_ub[j];
            k.increment();
        }

        // (3.5) Set full mesh control variables
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            if ( controls[j].optimal_control_type == FULL_MESH )
            {
                x_start[k] = controls[j].controls[i];
                x_lb[k]    = controls_lb[j];
                x_ub[k]    = controls_ub[j];
                k.increment();
            }
        }
    }

    // (3.6) Add the control variables corresponding to constant optimizations and hypermesh optimizations
    for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
    {
        switch (controls[j].optimal_control_type)
        {
         case (CONSTANT):
            for (size_t i = 0; i < controls[j].controls.size(); ++i)
            {
                x_start[k] = controls[j].controls[i];
                x_lb[k]    = controls_lb[j];
                x_ub[k]    = controls_ub[j];
                k.increment();
            } 
            break;

         case (HYPERMESH):
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
    Counter kc;
    for (size_t i = 1; i < n_points; ++i)
    {
        // Equality constraints: --------------- 

        // (4.1) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (4.2) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::TIME+1; j < Dynamic_model_t::NSTATE; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (4.3) Algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
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

        // (5.1) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (5.2) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::TIME+1; j < Dynamic_model_t::NSTATE; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (5.3) Algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
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
    input_states        = solution_exported.input_states;
    algebraic_states    = solution_exported.algebraic_states;
    controls            = solution_exported.controls;
    integral_quantities = solution_exported.integral_quantities;

    // (9.2) Export time, x, y, and psi
    const scalar& L = car.get_road().track_length();

    x_coord = std::vector<scalar>(fg.get_input_states().size());
    y_coord = std::vector<scalar>(fg.get_input_states().size());
    psi = std::vector<scalar>(fg.get_input_states().size());

    // (9.2.1) Compute the first point
    auto controls_i = fg.get_controls().control_array_at_s(car, 0, s.front());
    auto dtimeds_first = fg.get_car()(fg.get_input_states().front(),fg.get_algebraic_states().front(),controls_i,s.front())
                            .dstates_dt[Dynamic_model_t::Road_type::state_names::TIME];
    auto dtimeds_prev = dtimeds_first;

    x_coord.front() = Value(fg.get_car().get_road().get_x());
    y_coord.front() = Value(fg.get_car().get_road().get_y());
    psi.front() = Value(fg.get_car().get_road().get_psi());

    // (9.2.2) Compute the rest of the points
    for (size_t i = 1; i < fg.get_input_states().size(); ++i)
    {
        controls_i = fg.get_controls().control_array_at_s(car, i, s[i]);
        const auto dtimeds = fg.get_car()(fg.get_input_states()[i],fg.get_algebraic_states()[i],controls_i,s[i])
                                .dstates_dt[Dynamic_model_t::Road_type::state_names::TIME];
        input_states[i][Dynamic_model_t::Road_type::input_state_names::TIME] = input_states[i-1][Dynamic_model_t::Road_type::input_state_names::TIME] 
            + Value((s[i]-s[i-1])*(options.sigma*dtimeds + (1.0-options.sigma)*dtimeds_prev));
        dtimeds_prev = dtimeds;

        x_coord[i] = Value(fg.get_car().get_road().get_x());
        y_coord[i] = Value(fg.get_car().get_road().get_y());
        psi[i]     = Value(fg.get_car().get_road().get_psi());
    }

    // (9.2.4) Compute the laptime
    laptime = input_states.back()[Dynamic_model_t::Road_type::input_state_names::TIME];

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
        dinput_states_dp.resize(n_parameters);
        dalgebraic_states_dp.resize(n_parameters);
        dcontrols_dp.resize(n_parameters);
        dlaptime_dp.resize(n_parameters);

        for (size_t i = 0; i < n_parameters; ++i)
        {
            const auto solution_exported = export_solution(fg, dx_dp[i]);
            dinput_states_dp[i]     = solution_exported.input_states;
            dalgebraic_states_dp[i] = solution_exported.algebraic_states;
            dcontrols_dp[i]         = solution_exported.controls;

            // If open simulation, kill the first derivative
            if ( !isClosed )
            {
                std::fill(dinput_states_dp[i].front().begin()         , dinput_states_dp[i].front().end()         , 0.0);
                std::fill(dalgebraic_states_dp[i].front().begin()     , dalgebraic_states_dp[i].front().end()     , 0.0);
                std::fill(dcontrols_dp[i].front().controls.begin()    , dcontrols_dp[i].front().controls.end()    , 0.0);
                std::fill(dcontrols_dp[i].front().dcontrols_dt.begin(), dcontrols_dp[i].front().dcontrols_dt.end(), 0.0);
            }

            // Compute the derivative of the time
            fg.get_car().get_road().update_track(s[0]);
            const auto& kappa_i = fg.get_car().get_road().get_curvature();
            const auto& n_i = input_states[0][Dynamic_model_t::Road_type::input_state_names::N];
            const auto& dndp_i = dinput_states_dp[i][0][Dynamic_model_t::Road_type::input_state_names::N];

            const auto& u_i = input_states[0][Dynamic_model_t::Chassis_type::input_state_names::U];
            const auto& dudp_i = dinput_states_dp[i][0][Dynamic_model_t::Chassis_type::input_state_names::U];

            const auto& v_i = input_states[0][Dynamic_model_t::Chassis_type::input_state_names::V];
            const auto& dvdp_i = dinput_states_dp[i][0][Dynamic_model_t::Chassis_type::input_state_names::V];

            const auto& alpha_i = input_states[0][Dynamic_model_t::Road_type::input_state_names::ALPHA];
            const auto& dalphadp_i = dinput_states_dp[i][0][Dynamic_model_t::Road_type::input_state_names::ALPHA];
    
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
            for (size_t j = 1; j < fg.get_input_states().size(); ++j)
            {
                fg.get_car().get_road().update_track(s[j]);
                const auto& kappa_i = fg.get_car().get_road().get_curvature();
                const auto& n_i = input_states[j][Dynamic_model_t::Road_type::input_state_names::N];
                const auto& dndp_i = dinput_states_dp[i][j][Dynamic_model_t::Road_type::input_state_names::N];
    
                const auto& u_i = input_states[j][Dynamic_model_t::Chassis_type::input_state_names::U];
                const auto& dudp_i = dinput_states_dp[i][j][Dynamic_model_t::Chassis_type::input_state_names::U];
    
                const auto& v_i = input_states[j][Dynamic_model_t::Chassis_type::input_state_names::V];
                const auto& dvdp_i = dinput_states_dp[i][j][Dynamic_model_t::Chassis_type::input_state_names::V];
    
                const auto& alpha_i = input_states[j][Dynamic_model_t::Road_type::input_state_names::ALPHA];
                const auto& dalphadp_i = dinput_states_dp[i][j][Dynamic_model_t::Road_type::input_state_names::ALPHA];

                auto d2timedsdp_i = d2timedsdp(kappa_i, n_i, dndp_i, u_i, dudp_i, v_i, dvdp_i, alpha_i, dalphadp_i);
                dinput_states_dp[i][j][Dynamic_model_t::Road_type::input_state_names::TIME] = dinput_states_dp[i][j-1][Dynamic_model_t::Road_type::input_state_names::TIME] 
                    + (s[j]-s[j-1])*(options.sigma*d2timedsdp_i + (1.0-options.sigma)*d2timedsdp_prev);
                d2timedsdp_prev = d2timedsdp_i;
            }

            // (9.2.4) Compute the laptime
            dlaptime_dp[i] = dinput_states_dp[i].back()[Dynamic_model_t::Road_type::state_names::TIME];

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

    // (1) Get variable bounds and default control variables
    const auto& input_states_lb     = options.input_states_lb;
    const auto& input_states_ub     = options.input_states_ub;
    const auto& algebraic_states_lb = options.algebraic_states_lb;
    const auto& algebraic_states_ub = options.algebraic_states_ub;
    const auto& controls_lb         = options.controls_lb;
    const auto& controls_ub         = options.controls_ub;

    const auto [dcontrols_dt_lb, dcontrols_dt_ub] = car.optimal_laptime_derivative_control_bounds();

    // (1) Construct starting vector of control variables
    auto [controls_start, dcontrols_dt_start] = controls.control_array_and_derivative_at_s(car, 0, s.front());

    // (2) Construct fitness function functor
    FG_derivative<isClosed> fg(n_elements,n_points,car,s,input_states.front(),algebraic_states.front(),controls_start,dcontrols_dt_start,
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
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)    
        {
            x_start[k] = input_states[i][j];
            x_lb[k]    = input_states_lb[j];
            x_ub[k]    = input_states_ub[j];
            k.increment();
        }

        // (3.2) Set state to IN. Assert that ITIME = IN - 1
        static_assert(static_cast<size_t>(Dynamic_model_t::Road_type::input_state_names::TIME) 
                          == static_cast<size_t>(( Dynamic_model_t::Road_type::input_state_names::N - 1 )));
        static_assert(static_cast<size_t>(Dynamic_model_t::Road_type::input_state_names::TIME) 
                          == static_cast<size_t>(Dynamic_model_t::Road_type::state_names::TIME));
        static_assert(static_cast<size_t>(Dynamic_model_t::Road_type::input_state_names::N) 
                          == static_cast<size_t>(Dynamic_model_t::Road_type::state_names::N));

        x_start[k] = input_states[i][Dynamic_model_t::Road_type::input_state_names::N];
        x_lb[k]    = -car.get_road().get_left_track_limit(s[i]);
        x_ub[k]    = car.get_road().get_right_track_limit(s[i]);
        k.increment();

        // (3.3) Set state after IN
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::N+1; j < Dynamic_model_t::NSTATE; ++j)    
        {
            x_start[k] = input_states[i][j];
            x_lb[k]    = input_states_lb[j];
            x_ub[k]    = input_states_ub[j];
            k.increment();
        }

        // (3.4) Set algebraic variables
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
        {
            x_start[k] = algebraic_states[i][j];
            x_lb[k]    = algebraic_states_lb[j];
            x_ub[k]    = algebraic_states_ub[j];
            k.increment();
        }

        // (3.5) Set full mesh control variables
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            if ( controls[j].optimal_control_type == FULL_MESH )
            {
                x_start[k] = controls[j].controls[i];
                x_lb[k]    = controls_lb[j];
                x_ub[k]    = controls_ub[j];
                k.increment();
            }
        }

        // (3.6) Set full mesh control variables derivatives
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            if ( controls[j].optimal_control_type == FULL_MESH )
            {
                x_start[k] = controls[j].dcontrols_dt[i];
                x_lb[k]    = dcontrols_dt_lb[j];
                x_ub[k]    = dcontrols_dt_ub[j];
                k.increment();
            }
        }
    }

    // (3.7) Add the control variables corresponding to constant optimizations and hypermesh optimizations
    for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
    {
        switch (controls[j].optimal_control_type)
        {
         case (CONSTANT):
            for (size_t i = 0; i < controls[j].controls.size(); ++i)
            {
                x_start[k] = controls[j].controls[i];
                x_lb[k]    = controls_lb[j];
                x_ub[k]    = controls_ub[j];
                k.increment();
            } 

         case (HYPERMESH):
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
    std::vector<scalar> c_lb(fg.get_n_constraints(),0.0);
    std::vector<scalar> c_ub(fg.get_n_constraints(),0.0);
    Counter kc;
    for (size_t i = 1; i < n_points; ++i)
    {
        // Equality constraints: --------------- 

        // (4.1) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (4.2) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::TIME+1; j < Dynamic_model_t::NSTATE; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (4.3) Algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
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
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {   
            if ( controls[j].optimal_control_type == FULL_MESH )
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

        // (5.1) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (5.2) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::TIME+1; j < Dynamic_model_t::NSTATE; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc.increment();
        }

        // (5.3) Algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
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
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {   
            if ( controls[j].optimal_control_type == FULL_MESH )
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
    input_states        = solution_exported.input_states;
    algebraic_states    = solution_exported.algebraic_states;
    controls            = solution_exported.controls;
    integral_quantities = solution_exported.integral_quantities;


    // (9.2) Load the latest solution vector to the fitness function object
    std::vector<CppAD::AD<scalar>> x_final(result.x.size(),0.0),fg_final(fg.get_n_constraints()+1,0.0);
    std::copy(result.x.cbegin(), result.x.cend(), x_final.begin());
    fg(fg_final,x_final);
  
    assert(fg.get_input_states().size() == n_points);
    assert(fg.get_algebraic_states().size() == n_points);

    // (9.5) Export time, x, y, and psi
    const scalar& L = car.get_road().track_length();

    x_coord = std::vector<scalar>(fg.get_input_states().size());
    y_coord = std::vector<scalar>(fg.get_input_states().size());
    psi = std::vector<scalar>(fg.get_input_states().size());

    // (9.5.1) Compute the first point
    auto controls_i = fg.get_controls().control_array_at_s(car, 0, s.front());
    auto dtimeds_first = fg.get_car()(fg.get_input_states().front(),fg.get_algebraic_states().front(),controls_i,s.front())
                            .dstates_dt[Dynamic_model_t::Road_type::state_names::TIME];
    auto dtimeds_prev = dtimeds_first;

    x_coord.front() = Value(fg.get_car().get_road().get_x());
    y_coord.front() = Value(fg.get_car().get_road().get_y());
    psi.front() = Value(fg.get_car().get_road().get_psi());

    // (9.5.2) Compute the rest of the points
    for (size_t i = 1; i < fg.get_input_states().size(); ++i)
    {
        controls_i = fg.get_controls().control_array_at_s(car, i, s[i]);
        const auto dtimeds = fg.get_car()(fg.get_input_states()[i],fg.get_algebraic_states()[i],controls_i,s[i])
                                .dstates_dt[Dynamic_model_t::Road_type::state_names::TIME];
        input_states[i][Dynamic_model_t::Road_type::input_state_names::TIME] = input_states[i-1][Dynamic_model_t::Road_type::input_state_names::TIME] 
            + Value((s[i]-s[i-1])*(options.sigma*dtimeds + (1.0-options.sigma)*dtimeds_prev));
        dtimeds_prev = dtimeds;

        x_coord[i] = Value(fg.get_car().get_road().get_x());
        y_coord[i] = Value(fg.get_car().get_road().get_y());
        psi[i]     = Value(fg.get_car().get_road().get_psi());
    }

    // (9.5.4) Compute the laptime
    laptime = input_states.back()[Dynamic_model_t::Road_type::input_state_names::TIME];

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
        dinput_states_dp.resize(n_parameters);
        dalgebraic_states_dp.resize(n_parameters);
        dcontrols_dp.resize(n_parameters);
        dlaptime_dp.resize(n_parameters);

        for (size_t i = 0; i < n_parameters; ++i)
        {
            const auto solution_exported = export_solution(fg, dx_dp[i]);
            dinput_states_dp[i]     = solution_exported.input_states;
            dalgebraic_states_dp[i] = solution_exported.algebraic_states;
            dcontrols_dp[i]         = solution_exported.controls;

            // If open simulation, kill the first derivative
            if ( !isClosed )
            {
                std::fill(dinput_states_dp[i].front().begin()         , dinput_states_dp[i].front().end()         , 0.0);
                std::fill(dalgebraic_states_dp[i].front().begin()     , dalgebraic_states_dp[i].front().end()     , 0.0);
                std::fill(dcontrols_dp[i].front().controls.begin()    , dcontrols_dp[i].front().controls.end()    , 0.0);
                std::fill(dcontrols_dp[i].front().dcontrols_dt.begin(), dcontrols_dp[i].front().dcontrols_dt.end(), 0.0);
            }

            // Compute the derivative of the time
            fg.get_car().get_road().update_track(s[0]);
            const auto& kappa_i = fg.get_car().get_road().get_curvature();
            const auto& n_i = input_states[0][Dynamic_model_t::Road_type::input_state_names::N];
            const auto& dndp_i = dinput_states_dp[i][0][Dynamic_model_t::Road_type::input_state_names::N];

            const auto& u_i = input_states[0][Dynamic_model_t::Chassis_type::input_state_names::U];
            const auto& dudp_i = dinput_states_dp[i][0][Dynamic_model_t::Chassis_type::input_state_names::U];

            const auto& v_i = input_states[0][Dynamic_model_t::Chassis_type::input_state_names::V];
            const auto& dvdp_i = dinput_states_dp[i][0][Dynamic_model_t::Chassis_type::input_state_names::V];

            const auto& alpha_i = input_states[0][Dynamic_model_t::Road_type::input_state_names::ALPHA];
            const auto& dalphadp_i = dinput_states_dp[i][0][Dynamic_model_t::Road_type::input_state_names::ALPHA];
    
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
            for (size_t j = 1; j < fg.get_input_states().size(); ++j)
            {
                fg.get_car().get_road().update_track(s[j]);
                const auto& kappa_i = fg.get_car().get_road().get_curvature();
                const auto& n_i = input_states[j][Dynamic_model_t::Road_type::input_state_names::N];
                const auto& dndp_i = dinput_states_dp[i][j][Dynamic_model_t::Road_type::input_state_names::N];
    
                const auto& u_i = input_states[j][Dynamic_model_t::Chassis_type::input_state_names::U];
                const auto& dudp_i = dinput_states_dp[i][j][Dynamic_model_t::Chassis_type::input_state_names::U];
    
                const auto& v_i = input_states[j][Dynamic_model_t::Chassis_type::input_state_names::V];
                const auto& dvdp_i = dinput_states_dp[i][j][Dynamic_model_t::Chassis_type::input_state_names::V];
    
                const auto& alpha_i = input_states[j][Dynamic_model_t::Road_type::input_state_names::ALPHA];
                const auto& dalphadp_i = dinput_states_dp[i][j][Dynamic_model_t::Road_type::input_state_names::ALPHA];

                auto d2timedsdp_i = d2timedsdp(kappa_i, n_i, dndp_i, u_i, dudp_i, v_i, dvdp_i, alpha_i, dalphadp_i);
                dinput_states_dp[i][j][Dynamic_model_t::Road_type::input_state_names::TIME] = dinput_states_dp[i][j-1][Dynamic_model_t::Road_type::input_state_names::TIME] 
                    + (s[j]-s[j-1])*(options.sigma*d2timedsdp_i + (1.0-options.sigma)*d2timedsdp_prev);
                d2timedsdp_prev = d2timedsdp_i;
            }

            // (9.2.4) Compute the laptime
            dlaptime_dp[i] = dinput_states_dp[i].back()[Dynamic_model_t::Road_type::state_names::TIME];

            if (isClosed)
                dlaptime_dp[i] += Value((L-s.back())*((1.0-options.sigma)*d2timedsdp_prev + options.sigma*d2timedsdp_first));

        } 
    }
}


template<typename Dynamic_model_t>
std::unique_ptr<Xml_document> Optimal_laptime<Dynamic_model_t>::xml() const 
{
    std::unique_ptr<Xml_document> doc_ptr(std::make_unique<Xml_document>());


    std::ostringstream s_out;
    s_out.precision(17);

    doc_ptr->create_root_element("optimal_laptime");

    auto root = doc_ptr->get_root_element();
    
    root.set_attribute("n_points",std::to_string(n_points));
    
    if ( is_closed )
        root.set_attribute("type", "closed");
    else
        root.set_attribute("type", "open");

    if ( is_direct )
        root.set_attribute("is_direct","true");
    else
        root.set_attribute("is_direct","false");
    

    // Save arclength
    for (size_t j = 0; j < s.size()-1; ++j)
        s_out << s[j] << ", ";

    s_out << s.back();

    root.add_child("laptime").set_value(std::to_string(laptime));

    const auto [key_name, q_names, qa_names, u_names] = Dynamic_model_t{}.get_state_and_control_names();

    root.add_child(key_name).set_value(s_out.str());
    s_out.str(""); s_out.clear();


    // Save state
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        for (size_t j = 0; j < input_states.size()-1; ++j)
            s_out << input_states[j][i] << ", " ;

        s_out << input_states.back()[i];

        root.add_child(q_names[i]).set_value(s_out.str());
        s_out.str(""); s_out.clear();
    }

    // Save algebraic state
    for (size_t i = 0; i < Dynamic_model_t::NALGEBRAIC; ++i)
    {
        for (size_t j = 0; j < algebraic_states.size()-1; ++j)
            s_out << algebraic_states[j][i] << ", " ;

        s_out << algebraic_states.back()[i];

        root.add_child(qa_names[i]).set_value(s_out.str());
        s_out.str(""); s_out.clear();
    }

    // Save controls
    auto node_control_variables = root.add_child("control_variables");
    for (size_t i = 0; i < Dynamic_model_t::NCONTROL; ++i)
    {
        auto node_variable = node_control_variables.add_child(u_names[i]);
        if ( controls[i].optimal_control_type == DONT_OPTIMIZE )
        {
            node_variable.set_attribute("optimal_control_type","dont optimize");
            node_variable.add_child("values");
        }
        else if ( controls[i].optimal_control_type == CONSTANT )
        {   
            node_variable.set_attribute("optimal_control_type","constant");
            s_out << controls[i].controls.front();
            node_variable.add_child("values").set_value(s_out.str());
            s_out.str(""); s_out.clear();
        }
        else if ( controls[i].optimal_control_type == HYPERMESH )
        {
            node_variable.set_attribute("optimal_control_type","hypermesh");
            for (size_t j = 0; j < controls[i].controls.size()-1; ++j)
                s_out << controls[i].controls[j] << ", " ;

            s_out << controls[i].controls.back();

            node_variable.add_child("values").set_value(s_out.str());
            s_out.str(""); s_out.clear();

            for (size_t j = 0; j < controls[i].s_hypermesh.size()-1; ++j)
                s_out << controls[i].s_hypermesh[j] << ", " ;

            s_out << controls[i].s_hypermesh.back();

            node_variable.add_child("hypermesh").set_value(s_out.str());
            s_out.str(""); s_out.clear();
        }
        else if ( controls[i].optimal_control_type == FULL_MESH )
        {
            node_variable.set_attribute("optimal_control_type", "full-mesh");
            for (size_t j = 0; j < controls[i].controls.size()-1; ++j)
                s_out << controls[i].controls[j] << ", " ;

            s_out << controls[i].controls.back();

            node_variable.add_child("values").set_value(s_out.str());
            s_out.str(""); s_out.clear();

            // Save controls derivatives if not direct
            if ( !is_direct )
            {
                for (size_t j = 0; j < controls[i].dcontrols_dt.size()-1; ++j)
                    s_out << controls[i].dcontrols_dt[j] << ", " ;
        
                s_out << controls[i].dcontrols_dt.back();

                node_variable.add_child("derivatives").set_value(s_out.str());
                s_out.str(""); s_out.clear();
            }
        }
    }

    // Save x, y, and psi if they are not contained
    for (size_t j = 0; j < input_states.size()-1; ++j)
        s_out << x_coord[j] << ", " ;

    s_out << x_coord.back();
    root.add_child("x").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < input_states.size()-1; ++j)
        s_out << y_coord[j] << ", " ;

    s_out << y_coord.back();
    root.add_child("y").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < input_states.size()-1; ++j)
        s_out << psi[j] << ", " ;

    s_out << psi.back();
    root.add_child("psi").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    // Save optimization data
    auto opt_data = root.add_child("optimization_data");
    size_t n_vars(0);
    size_t n_cons(0);

    if ( is_direct )
    {
        n_vars = n_variables_per_point<true>(controls);
        n_cons = n_constraints_per_element<true>(controls);
    }
    else
    {
        n_vars = n_variables_per_point<false>(controls);
        n_cons = n_constraints_per_element<false>(controls);
    }

    opt_data.set_attribute("n_variables_per_point", std::to_string(n_vars));
    opt_data.set_attribute("n_constraints_per_element", std::to_string(n_cons));

    opt_data.add_child("number_of_iterations").set_value(std::to_string(iter_count));

    for (size_t j = 0; j < optimization_data.zl.size()-1; ++j)
        s_out << optimization_data.zl[j] << ", " ;

    s_out << optimization_data.zl.back();

    opt_data.add_child("zl").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < optimization_data.zu.size()-1; ++j)
        s_out << optimization_data.zu[j] << ", " ;

    s_out << optimization_data.zu.back();

    opt_data.add_child("zu").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < optimization_data.lambda.size()-1; ++j)
        s_out << optimization_data.lambda[j] << ", " ;

    s_out << optimization_data.lambda.back();

    opt_data.add_child("lambda").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    return doc_ptr;
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
    auto& _input_states_open_initial_point     = FG::_input_states_open_initial_point;
    auto& _algebraic_states_open_initial_point = FG::_algebraic_states_open_initial_point;
    auto& _controls_open_initial_point         = FG::_controls_open_initial_point;
    auto& _input_states                        = FG::_input_states;
    auto& _algebraic_states                    = FG::_algebraic_states;
    auto& _controls                            = FG::_controls;
    auto& _integral_quantities                 = FG::_integral_quantities;
    auto& _integral_quantities_values          = FG::_integral_quantities_values;
    auto& _integral_quantities_integrands      = FG::_integral_quantities_integrands;
    auto& _states                              = FG::_states;
    auto& _dstates_dt                          = FG::_dstates_dt;
    auto& _algebraic_equations                 = FG::_algebraic_equations;
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
        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
            _input_states.front()[j] = _input_states_open_initial_point[j];

        // (3.1.2) Algebraic state
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            _algebraic_states.front()[j] = _algebraic_states_open_initial_point[j];
    
        // (3.1.3) Control for full mesh calculations
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            if ( _controls[j].optimal_control_type == FULL_MESH )
                _controls[j].controls.front() = _controls_open_initial_point[j];
        }
    }


    // (3.2) Load the rest of the points
    constexpr const size_t offset = (isClosed ? 0 : 1);

    for (size_t i = offset; i < _n_points; ++i)
    {
        // (3.2.1) Load state (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)
            _input_states[i][j] = x[k++];

        // (3.2.2) Load state (after time)
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::TIME+1; j < Dynamic_model_t::NSTATE; ++j)
            _input_states[i][j] = x[k++];

        // (3.2.3) Load algebraic state
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            _algebraic_states[i][j] = x[k++];
            
        // (3.2.4) Load full mesh controls
        for (auto& control_variable : _controls)
        {
            if ( control_variable.optimal_control_type == FULL_MESH )
                control_variable.controls[i] = x[k++];
        }
    }

    // (3.3) Load the rest of the controls
    for (auto& control_variable : _controls)
    {
        switch (control_variable.optimal_control_type)
        {
         case (CONSTANT):
            for (size_t i = 0; i < control_variable.controls.size(); ++i)
                control_variable.controls[i] = x[k++];
            break;
         case (HYPERMESH):
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
    const auto equations_front = _car(_input_states.front(),_algebraic_states.front(),_controls.control_array_at_s(_car, 0,_s.front()),_s.front());
    _states.front()              = equations_front.states;
    _dstates_dt.front()          = equations_front.dstates_dt;
    _algebraic_equations.front() = equations_front.algebraic_equations;

    if constexpr (compute_integrated_quantities)
    {
        std::fill(_integral_quantities_values.begin(), _integral_quantities_values.end(), 0.0);
        _integral_quantities_integrands.front() = _dstates_dt.front()[Dynamic_model_t::Road_type::input_state_names::TIME]*_car.compute_integral_quantities();
    }

    k = 1;  // Reset the counter
    for (size_t i = 1; i < _n_points; ++i)
    {
        const auto equations    = _car(_input_states[i],_algebraic_states[i],_controls.control_array_at_s(_car, i, _s[i]),_s[i]);
        _states[i]              = equations.states;
        _dstates_dt[i]          = equations.dstates_dt;
        _algebraic_equations[i] = equations.algebraic_equations;

        // (5.1) Fitness function: integral of time
        const auto& dtime_ds_left = _dstates_dt[i-1][Dynamic_model_t::Road_type::input_state_names::TIME];
        const auto& dtime_ds_right = _dstates_dt[i][Dynamic_model_t::Road_type::input_state_names::TIME];

        fg.front() += (_s[i]-_s[i-1])*((1.0-_sigma) * dtime_ds_left + _sigma * dtime_ds_right);

        // Equality constraints: --------------- 

        // (5.2) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)
            fg[k++] = _states[i][j] - _states[i-1][j] - (_s[i]-_s[i-1])*((1.0-_sigma)*_dstates_dt[i-1][j] + _sigma*_dstates_dt[i][j]);

        // (5.3) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::TIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _states[i][j] - _states[i-1][j] - (_s[i]-_s[i-1])*((1.0-_sigma)*_dstates_dt[i-1][j] + _sigma*_dstates_dt[i][j]);

        // (5.4) algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            fg[k++] = _algebraic_equations[i][j];
        
        // (5.5) Problem dependent extra constraints
        auto c_extra = _car.optimal_laptime_extra_constraints();

        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        if constexpr ( compute_integrated_quantities )
        {
            // (5.6) Compute integral quantities
            _integral_quantities_integrands[i] = _dstates_dt[i][Dynamic_model_t::Road_type::input_state_names::TIME]*_car.compute_integral_quantities();
            _integral_quantities_values += (_s[i]-_s[i-1])*((1.0-_sigma)*_integral_quantities_integrands[i-1] + _sigma*_integral_quantities_integrands[i]);
        }
    }

    // (5.6) Add a penalisation to the controls
    for (const auto& control_variable : _controls)
    {
        if (control_variable.optimal_control_type == FULL_MESH)
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
        const auto& dtime_ds_left = _dstates_dt.back()[Dynamic_model_t::Road_type::input_state_names::TIME];
        const auto& dtime_ds_right = _dstates_dt.front()[Dynamic_model_t::Road_type::input_state_names::TIME];

        fg.front() += (L-_s.back())*(_sigma * dtime_ds_right + (1.0-_sigma) * dtime_ds_left);

        // Equality constraints: 

        // (5.7.2) q^{0} = q^{n-1} + 0.5.ds.[dqdt^{0} + dqdt^{n-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)
            fg[k++] = _states.front()[j] - _states.back()[j] - (L-_s.back())*((1.0-_sigma)*_dstates_dt.back()[j] + _sigma*_dstates_dt.front()[j]);

        // (5.7.3) q^{0} = q^{n-1} + 0.5.ds.[dqdt^{0} + dqdt^{n-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::TIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _states.front()[j] - _states.back()[j] - (L-_s.back())*((1.0-_sigma)*_dstates_dt.back()[j] + _sigma*_dstates_dt.front()[j]);

        // (5.7.4) algebraic constraints: dqa^{0} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            fg[k++] = _algebraic_equations.front()[j];

        // (5.7.5) Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        _car(_input_states.front(),_algebraic_states.front(),_controls.control_array_at_s(_car,0,_s.front()),_s.front());
        const auto c_extra = _car.optimal_laptime_extra_constraints();
    
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        // (5.7.6) Add a penalisation to the controls
        for (const auto& control_variable : _controls)
        {
            if (control_variable.optimal_control_type == FULL_MESH)
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
    auto& _input_states_open_initial_point     = FG::_input_states_open_initial_point;
    auto& _algebraic_states_open_initial_point = FG::_algebraic_states_open_initial_point;
    auto& _controls_open_initial_point         = FG::_controls_open_initial_point;
    auto& _input_states                        = FG::_input_states;
    auto& _algebraic_states                    = FG::_algebraic_states;
    auto& _controls                            = FG::_controls;
    auto& _integral_quantities                 = FG::_integral_quantities;
    auto& _integral_quantities_values          = FG::_integral_quantities_values;
    auto& _integral_quantities_integrands      = FG::_integral_quantities_integrands;
    auto& _states                              = FG::_states;
    auto& _dstates_dt                          = FG::_dstates_dt;
    auto& _algebraic_equations                 = FG::_algebraic_equations;
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
        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
            _input_states[0][j] = _input_states_open_initial_point[j];

        // (3.1.2) Algebraic state
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            _algebraic_states[0][j] = _algebraic_states_open_initial_point[j];
    
        // (3.1.3) Control for full mesh calculations
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            if ( _controls[j].optimal_control_type == FULL_MESH )
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
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)
            _input_states[i][j] = x[k++];

        // (3.2.2) Load state (after time)
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::TIME+1; j < Dynamic_model_t::NSTATE; ++j)
            _input_states[i][j] = x[k++];

        // (3.2.3) Load algebraic state
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            _algebraic_states[i][j] = x[k++];
            
        // (3.2.4) Load full mesh controls
        for (auto& control_variable : _controls)
        {
            if ( control_variable.optimal_control_type == FULL_MESH )
                control_variable.controls[i] = x[k++];
        }

        // (3.2.4) Load full mesh control derivatives
        for (auto& control_variable : _controls)
        {
            if ( control_variable.optimal_control_type == FULL_MESH )
                control_variable.dcontrols_dt[i] = x[k++];
        }
    }

    // (3.3) Load the rest of the controls
    for (auto& control_variable : _controls)
    {
        switch (control_variable.optimal_control_type)
        {
         case (CONSTANT):
            for (size_t i = 0; i < control_variable.controls.size(); ++i)
                control_variable.controls[i] = x[k++];
            break;
         case (HYPERMESH):
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
    const auto equations_front = _car(_input_states.front(),_algebraic_states.front(),_controls.control_array_at_s(_car, 0,_s.front()),_s.front());
    _states.front()              = equations_front.states;
    _dstates_dt.front()          = equations_front.dstates_dt;
    _algebraic_equations.front() = equations_front.algebraic_equations;

    if constexpr ( compute_integrated_quantities )
    {
        std::fill(_integral_quantities_values.begin(), _integral_quantities_values.end(), 0.0);
        _integral_quantities_integrands.front() = _dstates_dt.front()[Dynamic_model_t::Road_type::input_state_names::TIME]*_car.compute_integral_quantities();
    }

    k = 1;  // Reset the counter
    for (size_t i = 1; i < _n_points; ++i)
    {
        const auto equations    = _car(_input_states[i],_algebraic_states[i],_controls.control_array_at_s(_car, i, _s[i]),_s[i]);
        _states[i]              = equations.states;
        _dstates_dt[i]          = equations.dstates_dt;
        _algebraic_equations[i] = equations.algebraic_equations;

        // (5.1) Fitness function: 
        const auto& dtime_ds_left = _dstates_dt[i-1][Dynamic_model_t::Road_type::input_state_names::TIME];
        const auto& dtime_ds_right = _dstates_dt[i][Dynamic_model_t::Road_type::input_state_names::TIME];

        // (5.1.1) Integral of time
        fg[0] += (_s[i]-_s[i-1]) * ( (1.0-_sigma)* dtime_ds_left + _sigma * dtime_ds_right ); 

        // (5.1.2) Penalisation of controls
        for (const auto& control_variable : _controls)
        {
            if ( control_variable.optimal_control_type == FULL_MESH )
            {
                const auto& dcontrols_dt = control_variable.dcontrols_dt;
                const auto& dissipation = control_variable.dissipation;
                fg[0] += 0.5*dissipation*(dcontrols_dt[i-i]*dcontrols_dt[i-i]+dcontrols_dt[i]*dcontrols_dt[i])*(_s[i]-_s[i-1]);
            }
        }

        // Equality constraints: 

        // (5.2) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)
            fg[k++] = _states[i][j] - _states[i-1][j] - (_s[i]-_s[i-1])*((1.0-_sigma)*_dstates_dt[i-1][j] + _sigma*_dstates_dt[i][j]);

        // (5.3) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::TIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _states[i][j] - _states[i-1][j] - (_s[i]-_s[i-1])*((1.0-_sigma)*_dstates_dt[i-1][j] + _sigma*_dstates_dt[i][j]);

        // (5.4) algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            fg[k++] = _algebraic_equations[i][j];

        // (5.5) Problem dependent extra constraints
        auto c_extra = _car.optimal_laptime_extra_constraints();

        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        // (5.6) Time derivative of full mesh controls
        for (const auto& control_variable : _controls)
        {
            if (control_variable.optimal_control_type == FULL_MESH)
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
            _integral_quantities_integrands[i] = _dstates_dt[i][Dynamic_model_t::Road_type::input_state_names::TIME]*_car.compute_integral_quantities();
            _integral_quantities_values += (_s[i]-_s[i-1])*((1.0-_sigma)*_integral_quantities_integrands[i-1] + _sigma*_integral_quantities_integrands[i]);
        }
    }

    // (5.7) Add the periodic element if track is closed
    const scalar& L = _car.get_road().track_length();
    if constexpr (isClosed)
    {
        // (5.7.1) Fitness function: 
        const auto& dtime_ds_left = _dstates_dt.back()[Dynamic_model_t::Road_type::input_state_names::TIME];
        const auto& dtime_ds_right = _dstates_dt.front()[Dynamic_model_t::Road_type::input_state_names::TIME];

        // (5.7.1.1) Integral of time
        fg[0] += (L-_s.back())*((1.0-_sigma) * dtime_ds_left + _sigma * dtime_ds_right);

        // (5.7.1.2) Penalisation to the controls
        for (const auto& control_variable : _controls)
        {
            if ( control_variable.optimal_control_type == FULL_MESH )
            {
                const auto& dcontrols_dt = control_variable.dcontrols_dt;
                const auto& dissipation = control_variable.dissipation;
                fg[0] += 0.5*dissipation*(dcontrols_dt.back()*dcontrols_dt.back() + dcontrols_dt.front()*dcontrols_dt.front())*(L-_s.back());
            }
        }

        // Equality constraints: 

        // (5.7.2) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::input_state_names::TIME; ++j)
            fg[k++] = _states.front()[j] - _states.back()[j] - (L-_s.back())*((1.0-_sigma)*_dstates_dt.back()[j] + _sigma*_dstates_dt.front()[j]);

        // (5.7.3) q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = Dynamic_model_t::Road_type::input_state_names::TIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _states.front()[j] - _states.back()[j] - (L-_s.back())*((1.0-_sigma)*_dstates_dt.back()[j] + _sigma*_dstates_dt.front()[j]);

        // (5.7.3) algebraic constraints: dqa^{0} = 0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            fg[k++] = _algebraic_equations.front()[j];

        // (5.7.4) Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        _car(_input_states.front(),_algebraic_states.front(),_controls.control_array_at_s(_car,0,_s.front()),_s.front());
        auto c_extra = _car.optimal_laptime_extra_constraints();

        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        // (5.7.5) Time derivative of the control variables
        for (const auto& control_variable : _controls)
        {
            if (control_variable.optimal_control_type == FULL_MESH)
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

//
// Implementation of the helper class Control_variables ----------------------------------------------------:-
//

template<typename Dynamic_model_t>
template<typename T>
void Optimal_laptime<Dynamic_model_t>::Control_variables<T>::check_inputs()
{
    for (auto& control_variable : *this)
    {
        switch(control_variable.optimal_control_type)
        {
         case(DONT_OPTIMIZE):
            // Check that s_hypermesh is empty, and that controls only contains one value 
            if ( control_variable.s_hypermesh.size() > 0 ) 
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                    "In \"don't optimize\" mode, s_hypermesh should be empty");

            if ( control_variable.controls.size() > 0 )
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                    "In \"don't optimize\" mode, controls should be empty. Its size is " + std::to_string(control_variable.controls.size()) );

            if ( control_variable.dcontrols_dt.size() != 0 )
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                    "In \"don't optimize\" mode, dcontrols_dt should be empty");

            control_variable.dissipation = 0.0;

            break;

         case(CONSTANT):
            // Check that s_hypermesh is empty, and that controls only contains one value 
            if ( control_variable.s_hypermesh.size() > 0 ) 
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                    "In \"constant optimization\" mode, s_hypermesh should be empty");

            if ( control_variable.controls.size() != 1 )
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                    "In \"constant optimization\" mode, controls should contain only one value");

            if ( control_variable.dcontrols_dt.size() != 0 )
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                    "In \"constant optimization\" mode, dcontrols_dt should be empty");

            control_variable.dissipation = 0.0;

            break;
         case(HYPERMESH):
            // Check that the size of s_hypermesh equals the size of controls
            if ( control_variable.s_hypermesh.size() != control_variable.controls.size() )
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                    "In \"hypermesh optimization\" mode, size(s_hypermesh) should be equal to size(controls)");

            if ( control_variable.dcontrols_dt.size() != 0 )
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                    "In \"hypermesh optimization\" mode, dcontrols_dt should be empty");

            control_variable.dissipation = 0.0;

            break;
         case(FULL_MESH): 
            // Check that s_hypermesh is empty, the size of controls will be checked later 
            if ( control_variable.s_hypermesh.size() > 0 ) 
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                    "In \"full-mesh optimize\" mode, s_hypermesh should be empty");

            if ( (control_variable.controls.size() != control_variable.dcontrols_dt.size()) && (control_variable.dcontrols_dt.size() != 0) )
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> "
                    "In \"full-mesh optimize\" mode, dcontrols_dt should either have the same size as controls, or be empty");

            if ( control_variable.dissipation < 0.0 )
                throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> dissipation must be non-negative");

            break;
         default:
            throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> optimization mode not recognized");
    
            break;
        }
    }
}


template<typename Dynamic_model_t>
template<typename T>
void Optimal_laptime<Dynamic_model_t>::Control_variables<T>::compute_statistics()
{
    // (1) Reset values
    number_of_constant_optimizations        = 0;
    number_of_hypermesh_optimization_points = 0;
    number_of_full_optimizations            = 0;

    for (const auto& control_variable : *this)
    {
        switch(control_variable.optimal_control_type)
        {
         case(DONT_OPTIMIZE):
            // Do nothing
            break;
         case(CONSTANT):
            ++number_of_constant_optimizations;
            break;
         case(HYPERMESH):
            number_of_hypermesh_optimization_points += control_variable.s_hypermesh.size();
            break;
         case(FULL_MESH): 
            ++number_of_full_optimizations;
            break;
         default:
            throw fastest_lap_exception("[ERROR] Control_variables::check_inputs() -> optimization mode not recognized");
    
            break;
        }
    }
}


template<typename Dynamic_model_t>
template<typename T>
template<typename U>
std::enable_if_t<std::is_same_v<U,scalar>, typename Optimal_laptime<Dynamic_model_t>::template Control_variables<CppAD::AD<scalar>>> 
    Optimal_laptime<Dynamic_model_t>::Control_variables<T>::to_CppAD() const
{
    // (1) Create return value
    Control_variables<CppAD::AD<scalar>> output;

    // (2) Copy the std::array storage
    std::transform(base_type::cbegin(), base_type::cend(), output.begin(), [](const auto& scalar_control) -> auto { return scalar_control.to_CppAD(); });

    // (3) Copy the statistics
    output.number_of_constant_optimizations        = number_of_constant_optimizations;
    output.number_of_hypermesh_optimization_points = number_of_hypermesh_optimization_points;
    output.number_of_full_optimizations            = number_of_full_optimizations;

    // (4) Return
    return output;
}

template<typename Dynamic_model_t>
template<typename T>
typename Optimal_laptime<Dynamic_model_t>::template Control_variables<T>& Optimal_laptime<Dynamic_model_t>::Control_variables<T>::clear()
{
    std::transform(base_type::begin(), base_type::end(), base_type::begin(), [](auto& input) -> auto { return input.clear(); });
    return *this;
}


template<typename Dynamic_model_t>
template<typename T>
std::array<T,Dynamic_model_t::NCONTROL> Optimal_laptime<Dynamic_model_t>::Control_variables<T>::control_array_at_s(const Dynamic_model_t& car, const size_t i_fullmesh, const scalar s) const
{
    std::array<scalar,Dynamic_model_t::NCONTROL> controls_scalar = car.get_state_and_control_upper_lower_and_default_values().controls_def;
    std::array<T,Dynamic_model_t::NCONTROL> controls;

    std::copy(controls_scalar.cbegin(), controls_scalar.cend(), controls.begin());

    for (size_t j = 0; j < Dynamic_model_t::Dynamic_model_t::NCONTROL; ++j)
    {
        switch((*this)[j].optimal_control_type)
        {
         case (DONT_OPTIMIZE):
            // Keep the default value
            break;

         case (CONSTANT):
            controls[j] = (*this)[j].controls.front();
            break;

         case (HYPERMESH):
            controls[j] = (*this)[j].get_hypermesh_control_value_for_s(s);
            break;

         case (FULL_MESH):
            controls[j] = (*this)[j].controls[i_fullmesh];
            break;
        }
    }
    
    return controls;
}


template<typename Dynamic_model_t>
template<typename T>
std::pair<std::array<T,Dynamic_model_t::NCONTROL>,std::array<T,Dynamic_model_t::NCONTROL>>
    Optimal_laptime<Dynamic_model_t>::Control_variables<T>::control_array_and_derivative_at_s
        (const Dynamic_model_t& car, const size_t i_fullmesh, const scalar s) const
{
    std::array<T,Dynamic_model_t::NCONTROL> controls = car.get_state_and_control_upper_lower_and_default_values().controls_def;
    std::array<T,Dynamic_model_t::NCONTROL> dcontrols_dt{0.0};

    for (size_t j = 0; j < Dynamic_model_t::Dynamic_model_t::NCONTROL; ++j)
    {
        switch((*this)[j].optimal_control_type)
        {
         case (DONT_OPTIMIZE):
            dcontrols_dt[j] = 0.0;
            break;

         case (CONSTANT):
            controls[j] = (*this)[j].controls.front();
            dcontrols_dt[j] = 0.0;
            break;

         case (HYPERMESH):
            controls[j] = (*this)[j].get_hypermesh_control_value_for_s(s);
            dcontrols_dt[j] = 0.0;
            break;

         case (FULL_MESH):
            controls[j] = (*this)[j].controls[i_fullmesh];
            dcontrols_dt[j] = (*this)[j].dcontrols_dt[i_fullmesh];
            break;
        }
    }

    return {controls,dcontrols_dt};
}


template<typename Dynamic_model_t>
template<typename T>
typename Optimal_laptime<Dynamic_model_t>::template Control_variables<T>& Optimal_laptime<Dynamic_model_t>::Control_variables<T>::check()
{
    check_inputs();
    compute_statistics();
    return *this;
}



//
// Implementation of the helper class Control_variable -----------------------------------------------------:-
//

template<typename Dynamic_model_t>
template<typename T>
template<typename U>
std::enable_if_t<std::is_same_v<U,scalar>, typename Optimal_laptime<Dynamic_model_t>::template Control_variable<CppAD::AD<scalar>>> 
    Optimal_laptime<Dynamic_model_t>::Control_variable<T>::to_CppAD() const
{
    // (1) Define the new variable of type CppAD::AD
    Control_variable<CppAD::AD<scalar>> output;

    // (2) Copy optimal control type
    output.optimal_control_type = optimal_control_type; 

    // (3) Copy hypermesh
    output.s_hypermesh          = s_hypermesh; 

    // (4) Copy control variable values
    output.controls = std::vector<CppAD::AD<scalar>>(controls.size());
    std::copy(controls.cbegin(), controls.cend(), output.controls.begin());

    output.dcontrols_dt = std::vector<CppAD::AD<scalar>>(dcontrols_dt.size());
    std::copy(dcontrols_dt.cbegin(), dcontrols_dt.cend(), output.dcontrols_dt.begin());

    // (5) Copy dissipation
    output.dissipation = dissipation;

    // (6) Return
    return output;
}


template<typename Dynamic_model_t>
template<typename T>
size_t Optimal_laptime<Dynamic_model_t>::Control_variable<T>::get_hypermesh_position_for_s(const std::vector<scalar>& s_hypermesh, scalar s)
{
    // (1) Get an iterator to the first point in the hypermesh grid that *it > s
    auto it = std::upper_bound(s_hypermesh.begin(), s_hypermesh.end(), s);

    // (2) Error if the point found is the first node of the hypermesh
    if ( it == s_hypermesh.cbegin() )
        throw fastest_lap_exception("[ERROR] Optimal_laptime::Control_variable::get_hypermesh_value_for_s -> input s(" 
            + std::to_string(s) + ") is smaller than the first hypermesh checkpoint (" + std::to_string(s_hypermesh.front()) + ")");

    // (3) Compute position
    const size_t i_position = std::distance(s_hypermesh.begin(), it) - 1;

    return i_position;
}


template<typename Dynamic_model_t>
template<typename T>
const T& Optimal_laptime<Dynamic_model_t>::Control_variable<T>::get_hypermesh_control_value_for_s(const scalar s) const
{
    // (1) Error if control variable is not of type hypermesh
    if (optimal_control_type != HYPERMESH )
        throw fastest_lap_exception("[ERROR] Optimal_laptime::Control_variable::get_hypermesh_value_for_s can only be called for HYPERMESH parameters");

    // (2) Return
    return controls[get_hypermesh_position_for_s(s_hypermesh,s)]; 
}


template<typename Dynamic_model_t>
template<typename T>
std::pair<const T&,const T&> Optimal_laptime<Dynamic_model_t>::Control_variable<T>::get_hypermesh_control_value_and_derivative_for_s(const scalar s) const
{
    // (1) Error if control variable is not of type hypermesh
    if (optimal_control_type != HYPERMESH )
        throw fastest_lap_exception("[ERROR] Optimal_laptime::Control_variable::get_hypermesh_value_for_s can only be called for HYPERMESH parameters");

    // (2) Return
    const size_t i_position = get_hypermesh_position_for_s(s_hypermesh,s);
    return {controls[i_position], dcontrols_dt[i_position]}; 
}


template<typename Dynamic_model_t>
template<typename T>
typename Optimal_laptime<Dynamic_model_t>::template Control_variable<T>& Optimal_laptime<Dynamic_model_t>::Control_variable<T>::clear()
{
    std::fill(controls.begin(), controls.end(), 0.0);
    std::fill(dcontrols_dt.begin(), dcontrols_dt.end(), 0.0);

    return *this;
}
