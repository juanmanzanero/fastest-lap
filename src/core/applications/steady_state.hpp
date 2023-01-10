#ifndef STEADY_STATE_HPP
#define STEADY_STATE_HPP

#include "lion/math/optimise.h"
#include "lion/math/solve_nonlinear_system.h"
#include "lion/foundation/constants.h"
#include "lion/thirdparty/include/logger.hpp"
#include "lion/math/ipopt_cppad_handler.hpp"
#include "src/core/foundation/fastest_lap_exception.h"

template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,scalar>::value,typename Steady_state<Dynamic_model_t>::Solution> 
    Steady_state<Dynamic_model_t>::solve(scalar v, scalar ax, scalar ay, const size_t n_steps, const bool provide_x0, const std::vector<scalar>& x0_provided, bool throw_if_fail)
{
    std::vector<scalar> x0 = Dynamic_model_t::steady_state_initial_guess();

    if ( provide_x0 )
    {
        assert(x0_provided.size() == Dynamic_model_t::number_of_steady_state_variables);
        x0 = x0_provided;
    }

    // Solve the problem
    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
    auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

    // Aim for higher constraint voilation tolerance for steady-state computations
    Solve_nonlinear_system_options options;
    options.throw_if_fail = throw_if_fail;
    typename Solve_nonlinear_system<Solve_constraints>::Nonlinear_system_solution result;
    for (size_t i = 1; i <= n_steps; ++i)
    {
        const double factor = ((double) i)/((double) n_steps);
        Solve_constraints c(_car,v,factor*ax,factor*ay);
        result = Solve_nonlinear_system<Solve_constraints>::solve(Dynamic_model_t::number_of_steady_state_variables,Dynamic_model_t::number_of_steady_state_equations,x0,c,x_lb,x_ub,c_lb,c_ub,options);

        // Set x0 for the next iteration
        x0 = result.x;
    }

    Solve_constraints c(_car,v,ax,ay);

    typename Solve_constraints::argument_type x;
    std::copy(result.x.cbegin(), result.x.cend(), x.begin());
    c(x);
    std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> inputs = c.get_inputs();
    std::array<Timeseries_t,Dynamic_model_t::number_of_controls> controls = c.get_controls();
    auto [states,dstates_dt] = _car(inputs, controls, 0.0);

    return Solution
    {
        .solved           = result.solved,
        .v                = v,
        .ax               = ax,
        .ay               = ay,
        .inputs           = inputs,
        .controls         = controls,
        .dstates_dt       = dstates_dt
    };
}

template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,typename Steady_state<Dynamic_model_t>::Solution> 
    Steady_state<Dynamic_model_t>::solve(scalar v, scalar ax, scalar ay, const size_t, const bool provide_x0, const std::vector<scalar>& x0_provided, bool throw_if_fail)
{
    std::vector<scalar> x0 = Dynamic_model_t::steady_state_initial_guess();

    if ( provide_x0 )
    {
        assert(x0_provided.size() == Dynamic_model_t::number_of_steady_state_variables);
        x0 = x0_provided;
    }

    // Solve the problem
    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
    auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    options += "Numeric tol          1e-8\n";
    options += "Numeric constr_viol_tol  1e-8\n";
    options += "Numeric acceptable_tol  1e-6\n";

    // place to return solution
    CppAD::ipopt_cppad_result<std::vector<scalar>> solution;

    // solve the problem
    Solve f(_car,v,ax/Dynamic_model_t::acceleration_units,ay/Dynamic_model_t::acceleration_units);
    CppAD::ipopt_cppad_solve<std::vector<scalar>, Solve>(options, x0, x_lb, x_ub, c_lb, c_ub, f, solution);

    // write outputs
    Solve_constraints c(_car,v,ax/Dynamic_model_t::acceleration_units,ay/Dynamic_model_t::acceleration_units);

    typename Solve_constraints::argument_type x;
    std::copy(solution.x.cbegin(), solution.x.cend(), x.begin());
    c(x);
    std::array<CppAD::AD<scalar>,Dynamic_model_t::number_of_inputs>      inputs = c.get_inputs();
    std::array<CppAD::AD<scalar>,Dynamic_model_t::number_of_controls>    controls = c.get_controls();
    auto [states,dstates_dt] = _car(inputs,controls,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::number_of_inputs> inputs_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_inputs; ++i)
    {
        inputs_sc[i] = Value(inputs[i]);
    }

    std::array<scalar,Dynamic_model_t::number_of_controls> controls_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_controls; ++i)
    {
        controls_sc[i] = Value(controls[i]);
    }

    std::array<scalar,Dynamic_model_t::number_of_states> dstates_dt_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_states; ++i)
    {
        dstates_dt_sc[i] = Value(dstates_dt[i]);
    }

    return { solution.status == CppAD::ipopt_cppad_result<std::vector<scalar>>::success, v, ax, ay, inputs_sc, controls_sc, dstates_dt_sc };
}

template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,scalar>::value,typename Steady_state<Dynamic_model_t>::Solution> 
    Steady_state<Dynamic_model_t>::solve_max_lat_acc(scalar v)
{
    Max_lat_acc_fitness f;
    Max_lat_acc_constraints c(_car,v);
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax, ay]

    // Get the solution with ax = ay = 0 as initial point
    auto result_0g = solve(v,0.0,0.0);

    std::vector<scalar> x0 = _car.get_x(result_0g.inputs, result_0g.controls, v);
    x0.push_back(0.0);
    x0.push_back(0.0);

    // Solve the problem using the optimizer
    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-2.0); x_lb.push_back(-20.0);
    x_ub.push_back( 2.0); x_ub.push_back( 20.0);

    auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

    Optimise_options options;
    auto result = Optimise<Max_lat_acc_fitness,Max_lat_acc_constraints>::optimise(Dynamic_model_t::number_of_steady_state_variables+2,Dynamic_model_t::number_of_steady_state_equations,x0,f,c,x_lb,x_ub,c_lb,c_ub,options);

    typename Max_lat_acc_constraints::argument_type x;
    std::copy(result.x.cbegin(), result.x.cend(), x.begin());
    c(x);
    std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> inputs = c.get_inputs();
    std::array<Timeseries_t,Dynamic_model_t::number_of_controls> controls = c.get_controls();
    auto [states,dstates_dt] = _car(inputs,controls,0.0);

    return { result.solved, v, result.x[Dynamic_model_t::number_of_steady_state_variables], result.x[Dynamic_model_t::number_of_steady_state_variables+1], inputs, controls, dstates_dt };
}

template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,typename Steady_state<Dynamic_model_t>::Solution> 
    Steady_state<Dynamic_model_t>::solve_max_lat_acc(scalar v)
{
    // The content of x is: x = [x, ax, ay]

    // Get the solution with ax = ay = 0 as initial point
    auto result_0g = solve(v,0.0,0.0);

    std::vector<scalar> x0 = _car.get_x(result_0g.inputs, result_0g.controls, v);
    x0.push_back(0.0);
    x0.push_back(0.0);

    // Solve the problem using the optimizer
    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-2.0/Dynamic_model_t::acceleration_units); x_lb.push_back(-2.0/Dynamic_model_t::acceleration_units);
    x_ub.push_back( 20.0/Dynamic_model_t::acceleration_units); x_ub.push_back( 20.0/Dynamic_model_t::acceleration_units);

    auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    options += "Numeric tol          1e-8\n";
    options += "Numeric constr_viol_tol  1e-8\n";
    options += "Numeric acceptable_tol  1e-6\n";

    // place to return solution
    CppAD::ipopt_cppad_result<std::vector<scalar>> solution;

    // solve the problem
    Max_lat_acc f(_car,v);

    bool success = false;
    for (size_t attempt = 0; attempt < 32; ++attempt)
    {
        CppAD::ipopt_cppad_solve<std::vector<scalar>, Max_lat_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f, solution);

        // Check if the solution is close to the bounds imposed in acceleration, repeat otherwise
        success = true;

        if ( std::abs(x_lb[Dynamic_model_t::number_of_steady_state_variables] - solution.x[Dynamic_model_t::number_of_steady_state_variables]) < 1.0e-1 )
        {
            success = false;
            x_lb[Dynamic_model_t::number_of_steady_state_variables] -= 0.25*(x_ub[Dynamic_model_t::number_of_steady_state_variables] - x_lb[Dynamic_model_t::number_of_steady_state_variables]);

            x0 = solution.x;
        }

        if ( std::abs(x_ub[Dynamic_model_t::number_of_steady_state_variables] - solution.x[Dynamic_model_t::number_of_steady_state_variables]) < 1.0e-1 )
        {
            success = false;
            x_ub[Dynamic_model_t::number_of_steady_state_variables] += 0.25*(x_ub[Dynamic_model_t::number_of_steady_state_variables] - x_lb[Dynamic_model_t::number_of_steady_state_variables]);

            x0 = solution.x;
        }

        if ( std::abs(x_lb[Dynamic_model_t::number_of_steady_state_variables+1] - solution.x[Dynamic_model_t::number_of_steady_state_variables+1]) < 1.0e-1 )
        {
            success = false;
            x_lb[Dynamic_model_t::number_of_steady_state_variables+1] -= 0.25*(x_ub[Dynamic_model_t::number_of_steady_state_variables+1] - x_lb[Dynamic_model_t::number_of_steady_state_variables+1]);

            x0 = solution.x;
        }

        if ( std::abs(x_ub[Dynamic_model_t::number_of_steady_state_variables+1] - solution.x[Dynamic_model_t::number_of_steady_state_variables+1]) < 1.0e-1 )
        {
            success = false;
            x_ub[Dynamic_model_t::number_of_steady_state_variables+1] += 0.25*(x_ub[Dynamic_model_t::number_of_steady_state_variables+1] - x_lb[Dynamic_model_t::number_of_steady_state_variables+1]);

            x0 = solution.x;
        }
    
        if ( success ) break;

        // Set the new starting point as the final point in the previous attempt
        //x0 = solution.x;
    }



    // write outputs
    Max_lat_acc_constraints c(_car,v);

    typename Max_lat_acc_constraints::argument_type x;
    std::copy(solution.x.cbegin(), solution.x.cend(), x.begin());
    auto constraints = c(x);
    std::array<CppAD::AD<scalar>,Dynamic_model_t::number_of_inputs> inputs = c.get_inputs();
    std::array<CppAD::AD<scalar>,Dynamic_model_t::number_of_controls> controls = c.get_controls();
    auto [states,dstates_dt] = _car(inputs,controls,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::number_of_inputs> inputs_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_inputs; ++i)
    {
        inputs_sc[i] = Value(inputs[i]);
    }

    std::array<scalar,Dynamic_model_t::number_of_controls> controls_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_controls; ++i)
    {
        controls_sc[i] = Value(controls[i]);
    }

    std::array<scalar,Dynamic_model_t::number_of_states> dstates_dt_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_states; ++i)
    {
        dstates_dt_sc[i] = Value(dstates_dt[i]);
    }

    if ( solution.status != CppAD::ipopt_cppad_result<std::vector<scalar>>::success )
    {
        std::cout << "solve_max_lat_acc -> Ipopt was not successful" << std::endl;

        std::cout << std::setprecision(16);
        for (size_t i = 0; i < x.size(); ++i)
        {
            std::cout << x_lb[i] << " < x[" << i << "]: " << x[i] << " < " << x_ub[i] << std::endl;
        }
        for (size_t i = 0; i < constraints.size(); ++i)
        {
            std::cout << c_lb[i] << " < c[" << i << "]: " << constraints[i] << " < " << c_ub[i] << std::endl;
        }

    }

    return { solution.status == CppAD::ipopt_cppad_result<std::vector<scalar>>::success, v, Value(solution.x[Dynamic_model_t::number_of_steady_state_variables]*Dynamic_model_t::acceleration_units), Value(solution.x[Dynamic_model_t::number_of_steady_state_variables+1]*Dynamic_model_t::acceleration_units), inputs_sc, controls_sc, dstates_dt_sc };
}


template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,scalar>::value,std::pair<typename Steady_state<Dynamic_model_t>::Solution, typename Steady_state<Dynamic_model_t>::Solution>>
    Steady_state<Dynamic_model_t>::solve_max_lon_acc(scalar v, scalar ay)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax]
    Optimise_options options;

    // (1)
    // Get the solution with ax = ay = 0 as initial point
    auto result_0g = solve(v,0.0,0.0);

    // (2)
    // Compute the maximum lateral acceleration, and its corresponding longitudinal
    auto result_max_lat_acc = solve_max_lat_acc(v);

    // Check that the lateral acceleration is lower than the maximum
    if ( ay > result_max_lat_acc.ay )
        throw fastest_lap_exception("Acceleraton provided is higher than the maximum achievable");

    // (3)
    // Compute the steady state with ax = ax_aymax.ay/ay_max
    std::vector<scalar> x0_ss_ay = _car.get_x(result_0g.inputs, result_0g.controls, v);

    auto result_ss_ay = solve(v,result_max_lat_acc.ax*ay/result_max_lat_acc.ay, ay, 1, true, x0_ss_ay);
    
    // (4)
    // Optimise using the last optimization
    std::vector<scalar> x0 = _car.get_x(result_ss_ay.inputs, result_ss_ay.controls, v);
    x0.push_back(result_ss_ay.ax/Dynamic_model_t::acceleration_units);

    Max_lon_acc_fitness fmax;
    Min_lon_acc_fitness fmin;
    Max_lon_acc_constraints c(_car,v,ay);

    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-2.0);
    x_ub.push_back(10.0);

    auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

    // Solve maximum acceleration
    auto result_max = Optimise<Max_lon_acc_fitness,Max_lon_acc_constraints>::optimise(Dynamic_model_t::number_of_steady_state_variables+1,Dynamic_model_t::number_of_steady_state_equations,x0,fmax,c,x_lb,x_ub,c_lb,c_ub,options);

    typename Max_lon_acc_constraints::argument_type x_max;
    std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
    c(x_max);
    std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> inputs_max = c.get_inputs();
    std::array<Timeseries_t,Dynamic_model_t::number_of_controls> controls_max = c.get_controls();
    auto [states_max,dstates_dt_max] = _car(inputs_max,controls_max,0.0);

    Solution solution_max = {result_max.solved, v, result_max.x[Dynamic_model_t::number_of_steady_state_variables], ay, inputs_max, controls_max, dstates_dt_max};

    // Solve minimum acceleration
    std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-10.0);
    x_ub.push_back( 2.0);

    auto result_min = Optimise<Min_lon_acc_fitness,Max_lon_acc_constraints>::optimise(Dynamic_model_t::number_of_steady_state_variables+1,Dynamic_model_t::number_of_steady_state_equations,x0,fmin,c,x_lb,x_ub,c_lb,c_ub,options);

    typename Max_lon_acc_constraints::argument_type x_min;
    std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
    c(x_min);
    std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> inputs_min = c.get_inputs();
    std::array<Timeseries_t,Dynamic_model_t::number_of_controls> controls_min = c.get_controls();
    auto [states_min,dstates_dt_min] = _car(inputs_min,controls_min,0.0);

    Solution solution_min = {result_min.solved, v, result_min.x[Dynamic_model_t::number_of_steady_state_variables], ay, inputs_min, controls_min, dstates_dt_min};

    return {solution_max, solution_min};
}

template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,std::pair<typename Steady_state<Dynamic_model_t>::Solution, typename Steady_state<Dynamic_model_t>::Solution>>
    Steady_state<Dynamic_model_t>::solve_max_lon_acc(scalar v, scalar ay)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax]

    // (1)
    // Get the solution with ax = ay = 0 as initial point
    auto result_0g = solve(v,0.0,0.0);

    // (2)
    // Compute the maximum lateral acceleration, and its corresponding longitudinal
    auto result_max_lat_acc = solve_max_lat_acc(v);

    // Check that the lateral acceleration is lower than the maximum
    if ( ay > result_max_lat_acc.ay )
        throw fastest_lap_exception("Acceleraton provided is higher than the maximum achievable");

    // (3)
    // Compute the steady state with ax = ax_aymax.ay/ay_max
    std::vector<scalar> x0_ss_ay = _car.get_x(result_0g.inputs, result_0g.controls, v);

    auto result_ss_ay = solve(v,result_max_lat_acc.ax*ay/result_max_lat_acc.ay, ay, 1, true, x0_ss_ay);

    // (4)
    // Optimise using the last optimization
    std::vector<scalar> x0 = _car.get_x(result_ss_ay.inputs,result_ss_ay.controls, v);
    x0.push_back(result_ss_ay.ax/Dynamic_model_t::acceleration_units);

    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds_accelerate();
    x_lb.push_back(-2.0/Dynamic_model_t::acceleration_units);
    x_ub.push_back(10.0/Dynamic_model_t::acceleration_units);

    auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    options += "Numeric tol          1e-8\n";
    options += "Numeric constr_viol_tol  1e-8\n";
    options += "Numeric acceptable_tol  1e-6\n";
    options += "Integer max_iter 5000\n";

    // place to return solution
    CppAD::ipopt_cppad_result<std::vector<scalar>> result_max;

    // solve the problem
    Max_lon_acc f_max(_car,v,ay/Dynamic_model_t::acceleration_units);

    bool success = false;
    for (size_t attempt = 0; attempt < 8; ++attempt)
    {
        CppAD::ipopt_cppad_solve<std::vector<scalar>, Max_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_max, result_max);

        // Check if the solution is close to the bounds imposed in acceleration, repeat otherwise
        success = true;

        if ( std::abs(x_lb[Dynamic_model_t::number_of_steady_state_variables] - result_max.x[Dynamic_model_t::number_of_steady_state_variables]) < 1.0e-2 )
        {
            success = false;
            x_lb[Dynamic_model_t::number_of_steady_state_variables] *= 2.0;
        }

        if ( std::abs(x_ub[Dynamic_model_t::number_of_steady_state_variables] - result_max.x[Dynamic_model_t::number_of_steady_state_variables]) < 2.0e-1 )
        {
            success = false;
            x_ub[Dynamic_model_t::number_of_steady_state_variables] *= 1.2;
        }

        if ( success ) break;
    }

    Max_lon_acc_constraints c(_car,v,ay/Dynamic_model_t::acceleration_units);
    typename Max_lon_acc_constraints::argument_type x_max;
    std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
    auto constraints = c(x_max);
    std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> inputs_max = c.get_inputs();
    std::array<Timeseries_t,Dynamic_model_t::number_of_controls> controls_max = c.get_controls();
    auto [states_max,dstates_dt_max] = _car(inputs_max,controls_max,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::number_of_inputs> inputs_max_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_inputs; ++i)
    {
        inputs_max_sc[i] = Value(inputs_max[i]);
    }

    std::array<scalar,Dynamic_model_t::number_of_controls> controls_max_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_controls; ++i)
    {
        controls_max_sc[i] = Value(controls_max[i]);
    }

    std::array<scalar,Dynamic_model_t::number_of_states> dstates_dt_max_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_states; ++i)
    {
        dstates_dt_max_sc[i] = Value(dstates_dt_max[i]);
    }

    if ( result_max.status != CppAD::ipopt_cppad_result<std::vector<scalar>>::success )
    {
        std::cout << "solve_max_lon_acc -> Ipopt was not successful" << std::endl;
        const auto& x = result_max.x;

        std::cout << std::setprecision(16);
        for (size_t i = 0; i < x.size(); ++i)
        {
            std::cout << x_lb[i] << " < x[" << i << "]: " << x[i] << " < " << x_ub[i] << std::endl;
        }
        for (size_t i = 0; i < constraints.size(); ++i)
        {
            std::cout << c_lb[i] << " < c[" << i << "]: " << constraints[i] << " < " << c_ub[i] << std::endl;
        }

    }

    const bool max_solved = result_max.status == CppAD::ipopt_cppad_result<std::vector<scalar>>::success;
    Solution solution_max = {max_solved, v, Value(result_max.x[Dynamic_model_t::number_of_steady_state_variables])*Dynamic_model_t::acceleration_units, ay, inputs_max_sc, controls_max_sc, dstates_dt_max_sc};

    // Solve minimum acceleration
    std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds_brake();
    x_lb.push_back(result_max_lat_acc.ax/Dynamic_model_t::acceleration_units-10.0/Dynamic_model_t::acceleration_units);
    x_ub.push_back(result_max_lat_acc.ax/Dynamic_model_t::acceleration_units);

    if ( (x_ub.back()-1.0e-3) < x_lb.back() )
    {
        x_lb.back() = x_ub.back() - 10.0/Dynamic_model_t::acceleration_units;
    }


    // Restore the initial point
    x0 = _car.get_x(result_ss_ay.inputs, result_ss_ay.controls, v);
    x0.push_back(result_ss_ay.ax/Dynamic_model_t::acceleration_units);

    // place to return solution
    CppAD::ipopt_cppad_result<std::vector<scalar>> result_min;

    // solve the problem
    Min_lon_acc f_min(_car,v,ay/Dynamic_model_t::acceleration_units);
    success = false;
    for (size_t attempt = 0; attempt < 30; ++attempt)
    {
        CppAD::ipopt_cppad_solve<std::vector<scalar>, Min_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_min, result_min);

        // Check if the solution is close to the bounds imposed in acceleration, repeat otherwise
        success = (result_min.status == CppAD::ipopt_cppad_result<std::vector<scalar>>::success);

        if ( std::abs(x_lb[Dynamic_model_t::number_of_steady_state_variables] - result_min.x[Dynamic_model_t::number_of_steady_state_variables]) < 1.0 )
        {
            success = false;
            x_lb[Dynamic_model_t::number_of_steady_state_variables] -= 0.25*(x_ub[Dynamic_model_t::number_of_steady_state_variables] - x_lb[Dynamic_model_t::number_of_steady_state_variables]);

            x0 = result_min.x;
        }

        if ( success ) break;
    }


    typename Max_lon_acc_constraints::argument_type x_min;
    std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
    constraints = c(x_min);
    std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> inputs_min = c.get_inputs();
    std::array<Timeseries_t,Dynamic_model_t::number_of_controls> controls_min = c.get_controls();
    auto [states_min,dstates_dt_min] = _car(inputs_min,controls_min,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::number_of_inputs> inputs_min_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_inputs; ++i)
    {
        inputs_min_sc[i] = Value(inputs_min[i]);
    }

    std::array<scalar,Dynamic_model_t::number_of_controls> controls_min_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_controls; ++i)
    {
        controls_min_sc[i] = Value(controls_min[i]);
    }

    std::array<scalar,Dynamic_model_t::number_of_states> dstates_dt_min_sc;
    for (size_t i = 0; i < Dynamic_model_t::number_of_states; ++i)
    {
        dstates_dt_min_sc[i] = Value(dstates_dt_min[i]);
    }

    if ( result_min.status != CppAD::ipopt_cppad_result<std::vector<scalar>>::success )
    {
        std::cout << "solve_max_lon_acc -> Ipopt was not successful" << std::endl;
        std::cout << result_min.status << std::endl;
        const auto& x = result_min.x;

        std::cout << std::setprecision(16);
        for (size_t i = 0; i < x.size(); ++i)
        {
            std::cout << x_lb[i] << " < x[" << i << "]: " << x[i] << " < " << x_ub[i] << std::endl;
        }
        for (size_t i = 0; i < constraints.size(); ++i)
        {
            std::cout << c_lb[i] << " < c[" << i << "]: " << constraints[i] << " < " << c_ub[i] << std::endl;
        }

        std::cout << "lambda = " << result_min.lambda << std::endl;
        std::cout << "g = " << result_min.g << std::endl;
        std::cout << "zl = " << result_min.zl << std::endl;
        std::cout << "zu = " << result_min.zu << std::endl;
        std::cout << "s = " << result_min.s << std::endl;
        std::cout << "vl = " << result_min.vl << std::endl;
        std::cout << "vu = " << result_min.vu << std::endl;
    }

    const bool min_solved = result_min.status == CppAD::ipopt_cppad_result<std::vector<scalar>>::success;
    Solution solution_min = {min_solved, v, Value(result_min.x[Dynamic_model_t::number_of_steady_state_variables])*Dynamic_model_t::acceleration_units, ay, inputs_min_sc, controls_min_sc, dstates_dt_min_sc};

    return {solution_max, solution_min};
}


template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,scalar>::value,
    std::pair<std::vector<typename Steady_state<Dynamic_model_t>::Solution>, 
              std::vector<typename Steady_state<Dynamic_model_t>::Solution>>> 
    Steady_state<Dynamic_model_t>::gg_diagram(scalar v, const size_t n_points)
{
    // Initialize outputs
    std::vector<Solution> solution_max(n_points);
    std::vector<Solution> solution_min(n_points);

    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax]
    Optimise_options options;

    // (1)
    // Get the solution with ax = ay = 0 as initial point
    auto result_0g = solve(v,0.0,0.0);

    // (2)
    // Compute the maximum lateral acceleration, and its corresponding longitudinal. Create vector of lateral accelerations
    auto result_max_lat_acc = solve_max_lat_acc(v);

    std::vector<scalar> ay_gg = linspace(0.0,result_max_lat_acc.ay,n_points);

    // (3)
    // Loop on the requested lateral accelerations
    std::vector<scalar> x0_ss_ay = _car.get_x(result_0g.inputs, result_0g.controls, v);

    Solution result_ss_ay = result_0g;

    for (size_t i = 0; i < n_points; ++i)
    {
        out(2).progress_bar("g-g diagram computation: ", i, n_points);
        auto result_ss_ay_candidate = solve(v,result_max_lat_acc.ax*ay_gg[i]/result_max_lat_acc.ay, ay_gg[i], 1, true, x0_ss_ay, false);

        if ( result_ss_ay_candidate.solved )
        {
            result_ss_ay = result_ss_ay_candidate;
        }
    
    
        // (4)
        // Optimise using the last optimization
        std::vector<scalar> x0 = _car.get_x(result_ss_ay.inputs, result_ss_ay.controls, v);
        x0.push_back(result_ss_ay.ax);

        Max_lon_acc_fitness fmax;
        Min_lon_acc_fitness fmin;
        Max_lon_acc_constraints c(_car,v,ay_gg[i]);
    
        auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
        x_lb.push_back(result_max_lat_acc.ax - 0.5);
        x_ub.push_back(8.0);

        auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();
    
        // Solve maximum acceleration
        auto result_max = Optimise<Max_lon_acc_fitness,Max_lon_acc_constraints>::optimise(Dynamic_model_t::number_of_steady_state_variables+1,Dynamic_model_t::number_of_steady_state_equations,x0,fmax,c,x_lb,x_ub,c_lb,c_ub,options);
    
        typename Max_lon_acc_constraints::argument_type x_max;
        std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
        c(x_max);
        std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> inputs_max = c.get_inputs();
        std::array<Timeseries_t,Dynamic_model_t::number_of_controls> controls_max = c.get_controls();
        auto [states_max,dstates_dt_max] = _car(inputs_max,controls_max,0.0);
    
        solution_max[i] = {result_max.solved, v, result_max.x[Dynamic_model_t::number_of_steady_state_variables], ay_gg[i], inputs_max, controls_max, dstates_dt_max};
    
        // Solve minimum acceleration
        std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds();
        x_lb.push_back(-8.0);
        x_ub.push_back(result_max_lat_acc.ax + 0.5);

        auto result_min = Optimise<Min_lon_acc_fitness,Max_lon_acc_constraints>::optimise(Dynamic_model_t::number_of_steady_state_variables+1,Dynamic_model_t::number_of_steady_state_equations,x0,fmin,c,x_lb,x_ub,c_lb,c_ub,options);
    
        typename Max_lon_acc_constraints::argument_type x_min;
        std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
        c(x_min);
        std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> inputs_min = c.get_inputs();
        std::array<Timeseries_t,Dynamic_model_t::number_of_controls> controls_min = c.get_controls();
        auto [states_min,dstates_dt_min] = _car(inputs_min,controls_min,0.0);
    
        solution_min[i] = {result_min.solved, v, result_min.x[Dynamic_model_t::number_of_steady_state_variables], ay_gg[i], inputs_min, controls_min, dstates_dt_min};

        std::vector<scalar> x0_ss_ay = _car.get_x(result_ss_ay.inputs, result_ss_ay.controls, v);
    }

    out(2).stop_progress_bar();

    return {solution_max, solution_min};
} 


template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,
    std::pair<std::vector<typename Steady_state<Dynamic_model_t>::Solution>, 
              std::vector<typename Steady_state<Dynamic_model_t>::Solution>>> 
    Steady_state<Dynamic_model_t>::gg_diagram(scalar v, const size_t n_points)
{
    // Initialize outputs
    std::vector<Solution> solution_max(n_points);
    std::vector<Solution> solution_min(n_points);

    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax]

    // (1)
    // Get the solution with ax = ay = 0 as initial point
    auto result_0g = solve(v,0.0,0.0);

    // (2)
    // Compute the maximum lateral acceleration, and its corresponding longitudinal. Create vector of lateral accelerations
    auto result_max_lat_acc = solve_max_lat_acc(v);

    std::vector<scalar> ay_gg = linspace(0.0,result_max_lat_acc.ay,n_points);

    // (3) 
    // Compute the maximum longitudinal acceleration at 0g-lateral
    auto [result_max_lon_acc,result_min_lon_acc] = solve_max_lon_acc(v,0.0);

    // (3)
    // Loop on the requested lateral accelerations
    std::vector<scalar> x0_ss_ay = _car.get_x(result_0g.inputs, result_0g.controls, v);

    Solution result_ss_ay = result_0g;

    for (size_t i = 0; i < n_points-1; ++i)
    {
        out(2).progress_bar("g-g diagram computation: ", i, n_points);
        auto result_ss_ay_candidate = solve(v,result_max_lat_acc.ax*ay_gg[i]/result_max_lat_acc.ay, ay_gg[i], 1, true, x0_ss_ay, false);

        if ( result_ss_ay_candidate.solved )
        {
            result_ss_ay = result_ss_ay_candidate;
        }
    
        // (4)
        // Optimise using the last optimization
        std::vector<scalar> x0 = _car.get_x(result_ss_ay.inputs, result_ss_ay.controls, v);
        x0.push_back(result_ss_ay.ax/Dynamic_model_t::acceleration_units);

        auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds_accelerate();
        x_lb.push_back(result_max_lat_acc.ax/Dynamic_model_t::acceleration_units-0.5/Dynamic_model_t::acceleration_units);
        x_ub.push_back(result_max_lon_acc.ax/Dynamic_model_t::acceleration_units+0.1/Dynamic_model_t::acceleration_units);

        auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

        // options
        std::string options;
        // turn off any printing
        options += "Integer print_level  0\n";
        options += "String  sb           yes\n";
        options += "Numeric tol          1e-8\n";
        options += "Numeric constr_viol_tol  1e-8\n";
        options += "Numeric acceptable_tol  1e-6\n";

        // place to return solution
        CppAD::ipopt_cppad_result<std::vector<scalar>> result_max;

        // solve the problem
        Max_lon_acc f_max(_car,v,ay_gg[i]/Dynamic_model_t::acceleration_units);
        CppAD::ipopt_cppad_solve<std::vector<scalar>, Max_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_max, result_max);

        Max_lon_acc_constraints c(_car,v,ay_gg[i]/Dynamic_model_t::acceleration_units);
        typename Max_lon_acc_constraints::argument_type x_max;
        std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
        const auto constraints_max = c(x_max);
        std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> inputs_max = c.get_inputs();
        std::array<Timeseries_t,Dynamic_model_t::number_of_controls> controls_max = c.get_controls();
        auto [states_max, dstates_dt_max] = _car(inputs_max,controls_max,0.0);

        // Transform all AD to scalar
        std::array<scalar,Dynamic_model_t::number_of_inputs> inputs_max_sc;
        for (size_t i = 0; i < Dynamic_model_t::number_of_inputs; ++i)
        {
            inputs_max_sc[i] = Value(inputs_max[i]);
        }

        std::array<scalar,Dynamic_model_t::number_of_controls> controls_max_sc;
        for (size_t i = 0; i < Dynamic_model_t::number_of_controls; ++i)
        {
            controls_max_sc[i] = Value(controls_max[i]);
        }
    
        std::array<scalar,Dynamic_model_t::number_of_states> dstates_dt_max_sc;
        for (size_t i = 0; i < Dynamic_model_t::number_of_states; ++i)
        {
            dstates_dt_max_sc[i] = Value(dstates_dt_max[i]);
        }

        if ( result_max.status != CppAD::ipopt_cppad_result<std::vector<scalar>>::success )
        {
            std::cout << "gg_diagram -> Ipopt was not successful" << std::endl;
            std::cout << "Error code: " << result_max.status << std::endl;
            const auto& x = result_max.x;
    
            std::cout << std::setprecision(16);
            for (size_t i = 0; i < x.size(); ++i)
            {
                std::cout << x_lb[i] << " < x[" << i << "]: " << x[i] << " < " << x_ub[i] << std::endl;
            }
            for (size_t i = 0; i < constraints_max.size(); ++i)
            {
                std::cout << c_lb[i] << " < c[" << i << "]: " << constraints_max[i] << " < " << c_ub[i] << std::endl;
            }
    
        }
    
        const bool max_solved = result_max.status == CppAD::ipopt_cppad_result<std::vector<scalar>>::success;
        solution_max[i] = {max_solved, v, Value(result_max.x[Dynamic_model_t::number_of_steady_state_variables])*Dynamic_model_t::acceleration_units, ay_gg[i], inputs_max_sc, controls_max_sc, dstates_dt_max_sc};

        // Solve minimum acceleration
        std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds_brake();
        x_lb.push_back(result_min_lon_acc.ax/Dynamic_model_t::acceleration_units-0.1/Dynamic_model_t::acceleration_units);
        x_ub.push_back(result_max_lat_acc.ax/Dynamic_model_t::acceleration_units+0.1/Dynamic_model_t::acceleration_units);

        // place to return solution
        CppAD::ipopt_cppad_result<std::vector<scalar>> result_min;
    
        // solve the problem
        Min_lon_acc f_min(_car,v,ay_gg[i]/Dynamic_model_t::acceleration_units);

        CppAD::ipopt_cppad_solve<std::vector<scalar>, Min_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_min, result_min);

        if ( result_min.status != CppAD::ipopt_cppad_result<std::vector<scalar>>::success )
        {
            // Second attempt using the previous solution as initial point
            if ( i > 0 )
            {
                auto x = _car.get_x(solution_min[i-1].inputs, solution_min[i-1].controls, v);
                x.push_back(solution_min[i-1].ax/Dynamic_model_t::acceleration_units);
                CppAD::ipopt_cppad_solve<std::vector<scalar>, Min_lon_acc>(options, x, x_lb, x_ub, c_lb, c_ub, f_min, result_min);
            }
        }

        typename Max_lon_acc_constraints::argument_type x_min;
        std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
        auto constraints = c(x_min);
        std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> inputs_min = c.get_inputs();
        std::array<Timeseries_t,Dynamic_model_t::number_of_controls> controls_min = c.get_controls();
        auto [states_min,dstates_dt_min] = _car(inputs_min,controls_min,0.0);
    
        // Transform all AD to scalar
        std::array<scalar,Dynamic_model_t::number_of_inputs> inputs_min_sc;
        for (size_t i = 0; i < Dynamic_model_t::number_of_inputs; ++i)
        {
            inputs_min_sc[i] = Value(inputs_min[i]);
        }

        std::array<scalar,Dynamic_model_t::number_of_controls> controls_min_sc;
        for (size_t i = 0; i < Dynamic_model_t::number_of_controls; ++i)
        {
            controls_min_sc[i] = Value(controls_min[i]);
        }
    
        std::array<scalar,Dynamic_model_t::number_of_states> dstates_dt_min_sc;
        for (size_t i = 0; i < Dynamic_model_t::number_of_states; ++i)
        {
            dstates_dt_min_sc[i] = Value(dstates_dt_min[i]);
        }

        if ( result_min.status != CppAD::ipopt_cppad_result<std::vector<scalar>>::success )
        {
            std::cout << "gg_diagram -> Ipopt was not successful" << std::endl;
            std::cout << result_min.status << std::endl;
            const auto& x = result_min.x;
    
            std::cout << std::setprecision(16);
            for (size_t i = 0; i < x.size(); ++i)
            {
                std::cout << x_lb[i] << " < x[" << i << "]: " << x[i] << " < " << x_ub[i] << std::endl;
            }
            for (size_t i = 0; i < constraints.size(); ++i)
            {
                std::cout << c_lb[i] << " < c[" << i << "]: " << constraints[i] << " < " << c_ub[i] << std::endl;
            }
    
        }

        const bool min_solved = result_min.status == CppAD::ipopt_cppad_result<std::vector<scalar>>::success;
        solution_min[i] = {min_solved, v, Value(result_min.x[Dynamic_model_t::number_of_steady_state_variables])*Dynamic_model_t::acceleration_units, ay_gg[i], inputs_min_sc, controls_min_sc, dstates_dt_min_sc};

        std::vector<scalar> x0_ss_ay = _car.get_x(result_ss_ay.inputs, result_ss_ay.controls, v);
    }

    // Add the last point corresponding to the maximum lateral acceleration 
    solution_max.back() = result_max_lat_acc;
    solution_min.back() = result_max_lat_acc;

    out(2).stop_progress_bar();

    return {solution_max, solution_min};
} 


template<typename Dynamic_model_t>
typename Steady_state<Dynamic_model_t>::Solve_constraints::output_type Steady_state<Dynamic_model_t>::Solve_constraints::operator()
    (const typename Steady_state<Dynamic_model_t>::Solve_constraints::argument_type& x)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta]
    std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_equations> constraints;
    std::tie(constraints,_inputs,_controls) = _car->steady_state_equations(x,_ax,_ay,_v);

    return constraints;
}

template<typename Dynamic_model_t>
typename Steady_state<Dynamic_model_t>::Max_lat_acc_constraints::output_type Steady_state<Dynamic_model_t>::Max_lat_acc_constraints::operator()
    (const typename Steady_state<Dynamic_model_t>::Max_lat_acc_constraints::argument_type& x)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax, ay]
    const Timeseries_t& ax = x[Dynamic_model_t::number_of_steady_state_variables+0];
    const Timeseries_t& ay = x[Dynamic_model_t::number_of_steady_state_variables+1];

    // Get x
    std::array<Timeseries_t, Dynamic_model_t::number_of_steady_state_variables> x_reduced;  
    std::copy(x.data(), x.data() + Dynamic_model_t::number_of_steady_state_variables, x_reduced.begin());

    std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_equations> constraints;
    std::tie(constraints,_inputs,_controls) = _car->steady_state_equations(x_reduced,ax,ay,_v);

    return constraints;
}

template<typename Dynamic_model_t>
typename Steady_state<Dynamic_model_t>::Max_lon_acc_constraints::output_type Steady_state<Dynamic_model_t>::Max_lon_acc_constraints::operator()
    (const typename Steady_state<Dynamic_model_t>::Max_lon_acc_constraints::argument_type& x)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax]
    const Timeseries_t& ax = x[Dynamic_model_t::number_of_steady_state_variables+0];

    // Get x
    std::array<Timeseries_t, Dynamic_model_t::number_of_steady_state_variables> x_reduced;  
    std::copy(x.data(), x.data() + Dynamic_model_t::number_of_steady_state_variables, x_reduced.begin());

    std::array<Timeseries_t,Dynamic_model_t::number_of_steady_state_equations> constraints;
    std::tie(constraints,_inputs,_controls) = _car->steady_state_equations(x_reduced,ax,_ay,_v);

    return constraints;
}


#endif
