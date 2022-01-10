#ifndef __STEADY_STATE_HPP__
#define __STEADY_STATE_HPP__

#include "lion/math/optimise.h"
#include "lion/math/solve_nonlinear_system.h"
#include "lion/foundation/constants.h"
#include "lion/thirdparty/include/logger.hpp"
#include "lion/thirdparty/include/cppad/ipopt/solve.hpp"

template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,scalar>::value,typename Steady_state<Dynamic_model_t>::Solution> 
    Steady_state<Dynamic_model_t>::solve(scalar v, scalar ax, scalar ay, const size_t n_steps, const bool provide_x0, const std::vector<scalar>& x0_provided, bool throw_if_fail)
{
    std::vector<scalar> x0 = Dynamic_model_t::steady_state_initial_guess();

    if ( provide_x0 )
    {
        assert(x0_provided.size() == Dynamic_model_t::N_SS_VARS);
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
        result = Solve_nonlinear_system<Solve_constraints>::solve(Dynamic_model_t::N_SS_VARS,Dynamic_model_t::N_SS_EQNS,x0,c,x_lb,x_ub,c_lb,c_ub,options);

        // Set x0 for the next iteration
        x0 = result.x;
    }

    Solve_constraints c(_car,v,ax,ay);

    typename Solve_constraints::argument_type x;
    std::copy(result.x.cbegin(), result.x.cend(), x.begin());
    c(x);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa = c.get_qa();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u = c.get_u();
    auto [dqdt,dqa] = _car(q,qa,u,0.0);

    return { result.solved, v, ax, ay, q, qa, u, dqdt };
}

template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,typename Steady_state<Dynamic_model_t>::Solution> 
    Steady_state<Dynamic_model_t>::solve(scalar v, scalar ax, scalar ay, const size_t, const bool provide_x0, const std::vector<scalar>& x0_provided, bool throw_if_fail)
{
    std::vector<scalar> x0 = Dynamic_model_t::steady_state_initial_guess();

    if ( provide_x0 )
    {
        assert(x0_provided.size() == Dynamic_model_t::N_SS_VARS);
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
    CppAD::ipopt::solve_result<std::vector<scalar>> solution;

    // solve the problem
    Solve f(_car,v,ax,ay);
    CppAD::ipopt::solve<std::vector<scalar>, Solve>(options, x0, x_lb, x_ub, c_lb, c_ub, f, solution);

    // write outputs
    Solve_constraints c(_car,v,ax,ay);

    typename Solve_constraints::argument_type x;
    std::copy(solution.x.cbegin(), solution.x.cend(), x.begin());
    c(x);
    std::array<CppAD::AD<scalar>,Dynamic_model_t::NSTATE>      q = c.get_q();
    std::array<CppAD::AD<scalar>,Dynamic_model_t::NALGEBRAIC> qa = c.get_qa();
    std::array<CppAD::AD<scalar>,Dynamic_model_t::NCONTROL>    u = c.get_u();
    auto [dqdt,dqa] = _car(q,qa,u,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::NSTATE> q_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        q_sc[i] = Value(q[i]);
    }

    std::array<scalar,Dynamic_model_t::NALGEBRAIC> qa_sc;
    for (size_t i = 0; i < Dynamic_model_t::NALGEBRAIC; ++i)
    {
        qa_sc[i] = Value(qa[i]);
    }

    std::array<scalar,Dynamic_model_t::NCONTROL> u_sc;
    for (size_t i = 0; i < Dynamic_model_t::NCONTROL; ++i)
    {
        u_sc[i] = Value(u[i]);
    }

    std::array<scalar,Dynamic_model_t::NSTATE> dqdt_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        dqdt_sc[i] = Value(dqdt[i]);
    }

    return { solution.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success, v, ax, ay, q_sc, qa_sc, u_sc, dqdt_sc };
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

    std::vector<scalar> x0 = Dynamic_model_t::get_x(result_0g.q, result_0g.qa, result_0g.u, v);
    x0.push_back(0.0);
    x0.push_back(0.0);

    // Solve the problem using the optimizer
    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-2.0); x_lb.push_back(-20.0);
    x_ub.push_back( 2.0); x_ub.push_back( 20.0);

    auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

    Optimise_options options;
    auto result = Optimise<Max_lat_acc_fitness,Max_lat_acc_constraints>::optimise(Dynamic_model_t::N_SS_VARS+2,Dynamic_model_t::N_SS_EQNS,x0,f,c,x_lb,x_ub,c_lb,c_ub,options);

    typename Max_lat_acc_constraints::argument_type x;
    std::copy(result.x.cbegin(), result.x.cend(), x.begin());
    c(x);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa = c.get_qa();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u = c.get_u();
    auto [dqdt,dqa] = _car(q,qa,u,0.0);

    return { result.solved, v, result.x[Dynamic_model_t::N_SS_VARS], result.x[Dynamic_model_t::N_SS_VARS+1], q, qa, u, dqdt };
}

template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,typename Steady_state<Dynamic_model_t>::Solution> 
    Steady_state<Dynamic_model_t>::solve_max_lat_acc(scalar v)
{
    // The content of x is: x = [x, ax, ay]

    // Get the solution with ax = ay = 0 as initial point
    auto result_0g = solve(v,0.0,0.0);

    std::vector<scalar> x0 = Dynamic_model_t::get_x(result_0g.q, result_0g.qa, result_0g.u, v);
    x0.push_back(0.0);
    x0.push_back(0.0);

    // Solve the problem using the optimizer
    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-2.0); x_lb.push_back(-20.0);
    x_ub.push_back( 2.0); x_ub.push_back( 20.0);

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
    CppAD::ipopt::solve_result<std::vector<scalar>> solution;

    // solve the problem
    Max_lat_acc f(_car,v);

    bool success = false;
    for (size_t attempt = 0; attempt < 6; ++attempt)
    {
        CppAD::ipopt::solve<std::vector<scalar>, Max_lat_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f, solution);

        // Check if the solution is close to the bounds imposed in acceleration, repeat otherwise
        success = true;

        if ( std::abs(x_lb[Dynamic_model_t::N_SS_VARS] - solution.x[Dynamic_model_t::N_SS_VARS]) < 1.0e-3 )
        {
            success = false;
            x_lb[Dynamic_model_t::N_SS_VARS] *= 2.0;
        }

        if ( std::abs(x_ub[Dynamic_model_t::N_SS_VARS] - solution.x[Dynamic_model_t::N_SS_VARS]) < 1.0e-3 )
        {
            success = false;
            x_ub[Dynamic_model_t::N_SS_VARS] *= 2.0;
        }

        if ( std::abs(x_lb[Dynamic_model_t::N_SS_VARS+1] - solution.x[Dynamic_model_t::N_SS_VARS+1]) < 1.0e-3 )
        {
            success = false;
            x_lb[Dynamic_model_t::N_SS_VARS+1] *= 2.0;
        }

        if ( std::abs(x_ub[Dynamic_model_t::N_SS_VARS+1] - solution.x[Dynamic_model_t::N_SS_VARS+1]) < 1.0e-3 )
        {
            success = false;
            x_ub[Dynamic_model_t::N_SS_VARS+1] *= 2.0;
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
    std::array<CppAD::AD<scalar>,Dynamic_model_t::NSTATE> q = c.get_q();
    std::array<CppAD::AD<scalar>,Dynamic_model_t::NALGEBRAIC> qa = c.get_qa();
    std::array<CppAD::AD<scalar>,Dynamic_model_t::NCONTROL> u = c.get_u();
    auto [dqdt,dqa] = _car(q,qa,u,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::NSTATE> q_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        q_sc[i] = Value(q[i]);
    }

    std::array<scalar,Dynamic_model_t::NALGEBRAIC> qa_sc;
    for (size_t i = 0; i < Dynamic_model_t::NALGEBRAIC; ++i)
    {
        qa_sc[i] = Value(qa[i]);
    }

    std::array<scalar,Dynamic_model_t::NCONTROL> u_sc;
    for (size_t i = 0; i < Dynamic_model_t::NCONTROL; ++i)
    {
        u_sc[i] = Value(u[i]);
    }

    std::array<scalar,Dynamic_model_t::NSTATE> dqdt_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        dqdt_sc[i] = Value(dqdt[i]);
    }

    if ( solution.status != CppAD::ipopt::solve_result<std::vector<scalar>>::success )
    {
        std::cout << "Ipopt was not successful" << std::endl;

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

    return { solution.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success, v, Value(solution.x[Dynamic_model_t::N_SS_VARS]), Value(solution.x[Dynamic_model_t::N_SS_VARS+1]), q_sc, qa_sc, u_sc, dqdt_sc };
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
        throw std::runtime_error("Acceleraton provided is higher than the maximum achievable");

    // (3)
    // Compute the steady state with ax = ax_aymax.ay/ay_max
    std::vector<scalar> x0_ss_ay = Dynamic_model_t::get_x(result_0g.q, result_0g.qa, result_0g.u, v);

    auto result_ss_ay = solve(v,result_max_lat_acc.ax*ay/result_max_lat_acc.ay, ay, 1, true, x0_ss_ay);
    
    // (4)
    // Optimise using the last optimization
    std::vector<scalar> x0 = Dynamic_model_t::get_x(result_ss_ay.q, result_ss_ay.qa, result_ss_ay.u, v);
    x0.push_back(result_ss_ay.ax);

    Max_lon_acc_fitness fmax;
    Min_lon_acc_fitness fmin;
    Max_lon_acc_constraints c(_car,v,ay);

    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-2.0);
    x_ub.push_back(10.0);

    auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

    // Solve maximum acceleration
    auto result_max = Optimise<Max_lon_acc_fitness,Max_lon_acc_constraints>::optimise(Dynamic_model_t::N_SS_VARS+1,Dynamic_model_t::N_SS_EQNS,x0,fmax,c,x_lb,x_ub,c_lb,c_ub,options);

    typename Max_lon_acc_constraints::argument_type x_max;
    std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
    c(x_max);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_max = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa_max = c.get_qa();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_max = c.get_u();
    auto [dqdt_max,dqa] = _car(q_max,qa_max,u_max,0.0);

    Solution solution_max = {result_max.solved, v, result_max.x[Dynamic_model_t::N_SS_VARS], ay, q_max, qa_max, u_max, dqdt_max};

    // Solve minimum acceleration
    std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-10.0);
    x_ub.push_back( 2.0);

    auto result_min = Optimise<Min_lon_acc_fitness,Max_lon_acc_constraints>::optimise(Dynamic_model_t::N_SS_VARS+1,Dynamic_model_t::N_SS_EQNS,x0,fmin,c,x_lb,x_ub,c_lb,c_ub,options);

    typename Max_lon_acc_constraints::argument_type x_min;
    std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
    c(x_min);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_min = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa_min = c.get_qa();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_min = c.get_u();
    auto [dqdt_min,dqa_min] = _car(q_min,qa_min,u_min,0.0);

    Solution solution_min = {result_min.solved, v, result_min.x[Dynamic_model_t::N_SS_VARS], ay, q_min, qa_min, u_min, dqdt_min};

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
        throw std::runtime_error("Acceleraton provided is higher than the maximum achievable");

    // (3)
    // Compute the steady state with ax = ax_aymax.ay/ay_max
    std::vector<scalar> x0_ss_ay = Dynamic_model_t::get_x(result_0g.q, result_0g.qa, result_0g.u, v);

    auto result_ss_ay = solve(v,result_max_lat_acc.ax*ay/result_max_lat_acc.ay, ay, 1, true, x0_ss_ay);
    
    // (4)
    // Optimise using the last optimization
    std::vector<scalar> x0 = Dynamic_model_t::get_x(result_ss_ay.q, result_ss_ay.qa, result_ss_ay.u, v);
    x0.push_back(result_ss_ay.ax);

    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds_accelerate();
    x_lb.push_back(-2.0);
    x_ub.push_back(10.0);

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
    CppAD::ipopt::solve_result<std::vector<scalar>> result_max;

    // solve the problem
    Max_lon_acc f_max(_car,v,ay);
    bool success = false;
    for (size_t attempt = 0; attempt < 6; ++attempt)
    {
        CppAD::ipopt::solve<std::vector<scalar>, Max_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_max, result_max);

        // Check if the solution is close to the bounds imposed in acceleration, repeat otherwise
        success = true;

        if ( std::abs(x_lb[Dynamic_model_t::N_SS_VARS] - result_max.x[Dynamic_model_t::N_SS_VARS]) < 1.0e-2 )
        {
            success = false;
            x_lb[Dynamic_model_t::N_SS_VARS] *= 2.0;
        }

        if ( std::abs(x_ub[Dynamic_model_t::N_SS_VARS] - result_max.x[Dynamic_model_t::N_SS_VARS]) < 1.0e-2 )
        {
            success = false;
            x_ub[Dynamic_model_t::N_SS_VARS] *= 2.0;
        }

        if ( success ) break;
    }

    Max_lon_acc_constraints c(_car,v,ay);
    typename Max_lon_acc_constraints::argument_type x_max;
    std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
    auto constraints = c(x_max);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_max = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa_max = c.get_qa();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_max = c.get_u();
    auto [dqdt_max,dqa_max] = _car(q_max,qa_max,u_max,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::NSTATE> q_max_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        q_max_sc[i] = Value(q_max[i]);
    }

    std::array<scalar,Dynamic_model_t::NALGEBRAIC> qa_max_sc;
    for (size_t i = 0; i < Dynamic_model_t::NALGEBRAIC; ++i)
    {
        qa_max_sc[i] = Value(qa_max[i]);
    }

    std::array<scalar,Dynamic_model_t::NCONTROL> u_max_sc;
    for (size_t i = 0; i < Dynamic_model_t::NCONTROL; ++i)
    {
        u_max_sc[i] = Value(u_max[i]);
    }

    std::array<scalar,Dynamic_model_t::NSTATE> dqdt_max_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        dqdt_max_sc[i] = Value(dqdt_max[i]);
    }

    if ( result_max.status != CppAD::ipopt::solve_result<std::vector<scalar>>::success )
    {
        std::cout << "Ipopt was not successful" << std::endl;
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

    const bool max_solved = result_max.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success;
    Solution solution_max = {max_solved, v, Value(result_max.x[Dynamic_model_t::N_SS_VARS]), ay, q_max_sc, qa_max_sc, u_max_sc, dqdt_max_sc};

    // Solve minimum acceleration
    std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds_brake();
    x_lb.push_back(-10.0);
    x_ub.push_back(result_max_lat_acc.ax*ay/result_max_lat_acc.ay);

    if ( (x_ub.back()-1.0e-3) < x_lb.back() )
    {
        x_lb.back() = x_ub.back() - 10.0;
    }


    // Restore the initial point
    x0 = Dynamic_model_t::get_x(result_ss_ay.q, result_ss_ay.qa, result_ss_ay.u, v);
    x0.push_back(result_ss_ay.ax);

    // place to return solution
    CppAD::ipopt::solve_result<std::vector<scalar>> result_min;

    // solve the problem
    Min_lon_acc f_min(_car,v,ay);
    success = false;
    for (size_t attempt = 0; attempt < 6; ++attempt)
    {
        CppAD::ipopt::solve<std::vector<scalar>, Min_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_min, result_min);

        // Check if the solution is close to the bounds imposed in acceleration, repeat otherwise
        success = (result_min.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success);

        if ( std::abs(x_lb[Dynamic_model_t::N_SS_VARS] - result_min.x[Dynamic_model_t::N_SS_VARS]) < 1.0e-2 )
        {
            success = false;
            x_lb[Dynamic_model_t::N_SS_VARS] *= 2.0;
        }

        if ( success ) break;
    }


    typename Max_lon_acc_constraints::argument_type x_min;
    std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
    constraints = c(x_min);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_min = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa_min = c.get_qa();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_min = c.get_u();
    auto [dqdt_min,dqa] = _car(q_min,qa_min,u_min,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::NSTATE> q_min_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        q_min_sc[i] = Value(q_min[i]);
    }

    std::array<scalar,Dynamic_model_t::NALGEBRAIC> qa_min_sc;
    for (size_t i = 0; i < Dynamic_model_t::NALGEBRAIC; ++i)
    {
        qa_min_sc[i] = Value(qa_min[i]);
    }

    std::array<scalar,Dynamic_model_t::NCONTROL> u_min_sc;
    for (size_t i = 0; i < Dynamic_model_t::NCONTROL; ++i)
    {
        u_min_sc[i] = Value(u_min[i]);
    }

    std::array<scalar,Dynamic_model_t::NSTATE> dqdt_min_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        dqdt_min_sc[i] = Value(dqdt_min[i]);
    }

    if ( result_min.status != CppAD::ipopt::solve_result<std::vector<scalar>>::success )
    {
        std::cout << "Ipopt was not successful" << std::endl;
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

    const bool min_solved = result_min.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success;
    Solution solution_min = {min_solved, v, Value(result_min.x[Dynamic_model_t::N_SS_VARS]), ay, q_min_sc, qa_min_sc, u_min_sc, dqdt_min_sc};

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
    std::vector<scalar> x0_ss_ay = Dynamic_model_t::get_x(result_0g.q, result_0g.qa, result_0g.u, v);

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
        std::vector<scalar> x0 = Dynamic_model_t::get_x(result_ss_ay.q, result_ss_ay.qa, result_ss_ay.u, v);
        x0.push_back(result_ss_ay.ax);

        Max_lon_acc_fitness fmax;
        Min_lon_acc_fitness fmin;
        Max_lon_acc_constraints c(_car,v,ay_gg[i]);
    
        auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
        x_lb.push_back(result_max_lat_acc.ax - 0.5);
        x_ub.push_back(8.0);

        auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();
    
        // Solve maximum acceleration
        auto result_max = Optimise<Max_lon_acc_fitness,Max_lon_acc_constraints>::optimise(Dynamic_model_t::N_SS_VARS+1,Dynamic_model_t::N_SS_EQNS,x0,fmax,c,x_lb,x_ub,c_lb,c_ub,options);
    
        typename Max_lon_acc_constraints::argument_type x_max;
        std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
        c(x_max);
        std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_max = c.get_q();
        std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa_max = c.get_qa();
        std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_max = c.get_u();
        auto [dqdt_max,dqa_max] = _car(q_max,qa_max,u_max,0.0);
    
        solution_max[i] = {result_max.solved, v, result_max.x[Dynamic_model_t::N_SS_VARS], ay_gg[i], q_max, qa_max, u_max, dqdt_max};
    
        // Solve minimum acceleration
        std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds();
        x_lb.push_back(-8.0);
        x_ub.push_back(result_max_lat_acc.ax + 0.5);

        auto result_min = Optimise<Min_lon_acc_fitness,Max_lon_acc_constraints>::optimise(Dynamic_model_t::N_SS_VARS+1,Dynamic_model_t::N_SS_EQNS,x0,fmin,c,x_lb,x_ub,c_lb,c_ub,options);
    
        typename Max_lon_acc_constraints::argument_type x_min;
        std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
        c(x_min);
        std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_min = c.get_q();
        std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa_min = c.get_qa();
        std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_min = c.get_u();
        auto [dqdt_min,dqa_min] = _car(q_min,qa_min,u_min,0.0);
    
        solution_min[i] = {result_min.solved, v, result_min.x[Dynamic_model_t::N_SS_VARS], ay_gg[i], q_min, qa_min, u_min, dqdt_min};

        std::vector<scalar> x0_ss_ay = Dynamic_model_t::get_x(result_ss_ay.q, result_ss_ay.qa, result_ss_ay.u, v);
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

    PRINTVARIABLE(JMT, result_max_lat_acc.ax);
    PRINTVARIABLE(JMT, result_max_lat_acc.ay);

    // (3) 
    // Compute the maximum longitudinal acceleration at 0g-lateral
    auto [result_max_lon_acc,result_min_lon_acc] = solve_max_lon_acc(v,0.0);

    // (3)
    // Loop on the requested lateral accelerations
    std::vector<scalar> x0_ss_ay = Dynamic_model_t::get_x(result_0g.q, result_0g.qa, result_0g.u, v);

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
        std::vector<scalar> x0 = Dynamic_model_t::get_x(result_ss_ay.q, result_ss_ay.qa, result_ss_ay.u, v);
        x0.push_back(result_ss_ay.ax);

        auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds_accelerate();
        x_lb.push_back(result_max_lat_acc.ax-0.1);
        x_ub.push_back(result_max_lon_acc.ax+0.1);

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
        CppAD::ipopt::solve_result<std::vector<scalar>> result_max;

        // solve the problem
        Max_lon_acc f_max(_car,v,ay_gg[i]);
        CppAD::ipopt::solve<std::vector<scalar>, Max_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_max, result_max);

        Max_lon_acc_constraints c(_car,v,ay_gg[i]);
        typename Max_lon_acc_constraints::argument_type x_max;
        std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
        c(x_max);
        std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_max = c.get_q();
        std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa_max = c.get_qa();
        std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_max = c.get_u();
        auto [dqdt_max, dqa_max] = _car(q_max,qa_max,u_max,0.0);

        // Transform all AD to scalar
        std::array<scalar,Dynamic_model_t::NSTATE> q_max_sc;
        for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
        {
            q_max_sc[i] = Value(q_max[i]);
        }

        std::array<scalar,Dynamic_model_t::NALGEBRAIC> qa_max_sc;
        for (size_t i = 0; i < Dynamic_model_t::NALGEBRAIC; ++i)
        {
            qa_max_sc[i] = Value(qa_max[i]);
        }
    
    
        std::array<scalar,Dynamic_model_t::NCONTROL> u_max_sc;
        for (size_t i = 0; i < Dynamic_model_t::NCONTROL; ++i)
        {
            u_max_sc[i] = Value(u_max[i]);
        }
    
        std::array<scalar,Dynamic_model_t::NSTATE> dqdt_max_sc;
        for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
        {
            dqdt_max_sc[i] = Value(dqdt_max[i]);
        }
    
        const bool max_solved = result_max.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success;
        solution_max[i] = {max_solved, v, Value(result_max.x[Dynamic_model_t::N_SS_VARS]), ay_gg[i], q_max_sc, qa_max_sc, u_max_sc, dqdt_max_sc};

        // Solve minimum acceleration
        std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds_brake();
        x_lb.push_back(result_min_lon_acc.ax-0.1);
        x_ub.push_back(result_max_lat_acc.ax+0.1);

        // place to return solution
        CppAD::ipopt::solve_result<std::vector<scalar>> result_min;
    
        // solve the problem
        Min_lon_acc f_min(_car,v,ay_gg[i]);
        CppAD::ipopt::solve<std::vector<scalar>, Min_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_min, result_min);

        typename Max_lon_acc_constraints::argument_type x_min;
        std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
        auto constraints = c(x_min);
        std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_min = c.get_q();
        std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa_min = c.get_qa();
        std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_min = c.get_u();
        auto [dqdt_min,dqa_min] = _car(q_min,qa_min,u_min,0.0);
    
        // Transform all AD to scalar
        std::array<scalar,Dynamic_model_t::NSTATE> q_min_sc;
        for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
        {
            q_min_sc[i] = Value(q_min[i]);
        }

        std::array<scalar,Dynamic_model_t::NALGEBRAIC> qa_min_sc;
        for (size_t i = 0; i < Dynamic_model_t::NALGEBRAIC; ++i)
        {
            qa_min_sc[i] = Value(qa_min[i]);
        }
       
        std::array<scalar,Dynamic_model_t::NCONTROL> u_min_sc;
        for (size_t i = 0; i < Dynamic_model_t::NCONTROL; ++i)
        {
            u_min_sc[i] = Value(u_min[i]);
        }
    
        std::array<scalar,Dynamic_model_t::NSTATE> dqdt_min_sc;
        for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
        {
            dqdt_min_sc[i] = Value(dqdt_min[i]);
        }

        if ( result_min.status != CppAD::ipopt::solve_result<std::vector<scalar>>::success )
        {
            std::cout << "Ipopt was not successful" << std::endl;
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

    
        const bool min_solved = result_min.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success;
        solution_min[i] = {min_solved, v, Value(result_min.x[Dynamic_model_t::N_SS_VARS]), ay_gg[i], q_min_sc, qa_min_sc, u_min_sc, dqdt_min_sc};

        std::vector<scalar> x0_ss_ay = Dynamic_model_t::get_x(result_ss_ay.q, result_ss_ay.qa, result_ss_ay.u, v);
    }

    out(2).stop_progress_bar();

    return {solution_max, solution_min};
} 


template<typename Dynamic_model_t>
typename Steady_state<Dynamic_model_t>::Solve_constraints::output_type Steady_state<Dynamic_model_t>::Solve_constraints::operator()
    (const typename Steady_state<Dynamic_model_t>::Solve_constraints::argument_type& x)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta]
    std::array<Timeseries_t,Dynamic_model_t::N_SS_EQNS> constraints;
    std::tie(constraints,_q,_u) = _car->steady_state_equations(x,_ax,_ay,_v);

    return constraints;
}

template<typename Dynamic_model_t>
typename Steady_state<Dynamic_model_t>::Max_lat_acc_constraints::output_type Steady_state<Dynamic_model_t>::Max_lat_acc_constraints::operator()
    (const typename Steady_state<Dynamic_model_t>::Max_lat_acc_constraints::argument_type& x)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax, ay]
    const Timeseries_t& ax = x[Dynamic_model_t::N_SS_VARS+0];
    const Timeseries_t& ay = x[Dynamic_model_t::N_SS_VARS+1];

    // Get x
    std::array<Timeseries_t, Dynamic_model_t::N_SS_VARS> x_reduced;  
    std::copy(x.data(), x.data() + Dynamic_model_t::N_SS_VARS, x_reduced.begin());

    std::array<Timeseries_t,Dynamic_model_t::N_SS_EQNS> constraints;
    std::tie(constraints,_q,_u) = _car->steady_state_equations(x_reduced,ax,ay,_v);

    return constraints;
}

template<typename Dynamic_model_t>
typename Steady_state<Dynamic_model_t>::Max_lon_acc_constraints::output_type Steady_state<Dynamic_model_t>::Max_lon_acc_constraints::operator()
    (const typename Steady_state<Dynamic_model_t>::Max_lon_acc_constraints::argument_type& x)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax]
    const Timeseries_t& ax = x[Dynamic_model_t::N_SS_VARS+0];

    // Get x
    std::array<Timeseries_t, Dynamic_model_t::N_SS_VARS> x_reduced;  
    std::copy(x.data(), x.data() + Dynamic_model_t::N_SS_VARS, x_reduced.begin());

    std::array<Timeseries_t,Dynamic_model_t::N_SS_EQNS> constraints;
    std::tie(constraints,_q,_u) = _car->steady_state_equations(x_reduced,ax,_ay,_v);

    return constraints;
}


#endif
