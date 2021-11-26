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
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta]
    std::vector<scalar> x0 = {0.0, 0.02, 0.0, 0.0, 0.0, 0.0};

    if ( provide_x0 )
    {
        assert(x0_provided.size() == 6);
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
        result = Solve_nonlinear_system<Solve_constraints>::solve(6,12,x0,c,x_lb,x_ub,c_lb,c_ub,options);

        // Set x0 for the next iteration
        x0 = result.x;
    }

    Solve_constraints c(_car,v,ax,ay);

    typename Solve_constraints::argument_type x;
    std::copy(result.x.cbegin(), result.x.cend(), x.begin());
    c(x);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u = c.get_u();
    auto dqdt = _car(q,u,0.0);

    return { result.solved, v, ax, ay, q, u, dqdt };
}

template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,typename Steady_state<Dynamic_model_t>::Solution> 
    Steady_state<Dynamic_model_t>::solve(scalar v, scalar ax, scalar ay, const size_t, const bool provide_x0, const std::vector<scalar>& x0_provided, bool throw_if_fail)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta]
    std::vector<scalar> x0 = {0.0, 0.02, 0.0, 0.0, 0.0, 0.0};

    if ( provide_x0 )
    {
        assert(x0_provided.size() == 6);
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
    std::array<CppAD::AD<scalar>,Dynamic_model_t::NSTATE> q = c.get_q();
    std::array<CppAD::AD<scalar>,Dynamic_model_t::NCONTROL> u = c.get_u();
    auto dqdt = _car(q,u,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::NSTATE> q_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        q_sc[i] = Value(q[i]);
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

    return { solution.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success, v, ax, ay, q_sc, u_sc, dqdt_sc };
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

    std::vector<scalar> x0 = {(result_0g.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                              result_0g.q[Dynamic_model_t::Chassis_type::IZ],
                              result_0g.q[Dynamic_model_t::Chassis_type::IPHI],
                              result_0g.q[Dynamic_model_t::Chassis_type::IMU],
                              result_0g.q[Dynamic_model_t::Road_type::IPSI],
                              result_0g.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING],
                              0.0, 0.0
                             };

    // Solve the problem using the optimizer
    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-2.0); x_lb.push_back(-20.0);
    x_ub.push_back( 2.0); x_ub.push_back( 20.0);

    auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

    Optimise_options options;
    auto result = Optimise<Max_lat_acc_fitness,Max_lat_acc_constraints>::optimise(8,12,x0,f,c,x_lb,x_ub,c_lb,c_ub,options);

    typename Max_lat_acc_constraints::argument_type x;
    std::copy(result.x.cbegin(), result.x.cend(), x.begin());
    c(x);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u = c.get_u();
    auto dqdt = _car(q,u,0.0);

    return { result.solved, v, result.x[6], result.x[7], q, u, dqdt };
}

template<typename Dynamic_model_t>
template<typename T>
std::enable_if_t<std::is_same<T,CppAD::AD<scalar>>::value,typename Steady_state<Dynamic_model_t>::Solution> 
    Steady_state<Dynamic_model_t>::solve_max_lat_acc(scalar v)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax, ay]

    // Get the solution with ax = ay = 0 as initial point
    auto result_0g = solve(v,0.0,0.0);

    std::vector<scalar> x0 = {(result_0g.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                              result_0g.q[Dynamic_model_t::Chassis_type::IZ],
                              result_0g.q[Dynamic_model_t::Chassis_type::IPHI],
                              result_0g.q[Dynamic_model_t::Chassis_type::IMU],
                              result_0g.q[Dynamic_model_t::Road_type::IPSI],
                              result_0g.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING],
                              0.0, 0.0
                             };

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
    CppAD::ipopt::solve<std::vector<scalar>, Max_lat_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f, solution);

    // write outputs
    Max_lat_acc_constraints c(_car,v);

    typename Max_lat_acc_constraints::argument_type x;
    std::copy(solution.x.cbegin(), solution.x.cend(), x.begin());
    c(x);
    std::array<CppAD::AD<scalar>,Dynamic_model_t::NSTATE> q = c.get_q();
    std::array<CppAD::AD<scalar>,Dynamic_model_t::NCONTROL> u = c.get_u();
    auto dqdt = _car(q,u,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::NSTATE> q_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        q_sc[i] = Value(q[i]);
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

    return { solution.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success, v, Value(solution.x[6]), Value(solution.x[7]), q_sc, u_sc, dqdt_sc };
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
    std::vector<scalar> x0_ss_ay = {(result_0g.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                              result_0g.q[Dynamic_model_t::Chassis_type::IZ],
                              result_0g.q[Dynamic_model_t::Chassis_type::IPHI],
                              result_0g.q[Dynamic_model_t::Chassis_type::IMU],
                              result_0g.q[Dynamic_model_t::Road_type::IPSI],
                              result_0g.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING] 
                             };

    auto result_ss_ay = solve(v,result_max_lat_acc.ax*ay/result_max_lat_acc.ay, ay, 1, true, x0_ss_ay);
    
    // (4)
    // Optimise using the last optimization
    std::vector<scalar> x0 = {(result_ss_ay.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IZ],
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IPHI],
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IMU],
                              result_ss_ay.q[Dynamic_model_t::Road_type::IPSI],
                              result_ss_ay.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING],
                              result_ss_ay.ax
                             };

    Max_lon_acc_fitness fmax;
    Min_lon_acc_fitness fmin;
    Max_lon_acc_constraints c(_car,v,ay);

    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-2.0);
    x_ub.push_back(10.0);

    auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();

    // Solve maximum acceleration
    auto result_max = Optimise<Max_lon_acc_fitness,Max_lon_acc_constraints>::optimise(7,12,x0,fmax,c,x_lb,x_ub,c_lb,c_ub,options);

    typename Max_lon_acc_constraints::argument_type x_max;
    std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
    c(x_max);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_max = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_max = c.get_u();
    auto dqdt_max = _car(q_max,u_max,0.0);


    Solution solution_max = {result_max.solved, v, result_max.x[6], ay, q_max, u_max, dqdt_max};

    // Solve minimum acceleration
    std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-10.0);
    x_ub.push_back( 2.0);

    auto result_min = Optimise<Min_lon_acc_fitness,Max_lon_acc_constraints>::optimise(7,12,x0,fmin,c,x_lb,x_ub,c_lb,c_ub,options);

    typename Max_lon_acc_constraints::argument_type x_min;
    std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
    c(x_min);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_min = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_min = c.get_u();
    auto dqdt_min = _car(q_min,u_min,0.0);

    Solution solution_min = {result_min.solved, v, result_min.x[6], ay, q_min, u_min, dqdt_min};

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
    std::vector<scalar> x0_ss_ay = {(result_0g.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                              result_0g.q[Dynamic_model_t::Chassis_type::IZ],
                              result_0g.q[Dynamic_model_t::Chassis_type::IPHI],
                              result_0g.q[Dynamic_model_t::Chassis_type::IMU],
                              result_0g.q[Dynamic_model_t::Road_type::IPSI],
                              result_0g.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING] 
                             };

    auto result_ss_ay = solve(v,result_max_lat_acc.ax*ay/result_max_lat_acc.ay, ay, 1, true, x0_ss_ay);
    
    // (4)
    // Optimise using the last optimization
    std::vector<scalar> x0 = {(result_ss_ay.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IZ],
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IPHI],
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IMU],
                              result_ss_ay.q[Dynamic_model_t::Road_type::IPSI],
                              result_ss_ay.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING],
                              result_ss_ay.ax
                             };

    auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
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
    CppAD::ipopt::solve<std::vector<scalar>, Max_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_max, result_max);

    Max_lon_acc_constraints c(_car,v,ay);
    typename Max_lon_acc_constraints::argument_type x_max;
    std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
    c(x_max);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_max = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_max = c.get_u();
    auto dqdt_max = _car(q_max,u_max,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::NSTATE> q_max_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        q_max_sc[i] = Value(q_max[i]);
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
    Solution solution_max = {max_solved, v, Value(result_max.x[6]), ay, q_max_sc, u_max_sc, dqdt_max_sc};

    // Solve minimum acceleration
    std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds();
    x_lb.push_back(-10.0);
    x_ub.push_back( 2.0);

    // place to return solution
    CppAD::ipopt::solve_result<std::vector<scalar>> result_min;

    // solve the problem
    Min_lon_acc f_min(_car,v,ay);
    CppAD::ipopt::solve<std::vector<scalar>, Min_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_min, result_min);

    typename Max_lon_acc_constraints::argument_type x_min;
    std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
    c(x_min);
    std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_min = c.get_q();
    std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_min = c.get_u();
    auto dqdt_min = _car(q_min,u_min,0.0);

    // Transform all AD to scalar
    std::array<scalar,Dynamic_model_t::NSTATE> q_min_sc;
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        q_min_sc[i] = Value(q_min[i]);
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

    const bool min_solved = result_min.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success;
    Solution solution_min = {min_solved, v, Value(result_min.x[6]), ay, q_min_sc, u_min_sc, dqdt_min_sc};

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
    std::vector<scalar> x0_ss_ay = {(result_0g.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                              result_0g.q[Dynamic_model_t::Chassis_type::IZ],
                              result_0g.q[Dynamic_model_t::Chassis_type::IPHI],
                              result_0g.q[Dynamic_model_t::Chassis_type::IMU],
                              result_0g.q[Dynamic_model_t::Road_type::IPSI],
                              result_0g.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING] 
                             };

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
        std::vector<scalar> x0 = {(result_ss_ay.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                                  result_ss_ay.q[Dynamic_model_t::Chassis_type::IZ],
                                  result_ss_ay.q[Dynamic_model_t::Chassis_type::IPHI],
                                  result_ss_ay.q[Dynamic_model_t::Chassis_type::IMU],
                                  result_ss_ay.q[Dynamic_model_t::Road_type::IPSI],
                                  result_ss_ay.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING],
                                  result_ss_ay.ax
                                 };
    
        Max_lon_acc_fitness fmax;
        Min_lon_acc_fitness fmin;
        Max_lon_acc_constraints c(_car,v,ay_gg[i]);

    
        auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
        x_lb.push_back(result_max_lat_acc.ax - 0.5);
        x_ub.push_back(8.0);

        auto [c_lb, c_ub] = Dynamic_model_t::steady_state_constraint_bounds();
    
        // Solve maximum acceleration
        auto result_max = Optimise<Max_lon_acc_fitness,Max_lon_acc_constraints>::optimise(7,12,x0,fmax,c,x_lb,x_ub,c_lb,c_ub,options);
    
        typename Max_lon_acc_constraints::argument_type x_max;
        std::copy(result_max.x.cbegin(), result_max.x.cend(), x_max.begin());
        c(x_max);
        std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_max = c.get_q();
        std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_max = c.get_u();
        auto dqdt_max = _car(q_max,u_max,0.0);
    
        solution_max[i] = {result_max.solved, v, result_max.x[6], ay_gg[i], q_max, u_max, dqdt_max};
    
        // Solve minimum acceleration
        std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds();
        x_lb.push_back(-8.0);
        x_ub.push_back(result_max_lat_acc.ax + 0.5);

        auto result_min = Optimise<Min_lon_acc_fitness,Max_lon_acc_constraints>::optimise(7,12,x0,fmin,c,x_lb,x_ub,c_lb,c_ub,options);
    
        typename Max_lon_acc_constraints::argument_type x_min;
        std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
        c(x_min);
        std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_min = c.get_q();
        std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_min = c.get_u();
        auto dqdt_min = _car(q_min,u_min,0.0);
    
        solution_min[i] = {result_min.solved, v, result_min.x[6], ay_gg[i], q_min, u_min, dqdt_min};

        x0_ss_ay = {(result_ss_ay.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IZ],
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IPHI],
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IMU],
                              result_ss_ay.q[Dynamic_model_t::Road_type::IPSI],
                              result_ss_ay.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING] 
                             };
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
    // Loop on the requested lateral accelerations
    std::vector<scalar> x0_ss_ay = {(result_0g.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                              result_0g.q[Dynamic_model_t::Chassis_type::IZ],
                              result_0g.q[Dynamic_model_t::Chassis_type::IPHI],
                              result_0g.q[Dynamic_model_t::Chassis_type::IMU],
                              result_0g.q[Dynamic_model_t::Road_type::IPSI],
                              result_0g.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING] 
                             };

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
        std::vector<scalar> x0 = {(result_ss_ay.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                                  result_ss_ay.q[Dynamic_model_t::Chassis_type::IZ],
                                  result_ss_ay.q[Dynamic_model_t::Chassis_type::IPHI],
                                  result_ss_ay.q[Dynamic_model_t::Chassis_type::IMU],
                                  result_ss_ay.q[Dynamic_model_t::Road_type::IPSI],
                                  result_ss_ay.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING],
                                  result_ss_ay.ax
                                 };

        auto [x_lb, x_ub] = Dynamic_model_t::steady_state_variable_bounds();
        x_lb.push_back(-2.0);
        x_ub.push_back(7.0);

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
        std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_max = c.get_u();
        auto dqdt_max = _car(q_max,u_max,0.0);

        // Transform all AD to scalar
        std::array<scalar,Dynamic_model_t::NSTATE> q_max_sc;
        for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
        {
            q_max_sc[i] = Value(q_max[i]);
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
        solution_max[i] = {max_solved, v, Value(result_max.x[6]), ay_gg[i], q_max_sc, u_max_sc, dqdt_max_sc};

        // Solve minimum acceleration
        std::tie(x_lb, x_ub) = Dynamic_model_t::steady_state_variable_bounds();
        x_lb.push_back(-7.0);
        x_ub.push_back( 2.0);
    
        // place to return solution
        CppAD::ipopt::solve_result<std::vector<scalar>> result_min;
    
        // solve the problem
        Min_lon_acc f_min(_car,v,ay_gg[i]);
        CppAD::ipopt::solve<std::vector<scalar>, Min_lon_acc>(options, x0, x_lb, x_ub, c_lb, c_ub, f_min, result_min);
    
        typename Max_lon_acc_constraints::argument_type x_min;
        std::copy(result_min.x.cbegin(), result_min.x.cend(), x_min.begin());
        c(x_min);
        std::array<Timeseries_t,Dynamic_model_t::NSTATE> q_min = c.get_q();
        std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u_min = c.get_u();
        auto dqdt_min = _car(q_min,u_min,0.0);
    
        // Transform all AD to scalar
        std::array<scalar,Dynamic_model_t::NSTATE> q_min_sc;
        for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
        {
            q_min_sc[i] = Value(q_min[i]);
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
    
        const bool min_solved = result_min.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success;
        solution_min[i] = {min_solved, v, Value(result_min.x[6]), ay_gg[i], q_min_sc, u_min_sc, dqdt_min_sc};

        x0_ss_ay = {(result_ss_ay.q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE]*0.139-v)/v,
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IZ],
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IPHI],
                              result_ss_ay.q[Dynamic_model_t::Chassis_type::IMU],
                              result_ss_ay.q[Dynamic_model_t::Road_type::IPSI],
                              result_ss_ay.u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING] 
                             };
    }

    out(2).stop_progress_bar();

    return {solution_max, solution_min};
} 


template<typename Dynamic_model_t>
typename Steady_state<Dynamic_model_t>::Solve_constraints::output_type Steady_state<Dynamic_model_t>::Solve_constraints::operator()
    (const typename Steady_state<Dynamic_model_t>::Solve_constraints::argument_type& x)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta]

    // Construct state and controls
    const Timeseries_t& psi = x[4];
    const Timeseries_t omega = _ay/_v;
     
    // Construct the state
    _q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE] = (x[0]+1.0)*_v/0.139;
    _q[Dynamic_model_t::Chassis_type::IU]                          = _v*cos(psi);
    _q[Dynamic_model_t::Chassis_type::IV]                          = -_v*sin(psi);
    _q[Dynamic_model_t::Chassis_type::IOMEGA]                      = omega;
    _q[Dynamic_model_t::Chassis_type::IZ]                          = x[1];
    _q[Dynamic_model_t::Chassis_type::IPHI]                        = x[2];
    _q[Dynamic_model_t::Chassis_type::IMU]                         = x[3];
    _q[Dynamic_model_t::Chassis_type::IDZ]                         = 0.0;
    _q[Dynamic_model_t::Chassis_type::IDPHI]                       = 0.0;
    _q[Dynamic_model_t::Chassis_type::IDMU]                        = 0.0;
    _q[Dynamic_model_t::Road_type::IX]                             = 0.0;
    _q[Dynamic_model_t::Road_type::IY]                             = 0.0;
    _q[Dynamic_model_t::Road_type::IPSI]                           = x[4];

    // Construct the controls
    _u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING] = x[5];
    _u[Dynamic_model_t::Chassis_type::rear_axle_type::ITORQUE]    = 0.0;

    // Compute time derivative
    auto dqdt = (*_car)(_q,_u,0.0);

    // Compute constraints
    std::array<Timeseries_t,12> constraints;

    constraints[0] = dqdt[Dynamic_model_t::Chassis_type::IID2Z];
    constraints[1] = dqdt[Dynamic_model_t::Chassis_type::IIDOMEGA];
    constraints[2] = dqdt[Dynamic_model_t::Chassis_type::IID2PHI];
    constraints[3] = dqdt[Dynamic_model_t::Chassis_type::IID2MU];
    constraints[4] = dqdt[Dynamic_model_t::Chassis_type::IIDU]*sin(psi)
                    + dqdt[Dynamic_model_t::Chassis_type::IIDV]*cos(psi);
    constraints[5] = _ax - dqdt[Dynamic_model_t::Chassis_type::IIDU]*cos(psi)
                         + dqdt[Dynamic_model_t::Chassis_type::IIDV]*sin(psi);
    
    constraints[6] = _car->get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
    constraints[7] = _car->get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
    constraints[8] = _car->get_chassis().get_rear_axle().template get_tire<0>().get_lambda();
    constraints[9] = _car->get_chassis().get_rear_axle().template get_tire<1>().get_lambda();
    constraints[10] = _car->get_chassis().get_front_axle().template get_tire<0>().get_lambda();
    constraints[11] = _car->get_chassis().get_front_axle().template get_tire<1>().get_lambda();

    return constraints;
}

template<typename Dynamic_model_t>
typename Steady_state<Dynamic_model_t>::Max_lat_acc_constraints::output_type Steady_state<Dynamic_model_t>::Max_lat_acc_constraints::operator()
    (const typename Steady_state<Dynamic_model_t>::Max_lat_acc_constraints::argument_type& x)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax, ay]
    const Timeseries_t& ax = x[6];
    const Timeseries_t& ay = x[7];

    // Construct state and controls
    const Timeseries_t& psi = x[4];
    const Timeseries_t omega = ay/_v;
     
    // Construct the state
    _q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE] = (x[0]+1.0)*_v/0.139;
    _q[Dynamic_model_t::Chassis_type::IU]                          = _v*cos(psi);
    _q[Dynamic_model_t::Chassis_type::IV]                          = -_v*sin(psi);
    _q[Dynamic_model_t::Chassis_type::IOMEGA]                      = omega;
    _q[Dynamic_model_t::Chassis_type::IZ]                          = x[1];
    _q[Dynamic_model_t::Chassis_type::IPHI]                        = x[2];
    _q[Dynamic_model_t::Chassis_type::IMU]                         = x[3];
    _q[Dynamic_model_t::Chassis_type::IDZ]                         = 0.0;
    _q[Dynamic_model_t::Chassis_type::IDPHI]                       = 0.0;
    _q[Dynamic_model_t::Chassis_type::IDMU]                        = 0.0;
    _q[Dynamic_model_t::Road_type::IX]                             = 0.0;
    _q[Dynamic_model_t::Road_type::IY]                             = 0.0;
    _q[Dynamic_model_t::Road_type::IPSI]                           = x[4];

    // Construct the controls
    _u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING] = x[5];
    _u[Dynamic_model_t::Chassis_type::rear_axle_type::ITORQUE]    = 0.0;

    // Compute time derivative
    auto dqdt = (*_car)(_q,_u,0.0);

    // Compute constraints
    std::array<Timeseries_t,12> constraints;

    constraints[0] = dqdt[Dynamic_model_t::Chassis_type::IID2Z];
    constraints[1] = dqdt[Dynamic_model_t::Chassis_type::IIDOMEGA];
    constraints[2] = dqdt[Dynamic_model_t::Chassis_type::IID2PHI];
    constraints[3] = dqdt[Dynamic_model_t::Chassis_type::IID2MU];
    constraints[4] = dqdt[Dynamic_model_t::Chassis_type::IIDU]*sin(psi)
                    + dqdt[Dynamic_model_t::Chassis_type::IIDV]*cos(psi);
    constraints[5] = ax - dqdt[Dynamic_model_t::Chassis_type::IIDU]*cos(psi)
                         + dqdt[Dynamic_model_t::Chassis_type::IIDV]*sin(psi);
    
    constraints[6] = _car->get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
    constraints[7] = _car->get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
    constraints[8] = _car->get_chassis().get_rear_axle().template get_tire<0>().get_lambda();
    constraints[9] = _car->get_chassis().get_rear_axle().template get_tire<1>().get_lambda();
    constraints[10] = _car->get_chassis().get_front_axle().template get_tire<0>().get_lambda();
    constraints[11] = _car->get_chassis().get_front_axle().template get_tire<1>().get_lambda();

    return constraints;
}

template<typename Dynamic_model_t>
typename Steady_state<Dynamic_model_t>::Max_lon_acc_constraints::output_type Steady_state<Dynamic_model_t>::Max_lon_acc_constraints::operator()
    (const typename Steady_state<Dynamic_model_t>::Max_lon_acc_constraints::argument_type& x)
{
    // The content of x is: x = [w_axle, z, phi, mu, psi, delta, ax]
    const Timeseries_t& ax = x[6];

    // Construct state and controls
    const Timeseries_t& psi = x[4];
    const Timeseries_t omega = _ay/_v;
     
    // Construct the state
    _q[Dynamic_model_t::Chassis_type::rear_axle_type::IOMEGA_AXLE] = (x[0]+1.0)*_v/0.139;
    _q[Dynamic_model_t::Chassis_type::IU]                          = _v*cos(psi);
    _q[Dynamic_model_t::Chassis_type::IV]                          = -_v*sin(psi);
    _q[Dynamic_model_t::Chassis_type::IOMEGA]                      = omega;
    _q[Dynamic_model_t::Chassis_type::IZ]                          = x[1];
    _q[Dynamic_model_t::Chassis_type::IPHI]                        = x[2];
    _q[Dynamic_model_t::Chassis_type::IMU]                         = x[3];
    _q[Dynamic_model_t::Chassis_type::IDZ]                         = 0.0;
    _q[Dynamic_model_t::Chassis_type::IDPHI]                       = 0.0;
    _q[Dynamic_model_t::Chassis_type::IDMU]                        = 0.0;
    _q[Dynamic_model_t::Road_type::IX]                             = 0.0;
    _q[Dynamic_model_t::Road_type::IY]                             = 0.0;
    _q[Dynamic_model_t::Road_type::IPSI]                           = x[4];

    // Construct the controls
    _u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING] = x[5];
    _u[Dynamic_model_t::Chassis_type::rear_axle_type::ITORQUE]    = 0.0;

    // Compute time derivative
    auto dqdt = (*_car)(_q,_u,0.0);

    // Compute constraints
    std::array<Timeseries_t,12> constraints;

    constraints[0] = dqdt[Dynamic_model_t::Chassis_type::IID2Z];
    constraints[1] = dqdt[Dynamic_model_t::Chassis_type::IIDOMEGA];
    constraints[2] = dqdt[Dynamic_model_t::Chassis_type::IID2PHI];
    constraints[3] = dqdt[Dynamic_model_t::Chassis_type::IID2MU];
    constraints[4] = dqdt[Dynamic_model_t::Chassis_type::IIDU]*sin(psi)
                    + dqdt[Dynamic_model_t::Chassis_type::IIDV]*cos(psi);
    constraints[5] = ax - dqdt[Dynamic_model_t::Chassis_type::IIDU]*cos(psi)
                         + dqdt[Dynamic_model_t::Chassis_type::IIDV]*sin(psi);
    
    constraints[6] = _car->get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
    constraints[7] = _car->get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
    constraints[8] = _car->get_chassis().get_rear_axle().template get_tire<0>().get_lambda();
    constraints[9] = _car->get_chassis().get_rear_axle().template get_tire<1>().get_lambda();
    constraints[10] = _car->get_chassis().get_front_axle().template get_tire<0>().get_lambda();
    constraints[11] = _car->get_chassis().get_front_axle().template get_tire<1>().get_lambda();

    return constraints;
}


#endif
