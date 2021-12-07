#include "lion/thirdparty/include/cppad/ipopt/solve.hpp"

template<typename Dynamic_model_t>
inline Optimal_laptime<Dynamic_model_t>::Optimal_laptime(const size_t n, const bool is_closed, const bool is_direct, const Dynamic_model_t& car, const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
        const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations) 
{
    if ( is_direct )
    {
        if ( is_closed )
            compute_direct<true>(n,car,q0,u0,dissipations);
        else
            compute_direct<false>(n,car,q0,u0,dissipations);
    }
    else
    {
        if ( is_closed )
            compute_derivative<true>(n,car,q0,u0,dissipations);
        else
            compute_derivative<false>(n,car,q0,u0,dissipations);
    }

}


template<typename Dynamic_model_t>
template<bool is_closed>
inline void Optimal_laptime<Dynamic_model_t>::compute_direct(const size_t n, const Dynamic_model_t& car, const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
        const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations) 
{
    FG_direct<is_closed> fg(n,car,q0,u0,dissipations);
    typename std::vector<scalar> x0(fg.get_n_variables(),0.0);

    // Constraints are equalities for now
    std::vector<scalar> c_lb(fg.get_n_constraints(),0.0);
    std::vector<scalar> c_ub(fg.get_n_constraints(),0.0);

    // Set minimum and maximum variables: set no bound atm
    std::vector<scalar> u_lb = {-20.0*DEG,-1.0};
    std::vector<scalar> u_ub = { 20.0*DEG, 1.0};

    if ( fg.get_car().get_chassis().get_rear_axle().is_direct_torque() )
    {
        u_lb.back() = -200.0;
        u_ub.back() =  200.0;
    }

    std::vector<scalar> q_lb = { 10.0*KMH/0.139, 10.0*KMH,-10.0*KMH, -10.0, 1.0e-5, -30.0*DEG, -30.0*DEG, -10.0, -10.0, -10.0, 0.0, -2.0, -30.0*DEG };
    std::vector<scalar> q_ub = { 200.0*KMH/0.139, 200.0*KMH,10.0*KMH, 10.0, 0.139,  30.0*DEG,  30.0*DEG,  10.0,  10.0,  10.0, 0.0,  2.0,  30.0*DEG };

    std::vector<scalar> x_lb(fg.get_n_variables(), -1.0e24);
    std::vector<scalar> x_ub(fg.get_n_variables(), +1.0e24);

    // Fill x0 with the initial condition as state vector, and zero controls
    size_t k = 0;
    size_t kc = 0;
    constexpr const size_t offset = is_closed ? 0 : 1;

    for (size_t i = offset; i < n + offset; ++i)
    {
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)    
        {
            x0[k] = q0[j];
            x_lb[k] = q_lb[j];
            x_ub[k] = q_ub[j];
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            k++; kc++;
        }

        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)    
        {
            x0[k] = q0[j];
            x_lb[k] = q_lb[j];
            x_ub[k] = q_ub[j];
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            k++; kc++;
        }

        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            x0[k] = u0[j];
            x_lb[k] = u_lb[j];
            x_ub[k] = u_ub[j];
            k++;
        }

        for (size_t j = 0; j < 6; ++j)
        {
            c_lb[kc] = -0.11;
            c_ub[kc] =  0.11;
            kc++;
        }
    }

    assert(k == fg.get_n_variables());
    assert(kc == fg.get_n_constraints());

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    options += "Sparse true forward\n";
    options += "Numeric tol          1e-10\n";
    options += "Numeric constr_viol_tol  1e-10\n";
    options += "Numeric acceptable_tol  1e-8\n";

    // place to return solution
    CppAD::ipopt::solve_result<std::vector<scalar>> result;

    // solve the problem
    CppAD::ipopt::solve<std::vector<scalar>, FG_direct<is_closed>>(options, x0, x_lb, x_ub, c_lb, c_ub, fg, result);

    if ( result.status != CppAD::ipopt::solve_result<std::vector<scalar>>::success )
    {
        throw std::runtime_error("Optimization did not succeed");
    }

    // Load the latest solution to be retrieved by the caller via get_state(i) and get_control(i)
    std::vector<CppAD::AD<scalar>> x_final(result.x.size());
    std::vector<CppAD::AD<scalar>> fg_final(fg.get_n_constraints()+1);

    for (size_t i = 0; i < result.x.size(); ++i)
        x_final[i] = result.x[i];

    fg(fg_final,x_final);
/*  
    for (size_t i = 0; i < n+offset; ++i)
    {
        fg.get_car()(fg.get_state(i), fg.get_control(i), 0.0);
        std::cout << fg.get_control(i)[0] << ", " << fg.get_control(i)[1] << std::endl;
    }

    std::cout << "Positions" << std::endl;
    for (size_t i = 0; i < n+offset; ++i)
    {
        const scalar& L = fg.get_car().get_road().track_length();
        fg.get_car()(fg.get_state(i), fg.get_control(i), ((double)i)*L/((double)n));
        std::cout << fg.get_car().get_road().get_x() << ", " << fg.get_car().get_road().get_y() << std::endl;
    }
*/

    // Export the solution
    q = {fg.get_states().size(),{{}}};
    
    for (size_t i = 0; i < fg.get_states().size(); ++i)
        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
            q[i][j] = Value(fg.get_state(i)[j]);

    u = {fg.get_controls().size(),{{}}};
    
    for (size_t i = 0; i < fg.get_controls().size(); ++i)
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            u[i][j] = Value(fg.get_control(i)[j]);

    // Compute the time and arclength
    s = std::vector<scalar>(fg.get_states().size(),0.0);
    const scalar& L = fg.get_car().get_road().track_length();
    const scalar ds = L/((scalar)(n));
    auto dtimeds_first = fg.get_car()(fg.get_state(0),fg.get_control(0),0.0)[Dynamic_model_t::Road_type::ITIME];
    auto dtimeds_prev = dtimeds_first;
    for (size_t i = 1; i < fg.get_states().size(); ++i)
    {
        s[i] = ((double)i)*ds;
        const auto dtimeds = fg.get_car()(fg.get_state(i),fg.get_control(i),s[i])[Dynamic_model_t::Road_type::ITIME];
        q[i][Dynamic_model_t::Road_type::ITIME] = q[i-1][Dynamic_model_t::Road_type::ITIME] + Value(0.5*ds*(dtimeds + dtimeds_prev));
        dtimeds_prev = dtimeds;
    }

    laptime = q.back()[Dynamic_model_t::Road_type::ITIME] + Value(0.5*ds*(dtimeds_prev*dtimeds_first));
}

template<typename Dynamic_model_t>
template<bool is_closed>
inline void Optimal_laptime<Dynamic_model_t>::compute_derivative(const size_t n, const Dynamic_model_t& car, const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
        const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations) 
{
    FG_derivative<is_closed> fg(n,car,q0,u0,dissipations);
    typename std::vector<scalar> x0(fg.get_n_variables(),0.0);

    // Initialise constraints
    std::vector<scalar> c_lb(fg.get_n_constraints(),0.0);
    std::vector<scalar> c_ub(fg.get_n_constraints(),0.0);

    // Set minimum and maximum variables
    std::vector<scalar> u_lb = {-20.0*DEG,-1.0};
    std::vector<scalar> u_ub = { 20.0*DEG, 1.0};

    // Set minimum and maximum control derivatives
    std::vector<scalar> dudt_lb = { -20.0*DEG, -4000.0/400.0 };
    std::vector<scalar> dudt_ub = {  20.0*DEG,  4000.0/400.0 };

    if ( fg.get_car().get_chassis().get_rear_axle().is_direct_torque() )
    {
        u_lb.back() = -200.0;
        u_ub.back() =  200.0;
        dudt_lb.back() = -4000.0;
        dudt_ub.back() =  4000.0;
    }


    // Set state bounds
    std::vector<scalar> q_lb = { 10.0*KMH/0.139, 10.0*KMH,-10.0*KMH, -10.0, 1.0e-5, -30.0*DEG, -30.0*DEG, -10.0, -10.0, -10.0, 0.0, -2.0, -30.0*DEG };
    std::vector<scalar> q_ub = { 200.0*KMH/0.139, 200.0*KMH,10.0*KMH, 10.0, 0.139,  30.0*DEG,  30.0*DEG,  10.0,  10.0,  10.0, 0.0,  2.0,  30.0*DEG };

    std::vector<scalar> x_lb(fg.get_n_variables(), -1.0e24);
    std::vector<scalar> x_ub(fg.get_n_variables(), +1.0e24);

    // Fill x0 with the initial condition as state vector, and zero controls
    size_t k = 0;
    size_t kc = 0;
    constexpr const size_t offset = is_closed ? 0 : 1;

    for (size_t i = offset; i < n + offset; ++i)
    {
        // Set state before time
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)    
        {
            x0[k] = q0[j];
            x_lb[k] = q_lb[j];
            x_ub[k] = q_ub[j];
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            k++; kc++;
        }

        // Set state after time
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)    
        {
            x0[k] = q0[j];
            x_lb[k] = q_lb[j];
            x_ub[k] = q_ub[j];
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            k++; kc++;
        }

        // Set tire constraints
        for (size_t j = 0; j < 6; ++j)
        {
            c_lb[kc] = -0.11;
            c_ub[kc] =  0.11;
            kc++;
        }

        // Set controls
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            x0[k] = u0[j];
            x_lb[k] = u_lb[j];
            x_ub[k] = u_ub[j];
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            k++; kc++;
        }

        // Set control derivatives
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            x0[k] = 0.0;
            x_lb[k] = dudt_lb[j];
            x_ub[k] = dudt_ub[j];
            k++;
        }
    }

    assert(k == fg.get_n_variables());
    assert(kc == fg.get_n_constraints());

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    options += "Sparse true forward\n";
    options += "Numeric tol          1e-6\n";
    options += "Numeric constr_viol_tol  1e-6\n";
    options += "Numeric acceptable_tol  1e-6\n";

    // place to return solution
    CppAD::ipopt::solve_result<std::vector<scalar>> result;

    // solve the problem
    CppAD::ipopt::solve<std::vector<scalar>, FG_derivative<is_closed>>(options, x0, x_lb, x_ub, c_lb, c_ub, fg, result);

    if ( result.status != CppAD::ipopt::solve_result<std::vector<scalar>>::success )
    {
        throw std::runtime_error("Optimization did not succeed");
    }

    // Load the latest solution to be retrieved by the caller via get_state(i) and get_control(i)
    std::vector<CppAD::AD<scalar>> x_final(result.x.size());
    std::vector<CppAD::AD<scalar>> fg_final(fg.get_n_constraints()+1);

    for (size_t i = 0; i < result.x.size(); ++i)
        x_final[i] = result.x[i];

    fg(fg_final,x_final);
  
/*

    for (size_t i = 0; i < n+offset; ++i)
    {
        fg.get_car()(fg.get_state(i), fg.get_control(i), 0.0);
        std::cout << fg.get_control(i)[0] << ", " << fg.get_control(i)[1] << std::endl;
    }

    std::cout << "Positions" << std::endl;
    for (size_t i = 0; i < n+offset; ++i)
    {
        const scalar& L = fg.get_car().get_road().track_length();
        fg.get_car()(fg.get_state(i), fg.get_control(i), ((double)i)*L/((double)n));
        std::cout << fg.get_car().get_road().get_x() << ", " << fg.get_car().get_road().get_y() << std::endl;
    }
*/

    // Export the solution
    q = {fg.get_states().size(),{{}}};
    
    for (size_t i = 0; i < fg.get_states().size(); ++i)
        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
            q[i][j] = Value(fg.get_state(i)[j]);

    u = {fg.get_controls().size(),{{}}};
    
    for (size_t i = 0; i < fg.get_controls().size(); ++i)
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            u[i][j] = Value(fg.get_control(i)[j]);

    // Compute the time and arclength
    s = std::vector<scalar>(fg.get_states().size(),0.0);
    const scalar& L = fg.get_car().get_road().track_length();
    const scalar ds = L/((scalar)(n));
    auto dtimeds_first = fg.get_car()(fg.get_state(0),fg.get_control(0),0.0)[Dynamic_model_t::Road_type::ITIME];
    auto dtimeds_prev = dtimeds_first;
    for (size_t i = 1; i < fg.get_states().size(); ++i)
    {
        s[i] = ((double)i)*ds;
        const auto dtimeds = fg.get_car()(fg.get_state(i),fg.get_control(i),s[i])[Dynamic_model_t::Road_type::ITIME];
        q[i][Dynamic_model_t::Road_type::ITIME] = q[i-1][Dynamic_model_t::Road_type::ITIME] + Value(0.5*ds*(dtimeds + dtimeds_prev));
        dtimeds_prev = dtimeds;
    }

    laptime = q.back()[Dynamic_model_t::Road_type::ITIME] + Value(0.5*ds*(dtimeds_prev*dtimeds_first));
}




template<typename Dynamic_model_t>
template<bool is_closed>
inline void Optimal_laptime<Dynamic_model_t>::FG_direct<is_closed>::operator()(FG_direct<is_closed>::ADvector& fg, const Optimal_laptime<Dynamic_model_t>::FG_direct<is_closed>::ADvector& x)
{
    auto& _n             = FG::_n;
    auto& _car           = FG::_car;
    auto& _q0            = FG::_q0;
    auto& _u0            = FG::_u0;
    auto& _q             = FG::_q;
    auto& _u             = FG::_u;
    auto& _dqdt          = FG::_dqdt;
    auto& _dissipations  = FG::_dissipations;


    assert(x.size() == FG::_n_variables);
    assert(fg.size() == (1 + FG::_n_constraints));

    // Load the state and control vectors
    size_t k = 0;

    if constexpr (!is_closed)
    {
        // Load the state in the first position
        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
            _q[0][j] = _q0[j];
    
        // Load the control in the first position
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            _u[0][j] = _u0[j];
    }

    constexpr const size_t offset = (is_closed ? 0 : 1);

    for (size_t i = offset; i < _n+offset; ++i)
    {
        // Load state (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            _q[i][j] = x[k++];

        // Load state (after time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            _q[i][j] = x[k++];

        // Load control
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            _u[i][j] = x[k++];
    }

    // Check that all variables in x were used
    assert(k == FG::_n_variables);

    // Initialize fitness function
    fg[0] = 0.0;

    // Loop over the nodes
    const scalar& L = _car.get_road().track_length();
    const scalar ds = L/((scalar)(_n));
    
    _dqdt[0] = _car(_q[0],_u[0],0.0);
    k = 1;  // Reset the counter
    for (size_t i = 1; i < _n+offset; ++i)
    {
        const scalar s = ((double)i)*ds;
        _dqdt[i] = _car(_q[i],_u[i],s);

        // Fitness function: integral of time
        fg[0] += 0.5*ds*(_dqdt[i-1][Dynamic_model_t::Road_type::ITIME] + _dqdt[i][Dynamic_model_t::Road_type::ITIME]);

        // Equality constraints: q^{i+1} = q^{i} + 0.5.ds.[dqdt^{i+1} + dqdt^{i}]
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            fg[k++] = _q[i][j] - _q[i-1][j] - 0.5*ds*(_dqdt[i-1][j] + _dqdt[i][j]);

        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _q[i][j] - _q[i-1][j] - 0.5*ds*(_dqdt[i-1][j] + _dqdt[i][j]);

        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        const auto& kappa_left = _car.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = _car.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        const auto& lambda_front_left = _car.get_chassis().get_front_axle().template get_tire<0>().get_lambda();
        const auto& lambda_front_right = _car.get_chassis().get_front_axle().template get_tire<1>().get_lambda();
        const auto& lambda_rear_left = _car.get_chassis().get_rear_axle().template get_tire<0>().get_lambda();
        const auto& lambda_rear_right = _car.get_chassis().get_rear_axle().template get_tire<1>().get_lambda();

        fg[k++] = kappa_left;
        fg[k++] = kappa_right;
        fg[k++] = lambda_front_left;
        fg[k++] = lambda_front_right;
        fg[k++] = lambda_rear_left;
        fg[k++] = lambda_rear_right;
    }

    // Add a penalisation to the controls
    const double coeff = 1.0/(ds);
    for (size_t i = 1; i < _n+offset; ++i)
    {
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            const auto derivative = coeff*(_u[i][j]-_u[i-1][j]);
            fg[0] += _dissipations[j]*(derivative*derivative)*ds;
        }
    }

    // Add the periodic element if track is closed
    if constexpr (is_closed)
    {
        // Fitness function: integral of time
        fg[0] += 0.5*ds*(_dqdt[0][Dynamic_model_t::Road_type::ITIME] + _dqdt[_n-1][Dynamic_model_t::Road_type::ITIME]);

        // Equality constraints: q^{i+1} = q^{i} + 0.5.ds.[dqdt^{i+1} + dqdt^{i}]
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            fg[k++] = _q[0][j] - _q[_n-1][j] - 0.5*ds*(_dqdt[_n-1][j] + _dqdt[0][j]);

        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _q[0][j] - _q[_n-1][j] - 0.5*ds*(_dqdt[_n-1][j] + _dqdt[0][j]);

        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        _car(_q[0],_u[0],0.0);
        const auto& kappa_left = _car.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = _car.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        const auto& lambda_front_left = _car.get_chassis().get_front_axle().template get_tire<0>().get_lambda();
        const auto& lambda_front_right = _car.get_chassis().get_front_axle().template get_tire<1>().get_lambda();
        const auto& lambda_rear_left = _car.get_chassis().get_rear_axle().template get_tire<0>().get_lambda();
        const auto& lambda_rear_right = _car.get_chassis().get_rear_axle().template get_tire<1>().get_lambda();

        fg[k++] = kappa_left;
        fg[k++] = kappa_right;
        fg[k++] = lambda_front_left;
        fg[k++] = lambda_front_right;
        fg[k++] = lambda_rear_left;
        fg[k++] = lambda_rear_right;

        // Add the penalisation to the controls
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            const auto derivative = coeff*(_u[0][j]-_u[_n-1][j]);
            fg[0] += _dissipations[j]*(derivative*derivative)*ds;
        }
    }

    assert(k == FG::_n_constraints+1);
}


template<typename Dynamic_model_t>
template<bool is_closed>
inline void Optimal_laptime<Dynamic_model_t>::FG_derivative<is_closed>::operator()(FG_derivative<is_closed>::ADvector& fg, const Optimal_laptime<Dynamic_model_t>::FG_derivative<is_closed>::ADvector& x)
{
    auto& _n             = FG::_n;
    auto& _car           = FG::_car;
    //auto& _q0            = FG::_q0;
    //auto& _u0            = FG::_u0;
    auto& _q             = FG::_q;
    auto& _u             = FG::_u;
    auto& _dqdt          = FG::_dqdt;
    auto& _dissipations  = FG::_dissipations;
    const scalar& L = _car.get_road().track_length();
    const scalar ds = L/((scalar)(_n));


    assert(x.size() == FG::_n_variables);
    assert(fg.size() == (1 + FG::_n_constraints));

    // Load the state and control vectors
    size_t k = 0;

    if constexpr (!is_closed)
    {
        throw std::runtime_error("Not implemented");
    }

    constexpr const size_t offset = (is_closed ? 0 : 1);

    for (size_t i = offset; i < _n+offset; ++i)
    {
        // Load state (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            _q[i][j] = x[k++];

        // Load state (after time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            _q[i][j] = x[k++];

        // Load control
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            _u[i][j] = x[k++];

        // Load control derivative
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            _dudt[i][j] = x[k++];
    }

    // Check that all variables in x were used
    assert(k == FG::_n_variables);

    // Initialize fitness function
    fg[0] = 0.0;

    // Loop over the nodes
    _dqdt[0] = _car(_q[0],_u[0],0.0);
    k = 1;  // Reset the counter
    for (size_t i = 1; i < _n+offset; ++i)
    {
        const scalar s = ((double)i)*ds;
        _dqdt[i] = _car(_q[i],_u[i],s);

        // Fitness function: integral of time
        fg[0] += 0.5*ds*(_dqdt[i-1][Dynamic_model_t::Road_type::ITIME] + _dqdt[i][Dynamic_model_t::Road_type::ITIME]);

        // Equality constraints: q^{i+1} = q^{i} + 0.5.ds.[dqdt^{i+1} + dqdt^{i}]
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            fg[k++] = _q[i][j] - _q[i-1][j] - 0.5*ds*(_dqdt[i-1][j] + _dqdt[i][j]);

        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _q[i][j] - _q[i-1][j] - 0.5*ds*(_dqdt[i-1][j] + _dqdt[i][j]);

        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        const auto& kappa_left = _car.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = _car.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        const auto& lambda_front_left = _car.get_chassis().get_front_axle().template get_tire<0>().get_lambda();
        const auto& lambda_front_right = _car.get_chassis().get_front_axle().template get_tire<1>().get_lambda();
        const auto& lambda_rear_left = _car.get_chassis().get_rear_axle().template get_tire<0>().get_lambda();
        const auto& lambda_rear_right = _car.get_chassis().get_rear_axle().template get_tire<1>().get_lambda();

        fg[k++] = kappa_left;
        fg[k++] = kappa_right;
        fg[k++] = lambda_front_left;
        fg[k++] = lambda_front_right;
        fg[k++] = lambda_rear_left;
        fg[k++] = lambda_rear_right;

        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            fg[k++] = _u[i][j] - _u[i-1][j] - 0.5*ds*(_dudt[i-1][j]*_dqdt[i-1][Dynamic_model_t::Road_type::ITIME]+_dudt[i][j]*_dqdt[i][Dynamic_model_t::Road_type::ITIME]);
    }

    // Add a penalisation to the controls
    for (size_t i = 1; i < _n+offset; ++i)
    {
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            fg[0] += _dissipations[j]*(_dudt[i][j]*_dudt[i][j])*ds;
        }
    }

    // Add the periodic element if track is closed
    if constexpr (is_closed)
    {
        // Fitness function: integral of time
        fg[0] += 0.5*ds*(_dqdt[0][Dynamic_model_t::Road_type::ITIME] + _dqdt[_n-1][Dynamic_model_t::Road_type::ITIME]);

        // Equality constraints: q^{i+1} = q^{i} + 0.5.ds.[dqdt^{i+1} + dqdt^{i}]
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            fg[k++] = _q[0][j] - _q[_n-1][j] - 0.5*ds*(_dqdt[_n-1][j] + _dqdt[0][j]);

        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _q[0][j] - _q[_n-1][j] - 0.5*ds*(_dqdt[_n-1][j] + _dqdt[0][j]);

        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        _car(_q[0],_u[0],0.0);
        const auto& kappa_left = _car.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
        const auto& kappa_right = _car.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();
        const auto& lambda_front_left = _car.get_chassis().get_front_axle().template get_tire<0>().get_lambda();
        const auto& lambda_front_right = _car.get_chassis().get_front_axle().template get_tire<1>().get_lambda();
        const auto& lambda_rear_left = _car.get_chassis().get_rear_axle().template get_tire<0>().get_lambda();
        const auto& lambda_rear_right = _car.get_chassis().get_rear_axle().template get_tire<1>().get_lambda();

        fg[k++] = kappa_left;
        fg[k++] = kappa_right;
        fg[k++] = lambda_front_left;
        fg[k++] = lambda_front_right;
        fg[k++] = lambda_rear_left;
        fg[k++] = lambda_rear_right;

        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            fg[k++] = _u[0][j] - _u[_n-1][j] - 0.5*ds*(_dudt[_n-1][j]*_dqdt[_n-1][Dynamic_model_t::Road_type::ITIME]+_dudt[0][j]*_dqdt[0][Dynamic_model_t::Road_type::ITIME]);

        // Add the penalisation to the controls
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            fg[0] += _dissipations[j]*(_dudt[0][j]*_dudt[0][j])*ds;
        }
    }

    assert(k == FG::_n_constraints+1);
}
