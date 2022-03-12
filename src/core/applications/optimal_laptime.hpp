#include "lion/thirdparty/include/cppad/ipopt/solve.hpp"

template<typename Dynamic_model_t>
inline Optimal_laptime<Dynamic_model_t>::Optimal_laptime(const size_t n, const bool is_closed_, const bool is_direct, 
    const Dynamic_model_t& car, const std::array<scalar,Dynamic_model_t::NSTATE>& q0, 
    const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& qa0, 
    const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations,
    const Options opts) 
: options(opts), is_closed(is_closed_)
{
    // (1) Compute number of elements and points
    n_elements = n;
    n_points = (is_closed ? n_elements : n_elements + 1);

    // (2) Compute vector of arclengths
    const scalar& L = car.get_road().track_length();
    const scalar ds = L/static_cast<scalar>(n_elements);
    s = std::vector<scalar>(n_points, 0.0);

    for (size_t i = 1; i < n_points; ++i)
        s[i] = static_cast<scalar>(i)*ds;

    if ( !is_closed ) s.back() = L; // If open circuit, simply override the last one by the accurate track length

    // (3) Set the initial condition
    q  = std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>(n_points,q0);
    qa = std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>(n_points,qa0);
    u  = std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>>(n_points,u0);

    // (4) Compute
    compute(is_direct, car, dissipations);
}


template<typename Dynamic_model_t>
inline Optimal_laptime<Dynamic_model_t>::Optimal_laptime(const std::vector<scalar>& s_, const bool is_closed_, const bool is_direct,
    const Dynamic_model_t& car, const std::array<scalar,Dynamic_model_t::NSTATE>& q0, const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& qa0,
    const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations,
    const Options opts)
: options(opts), is_closed(is_closed_)
{
    s = s_;
    if ( s.size() <= 1 )
        throw std::runtime_error("Provide at least two values of arclength");

    // (1) Compute number of elements and points
    n_points = s.size();
    n_elements = (is_closed ? n_points : n_points - 1);

    // (2) Verify the vector or arclength
    const scalar& L = car.get_road().track_length();

    if (is_closed)
    {
        if (std::abs(s.front()) > 1.0e-12)
            throw std::runtime_error("In closed circuits, s[0] should be 0.0");

        if (s.back() > L - 1.0e-10)
            throw std::runtime_error("In closed circuits, s[end] should be < track_length");
    }
    else
    {
        if (s[0] < -1.0e-12)
            throw std::runtime_error("s[0] must be >= 0");

        if (s.back() > L)
            throw std::runtime_error("s[end] must be <= L");
    }

    // If closed, replace the initial and final arclength by 0,L
    if (is_closed)
    {   
        s.front() = 0.0;
    }
        
    // (3) Set the initial condition
    q  = std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>(n_points,q0);
    qa = std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>(n_points,qa0);
    u  = std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>>(n_points,u0);

    // (4) Compute
    compute(is_direct, car, dissipations);
}

template<typename Dynamic_model_t>
inline Optimal_laptime<Dynamic_model_t>::Optimal_laptime(const scalar s_start, const scalar s_finish, const size_t n, const bool is_direct,
    const Dynamic_model_t& car, const std::array<scalar,Dynamic_model_t::NSTATE>& q0, const std::array<scalar,Dynamic_model_t::NALGEBRAIC>& qa0,
    const std::array<scalar,Dynamic_model_t::NCONTROL>& u0, const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations,
    const Options opts)
: options(opts)
{
    // (1) Check inputs
    if (s_start < -1.0e-12)
        throw std::runtime_error("s_start must be >= 0");

    const scalar& L = car.get_road().track_length();
    if (s.back() > L)
        throw std::runtime_error("s_finish must be <= L");

    // (2) Compute number of points
    n_elements = n;
    n_points   = n+1;

    // (3) Construct arclength
    s = linspace(s_start, s_finish, n+1); 

    // (4) Set the initial condition
    q  = std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>(n_points,q0);
    qa = std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>(n_points,qa0);
    u  = std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>>(n_points,u0);

    // (5) Compute
    compute(false, is_direct, car, dissipations);
}


template<typename Dynamic_model_t>
inline Optimal_laptime<Dynamic_model_t>::Optimal_laptime(const std::vector<scalar>& s_, const bool is_closed_, const bool is_direct,
    const Dynamic_model_t& car, const std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>& q0, 
    const std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>& qa0,
    const std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>>& u0, 
    const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations,
    const Options opts)
: options(opts), is_closed(is_closed_)
{
    s = s_;
    if ( s.size() <= 1 )
        throw std::runtime_error("Provide at least two values of arclength");

    // (1) Compute number of elements and points
    n_points = s.size();
    n_elements = (is_closed ? n_points : n_points - 1);

    // (2) Verify the vector or arclength
    const scalar& L = car.get_road().track_length();

    if (is_closed)
    {
        if (std::abs(s.front()) > 1.0e-12)
            throw std::runtime_error("In closed circuits, s[0] should be 0.0");

        if (s.back() > L - 1.0e-10)
            throw std::runtime_error("In closed circuits, s[end] should be < track_length");
    }
    else
    {
        if (s[0] < -1.0e-12)
            throw std::runtime_error("s[0] must be >= 0");

        if (s.back() > L)
            throw std::runtime_error("s[end] must be <= L");
    }

    // If closed, replace the initial arclength by 0
    if (is_closed)
    {   
        s.front() = 0.0;
    }

    // (3) Set the initial condition
    if ( q0.size() != n_points )
        throw std::runtime_error("q0 must have size of n_points");

    if ( qa0.size() != n_points )
        throw std::runtime_error("qa0 must have size of n_points");

    if ( u0.size() != n_points )
        throw std::runtime_error("u0 must have size of n_points");

    q  = q0;
    qa = qa0;
    u  = u0;

    // (4) Compute
    compute(is_direct, car, dissipations);
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
        throw std::runtime_error("Incorrect track type, should be \"open\" or \"closed\"");

    const auto [key_name, q_names, qa_names, u_names] = Dynamic_model_t::get_state_and_control_names();

    // Get the data
    n_points = std::stoi(root.get_attribute("n_points"));
    n_elements = (is_closed ? n_points : n_points - 1);

    laptime = root.get_child("laptime").get_value(scalar());

    s = root.get_child("arclength").get_value(std::vector<scalar>());

    // Get state
    q = std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>(n_points);
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        std::vector<scalar> data_in = root.get_child(q_names[i]).get_value(std::vector<scalar>());
        for (size_t j = 0; j < n_points; ++j)
            q[j][i] = data_in[j];
    }

    // Get algebraic states
    qa = std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>(n_points);
    for (size_t i = 0; i < Dynamic_model_t::NALGEBRAIC; ++i)
    {
        std::vector<scalar> data_in = root.get_child(qa_names[i]).get_value(std::vector<scalar>());
        for (size_t j = 0; j < n_points; ++j)
            qa[j][i] = data_in[j];
    }

    // Get controls
    u = std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>>(n_points);
    for (size_t i = 0; i < Dynamic_model_t::NCONTROL; ++i)
    {
        std::vector<scalar> data_in = root.get_child(u_names[i]).get_value(std::vector<scalar>());
        for (size_t j = 0; j < n_points; ++j)
            u[j][i] = data_in[j];
    }

    // Get x
    x_coord = root.get_child("x").get_value(std::vector<scalar>());
    y_coord = root.get_child("y").get_value(std::vector<scalar>());
    psi     = root.get_child("psi").get_value(std::vector<scalar>());
}


template<typename Dynamic_model_t>
inline void Optimal_laptime<Dynamic_model_t>::compute(const bool is_direct, const Dynamic_model_t& car, const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations)
{
    if ( is_direct )
    {
        if ( is_closed )
            compute_direct<true>(car,dissipations);
        else
            compute_direct<false>(car,dissipations);
    }
    else
    {
        if ( is_closed )
            compute_derivative<true>(car,dissipations);
        else
            compute_derivative<false>(car,dissipations);
    }
}


template<typename Dynamic_model_t>
template<bool isClosed>
inline void Optimal_laptime<Dynamic_model_t>::compute_direct(const Dynamic_model_t& car, const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations) 
{
    FG_direct<isClosed> fg(n_elements,n_points,car,s,q.front(),qa.front(),u.front(),dissipations);
    typename std::vector<scalar> x0(fg.get_n_variables(),0.0);

    // Set minimum and maximum variables
    std::vector<scalar> u_lb, u_ub;
    std::tie(u_lb, u_ub, std::ignore, std::ignore) = car.optimal_laptime_control_bounds();
    auto [q_lb, q_ub] = Dynamic_model_t::optimal_laptime_state_bounds();
    auto [qa_lb, qa_ub] = Dynamic_model_t::optimal_laptime_algebraic_state_bounds();

    // Correct the maximum bound in N to the track limits
    std::vector<scalar> x_lb(fg.get_n_variables(), -1.0e24);
    std::vector<scalar> x_ub(fg.get_n_variables(), +1.0e24);

    // Fill x0 with the initial condition as state vector, and zero controls
    size_t k = 0;
    constexpr const size_t offset = isClosed ? 0 : 1;

    for (size_t i = offset; i < n_points; ++i)
    {
        // Set state and initial condition from start to ITIME
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)    
        {
            x0[k] = q[i][j];
            x_lb[k] = q_lb[j];
            x_ub[k] = q_ub[j];
            k++;
        }

        // Set state to IN. Assert that ITIME = IN - 1
        assert(Dynamic_model_t::Road_type::ITIME == ( Dynamic_model_t::Road_type::IN - 1 ) );

        x0[k] = q[i][Dynamic_model_t::Road_type::IN];
        x_lb[k] = -car.get_road().get_left_track_limit(s[i]);
        x_ub[k] =  car.get_road().get_right_track_limit(s[i]);
        k++;

        // Set state after IN
        for (size_t j = Dynamic_model_t::Road_type::IN+1; j < Dynamic_model_t::NSTATE; ++j)    
        {
            x0[k] = q[i][j];
            x_lb[k] = q_lb[j];
            x_ub[k] = q_ub[j];
            k++;
        }

        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
        {
            x0[k] = qa[i][j];
            x_lb[k] = qa_lb[j];
            x_ub[k] = qa_ub[j];
            k++; 
        }

        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            x0[k] = u[i][j];
            x_lb[k] = u_lb[j];
            x_ub[k] = u_ub[j];
            k++;
        }
    }

    // Fill the constraints upper/lower bounds vector
    std::vector<scalar> c_lb(fg.get_n_constraints(),0.0);
    std::vector<scalar> c_ub(fg.get_n_constraints(),0.0);
    size_t kc = 0;
    for (size_t i = 1; i < n_points; ++i)
    {
        // Equality constraints: --------------- 
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }

        // q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }

        // algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }
        
        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        auto [c_extra_lb, c_extra_ub] = car.optimal_laptime_extra_constraints_bounds(s[i]); 
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
        {
            c_lb[kc] = c_extra_lb[j];
            c_ub[kc] = c_extra_ub[j];
            kc++;
        }
    }

    // Add the periodic element if track is closed
    if constexpr (isClosed)
    {
        // Equality constraints: --------------- 
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }

        // q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }

        // algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }
        
        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        auto [c_extra_lb, c_extra_ub] = car.optimal_laptime_extra_constraints_bounds(s[0]); 
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
        {
            c_lb[kc] = c_extra_lb[j];
            c_ub[kc] = c_extra_ub[j];
            kc++;
        }

    }

    if ( k != fg.get_n_variables() )
    {
        std::ostringstream s_out;
        s_out << "k(=" << k << ") != fg.get_n_variables()(=" << fg.get_n_variables() << ")" ;
        throw std::runtime_error(s_out.str());
    }
    
    if ( kc != fg.get_n_constraints() )
    {
        std::ostringstream s_out;
        s_out << "kc(=" << kc << ") != fg.get_n_constraints()(=" << fg.get_n_constraints() << ")" ;
        throw std::runtime_error(s_out.str());
    }

    // options
    std::string ipoptoptions;
    // turn off any printing
    ipoptoptions += "Integer print_level  ";
    ipoptoptions += std::to_string(options.print_level);
    ipoptoptions += "\n";
    ipoptoptions += "String  sb           yes\n";
    ipoptoptions += "Sparse true forward\n";
    ipoptoptions += "Numeric tol          1e-10\n";
    ipoptoptions += "Numeric constr_viol_tol  1e-10\n";
    ipoptoptions += "Numeric acceptable_tol  1e-8\n";

    // place to return solution
    CppAD::ipopt::solve_result<std::vector<scalar>> result;

    // solve the problem
    CppAD::ipopt::solve<std::vector<scalar>, FG_direct<isClosed>>(ipoptoptions, x0, x_lb, x_ub, c_lb, c_ub, fg, result);

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

    // Export the solution
    assert(fg.get_states().size() == n_points);
    assert(fg.get_algebraic_states().size() == n_points);
    assert(fg.get_controls().size() == n_points);

    q = std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
            q[i][j] = Value(fg.get_state(i)[j]);

    qa = std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            qa[i][j] = Value(fg.get_algebraic_state(i)[j]);

    u = std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>>(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            u[i][j] = Value(fg.get_control(i)[j]);

    // Compute the time, x, y, and psi
    x_coord = std::vector<scalar>(fg.get_states().size());
    y_coord = std::vector<scalar>(fg.get_states().size());
    psi = std::vector<scalar>(fg.get_states().size());

    const scalar& L = car.get_road().track_length();
    auto dtimeds_first = fg.get_car()(fg.get_state(0),fg.get_algebraic_state(0),fg.get_control(0),s[0]).first[Dynamic_model_t::Road_type::ITIME];
    auto dtimeds_prev = dtimeds_first;
    x_coord.front() = Value(fg.get_car().get_road().get_x());
    y_coord.front() = Value(fg.get_car().get_road().get_y());
    psi.front() = Value(fg.get_car().get_road().get_psi());
    for (size_t i = 1; i < fg.get_states().size(); ++i)
    {
        const auto dtimeds = fg.get_car()(fg.get_state(i),fg.get_algebraic_state(i),fg.get_control(i),s[i]).first[Dynamic_model_t::Road_type::ITIME];
        q[i][Dynamic_model_t::Road_type::ITIME] = q[i-1][Dynamic_model_t::Road_type::ITIME] + Value(0.5*(s[i]-s[i-1])*(dtimeds + dtimeds_prev));
        dtimeds_prev = dtimeds;

        x_coord[i] = Value(fg.get_car().get_road().get_x());
        y_coord[i] = Value(fg.get_car().get_road().get_y());
        psi[i]     = Value(fg.get_car().get_road().get_psi());
    }

    laptime = q.back()[Dynamic_model_t::Road_type::ITIME];

    if (isClosed)
        laptime += Value(0.5*(L-s.back())*(dtimeds_prev*dtimeds_first));
}

template<typename Dynamic_model_t>
template<bool isClosed>
inline void Optimal_laptime<Dynamic_model_t>::compute_derivative(const Dynamic_model_t& car, const std::array<scalar,Dynamic_model_t::NCONTROL>& dissipations) 
{
    FG_derivative<isClosed> fg(n_elements,n_points,car,s,q.front(),qa.front(),u.front(),dissipations);
    typename std::vector<scalar> x0(fg.get_n_variables(),0.0);

    // Set minimum and maximum variables
    auto [u_lb, u_ub, dudt_lb, dudt_ub] = car.optimal_laptime_control_bounds();
    auto [q_lb, q_ub] = Dynamic_model_t::optimal_laptime_state_bounds();
    auto [qa_lb, qa_ub] = Dynamic_model_t::optimal_laptime_algebraic_state_bounds();

    // Set state bounds
    std::vector<scalar> x_lb(fg.get_n_variables(), -1.0e24);
    std::vector<scalar> x_ub(fg.get_n_variables(), +1.0e24);

    // Fill x0 with the initial condition as state vector, and zero controls
    size_t k = 0;
    constexpr const size_t offset = isClosed ? 0 : 1;

    for (size_t i = offset; i < n_points; ++i)
    {
        // Set state before time
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)    
        {
            x0[k] = q[i][j];
            x_lb[k] = q_lb[j];
            x_ub[k] = q_ub[j];
            k++;
        }

        // Set state to IN. Assert that ITIME = IN - 1
        assert(Dynamic_model_t::Road_type::ITIME == ( Dynamic_model_t::Road_type::IN - 1 ) );

        x0[k] = q[i][Dynamic_model_t::Road_type::IN];
        x_lb[k] = -car.get_road().get_left_track_limit(s[i]);
        x_ub[k] =  car.get_road().get_right_track_limit(s[i]);
        k++;

        // Set state after the normal 
        for (size_t j = Dynamic_model_t::Road_type::IN+1; j < Dynamic_model_t::NSTATE; ++j)    
        {
            x0[k] = q[i][j];
            x_lb[k] = q_lb[j];
            x_ub[k] = q_ub[j];
            k++;
        }

        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
        {
            x0[k] = qa[i][j];
            x_lb[k] = qa_lb[j];
            x_ub[k] = qa_ub[j];
            k++;
        }

        // Set controls
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            x0[k] = u[i][j];
            x_lb[k] = u_lb[j];
            x_ub[k] = u_ub[j];
            k++;
        }

        // Set control derivatives: initially zero
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            x0[k] = 0.0;
            x_lb[k] = dudt_lb[j];
            x_ub[k] = dudt_ub[j];
            k++;
        }
    }

    // Fill the constraints upper/lower bounds vector
    std::vector<scalar> c_lb(fg.get_n_constraints(),0.0);
    std::vector<scalar> c_ub(fg.get_n_constraints(),0.0);
    size_t kc = 0;
    for (size_t i = 1; i < n_points; ++i)
    {
        // Equality constraints: --------------- 
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }

        // q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }

        // algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }
        
        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        auto [c_extra_lb, c_extra_ub] = car.optimal_laptime_extra_constraints_bounds(s[i]); 
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
        {
            c_lb[kc] = c_extra_lb[j];
            c_ub[kc] = c_extra_ub[j];
            kc++;
        }

        // control derivatives
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {   
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }
    }

    // Add the periodic element if track is closed
    if constexpr (isClosed)
    {
        // Equality constraints: --------------- 
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }

        // q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }

        // algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
        {
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }
        
        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        auto [c_extra_lb, c_extra_ub] = car.optimal_laptime_extra_constraints_bounds(s[0]); 
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
        {
            c_lb[kc] = c_extra_lb[j];
            c_ub[kc] = c_extra_ub[j];
            kc++;
        }

        // control derivatives
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {   
            c_lb[kc] = 0.0;
            c_ub[kc] = 0.0;
            kc++;
        }
    }

    assert(k == fg.get_n_variables());
    assert(kc == fg.get_n_constraints());

    // options
    std::string ipoptoptions;
    // turn off any printing
    ipoptoptions += "Integer print_level  ";
    ipoptoptions += std::to_string(options.print_level);
    ipoptoptions += "\n";
    ipoptoptions += "String  sb           yes\n";
    ipoptoptions += "Sparse true forward\n";
    ipoptoptions += "Numeric tol          1e-10\n";
    ipoptoptions += "Numeric constr_viol_tol  1e-10\n";
    ipoptoptions += "Numeric acceptable_tol  1e-8\n";

    // place to return solution
    CppAD::ipopt::solve_result<std::vector<scalar>> result;

    // solve the problem
    CppAD::ipopt::solve<std::vector<scalar>, FG_derivative<isClosed>>(ipoptoptions, x0, x_lb, x_ub, c_lb, c_ub, fg, result);

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
  
    // Export the solution
    assert(fg.get_states().size() == n_points);
    assert(fg.get_algebraic_states().size() == n_points);
    assert(fg.get_controls().size() == n_points);

    q = std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
            q[i][j] = Value(fg.get_state(i)[j]);

    qa = std::vector<std::array<scalar,Dynamic_model_t::NALGEBRAIC>>(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            qa[i][j] = Value(fg.get_algebraic_state(i)[j]);

    u = std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>>(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            u[i][j] = Value(fg.get_control(i)[j]);

    // Compute the time, x, y, and psi
    x_coord = std::vector<scalar>(fg.get_states().size());
    y_coord = std::vector<scalar>(fg.get_states().size());
    psi = std::vector<scalar>(fg.get_states().size());

    const scalar& L = car.get_road().track_length();
    auto dtimeds_first = fg.get_car()(fg.get_state(0),fg.get_algebraic_state(0),fg.get_control(0),0.0).first[Dynamic_model_t::Road_type::ITIME];
    auto dtimeds_prev = dtimeds_first;
    x_coord.front() = Value(fg.get_car().get_road().get_x());
    y_coord.front() = Value(fg.get_car().get_road().get_y());
    psi.front() = Value(fg.get_car().get_road().get_psi());
    for (size_t i = 1; i < fg.get_states().size(); ++i)
    {
        const auto dtimeds = fg.get_car()(fg.get_state(i),fg.get_algebraic_state(i),fg.get_control(i),s[i]).first[Dynamic_model_t::Road_type::ITIME];
        q[i][Dynamic_model_t::Road_type::ITIME] = q[i-1][Dynamic_model_t::Road_type::ITIME] + Value(0.5*(s[i]-s[i-1])*(dtimeds + dtimeds_prev));
        dtimeds_prev = dtimeds;

        x_coord[i] = Value(fg.get_car().get_road().get_x());
        y_coord[i] = Value(fg.get_car().get_road().get_y());
        psi[i]     = Value(fg.get_car().get_road().get_psi());
    }

    laptime = q.back()[Dynamic_model_t::Road_type::ITIME];

    if (isClosed)
        laptime += Value(0.5*(L-s.back())*(dtimeds_prev+dtimeds_first));
}


template<typename Dynamic_model_t>
std::unique_ptr<Xml_document> Optimal_laptime<Dynamic_model_t>::xml() const 
{
    const auto [key_name, q_names, qa_names, u_names] = Dynamic_model_t::get_state_and_control_names();

    std::ostringstream s_out;
    s_out.precision(17);
    std::unique_ptr<Xml_document> doc_ptr(std::make_unique<Xml_document>());

    doc_ptr->create_root_element("optimal_laptime");

    auto root = doc_ptr->get_root_element();
    
    root.add_attribute("n_points",std::to_string(n_points));
    
    if ( is_closed )
        root.add_attribute("type", "closed");
    else
        root.add_attribute("type", "open");

    // Save arclength
    for (size_t j = 0; j < s.size()-1; ++j)
        s_out << s[j] << ", ";

    s_out << s.back();

    root.add_child("laptime").set_value(std::to_string(laptime));

    root.add_child("arclength").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    // Save state
    for (size_t i = 0; i < Dynamic_model_t::NSTATE; ++i)
    {
        for (size_t j = 0; j < q.size()-1; ++j)
            s_out << q[j][i] << ", " ;

        s_out << q.back()[i];

        root.add_child(q_names[i]).set_value(s_out.str());
        s_out.str(""); s_out.clear();
    }


    // Save algebraic state
    for (size_t i = 0; i < Dynamic_model_t::NALGEBRAIC; ++i)
    {
        for (size_t j = 0; j < qa.size()-1; ++j)
            s_out << qa[j][i] << ", " ;

        s_out << qa.back()[i];

        root.add_child(qa_names[i]).set_value(s_out.str());
        s_out.str(""); s_out.clear();
    }

    // Save controls
    for (size_t i = 0; i < Dynamic_model_t::NCONTROL; ++i)
    {
        for (size_t j = 0; j < u.size()-1; ++j)
            s_out << u[j][i] << ", " ;

        s_out << u.back()[i];

        root.add_child(u_names[i]).set_value(s_out.str());
        s_out.str(""); s_out.clear();
    }

    // Save x, y, and psi if they are not contained
    for (size_t j = 0; j < q.size()-1; ++j)
        s_out << x_coord[j] << ", " ;

    s_out << x_coord.back();
    root.add_child("x").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < q.size()-1; ++j)
        s_out << y_coord[j] << ", " ;

    s_out << y_coord.back();
    root.add_child("y").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < q.size()-1; ++j)
        s_out << psi[j] << ", " ;

    s_out << psi.back();
    root.add_child("psi").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    return doc_ptr;
}


template<typename Dynamic_model_t>
template<bool isClosed>
inline void Optimal_laptime<Dynamic_model_t>::FG_direct<isClosed>::operator()(FG_direct<isClosed>::ADvector& fg, const Optimal_laptime<Dynamic_model_t>::FG_direct<isClosed>::ADvector& x)
{
    auto& _n_points      = FG::_n_points;
    auto& _car           = FG::_car;
    auto& _s             = FG::_s;
    auto& _q0            = FG::_q0;
    auto& _qa0           = FG::_qa0;
    auto& _u0            = FG::_u0;
    auto& _q             = FG::_q;
    auto& _qa            = FG::_qa;
    auto& _u             = FG::_u;
    auto& _dqdt          = FG::_dqdt;
    auto& _dqa           = FG::_dqa ;
    auto& _dissipations  = FG::_dissipations;


    assert(x.size() == FG::_n_variables);
    assert(fg.size() == (1 + FG::_n_constraints));

    // Load the state and control vectors
    size_t k = 0;

    if constexpr (!isClosed)
    {
        // Load the state in the first position
        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
            _q[0][j] = _q0[j];

        // Load the algebraic state in the first position
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            _qa[0][j] = _qa0[j];
    
        // Load the control in the first position
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            _u[0][j] = _u0[j];
    }

    constexpr const size_t offset = (isClosed ? 0 : 1);

    for (size_t i = offset; i < _n_points; ++i)
    {
        // Load state (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            _q[i][j] = x[k++];

        // Load state (after time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            _q[i][j] = x[k++];

        // Load algebraic state
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            _qa[i][j] = x[k++];
            
        // Load control
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            _u[i][j] = x[k++];
    }

    // Check that all variables in x were used
    assert(k == FG::_n_variables);

    // Initialize fitness function
    fg[0] = 0.0;

    // Loop over the nodes
    
    std::tie(_dqdt[0], _dqa[0]) = _car(_q[0],_qa[0],_u[0],_s[0]);
    k = 1;  // Reset the counter
    for (size_t i = 1; i < _n_points; ++i)
    {
        std::tie(_dqdt[i],_dqa[i]) = _car(_q[i],_qa[i],_u[i],_s[i]);

        // Fitness function: integral of time
        fg[0] += 0.5*(_s[i]-_s[i-1])*(_dqdt[i-1][Dynamic_model_t::Road_type::ITIME] + _dqdt[i][Dynamic_model_t::Road_type::ITIME]);

        // Equality constraints: --------------- 
        // q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            fg[k++] = _q[i][j] - _q[i-1][j] - 0.5*(_s[i]-_s[i-1])*(_dqdt[i-1][j] + _dqdt[i][j]);

        // q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _q[i][j] - _q[i-1][j] - 0.5*(_s[i]-_s[i-1])*(_dqdt[i-1][j] + _dqdt[i][j]);

        // algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            fg[k++] = _dqa[i][j];
        
        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        auto c_extra = _car.optimal_laptime_extra_constraints();

        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];
    }

    // Add a penalisation to the controls
    for (size_t i = 1; i < _n_points; ++i)
    {
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            const auto derivative = (_u[i][j]-_u[i-1][j])/(_s[i]-_s[i-1]);
            fg[0] += _dissipations[j]*(derivative*derivative)*(_s[i]-_s[i-1]);
        }
    }

    // Add the periodic element if track is closed
    const scalar& L = _car.get_road().track_length();
    if constexpr (isClosed)
    {
        // Fitness function: integral of time
        fg[0] += 0.5*(L-_s.back())*(_dqdt.front()[Dynamic_model_t::Road_type::ITIME] + _dqdt.back()[Dynamic_model_t::Road_type::ITIME]);

        // Equality constraints: 

        // q^{0} = q^{n-1} + 0.5.ds.[dqdt^{0} + dqdt^{n-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            fg[k++] = _q.front()[j] - _q.back()[j] - 0.5*(L-_s.back())*(_dqdt.back()[j] + _dqdt.front()[j]);

        // q^{0} = q^{n-1} + 0.5.ds.[dqdt^{0} + dqdt^{n-1}] (after time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _q.front()[j] - _q.back()[j] - 0.5*(L-_s.back())*(_dqdt.back()[j] + _dqdt.front()[j]);

        // algebraic constraints: dqa^{0} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            fg[k++] = _dqa.front()[j];

        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        _car(_q[0],_qa[0],_u[0],_s[0]);
        const auto c_extra = _car.optimal_laptime_extra_constraints();
    
        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        // Add the penalisation to the controls
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            const auto derivative = (_u.front()[j]-_u.back()[j])/(L-_s.back());
            fg[0] += _dissipations[j]*(derivative*derivative)*(L-_s.back());
        }
    }

    assert(k == FG::_n_constraints+1);
}


template<typename Dynamic_model_t>
template<bool isClosed>
inline void Optimal_laptime<Dynamic_model_t>::FG_derivative<isClosed>::operator()(FG_derivative<isClosed>::ADvector& fg, const Optimal_laptime<Dynamic_model_t>::FG_derivative<isClosed>::ADvector& x)
{
    auto& _n_points      = FG::_n_points;
    auto& _car           = FG::_car;
    auto& _s             = FG::_s;
    auto& _q0            = FG::_q0;
    auto& _qa0           = FG::_qa0;
    auto& _u0            = FG::_u0;
    auto& _q             = FG::_q;
    auto& _qa            = FG::_qa;
    auto& _u             = FG::_u;
    auto& _dqdt          = FG::_dqdt;
    auto& _dqa           = FG::_dqa ;
    auto& _dissipations  = FG::_dissipations;

    assert(x.size() == FG::_n_variables);
    assert(fg.size() == (1 + FG::_n_constraints));

    // Load the state and control vectors
    size_t k = 0;

    if constexpr (!isClosed)
    {
        // Load the state in the first position
        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
            _q[0][j] = _q0[j];

        // Load the algebraic state in the first position
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            _qa[0][j] = _qa0[j];
    
        // Load the control in the first position
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            _u[0][j] = _u0[j];

        // Load the control derivative in the first position
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            _dudt[0][j] = 0.0;
    }

    constexpr const size_t offset = (isClosed ? 0 : 1);

    for (size_t i = offset; i < _n_points; ++i)
    {
        // Load state (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            _q[i][j] = x[k++];

        // Load state (after time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            _q[i][j] = x[k++];

        // Load algebraic states
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            _qa[i][j] = x[k++];
            
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
    std::tie(_dqdt[0],_dqa[0]) = _car(_q[0],_qa[0],_u[0],_s[0]);
    k = 1;  // Reset the counter
    for (size_t i = 1; i < _n_points; ++i)
    {
        std::tie(_dqdt[i], _dqa[i]) = _car(_q[i],_qa[i],_u[i],_s[i]);

        // Fitness function: integral of time
        fg[0] += 0.5*(_s[i]-_s[i-1])*(_dqdt[i-1][Dynamic_model_t::Road_type::ITIME] + _dqdt[i][Dynamic_model_t::Road_type::ITIME]);

        // Equality constraints: 

        // q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            fg[k++] = _q[i][j] - _q[i-1][j] - 0.5*(_s[i]-_s[i-1])*(_dqdt[i-1][j] + _dqdt[i][j]);

        // q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _q[i][j] - _q[i-1][j] - 0.5*(_s[i]-_s[i-1])*(_dqdt[i-1][j] + _dqdt[i][j]);

        // algebraic constraints: dqa^{i} = 0.0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            fg[k++] = _dqa[i][j];

        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        auto c_extra = _car.optimal_laptime_extra_constraints();

        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            fg[k++] = _u[i][j] - _u[i-1][j] - 0.5*(_s[i]-_s[i-1])*(_dudt[i-1][j]*_dqdt[i-1][Dynamic_model_t::Road_type::ITIME]+_dudt[i][j]*_dqdt[i][Dynamic_model_t::Road_type::ITIME]);
    }

    // Add a penalisation to the controls (TODO do this properly)
    for (size_t i = 1; i < _n_points; ++i)
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            fg[0] += _dissipations[j]*(_dudt[i][j]*_dudt[i][j])*(_s[i]-_s[i-1]);

    // Add the periodic element if track is closed
    const scalar& L = _car.get_road().track_length();
    if constexpr (isClosed)
    {
        // Fitness function: integral of time
        fg[0] += 0.5*(L-_s.back())*(_dqdt.front()[Dynamic_model_t::Road_type::ITIME] + _dqdt.back()[Dynamic_model_t::Road_type::ITIME]);

        // Equality constraints: 

        // q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = 0; j < Dynamic_model_t::Road_type::ITIME; ++j)
            fg[k++] = _q.front()[j] - _q.back()[j] - 0.5*(L-_s.back())*(_dqdt.back()[j] + _dqdt.front()[j]);

        // q^{i} = q^{i-1} + 0.5.ds.[dqdt^{i} + dqdt^{i-1}] (before time)
        for (size_t j = Dynamic_model_t::Road_type::ITIME+1; j < Dynamic_model_t::NSTATE; ++j)
            fg[k++] = _q.front()[j] - _q.back()[j] - 0.5*(L-_s.back())*(_dqdt.back()[j] + _dqdt.front()[j]);

        // algebraic constraints: dqa^{0} = 0
        for (size_t j = 0; j < Dynamic_model_t::NALGEBRAIC; ++j)
            fg[k++] = _dqa.front()[j];

        // Inequality constraints: -0.11 < kappa < 0.11, -0.11 < lambda < 0.11
        _car(_q[0],_qa[0],_u[0],_s[0]);
        auto c_extra = _car.optimal_laptime_extra_constraints();

        for (size_t j = 0; j < Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS; ++j)
            fg[k++] = c_extra[j];

        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
            fg[k++] = _u.front()[j] - _u.back()[j] - 0.5*(L-_s.back())*(_dudt.back()[j]*_dqdt.back()[Dynamic_model_t::Road_type::ITIME]+_dudt.front()[j]*_dqdt.front()[Dynamic_model_t::Road_type::ITIME]);

        // Add the penalisation to the controls
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            fg[0] += _dissipations[j]*(_dudt[0][j]*_dudt[0][j])*(L-_s.back());
        }
    }

    assert(k == FG::_n_constraints+1);
}
