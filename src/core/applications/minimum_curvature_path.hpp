#ifndef MINIMUM_CURVATURE_PATH_HPP
#define MINIMUM_CURVATURE_PATH_HPP

#include "lion/thirdparty/include/cppad/ipopt/solve.hpp"

inline std::vector<scalar> Minimum_curvature_path::compute_track_by_arcs(const Track_by_arcs& track, const size_t N) 
{
    Fitness_fcn f(N,track);

    // initial value of the independent variables
    std::vector<scalar> xi(N,0.0);

    // lower and upper limits for x
    std::vector<scalar> xl(N,-10.0), xu(N,10.0);

    // lower and upper limits for g
    std::vector<scalar> gl, gu;

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    // maximum number of iterations
    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-6\n";
    // derivative testing
    options += "String  derivative_test            second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius  0.\n";

    options += "Sparse true forward\n";
    options += "Retape false\n";

    // place to return solution
    CppAD::ipopt::solve_result<std::vector<scalar>> solution;

    // solve the problem
    CppAD::ipopt::solve<std::vector<scalar>, Fitness_fcn>(
        options, xi, xl, xu, gl, gu, f, solution
    );

    _x = solution.x;
    success = solution.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success;
    
    return solution.x;
}


template<bool is_closed>
inline void Minimum_curvature_path::compute_track_by_polynomial(const Track_by_polynomial& track, const std::vector<scalar>& s_)
{
    s = s_;
    num_points = s.size();
    num_elements = (is_closed ? num_points : num_points - 1);

    // (2) Verify the vector of arclength
    const scalar& L = track.get_total_length();

    if constexpr (is_closed)
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
    if constexpr (is_closed)
    {   
        s.front() = 0.0;
    }

    // (3) Construct fitness function
    Fitness_fcn_by_polynomial<is_closed> fg(track, num_points, num_elements, s);

    // (4) Set initial condition and bounds
    std::vector<scalar> x_start(fg.get_n_variables(), 0.0);
    std::vector<scalar> x_lb(fg.get_n_variables(), 0.0);
    std::vector<scalar> x_ub(fg.get_n_variables(), 0.0);

    using input_names = Fitness_fcn_by_polynomial<is_closed>::input_names;

    for (size_t i_point = 0; i_point < num_points; ++i_point)
    {
        const auto [position, euler_angles, deuler_angles] = track(s[i_point]);

        x_start[input_names::end * i_point + input_names::n] = 0.0;
        x_lb[input_names::end * i_point + input_names::n] = -track.get_left_track_limit(s[i_point]);
        x_ub[input_names::end * i_point + input_names::n] =  track.get_right_track_limit(s[i_point]);

        x_start[input_names::end * i_point + input_names::theta] = euler_angles.yaw();
        x_lb[input_names::end * i_point + input_names::theta] = euler_angles.yaw() - 60.0*DEG;
        x_ub[input_names::end * i_point + input_names::theta] = euler_angles.yaw() + 60.0*DEG;

        x_start[input_names::end * i_point + input_names::kappa] = deuler_angles.yaw();
        x_lb[input_names::end * i_point + input_names::kappa] = deuler_angles.yaw() - 1.0;
        x_ub[input_names::end * i_point + input_names::kappa] = deuler_angles.yaw() + 1.0;
    }

    for (size_t i_point = 1; i_point < num_points; ++i_point)
    {
        x_start[input_names::end * (i_point-1) + input_names::delta_path_arclength] = s[i_point] - s[i_point-1];
        x_lb[input_names::end * (i_point-1) + input_names::delta_path_arclength] = 0.05*(s[i_point]-s[i_point-1]);
        x_ub[input_names::end * (i_point-1) + input_names::delta_path_arclength] = 5.00*(s[i_point]-s[i_point-1]);
    }

    if (is_closed)
    {
        x_start[input_names::end * (num_points-1) + input_names::delta_path_arclength] = track.get_total_length() - s.back();
        x_lb[input_names::end * (num_points-1) + input_names::delta_path_arclength] = 0.05*(track.get_total_length() - s.back());
        x_ub[input_names::end * (num_points-1) + input_names::delta_path_arclength] = 5.00*(track.get_total_length() - s.back());
    }

    // (5) Set constraints bounds
    std::vector<scalar> c_lb(fg.get_n_constraints(), 0.0), c_ub(fg.get_n_constraints(), 0.0);

    // (6) Run optimization

    // (6.1) Prepare options
    std::ostringstream ipoptoptions; ipoptoptions << std::setprecision(17);
    ipoptoptions << "Integer print_level " << options.print_level        << std::endl;
    ipoptoptions << "Integer max_iter "    << options.maximum_iterations << std::endl;
    ipoptoptions << "String  sb           yes\n";
    ipoptoptions << "Sparse true forward\n";
    ipoptoptions << "Numeric tol "             << options.nlp_tolerance              << std::endl;
    ipoptoptions << "Numeric constr_viol_tol " << options.constraints_viol_tolerance << std::endl;
    ipoptoptions << "Numeric acceptable_tol "  << options.acceptable_tolerance       << std::endl;

    // (8.2) Return object
    CppAD::ipopt_cppad_result<std::vector<scalar>> result;

    // (8.3) Solve the problem
    CppAD::ipopt_cppad_solve<std::vector<scalar>, Fitness_fcn_by_polynomial<is_closed>>(ipoptoptions.str(), x_start, x_lb, x_ub, c_lb, c_ub, fg, result);
 
    // (8.4) Check success flag
    success = result.status == CppAD::ipopt_cppad_result<std::vector<scalar>>::success; 
    iter_count = result.iter_count;

    if ( !success && options.throw_if_fail )
    {
        throw fastest_lap_exception("Optimization did not succeed");
    }

    // Export
    std::vector<CppAD::AD<scalar>> x_cppad(fg.get_n_variables(), 0.0), fg_cppad(fg.get_n_constraints() + 1, 0.0);
    std::copy(result.x.cbegin(), result.x.cend(), x_cppad.begin());
    fg(fg_cppad, x_cppad);

    delta_path_arclength = std::vector<scalar>(num_elements,0.0);
    std::transform(fg._delta_path_arclength.cbegin(), fg._delta_path_arclength.cend(), delta_path_arclength.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    n = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._n.cbegin(), fg._n.cend(), n.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    x = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._x.cbegin(), fg._x.cend(), x.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    y = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._y.cbegin(), fg._y.cend(), y.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    theta = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._theta.cbegin(), fg._theta.cend(), theta.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    kappa = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._kappa.cbegin(), fg._kappa.cend(), kappa.begin(), [](const auto& x_cppad) { return Value(x_cppad); });
}

std::unique_ptr<Xml_document> Minimum_curvature_path::xml() const
{
    std::unique_ptr<Xml_document> doc_ptr(std::make_unique<Xml_document>());

    std::ostringstream s_out;
    s_out.precision(17);

    doc_ptr->create_root_element("minimum_curvature");

    auto root = doc_ptr->get_root_element();

    for (size_t j = 0; j < x.size()-1; ++j)
        s_out << x[j] << ", " ;

    s_out << x.back();
    root.add_child("x").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < y.size() - 1; ++j)
        s_out << y[j] << ", " ;

    s_out << y.back();
    root.add_child("y").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < n.size() - 1; ++j)
        s_out << n[j] << ", " ;

    s_out << n.back();
    root.add_child("lateral_displacement").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < theta.size() - 1; ++j)
        s_out << theta[j] << ", " ;

    s_out << theta.back();
    root.add_child("theta").set_value(s_out.str());
    s_out.str(""); s_out.clear();


    for (size_t j = 0; j < kappa.size() - 1; ++j)
        s_out << kappa[j] << ", " ;

    s_out << kappa.back();
    root.add_child("kappa").set_value(s_out.str());
    s_out.str(""); s_out.clear();


    return doc_ptr;
}


inline void Minimum_curvature_path::Fitness_fcn::operator()(Minimum_curvature_path::Fitness_fcn::ADvector& fg, const Minimum_curvature_path::Fitness_fcn::ADvector& x) const
{
    assert(fg.size() == 1);
    assert(x.size() == _n);

    const scalar ds = _track.get_total_length() / ((scalar)_n); 
    const scalar inv_ds = 1.0/ds;

    const ADvector& w = x;
    
    // Compute derivatives of the normal distance
    ADvector dw(_n);
    ADvector d2w(_n);

    // Compute derivative of interior points
    for (size_t i = 1; i < _n-1; ++i) 
    {
        dw[i] = (w[i+1]-w[i-1])*0.5*inv_ds;
        d2w[i] = (w[i+1]+w[i-1]-2.0*w[i])*inv_ds*inv_ds;
    }

    // Compute derivatives of first point
    dw[0] = (w[1] - w[_n-1])*0.5*inv_ds;
    d2w[0] = (w[1] + w[_n-1] - 2.0*w[0])*inv_ds*inv_ds;

    dw[_n-1] = (w[0] - w[_n-2])*0.5*inv_ds;
    d2w[_n-1] = (w[0] + w[_n-2] - 2.0*w[_n-1])*inv_ds*inv_ds;

    // Compute the curvature
    CppAD::AD<scalar> total_curvature = 0.0;
    for (size_t i = 0; i < _n; ++i)
    {
        const scalar s = ((scalar)i)*ds;
        total_curvature += _track.pseudo_curvature2_at(s,w[i],dw[i],d2w[i])*ds;
    }

    fg[0] = total_curvature;
}


template<bool is_closed>
inline void Minimum_curvature_path::Fitness_fcn_by_polynomial<is_closed>::operator()(ADvector& fg, const ADvector& x)
{
    // (1) Check dimensions
    assert(x.size() == _num_variables);
    assert(fg.size() == _num_constraints + 1);

    // (2) Load variables
    for (size_t i_point = 0; i_point < _num_points; ++i_point)
    {
        _n[i_point] = x[input_names::end * i_point + input_names::n];
        _theta[i_point] = x[input_names::end * i_point + input_names::theta];
        _kappa[i_point] = x[input_names::end * i_point + input_names::kappa];
    }

    for (size_t i_element = 0; i_element < _num_elements; ++i_element)
    {
        _delta_path_arclength[i_element] = x[input_names::end * i_element + input_names::delta_path_arclength];
    }

    // (3) Compute auxiliary variables
    for (size_t i_point = 0; i_point < _num_points; ++i_point)
    {
        Vector3d<CppAD::AD<scalar>> normal_vector = { -sin(_theta[i_point]), cos(_theta[i_point]), 0.0};
        const auto centerline = _track(_s[i_point]).position;
        _x[i_point] = centerline.x() + _n[i_point] * normal_vector.x();
        _y[i_point] = centerline.y() + _n[i_point] * normal_vector.y();
    }

    // (4) Compute fitness function and constraints: the definition of theta and kappa
    size_t constraint_counter = 0;
    fg.front() = 0.0;
    for (size_t i_point = 1; i_point < _num_points; ++i_point)
    {
        const auto ds = _delta_path_arclength[i_point-1];
        fg.front() += 0.5 * ds * (_kappa[i_point-1] * _kappa[i_point-1] + _kappa[i_point] * _kappa[i_point]);
        fg[++constraint_counter] = _x[i_point] - _x[i_point-1] - 0.5 * ds * (cos(_theta[i_point]) + cos(_theta[i_point-1]));
        fg[++constraint_counter] = _y[i_point] - _y[i_point-1] - 0.5 * ds * (sin(_theta[i_point]) + sin(_theta[i_point-1]));
        fg[++constraint_counter] = _theta[i_point] - _theta[i_point - 1] - 0.5 * ds * (_kappa[i_point] + _kappa[i_point - 1]);
    }

    if constexpr (is_closed)
    {
        const auto ds = _delta_path_arclength.back();
        fg.front() += 0.5 * ds * (_kappa.back() * _kappa.back() + _kappa.front() * _kappa.front());
        fg[++constraint_counter] = _x.front() - _x.back() - 0.5 * ds * (cos(_theta.front()) + cos(_theta.back()));
        fg[++constraint_counter] = _y.front() - _y.back() - 0.5 * ds * (sin(_theta.front()) + sin(_theta.back()));
        fg[++constraint_counter] = _theta.front() - _theta.back() - 0.5 * ds * (_kappa.front() + _kappa.back()) + 2.0 * pi * _track.get_preprocessor().direction;
    }

    assert(constraint_counter == _num_constraints);

    return;
}

#endif
