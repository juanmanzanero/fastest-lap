#ifndef MINIMUM_CURVATURE_PATH_HPP
#define MINIMUM_CURVATURE_PATH_HPP

#include "lion/thirdparty/include/cppad/ipopt/solve.hpp"
#include "lion/math/polynomial.h"
#include "lion/math/ipopt_cppad_handler.hpp"

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
inline void Minimum_curvature_path::compute_track_by_polynomial(const Circuit_geometry& circuit_geometry, const std::vector<scalar>& s_compute_)
{
    s_compute = s_compute_;
    num_points = s_compute.size();
    num_elements = (is_closed ? num_points : num_points - 1);

    // (2) Verify the vector of arclength
    const scalar& L = circuit_geometry.track_length;

    if constexpr (is_closed)
    {
        if (std::abs(s_compute.front()) > 1.0e-12)
            throw fastest_lap_exception("In closed circuits, s_compute[0] should be 0.0");

        if (s_compute.back() > L - 1.0e-10)
            throw fastest_lap_exception("In closed circuits, s_compute[end] should be < track_length");
    }
    else
    {
        if (s_compute[0] < -1.0e-12)
            throw fastest_lap_exception("s_compute[0] must be >= 0");

        if (s_compute.back() > L)
            throw fastest_lap_exception("s_compute[end] must be <= L");
    }

    // If closed, replace the initial arclength by 0
    if constexpr (is_closed)
    {   
        s_compute.front() = 0.0;
    }

    using input_names = typename Fitness_fcn_by_polynomial<is_closed>::input_names;

    const auto centerline_poly = Polynomial(circuit_geometry.s, circuit_geometry.r_centerline, 1, false);

    const auto yaw_poly = sPolynomial(circuit_geometry.s, circuit_geometry.yaw, 1, false);
    const auto pitch_poly = circuit_geometry.pitch.size() > 0 ? sPolynomial(circuit_geometry.s, circuit_geometry.pitch, 1, false) : sPolynomial();
    const auto roll_poly = circuit_geometry.roll.size() > 0 ? sPolynomial(circuit_geometry.s, circuit_geometry.roll, 1, false) : sPolynomial();


    const auto yaw_dot_poly = sPolynomial(circuit_geometry.s, circuit_geometry.yaw_dot, 1, false);
    const auto pitch_dot_poly = circuit_geometry.pitch_dot.size() > 0 ? sPolynomial(circuit_geometry.s, circuit_geometry.pitch_dot, 1, false) : sPolynomial();
    const auto roll_dot_poly = circuit_geometry.roll_dot.size() > 0 ? sPolynomial(circuit_geometry.s, circuit_geometry.roll_dot, 1, false) : sPolynomial();

    const auto nl_poly = sPolynomial(circuit_geometry.s, circuit_geometry.nl, 1, false);
    const auto nr_poly = sPolynomial(circuit_geometry.s, circuit_geometry.nr, 1, false);

    std::vector<sVector3d> centerline(num_points);
    std::vector<sVector3d> normal_vector(num_points);


    for (size_t i_point = 0; i_point < num_points; ++i_point)
    {
        const auto yaw = yaw_poly(s_compute[i_point]);
        const auto pitch = pitch_poly(s_compute[i_point]);
        const auto roll = roll_poly(s_compute[i_point]);

        centerline[i_point] = centerline_poly(s_compute[i_point]);

        normal_vector[i_point] = sVector3d{ cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll),
                                   sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll),
                                   cos(pitch) * sin(roll) };
    }

    // (3) Construct fitness function
    Fitness_fcn_by_polynomial<is_closed> fg(num_points, num_elements, s_compute, centerline, normal_vector, circuit_geometry.direction);


    // (4) Set initial condition and bounds
    std::vector<scalar> x_start(fg.get_n_variables(), 0.0);
    std::vector<scalar> x_lb(fg.get_n_variables(), 0.0);
    std::vector<scalar> x_ub(fg.get_n_variables(), 0.0);


    for (size_t i_point = 0; i_point < num_points; ++i_point)
    {
        const auto yaw = yaw_poly(s_compute[i_point]);
        const auto pitch = pitch_poly(s_compute[i_point]);
        const auto roll = roll_poly(s_compute[i_point]);

        const auto yaw_dot = yaw_dot_poly(s_compute[i_point]);
        const auto pitch_dot = pitch_dot_poly(s_compute[i_point]);
        const auto roll_dot = roll_dot_poly(s_compute[i_point]);

        x_start[input_names::end * i_point + input_names::n] = 0.0;
        x_lb[input_names::end * i_point + input_names::n] = -nl_poly(s_compute[i_point]);
        x_ub[input_names::end * i_point + input_names::n] =  nr_poly(s_compute[i_point]);

        x_start[input_names::end * i_point + input_names::yaw] = yaw;
        x_lb[input_names::end * i_point + input_names::yaw] = yaw - 60.0*DEG;
        x_ub[input_names::end * i_point + input_names::yaw] = yaw + 60.0*DEG;

        x_start[input_names::end * i_point + input_names::pitch] = pitch;
        x_lb[input_names::end * i_point + input_names::pitch] = pitch - 30.0*DEG;
        x_ub[input_names::end * i_point + input_names::pitch] = pitch + 30.0*DEG;

        x_start[input_names::end * i_point + input_names::roll] = roll;
        x_lb[input_names::end * i_point + input_names::roll] = roll - 30.0*DEG;
        x_ub[input_names::end * i_point + input_names::roll] = roll + 30.0*DEG;

        x_start[input_names::end * i_point + input_names::yaw_dot] = yaw_dot;
        x_lb[input_names::end * i_point + input_names::yaw_dot] = yaw_dot - 1.0;
        x_ub[input_names::end * i_point + input_names::yaw_dot] = yaw_dot + 1.0;

        x_start[input_names::end * i_point + input_names::pitch_dot] = pitch_dot;
        x_lb[input_names::end * i_point + input_names::pitch_dot] = pitch_dot - 1.0;
        x_ub[input_names::end * i_point + input_names::pitch_dot] = pitch_dot + 1.0;

        x_start[input_names::end * i_point + input_names::roll_dot] = roll_dot;
        x_lb[input_names::end * i_point + input_names::roll_dot] = roll_dot - 1.0;
        x_ub[input_names::end * i_point + input_names::roll_dot] = roll_dot + 1.0;
    }

    for (size_t i_point = 1; i_point < num_points; ++i_point)
    {
        x_start[input_names::end * (i_point-1) + input_names::delta_path_arclength] = s_compute[i_point] - s_compute[i_point-1];
        x_lb[input_names::end * (i_point-1) + input_names::delta_path_arclength] = 0.05*(s_compute[i_point]-s_compute[i_point-1]);
        x_ub[input_names::end * (i_point-1) + input_names::delta_path_arclength] = 5.00*(s_compute[i_point]-s_compute[i_point-1]);
    }

    if (is_closed)
    {
        x_start[input_names::end * (num_points-1) + input_names::delta_path_arclength] = circuit_geometry.track_length - s_compute.back();
        x_lb[input_names::end * (num_points-1) + input_names::delta_path_arclength] = 0.05*(circuit_geometry.track_length - s_compute.back());
        x_ub[input_names::end * (num_points-1) + input_names::delta_path_arclength] = 5.00*(circuit_geometry.track_length - s_compute.back());
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

    z = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._z.cbegin(), fg._z.cend(), z.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    yaw = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._yaw.cbegin(), fg._yaw.cend(), yaw.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    pitch = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._pitch.cbegin(), fg._pitch.cend(), pitch.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    roll = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._roll.cbegin(), fg._roll.cend(), roll.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    yaw_dot = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._yaw_dot.cbegin(), fg._yaw_dot.cend(), yaw_dot.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    pitch_dot = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._pitch_dot.cbegin(), fg._pitch_dot.cend(), pitch_dot.begin(), [](const auto& x_cppad) { return Value(x_cppad); });

    roll_dot = std::vector<scalar>(num_points, 0.0);
    std::transform(fg._roll_dot.cbegin(), fg._roll_dot.cend(), roll_dot.begin(), [](const auto& x_cppad) { return Value(x_cppad); });
}

inline std::unique_ptr<Xml_document> Minimum_curvature_path::xml() const
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

    for (size_t j = 0; j < z.size() - 1; ++j)
        s_out << z[j] << ", " ;

    s_out << z.back();
    root.add_child("z").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < n.size() - 1; ++j)
        s_out << n[j] << ", " ;

    s_out << n.back();
    root.add_child("lateral_displacement").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < yaw.size() - 1; ++j)
        s_out << yaw[j] << ", " ;

    s_out << yaw.back();
    root.add_child("yaw").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < pitch.size() - 1; ++j)
        s_out << pitch[j] << ", " ;

    s_out << pitch.back();
    root.add_child("pitch").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < roll.size() - 1; ++j)
        s_out << roll[j] << ", " ;

    s_out << roll.back();
    root.add_child("roll").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < yaw_dot.size() - 1; ++j)
        s_out << yaw_dot[j] << ", " ;

    s_out << yaw_dot.back();
    root.add_child("yaw_dot").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < pitch_dot.size() - 1; ++j)
        s_out << pitch_dot[j] << ", " ;

    s_out << pitch_dot.back();
    root.add_child("pitch_dot").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    for (size_t j = 0; j < roll_dot.size() - 1; ++j)
        s_out << roll_dot[j] << ", " ;

    s_out << roll_dot.back();
    root.add_child("roll_dot").set_value(s_out.str());
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
        _yaw[i_point] = x[input_names::end * i_point + input_names::yaw];
        _pitch[i_point] = x[input_names::end * i_point + input_names::pitch];
        _roll[i_point] = x[input_names::end * i_point + input_names::roll];
        _yaw_dot[i_point] = x[input_names::end * i_point + input_names::yaw_dot];
        _pitch_dot[i_point] = x[input_names::end * i_point + input_names::pitch_dot];
        _roll_dot[i_point] = x[input_names::end * i_point + input_names::roll_dot];
    }

    for (size_t i_element = 0; i_element < _num_elements; ++i_element)
    {
        _delta_path_arclength[i_element] = x[input_names::end * i_element + input_names::delta_path_arclength];
    }

    // (3) Compute auxiliary variables
    for (size_t i_point = 0; i_point < _num_points; ++i_point)
    {
        _x[i_point] = _centerline[i_point].x() + _n[i_point] * _normal_vector[i_point].x();
        _y[i_point] = _centerline[i_point].y() + _n[i_point] * _normal_vector[i_point].y();
        _z[i_point] = _centerline[i_point].z() + _n[i_point] * _normal_vector[i_point].z();

        _curvature_x[i_point] = _roll_dot[i_point] - sin(_pitch[i_point]) * _yaw_dot[i_point];
        _curvature_y[i_point] = cos(_roll[i_point]) * _pitch_dot[i_point] + cos(_pitch[i_point]) * sin(_roll[i_point]) * _yaw_dot[i_point];
        _curvature_z[i_point] = -sin(_roll[i_point]) * _pitch_dot[i_point] + cos(_pitch[i_point]) * cos(_roll[i_point]) * _yaw_dot[i_point];
    }

    // (4) Compute fitness function and constraints: the definition of yaw and yaw_dot
    size_t constraint_counter = 0;
    fg.front() = 0.0;
    for (size_t i_point = 1; i_point < _num_points; ++i_point)
    {
        const auto ds = _delta_path_arclength[i_point-1];
        fg.front() += 0.5 * ds * (_curvature_x[i_point]*_curvature_x[i_point] + _curvature_y[i_point]*_curvature_y[i_point] + _curvature_z[i_point]*_curvature_z[i_point]
                + _curvature_x[i_point - 1]*_curvature_x[i_point - 1] + _curvature_y[i_point - 1]*_curvature_y[i_point - 1] + _curvature_z[i_point - 1]*_curvature_z[i_point - 1]);

        fg[++constraint_counter] = _x[i_point] - _x[i_point - 1] - 0.5 * ds * (cos(_yaw[i_point]) * cos(_pitch[i_point]) + cos(_yaw[i_point - 1]) * cos(_pitch[i_point - 1]));
        fg[++constraint_counter] = _y[i_point] - _y[i_point-1] - 0.5 * ds * (sin(_yaw[i_point])*cos(_pitch[i_point]) + sin(_yaw[i_point - 1])*cos(_pitch[i_point-1]));
        fg[++constraint_counter] = _z[i_point] - _z[i_point-1] - 0.5 * ds * (sin(_pitch[i_point]) + sin(_pitch[i_point - 1]));
        fg[++constraint_counter] = _yaw[i_point] - _yaw[i_point - 1] - 0.5 * ds * (_yaw_dot[i_point] + _yaw_dot[i_point - 1]);
        fg[++constraint_counter] = _pitch[i_point] - _pitch[i_point - 1] - 0.5 * ds * (_pitch_dot[i_point] + _pitch_dot[i_point - 1]);
        fg[++constraint_counter] = _roll[i_point] - _roll[i_point - 1] - 0.5 * ds * (_roll_dot[i_point] + _roll_dot[i_point - 1]);
    }

    if constexpr (is_closed)
    {
        const auto ds = _delta_path_arclength.back();
        fg.front() += 0.5 * ds * (_yaw_dot.back() * _yaw_dot.back() + _yaw_dot.front() * _yaw_dot.front());
        fg[++constraint_counter] = _x.front() - _x.back() - 0.5 * ds * (cos(_yaw.front())*cos(_pitch.front()) + cos(_yaw.back())*cos(_pitch.back()));
        fg[++constraint_counter] = _y.front() - _y.back() - 0.5 * ds * (sin(_yaw.front())*cos(_pitch.front()) + sin(_yaw.back())*cos(_pitch.back()));
        fg[++constraint_counter] = _z.front() - _z.back() - 0.5 * ds * (sin(_pitch.front()) + sin(_pitch.back()));
        fg[++constraint_counter] = _yaw.front() - _yaw.back() - 0.5 * ds * (_yaw_dot.front() + _yaw_dot.back()) + 2.0 * pi * _circuit_direction;
        fg[++constraint_counter] = _pitch.front() - _pitch.back() - 0.5 * ds * (_pitch_dot.front() + _pitch_dot.back());
        fg[++constraint_counter] = _roll.front() - _roll.back() - 0.5 * ds * (_roll_dot.front() + _roll_dot.back());
    }

    assert(constraint_counter == _num_constraints);

    return;
}

#endif
