#include "lion/thirdparty/include/cppad/ipopt/solve.hpp"

inline std::vector<scalar> Minimum_curvature_path::compute(const Track_by_arcs& track, const size_t N) 
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
    _success = solution.status == CppAD::ipopt::solve_result<std::vector<scalar>>::success;
    
    return solution.x;
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


