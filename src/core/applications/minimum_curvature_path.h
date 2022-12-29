#ifndef MINIMUM_CURVATURE_PATH_H
#define MINIMUM_CURVATURE_PATH_H

#include "src/core/vehicles/track_by_arcs.h"
#include "src/core/vehicles/track_by_polynomial.h"

#include "lion/thirdparty/include/cppad/cppad.hpp"

class Minimum_curvature_path
{
 public:
    struct Options
    {
        size_t print_level                = 0;
        size_t maximum_iterations         = 3000;
        bool   throw_if_fail              = true;
        scalar nlp_tolerance              = 1.0e-8;
        scalar constraints_viol_tolerance = 1.0e-8;
        scalar acceptable_tolerance       = 1.0e-8;
    };

    Minimum_curvature_path(const Track_by_arcs& track, const size_t N) { compute_track_by_arcs(track,N); }

    Minimum_curvature_path(const Track_by_polynomial& track, const std::vector<scalar>& s_, const bool is_closed, const Options& options_) : options(options_)
    { 
        if (is_closed)
            compute_track_by_polynomial<true>(track, s_); 
        else
            compute_track_by_polynomial<false>(track, s_); 
    }

    std::vector<double> compute_track_by_arcs(const Track_by_arcs& track, const size_t N);

    template<bool is_closed>
    void compute_track_by_polynomial(const Track_by_polynomial& track, const std::vector<scalar>& s_);

    //! Export to XML
    std::unique_ptr<Xml_document> xml() const;

    const std::vector<double>& get_x() const { return _x; }
 private:

    class Fitness_fcn
    {
     public:
        using ADvector = std::vector<CppAD::AD<scalar>>;

        Fitness_fcn(const size_t N, const Track_by_arcs& track) : _n(N), _track(track) {}

        void operator()(ADvector& fg, const ADvector& x) const;

     private:
        size_t _n;
        Track_by_arcs _track;
    };

    template<bool is_closed>
    class Fitness_fcn_by_polynomial
    {
    public:
        struct input_names
        {
            enum { n, theta, kappa, delta_path_arclength, end };
        };

        using ADvector = std::vector<CppAD::AD<scalar>>;

        Fitness_fcn_by_polynomial(const Track_by_polynomial& track, const size_t num_points, const size_t num_elements, const std::vector<double>& s)
            : _track(track), _num_points(num_points), _num_elements(num_elements), 
              _num_variables((input_names::end-1)*_num_points + _num_elements), _num_constraints(_num_elements*3), 
              _s(s), _delta_path_arclength(_num_elements, 0.0), _n(_num_points, 0.0), _theta(_num_points, 0.0), _kappa(_num_points, 0.0),
              _x(_num_points, 0.0), _y(_num_points, 0.0) {}

        void operator()(ADvector& fg, const ADvector& x);

        constexpr const size_t& get_n_variables() const { return _num_variables; }

        constexpr const size_t& get_n_constraints() const { return _num_constraints; }

    private:
        Track_by_polynomial _track;

        size_t _num_points;
        size_t _num_elements;
        size_t _num_variables;
        size_t _num_constraints;

        std::vector<double> _s;

    public:
        std::vector<CppAD::AD<scalar>> _delta_path_arclength;
        std::vector<CppAD::AD<scalar>> _n;
        std::vector<CppAD::AD<scalar>> _theta;
        std::vector<CppAD::AD<scalar>> _kappa;
        std::vector<CppAD::AD<scalar>> _x;
        std::vector<CppAD::AD<scalar>> _y;
    };

public:

    Options options;

    size_t num_points;
    size_t num_elements;

    std::vector<scalar> s;

    bool success;
    size_t iter_count;
    std::vector<scalar> delta_path_arclength;
    std::vector<scalar> x;
    std::vector<scalar> y;
    std::vector<scalar> n;
    std::vector<scalar> theta;
    std::vector<scalar> kappa;
    std::vector<double> _x;
};

#include "minimum_curvature_path.hpp"

#endif
