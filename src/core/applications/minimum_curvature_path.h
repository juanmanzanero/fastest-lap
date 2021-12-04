#ifndef __MINIMUM_CURVATURE_PATH_H__
#define __MINIMUM_CURVATURE_PATH_H__

#include "lion/thirdparty/include/cppad/cppad.hpp"

class Minimum_curvature_path
{
 public:
    Minimum_curvature_path(const Track_by_arcs& track, const size_t N) { compute(track,N); }

    std::vector<double> compute(const Track_by_arcs& track, const size_t N);

    const std::vector<double>& get_x() const { return _x; }
    const bool& get_success() const { return _success; } 
 private:

    class Fitness_fcn
    {
     public:
        using ADvector = std::vector<timeseries>;

        Fitness_fcn(const size_t N, const Track_by_arcs& track) : _n(N), _track(track) {}

        void operator()(ADvector& fg, const ADvector& x) const;

     private:
        size_t _n;
        Track_by_arcs _track;
    };

    bool _success;
    std::vector<double> _x;
};

#include "minimum_curvature_path.hpp"

#endif
