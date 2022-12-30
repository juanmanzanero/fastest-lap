#ifndef KERB_H
#define KERB_H

#include "lion/math/polynomial.h"
#include "src/core/foundation/fastest_lap_exception.h"

class Kerb
{
public: 
    struct Direction
    {
        constexpr const static int exterior = -1;
        constexpr const static int interior =  1;
    };

    struct Kerb_element
    {
        scalar arclength_start;
        scalar arclength_finish;
        scalar width;
        bool used;
    };

    Kerb() : _kerbs() {};

    Kerb(const std::vector<scalar>& arclength, const std::vector<scalar>& lateral_displacement_percentage, const std::vector<scalar>& curvature, const scalar kerb_width, const bool use_kerbs, const int exterior_kerbs_direction)
    { 
        compute(arclength, lateral_displacement_percentage, curvature, kerb_width, use_kerbs, exterior_kerbs_direction); 
    }

    void compute(const std::vector<scalar>& arclength, const std::vector<scalar>& lateral_displacement_percentage, const std::vector<scalar>& curvature, const scalar kerb_width, const bool use_kerbs, const int exterior_kerbs_direction);

    void append_kerb(const Kerb_element& kerb) 
    { 
        if (kerb.arclength_start < _kerbs.back().arclength_finish)
        {
            throw fastest_lap_exception("[ERROR] Kerb::append_kerb -> Kerbs must be ordered monotonically in arclength");
        }

        if (kerb.arclength_start > kerb.arclength_finish - 1.0e-8)
        {
            throw fastest_lap_exception("[ERROR] Kerb::append_kerb -> Kerb must have arclength_start > arclength_finish");
        }

        _kerbs.emplace_back(kerb); 
    }

    constexpr const std::vector<Kerb_element>& get_kerbs() const { return _kerbs; }

    scalar evaluate(scalar s) const { return evaluate_generic(s, false); }

    scalar evaluate_display(scalar s) const { return evaluate_generic(s, true); }

    scalar evaluate_generic(scalar s, bool display_mode) const;

private:
    std::vector<Kerb_element> _kerbs;
};


inline void Kerb::compute(const std::vector<scalar>& arclength, const std::vector<scalar>& lateral_displacement_percentage, const std::vector<scalar>& curvature, const scalar kerb_width, const bool use_kerbs, const int exterior_kerbs_direction)
{
    assert(arclength.size() == lateral_displacement_percentage.size());
    assert(arclength.size() == curvature.size());

    std::vector<scalar> current_kerb_arclength;
    std::vector<scalar> current_kerb_lateral_displacement;
    std::vector<scalar> current_kerb_curvature;
    for (size_t i_s = 0; i_s < arclength.size(); ++i_s)
    {
        if (lateral_displacement_percentage[i_s] > 0.8)
        {
            current_kerb_arclength.push_back(arclength[i_s]);
            current_kerb_lateral_displacement.push_back(lateral_displacement_percentage[i_s]);
            current_kerb_curvature.push_back(curvature[i_s]);
        }
        else
        {
            if (current_kerb_arclength.size() > 2)
            {
                // Decide whether the kerb is interior or exterior
                std::vector<double> integrand(current_kerb_arclength.size());
                std::transform(current_kerb_lateral_displacement.cbegin(),
                    current_kerb_lateral_displacement.cend(),
                    current_kerb_curvature.cbegin(),
                    integrand.begin(),
                    [](const auto& n, const auto& kappa) { return (n - 0.5) * kappa; }
                );

                bool is_kerb_used;
                int kerb_direction;
                if (trapz(current_kerb_arclength, integrand) / (current_kerb_arclength.back() - current_kerb_arclength.front()) > 1.0e-3)
                {
                    // Interior kerb
                    is_kerb_used = false;
                    kerb_direction = Direction::exterior;
                }
                else
                {
                    // Exterior kerb
                    is_kerb_used = true;
                    kerb_direction = exterior_kerbs_direction;
                }

                // Generate kerb
                append_kerb({ current_kerb_arclength.front(), current_kerb_arclength.back(), kerb_direction * kerb_width, is_kerb_used && use_kerbs });
                
                current_kerb_arclength.clear();
                current_kerb_lateral_displacement.clear();
                current_kerb_curvature.clear();
            }
            else if (current_kerb_arclength.size() > 0)
            {
                // Prevent having very short kerbs
                current_kerb_arclength.clear();
                current_kerb_lateral_displacement.clear();
                current_kerb_curvature.clear();
            }
        }
    }
}

inline scalar Kerb::evaluate_generic(scalar s, bool display_mode) const
{
    if (_kerbs.size() > 0)
    {
        for (const auto& kerb : _kerbs)
            if (s > kerb.arclength_start - 1.0e-8 && s < kerb.arclength_finish + 1.0e-8)
                return ((display_mode || kerb.used) ? kerb.width : 0.0);

        return 0.0;
    }
    else
    {
        return 0.0;
    }
}

#endif
