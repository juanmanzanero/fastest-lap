#ifndef TRACK_BY_POLYNOMIAL_HPP
#define TRACK_BY_POLYNOMIAL_HPP


inline Track_by_polynomial::Track_by_polynomial(Xml_document& doc)
{
    if ( doc.get_root_element().get_attribute("format") == "discrete" )
    {
        Circuit_preprocessor circuit(doc);
        *this = Track_by_polynomial(circuit);
    }
    else
        throw fastest_lap_exception("Format is not recognized. The only option supported is \"discrete\"");
}


inline Track_by_polynomial::Track_by_polynomial(const vPolynomial& position, const sPolynomial& wl, const sPolynomial& wr)
: _r(position),
  _wl(wl),
  _wr(wr)
{
    // (1) Check that the curve provided is planar
    const auto& n_blocks = position.get_n_blocks();

    for (size_t i_block = 0; i_block < n_blocks; ++i_block)
    {
        const auto& coeffs = position.get_coeffs(i_block);

        if (!std::accumulate(coeffs.cbegin(), coeffs.cend(), true, [](const auto& acc, const auto& v_i) { return acc && std::abs(v_i.z()) < 1.0e-12; }))
            throw fastest_lap_exception("[ERROR] Track_by_polynomial -> cannot construct track from polynomial since the polynomial provided is not planar.");
    }

    // (2) Get the derivatives
    const auto dp = position.derivative();
    const auto d2p = dp.derivative();

    // (3) For each block, compute the heading angle and the curvature
    std::vector<sPolynomial> heading_angle_polynomials(n_blocks);
    std::vector<sPolynomial> curvature_polynomials(n_blocks);

    for (size_t i_block = 0; i_block < n_blocks; ++i_block)
    {
        const auto dp_block = dp.get_block(i_block);
        const auto d2p_block = d2p.get_block(i_block);

        // (3.1) Compute the structure: the order of heading angle and curvature is the order of d(position)/dt
        const auto polynomial_order = dp_block.get_coeffs(0).size();
        const auto left_bounds = dp_block.get_left_bound();
        const auto right_bounds = dp_block.get_right_bound();

        // (3.2) Evaluate the derivatives in a set of points
        const auto x = linspace(left_bounds, right_bounds, 2 * polynomial_order + 1);
        auto yaw_values = std::vector<scalar>(2 * polynomial_order + 1);
        auto yaw_dot_values = std::vector<scalar>(2 * polynomial_order + 1);

        for (size_t i_point = 0; i_point < 2 * polynomial_order + 1; ++i_point)
        {
            const auto dp_i  = dp(x[i_point]);
            const auto d2p_i = d2p(x[i_point]);

            yaw_values[i_point] = atan2(dp_i.y(), dp_i.x());
            yaw_dot_values[i_point] = cross(dp_i, d2p_i).z() / (norm(dp_i) * dot(dp_i, dp_i));
        }

        heading_angle_polynomials[i_block] = { x, yaw_values, polynomial_order, true };
        curvature_polynomials[i_block]     = { x, yaw_dot_values, polynomial_order, true };
    }

    _yaw     = { heading_angle_polynomials };
    _dyaw_ds = { curvature_polynomials };

    _pitch     = { {position.get_left_bound(), position.get_right_bound()}, {0.0, 0.0}, 1, false };
    _roll      = { {position.get_left_bound(), position.get_right_bound()}, {0.0, 0.0}, 1, false };
    _dpitch_ds = { {position.get_left_bound(), position.get_right_bound()}, {0.0, 0.0}, 1, false };
    _droll_ds  = { {position.get_left_bound(), position.get_right_bound()}, {0.0, 0.0}, 1, false };
}

inline Track_by_polynomial::Track_by_polynomial(const Circuit_preprocessor& circuit)
{
    // (1) Get data from circuit preprocessor
    auto s            = circuit.s;
    auto r_centerline = circuit.r_centerline;
    auto yaw          = circuit.yaw;
    auto pitch        = circuit.pitch;
    auto roll         = circuit.roll;
    auto yaw_dot      = circuit.yaw_dot;
    auto dpitchds     = circuit.dpitch_dot;
    auto drollds      = circuit.droll_dot;
    auto nl           = circuit.nl;
    auto nr           = circuit.nr;

    // (2) Augment nl and nr with the kerbs
    for (size_t i_s = 0; i_s < s.size(); ++i_s)
    {
        for (const auto& kerb : circuit.left_kerb.get_kerbs())
            if (s[i_s] > kerb.arclength_start - 1.0e-12 && s[i_s] < kerb.arclength_finish + 1.0e-12 && kerb.used) 
            {
                nl[i_s] += kerb.width;
                break;
            }

        for (const auto& kerb : circuit.right_kerb.get_kerbs())
            if (s[i_s] > kerb.arclength_start - 1.0e-12 && s[i_s] < kerb.arclength_finish + 1.0e-12 && kerb.used) 
            {
                nr[i_s] += kerb.width;
                break;
            }
    }

    // (3) Close the circuit if needed
    if ( circuit.is_closed ) 
    { 
        s.push_back(circuit.track_length);
        r_centerline.push_back(r_centerline.front());
        yaw.push_back(yaw.front());
        yaw_dot.push_back(yaw_dot.front());
        nl.push_back(nl.front());
        nr.push_back(nr.front());

        if (circuit.options.with_elevation)
        {
            pitch.push_back(pitch.front());
            roll.push_back(roll.front());
            dpitchds.push_back(dpitchds.front());
            drollds.push_back(drollds.front());
        }
    }

    // Construct the polynomials
    _r       = {s,r_centerline,1,false};
    _yaw     = { s, yaw, 1, false };
    _dyaw_ds = { s, yaw_dot, 1, false };
    _wl      = {s,nl,1,false};
    _wr      = {s,nr,1,false};

    if (circuit.options.with_elevation)
    {
        _pitch     = { s, pitch, 1, false };
        _roll      = { s, roll, 1, false };
        _dpitch_ds = { s, dpitchds, 1, false };
        _droll_ds  = { s, drollds, 1, false };
    }
    else
    {
        _pitch     = { {s.front(), s.back()}, {0.0, 0.0}, 1, false };
        _roll      = { {s.front(), s.back()}, {0.0, 0.0}, 1, false };
        _dpitch_ds = { {s.front(), s.back()}, {0.0, 0.0}, 1, false };
        _droll_ds  = { {s.front(), s.back()}, {0.0, 0.0}, 1, false };
    }

    // Save the preprocessor
    _preprocessor = circuit;
}


constexpr bool Track_by_polynomial::has_elevation() const
{
    return !_pitch.is_zero() || !_roll.is_zero() || !_r.is_zero([](const auto& r) { return std::abs(r.z()) < 1.0e-12; })
        || !_dpitch_ds.is_zero() || !_droll_ds.is_zero();
}
#endif
