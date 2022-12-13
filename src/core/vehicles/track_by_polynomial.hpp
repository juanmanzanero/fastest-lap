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
        auto theta_values = std::vector<scalar>(2 * polynomial_order + 1);
        auto kappa_values = std::vector<scalar>(2 * polynomial_order + 1);

        for (size_t i_point = 0; i_point < 2 * polynomial_order + 1; ++i_point)
        {
            const auto dp_i = dp(x[i_point]);
            const auto d2p_i = d2p(x[i_point]);

            theta_values[i_point] = atan2(dp_i.y(), dp_i.x());
            kappa_values[i_point] = cross(dp_i, d2p_i).z() / (norm(dp_i) * dot(dp_i, dp_i));
        }

        heading_angle_polynomials[i_block] = { x, theta_values, polynomial_order, true };
        curvature_polynomials[i_block]     = { x, kappa_values, polynomial_order, true };
    }

    _theta = { heading_angle_polynomials };
    _dtheta_ds = { curvature_polynomials };

    _mu      = { {position.get_left_bound(), position.get_right_bound()}, {0.0, 0.0}, 1, false };
    _phi     = { {position.get_left_bound(), position.get_right_bound()}, {0.0, 0.0}, 1, false };
    _dmu_ds  = { {position.get_left_bound(), position.get_right_bound()}, {0.0, 0.0}, 1, false };
    _dphi_ds = { {position.get_left_bound(), position.get_right_bound()}, {0.0, 0.0}, 1, false };
}

inline Track_by_polynomial::Track_by_polynomial(const Circuit_preprocessor& circuit)
{
    auto s            = circuit.s;
    auto r_centerline = circuit.r_centerline;
    auto theta        = circuit.theta;
    auto mu           = circuit.mu;
    auto phi          = circuit.phi;
    auto kappa        = circuit.kappa;
    auto dmuds        = circuit.dmu_dot;
    auto dphids       = circuit.dphi_dot;
    auto nl           = circuit.nl;
    auto nr           = circuit.nr;

    if ( circuit.is_closed ) 
    { 
        s.push_back(circuit.track_length);
        r_centerline.push_back(r_centerline.front());
        theta.push_back(theta.front());
        kappa.push_back(kappa.front());
        nl.push_back(nl.front());
        nr.push_back(nr.front());

        if (circuit.options.with_elevation)
        {
            mu.push_back(mu.front());
            phi.push_back(phi.front());
            dmuds.push_back(dmuds.front());
            dphids.push_back(dphids.front());
        }
    }

    std::cout << "[WARNING] Track_by_polynomial -> flipping Y after reading file, this should be fixed!" << std::endl;

    std::transform(r_centerline.begin(), r_centerline.end(), r_centerline.begin(), [](const auto& v) -> sVector3d { return { v.x(), -v.y(), v.z() }; });
    theta = (-1.0)*theta;
    kappa = (-1.0)*kappa;

    // Construct the polynomials
    _r         = {s,r_centerline,1,false};
    _theta     = { s, theta, 1, false };
    _dtheta_ds = { s, kappa, 1, false };
    _wl        = {s,nl,1,false};
    _wr        = {s,nr,1,false};

    if (circuit.options.with_elevation)
    {
        _mu = { s, mu, 1, false };
        _phi = { s, phi, 1, false };
        _dmu_ds = { s, dmuds, 1, false };
        _dphi_ds = { s, dphids, 1, false };
    }
    else
    {
        _mu      = { {s.front(), s.back()}, {0.0, 0.0}, 1, false };
        _phi     = { {s.front(), s.back()}, {0.0, 0.0}, 1, false };
        _dmu_ds  = { {s.front(), s.back()}, {0.0, 0.0}, 1, false };
        _dphi_ds = { {s.front(), s.back()}, {0.0, 0.0}, 1, false };
    }

    // Save the preprocessor
    _preprocessor = circuit;
}


constexpr bool Track_by_polynomial::has_elevation() const
{
    return !_mu.is_zero() || !_phi.is_zero() || !_r.is_zero([](const auto& r) { return std::abs(r.z()) < 1.0e-12; })
        || !_dmu_ds.is_zero() || !_dphi_ds.is_zero();
}
#endif
