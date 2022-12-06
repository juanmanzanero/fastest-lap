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
        mu.push_back(mu.front());
        phi.push_back(phi.front());
        kappa.push_back(kappa.front());
        dmuds.push_back(dmuds.front());
        dphids.push_back(dphids.front());
        nl.push_back(nl.front());
        nr.push_back(nr.front());
    }

    // Construct the polynomials
    _r         = {s,r_centerline,1,false};
    _theta     = { s, theta, 1, false };
    _mu        = { s, mu, 1, false };
    _phi       = { s, phi, 1, false };
    _dtheta_ds = { s, kappa, 1, false };
    _dmu_ds    = { s, dmuds, 1, false };
    _dphi_ds   = { s, dphids, 1, false };
    _wl        = {s,nl,1,false};
    _wr        = {s,nr,1,false};

    // Save the preprocessor
    _preprocessor = circuit;
}

#endif
