#ifndef __TRACK_BY_POLYNOMIAL_HPP__
#define __TRACK_BY_POLYNOMIAL_HPP__


inline Track_by_polynomial::Track_by_polynomial(Xml_document& doc)
{
    if ( doc.get_root_element().get_attribute("format") == "by-polynomial" )
        *this = Track_by_polynomial(compute_track_polynomial(doc));
    
    else if ( doc.get_root_element().get_attribute("format") == "discrete" )
    {
        Circuit_preprocessor circuit(doc);
        *this = Track_by_polynomial(circuit);
    }
    else
        throw std::runtime_error("Format is not recognized. Options are \"by-polynomial\" and \"discrete\"");
}


inline Track_by_polynomial::Track_by_polynomial(const Circuit_preprocessor& circuit)
{
    auto s = circuit.s;
    auto r_centerline = circuit.r_centerline;
    auto theta = circuit.theta;
    auto kappa = circuit.kappa;
    auto nl = circuit.nl;
    auto nr = circuit.nr;

    if ( circuit.is_closed ) 
    { 
        s.push_back(circuit.track_length);
        r_centerline.push_back(r_centerline.front());
        theta.push_back(theta.front());
        kappa.push_back(kappa.front());
        nl.push_back(nl.front());
        nr.push_back(nr.front());
    }

    // Rotate 180 around +X (flip Y)
    for (size_t i = 0; i < r_centerline.size(); ++i)
    {
        r_centerline[i].y() *= -1.0;
        theta[i] *= -1.0;
        kappa[i] *= -1.0;
    }

    // Compute dr = (cos(theta), sin(theta), 0.0)
    std::vector<sVector3d> dr(theta.size());
    
    for (size_t i = 0; i < theta.size(); ++i)
        dr[i] = sVector3d(cos(theta[i]),sin(theta[i]),0.0);

    // Compute d2r = kappa.(-sin(theta), cos(theta), 0.0)
    std::vector<sVector3d> d2r(theta.size());

    for (size_t i = 0; i < theta.size(); ++i)
        d2r[i] = kappa[i]*sVector3d(-sin(theta[i]), cos(theta[i]), 0.0);
    
    
    // Construct the polynomials
    _r   = {s,r_centerline,1,false};
    _dr  = {s,dr,1,false};
    _d2r = {s,d2r,1,false};
    _wl  = {s,nl,1,false};
    _wr  = {s,nr,1,false};

    // Save the preprocessor
    _preprocessor = circuit;
}

inline std::tuple<vPolynomial,sPolynomial,sPolynomial> Track_by_polynomial::compute_track_polynomial(Xml_document& doc)   
{
    auto xml_segments = doc.get_root_element().get_children();

    const auto n_segments = xml_segments.size();
    std::vector<vPolynomial> track_segments(n_segments);

    std::vector<sPolynomial> w_left_poly(n_segments);
    std::vector<sPolynomial> w_right_poly(n_segments);

    // Loop over the segments, construct one polynomial for each
    scalar initial_arclength = 0.0; // Use the arclength of the previous element to start the new one
    for (size_t i = 0; i < n_segments; ++i)
    {
        std::vector<scalar> x = xml_segments[i].get_child("x").get_value(std::vector<scalar>());
        std::vector<scalar> y = xml_segments[i].get_child("y").get_value(std::vector<scalar>());

        std::vector<scalar> wl = xml_segments[i].get_child("w_left").get_value(std::vector<scalar>());
        std::vector<scalar> wr = xml_segments[i].get_child("w_right").get_value(std::vector<scalar>());

        std::vector<sVector3d> coordinates(x.size());

        for (size_t j = 0; j < x.size(); ++j)
            coordinates[j] = sVector3d(x[j],y[j],0.0);

        // Compute the pseudo-arclength
        std::vector<scalar> s(x.size());
        s[0] = initial_arclength;

        for (size_t j = 1; j < x.size(); ++j)
            s[j] = s[j-1] + sqrt((x[j]-x[j-1])*(x[j]-x[j-1]) + (y[j]-y[j-1])*(y[j]-y[j-1]));

        initial_arclength = s.back();

        // Compute the polynomial
        track_segments[i] = {s,coordinates,15,true};

        w_left_poly[i] = {s,wl,1,false};
        w_right_poly[i] = {s,wr,1,false};
    }

    // Combine all the polynomials in one
    return {{track_segments},{w_left_poly},{w_right_poly}};
}

#endif
