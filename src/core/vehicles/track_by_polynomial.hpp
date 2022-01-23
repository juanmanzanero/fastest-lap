#ifndef __TRACK_BY_POLYNOMIAL_HPP__
#define __TRACK_BY_POLYNOMIAL_HPP__

inline vPolynomial Track_by_polynomial::compute_track_polynomial(Xml_document& doc)   
{
    auto xml_segments = doc.get_root_element().get_children();

    const auto n_segments = xml_segments.size();
    std::vector<vPolynomial> track_segments(n_segments);

    // Loop over the segments, construct one polynomial for each

    scalar initial_arclength = 0.0; // Use the arclength of the previous element to start the new one
    for (size_t i = 0; i < n_segments; ++i)
    {
        std::vector<scalar> x = xml_segments[i].get_child("x").get_value(std::vector<scalar>());
        std::vector<scalar> y = xml_segments[i].get_child("y").get_value(std::vector<scalar>());

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
        track_segments[i] = {s,coordinates,10,true};
    }

    // Combine all the polynomials in one
    return {track_segments};
}

#endif
