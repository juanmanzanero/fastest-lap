#ifndef __CIRCUIT_PREPROCESSOR_HPP__
#define __CIRCUIT_PREPROCESSOR_HPP__

#include "lion/foundation/utils.hpp"
#include "lion/math/polynomial.h"
#include "lion/math/matrix_extensions.h"
#include "lion/math/ipopt_cppad_handler.hpp"

inline Circuit_preprocessor::Circuit_preprocessor(Xml_document& coord_left_kml, Xml_document& coord_right_kml, const size_t n, bool closed, const Options options)
 : _options(options)
{
    // Get child with data for the left boundary 
    const std::vector<scalar> coord_left_raw  = coord_left_kml.get_element("kml/Document/Placemark/LineString/coordinates").get_value(std::vector<scalar>());
    const std::vector<scalar> coord_right_raw = coord_right_kml.get_element("kml/Document/Placemark/LineString/coordinates").get_value(std::vector<scalar>());

    if ( coord_left_raw.size() % 3 != 0 )
        throw std::runtime_error("Error processing google-earth placemark: size must be multiple of 3");

    if ( coord_right_raw.size() % 3 != 0 )
        throw std::runtime_error("Error processing google-earth placemark: size must be multiple of 3");

    const size_t n_left = coord_left_raw.size()/3;
    std::vector<std::array<scalar,2>> coord_left(n_left);

    for (size_t i = 0; i < n_left; ++i)
        coord_left[i] = std::array<scalar,2>({coord_left_raw[3*i],coord_left_raw[3*i+1]});

    const size_t n_right = coord_right_raw.size()/3;
    std::vector<std::array<scalar,2>> coord_right(n_right);

    for (size_t i = 0; i < n_right; ++i)
        coord_right[i] = std::array<scalar,2>({coord_right_raw[3*i],coord_right_raw[3*i+1]});

    *this = Circuit_preprocessor(coord_left, coord_right, n, closed, options);
}

inline Circuit_preprocessor::Circuit_preprocessor(const std::vector<std::array<scalar,2>>& coord_left, const std::vector<std::array<scalar,2>>& coord_right, 
    const size_t n, const bool closed, const Options options)
: _options(options), is_closed(closed)
{
    if (closed)
        compute<true>(coord_left, coord_right, n);
    else
        compute<false>(coord_left, coord_right, n);
}

template<bool closed>
inline void Circuit_preprocessor::compute(const std::vector<std::array<scalar,2>>& coord_left, const std::vector<std::array<scalar,2>>& coord_right, 
    const size_t n)
{
    // (1) Get reference latitude (arbitrarily, that of the first right curve)
    phi_ref = coord_right.front()[LAT]*DEG;
    phi0    = coord_right.front()[LAT]*DEG;
    theta0  = coord_right.front()[LON]*DEG;

    x0   = theta0*R_earth*cos(phi_ref);
    y0   = phi0*R_earth;

    // (2) Transform coordinates to cartesian (x,y)
    r_left_measured = std::vector<sVector3d>(coord_left.size(),{0.0,0.0,0.0});
    for (size_t i = 0; i < coord_left.size(); ++i)
        r_left_measured[i] = sVector3d((coord_left[i][LON]*DEG-theta0)*R_earth*cos(phi_ref), (coord_left[i][LAT]*DEG-phi0)*R_earth, 0.0);

    r_right_measured = std::vector<sVector3d>(coord_right.size(),{0.0,0.0,0.0});
    for (size_t i = 0; i < coord_right.size(); ++i)
        r_right_measured[i] = sVector3d((coord_right[i][LON]*DEG-theta0)*R_earth*cos(phi_ref), (coord_right[i][LAT]*DEG-phi0)*R_earth, 0.0);

    // (3) Compute the centerline estimate
    const auto [s_center,r_center] = compute_averaged_centerline<closed>(r_left_measured,r_right_measured,n);
    const scalar track_length_estimate = s_center.back() + norm(r_center.front() - r_center.back());

    // (5) Compute the initial condition via finite differences
    n_points = closed ? n : n+1;
    std::vector<scalar> x_init(n_points,0.0);
    std::vector<scalar> y_init(n_points,0.0);
    std::vector<scalar> theta_init(n_points,0.0);
    std::vector<scalar> kappa_init(n_points,0.0);
    std::vector<scalar> nl_init(n_points,0.0);
    std::vector<scalar> nr_init(n_points,0.0);
    std::vector<scalar> dkappa_init(n_points,0.0);
    std::vector<scalar> dnl_init(n_points,0.0);
    std::vector<scalar> dnr_init(n_points,0.0);

    // x and y
    for (size_t i = 0; i < n_points; ++i)
    {
        x_init[i] = r_center[i][0];
        y_init[i] = r_center[i][1];
    }

    // theta
    theta_init[0] = atan2(y_init[1]-y_init[0],x_init[1]-x_init[0]);
    for (size_t i = 1; i < n_points-1; ++i)
        theta_init[i] = theta_init[i-1] + wrap_to_pi(atan2(y_init[i+1]-y_init[i],x_init[i+1]-x_init[i])-theta_init[i-1]);
    
    if (closed)
    {
        theta_init[n-1] = theta_init[n-2] + wrap_to_pi(atan2(y_init[0]-y_init[n-1],x_init[0]-x_init[n-1])-theta_init[n-2]);
    
        // compute the circuit direction (clockwise/counter clockwise)
        if ( theta_init[n-1] > theta_init[0] )
            direction = 1.0;
        else
            direction = -1.0;
    }
    else
        theta_init[n] = theta_init[n-1];


    // kappa
    for (size_t i = 0; i < n_points-1; ++i)
        kappa_init[i] = (theta_init[i+1]-theta_init[i])/(s_center[i+1]-s_center[i]);

    if (closed)
        kappa_init[n-1] = (theta_init[0] + 2.0*pi*direction - theta_init[n-1])/norm(r_center.front() - r_center.back());
    else
        kappa_init[n] = kappa_init[n-1];

    // nl and nr
    for (size_t i = 0; i < n_points; ++i)
    {
        nl_init[i] = sqrt(find_closest_point<scalar>(r_left_measured, r_center[i], closed).second);
        nr_init[i] = sqrt(find_closest_point<scalar>(r_right_measured, r_center[i], closed).second);
    }

    // dkappa, dnl, and dnr
    for (size_t i = 0; i < n_points-1; ++i)
    {
        dkappa_init[i] = (kappa_init[i+1]-kappa_init[i])/(s_center[i+1]-s_center[i]);
        dnl_init[i] = (nl_init[i+1]-nl_init[i])/(s_center[i+1]-s_center[i]);
        dnr_init[i] = (nr_init[i+1]-nr_init[i])/(s_center[i+1]-s_center[i]);
    }

    if (closed)
    {
        dkappa_init[n-1] = (kappa_init[0] - kappa_init[n-1])/norm(r_center.front()-r_center.back());
        dnl_init[n-1] = (nl_init[0] - nl_init[n-1])/norm(r_center.front()-r_center.back());
        dnr_init[n-1] = (nr_init[0] - nr_init[n-1])/norm(r_center.front()-r_center.back());
    }
    else
    {
        dkappa_init[n] = dkappa_init[n-1];
        dnl_init[n] = dnl_init[n-1];
        dnr_init[n] = dnr_init[n-1];
    }

    // (4) Create the FG object
    FG<closed> fg(n, track_length_estimate, r_left_measured, r_right_measured, r_center, direction, _options);

    // load them into an x vector
    std::vector<scalar> x(fg.get_n_variables());
    std::vector<scalar> x_lb(fg.get_n_variables());
    std::vector<scalar> x_ub(fg.get_n_variables());
    size_t k = 0;
    size_t k_ub = 0;
    size_t k_lb = 0;
    x[k++] = track_length_estimate/n;
    x_lb[k_lb++] = 0.9*track_length_estimate/n;
    x_ub[k_ub++] = 1.11*track_length_estimate/n;
    for (size_t i = 0; i < n_points; ++i)
    {
        x[k++] = x_init[i];
        x[k++] = y_init[i];
        x[k++] = theta_init[i];
        x[k++] = kappa_init[i];
        x[k++] = nl_init[i];
        x[k++] = nr_init[i];
        x[k++] = dkappa_init[i];
        x[k++] = dnl_init[i];
        x[k++] = dnr_init[i];

        x_lb[k_lb++] = x_init[i]-100.0;
        x_lb[k_lb++] = y_init[i]-100.0;
        x_lb[k_lb++] = theta_init[i]-30.0*DEG;
        x_lb[k_lb++] = -_options.maximum_kappa;
        x_lb[k_lb++] = 1.0;
        x_lb[k_lb++] = 1.0;
        x_lb[k_lb++] = -_options.maximum_dkappa;
        x_lb[k_lb++] = -1.0;
        x_lb[k_lb++] = -1.0;

        x_ub[k_ub++] = x_init[i]+100.0;
        x_ub[k_ub++] = y_init[i]+100.0;
        x_ub[k_ub++] = theta_init[i]+30.0*DEG;
        x_ub[k_ub++] = _options.maximum_kappa;
        x_ub[k_ub++] = nl_init[i]+nr_init[i];
        x_ub[k_ub++] = nl_init[i]+nr_init[i];
        x_ub[k_ub++] = _options.maximum_dkappa;
        x_ub[k_ub++] = 1.0;
        x_ub[k_ub++] = 1.0;
    }

    assert(k == fg.get_n_variables());
    assert(k_lb == fg.get_n_variables());
    assert(k_ub == fg.get_n_variables());


    // (7) Run the optimization
    std::string ipopt_options;
    ipopt_options += "Integer print_level  5\n";
    ipopt_options += "String  sb           yes\n";
    ipopt_options += "Sparse true forward\n";
    ipopt_options += "Retape true\n";
    ipopt_options += "Numeric tol          1e-10\n";
    ipopt_options += "Numeric constr_viol_tol  1e-10\n";
    ipopt_options += "Numeric acceptable_tol  1e-8\n";

    // place to return solution
    CppAD::ipopt_cppad_result<std::vector<scalar>> result;

    // solve the problem
    CppAD::ipopt_cppad_solve(ipopt_options, x, x_lb, x_ub, std::vector<scalar>(fg.get_n_constraints(),0.0), std::vector<scalar>(fg.get_n_constraints(),0.0), fg, result);

    if ( result.status != CppAD::ipopt_cppad_result<std::vector<scalar>>::success )
    {
        throw std::runtime_error("Optimization did not succeed");
    }

    // Load the solution
    ds           = x[0];
    s            = std::vector<scalar>(n_points);
    r_left       = std::vector<sVector3d>(n_points);
    r_right      = std::vector<sVector3d>(n_points);
    r_centerline = std::vector<sVector3d>(n_points);
    theta        = std::vector<scalar>(n_points);
    kappa        = std::vector<scalar>(n_points);
    nl           = std::vector<scalar>(n_points);
    nr           = std::vector<scalar>(n_points);
    dkappa       = std::vector<scalar>(n_points);
    dnl          = std::vector<scalar>(n_points);
    dnr          = std::vector<scalar>(n_points);

    const auto& IX        = FG<closed>::IX;
    const auto& IY        = FG<closed>::IY;
    const auto& ITHETA    = FG<closed>::ITHETA;
    const auto& IKAPPA    = FG<closed>::IKAPPA;
    const auto& INL       = FG<closed>::INL;
    const auto& INR       = FG<closed>::INR;
    const auto& NSTATE    = FG<closed>::NSTATE;
    const auto& IDKAPPA   = FG<closed>::IDKAPPA;
    const auto& IDNL      = FG<closed>::IDNL;
    const auto& IDNR      = FG<closed>::IDNR;
    const auto& NCONTROLS = FG<closed>::NCONTROLS;

    for (size_t i = 0; i < n_points; ++i)
    {
        // Get indexes
        const size_t ix      = 1  + (NSTATE + NCONTROLS)*i          + IX;
        const size_t iy      = 1  + (NSTATE + NCONTROLS)*i          + IY;
        const size_t itheta  = 1  + (NSTATE + NCONTROLS)*i          + ITHETA;
        const size_t ikappa  = 1  + (NSTATE + NCONTROLS)*i          + IKAPPA;
        const size_t inl     = 1  + (NSTATE + NCONTROLS)*i          + INL;
        const size_t inr     = 1  + (NSTATE + NCONTROLS)*i          + INR;
        const size_t idkappa = 1  + (NSTATE + NCONTROLS)*i + NSTATE + IDKAPPA;
        const size_t idnl    = 1  + (NSTATE + NCONTROLS)*i + NSTATE + IDNL;
        const size_t idnr    = 1  + (NSTATE + NCONTROLS)*i + NSTATE + IDNR;

        s[i] = i*ds;

        r_left[i] = sVector3d(result.x[ix] - sin(result.x[itheta])*result.x[inl],
                              result.x[iy] + cos(result.x[itheta])*result.x[inl],
                              0.0);
        r_right[i] = sVector3d(result.x[ix] + sin(result.x[itheta])*result.x[inr],
                               result.x[iy] - cos(result.x[itheta])*result.x[inr],
                               0.0); 
        r_centerline[i] = sVector3d(result.x[ix], result.x[iy], 0.0);

        theta[i]  = result.x[itheta];
        kappa[i]  = result.x[ikappa];
        nl[i]     = result.x[inl];
        nr[i]     = result.x[inr];
        dkappa[i] = result.x[idkappa];
        dnl[i]    = result.x[idnl];
        dnr[i]    = result.x[idnr];
    }

    track_length = n*ds;

    // Compute the errors
    left_boundary_max_error   = sqrt(find_closest_point<scalar>(r_left_measured,r_left.front(), closed).second);
    right_boundary_max_error  = sqrt(find_closest_point<scalar>(r_right_measured,r_right.front(), closed).second);
    left_boundary_L2_error   = 0.0;
    right_boundary_L2_error  = 0.0;

    scalar prev_left_error  = left_boundary_max_error;
    scalar prev_right_error = right_boundary_max_error;
    for (size_t i = 1; i < n_points; ++i)
    {
        // Compute current error
        scalar current_left_error = sqrt(find_closest_point<scalar>(r_left_measured,r_left[i], closed).second); 
        scalar current_right_error = sqrt(find_closest_point<scalar>(r_right_measured,r_right[i], closed).second); 

        // Compute maximum error
        left_boundary_max_error = max(current_left_error, left_boundary_max_error);
        right_boundary_max_error = max(current_right_error, right_boundary_max_error);

        // Compute L2 error
        left_boundary_L2_error  += 0.5*ds*(prev_left_error*prev_left_error + current_left_error*current_left_error);
        right_boundary_L2_error += 0.5*ds*(prev_right_error*prev_right_error + current_right_error*current_right_error);

        // Update previous errors
        prev_left_error = current_left_error;
        prev_right_error = current_right_error;
    }

    if (closed)
    {
        scalar current_left_error = sqrt(find_closest_point<scalar>(r_left_measured,r_left.front(), closed).second);
        scalar current_right_error = sqrt(find_closest_point<scalar>(r_right_measured,r_right.front(), closed).second);

        // Compute L2 error
        left_boundary_L2_error  += 0.5*ds*(prev_left_error*prev_left_error + current_left_error*current_left_error);
        right_boundary_L2_error += 0.5*ds*(prev_right_error*prev_right_error + current_right_error*current_right_error);
    }

    left_boundary_L2_error = sqrt(left_boundary_L2_error/track_length);
    right_boundary_L2_error = sqrt(right_boundary_L2_error/track_length);
}

inline std::unique_ptr<Xml_document> Circuit_preprocessor::xml() const
{
    std::ostringstream s_out;
    s_out.precision(17);
    std::unique_ptr<Xml_document> doc_ptr(std::make_unique<Xml_document>());

    doc_ptr->create_root_element("circuit");

    Xml_element root = doc_ptr->get_root_element();

    root.add_attribute("format","discrete");

    if (is_closed)
        root.add_attribute("type","closed");
    else
        root.add_attribute("type","open");
    
    // Add a header with the errors 
    auto header = root.add_child("header");

    s_out << track_length;
    header.add_child("track_length").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();

    s_out << left_boundary_L2_error;
    header.add_child("L2_error_left").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    s_out << right_boundary_L2_error;
    header.add_child("L2_error_right").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    s_out << left_boundary_max_error;
    header.add_child("max_error_left").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    s_out << right_boundary_max_error;
    header.add_child("max_error_right").set_value(s_out.str());
    s_out.str(""); s_out.clear();

    // Add optimization section
    auto opt = root.add_child("optimization");
    opt.add_child("cost_curvature").set_value(std::to_string(_options.eps_k));
    opt.add_child("cost_track_limits_smoothness").set_value(std::to_string(_options.eps_n));
    opt.add_child("cost_track_limits_errors").set_value(std::to_string(_options.eps_d));
    opt.add_child("cost_centerline").set_value(std::to_string(_options.eps_c));

    // Add the GPS coordinates conversion used
    auto gps_param = root.add_child("GPS_parameters");
    gps_param.add_comment(" x = earth_radius.cos(reference_latitude).(longitude - origin_longitude) ");
    gps_param.add_comment(" y = earth_radius.(latitude - origin_latitude) ");

    s_out << theta0*RAD;
    gps_param.add_child("origin_longitude").set_value(s_out.str()).add_attribute("units","deg");
    s_out.str(""); s_out.clear();

    s_out << phi0*RAD;
    gps_param.add_child("origin_latitude").set_value(s_out.str()).add_attribute("units","deg");
    s_out.str(""); s_out.clear();

    s_out << R_earth;
    gps_param.add_child("earth_radius").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();

    s_out << phi_ref*RAD;
    gps_param.add_child("reference_latitude").set_value(s_out.str()).add_attribute("units","deg");
    s_out.str(""); s_out.clear();

    // Add the data
    auto data = root.add_child("data");
    data.add_attribute("number_of_points",std::to_string(n_points));

    // Arc-length
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << s[i] << ", " ;
    s_out << s.back();

    data.add_child("arclength").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();

    // Centerline
    auto centerline = data.add_child("centerline");

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_centerline[i].x() << ", " ;
    s_out << r_centerline.back().x();

    centerline.add_child("x").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_centerline[i].y() << ", " ;
    s_out << r_centerline.back().y();

    centerline.add_child("y").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();

    // Left boundary
    auto left = data.add_child("left_boundary");

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_left[i].x() << ", " ;
    s_out << r_left.back().x();

    left.add_child("x").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_left[i].y() << ", " ;
    s_out << r_left.back().y();

    left.add_child("y").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();
    
    // Right boundary
    auto right = data.add_child("right_boundary");

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_right[i].x() << ", " ;
    s_out << r_right.back().x();

    right.add_child("x").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_right[i].y() << ", " ;
    s_out << r_right.back().y();

    right.add_child("y").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();
    
    // Left measured boundary
    auto left_measured = data.add_child("left_measured_boundary");

    for (size_t i = 0; i < r_left_measured.size()-1; ++i)
        s_out << r_left_measured[i].x() << ", " ;
    s_out << r_left_measured.back().x();

    left_measured.add_child("x").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < r_left_measured.size()-1; ++i)
        s_out << r_left_measured[i].y() << ", " ;
    s_out << r_left_measured.back().y();

    left_measured.add_child("y").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();
    
    // Left measured boundary
    auto right_measured = data.add_child("right_measured_boundary");

    for (size_t i = 0; i < r_right_measured.size()-1; ++i)
        s_out << r_right_measured[i].x() << ", " ;
    s_out << r_right_measured.back().x();

    right_measured.add_child("x").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < r_right_measured.size()-1; ++i)
        s_out << r_right_measured[i].y() << ", " ;
    s_out << r_right_measured.back().y();

    right_measured.add_child("y").set_value(s_out.str()).add_attribute("units","m");
    s_out.str(""); s_out.clear();

    // Theta
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << theta[i] << ", " ;
    s_out << theta.back();

    data.add_child("theta").set_value(s_out.str()).add_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // Kappa
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << kappa[i] << ", " ;
    s_out << kappa.back();

    data.add_child("kappa").set_value(s_out.str()).add_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // nl
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << nl[i] << ", " ;
    s_out << nl.back();

    data.add_child("nl").set_value(s_out.str()).add_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // nr
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << nr[i] << ", " ;
    s_out << nr.back();

    data.add_child("nr").set_value(s_out.str()).add_attribute("units","rad");
    s_out.str(""); s_out.clear();


    // dkappa
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << dkappa[i] << ", " ;
    s_out << dkappa.back();

    data.add_child("dkappa").set_value(s_out.str()).add_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // dnl
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << dnl[i] << ", " ;
    s_out << dnl.back();

    data.add_child("dnl").set_value(s_out.str()).add_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // dnr
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << dnr[i] << ", " ;
    s_out << dnr.back();

    data.add_child("dnr").set_value(s_out.str()).add_attribute("units","rad");
    s_out.str(""); s_out.clear();

    return doc_ptr;
}

template<bool closed>
inline std::pair<std::vector<scalar>, std::vector<sVector3d>> Circuit_preprocessor::compute_averaged_centerline
    (std::vector<sVector3d> r_left, std::vector<sVector3d> r_right, const size_t n)
{
    // (1) Compute the number of points: n for closed circuits, n+1 for open circuits
    const size_t n_points = (closed ? n : n+1);

    // (2) If closed, add the first point as last point to close the track (needed to construct a polynomial version)
    if constexpr (closed)
    {
        r_left.push_back(r_left.front());
        r_right.push_back(r_right.front());
    }

    // (3) Compute the approximated arclength
    std::vector<scalar> s_left(r_left.size());
    std::vector<scalar> s_right(r_right.size());

    for (size_t i = 1; i < r_left.size(); ++i)
        s_left[i] = s_left[i-1] + norm(r_left[i]-r_left[i-1]);

    for (size_t i = 1; i < r_right.size(); ++i)
        s_right[i] = s_right[i-1] + norm(r_right[i]-r_right[i-1]);

    // (4) Project the right boundary into equally-spaced nodes
    std::vector<scalar> s_right_equispaced = linspace(0.0,s_right.back(),n+1);
    if constexpr (closed)
        s_right_equispaced.pop_back();

    Polynomial<sVector3d> track_right(s_right, r_right, 1); 
    std::vector<sVector3d> r_right_equispaced(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        r_right_equispaced[i] = track_right(s_right_equispaced[i]);

    // (5) Get the closest point in the left boundary to each point of the right boundary
    std::vector<sVector3d> r_left_equispaced(n_points);
    for (size_t i = 0; i < n_points; ++i)
        r_left_equispaced[i] = find_closest_point<scalar>(r_left,r_right_equispaced[i], closed).first;

    // (6) Compute the centerline estimation, and close it
    std::vector<sVector3d> r_center = 0.5*(r_left_equispaced + r_right_equispaced);
    if constexpr (closed)
        r_center.push_back(r_center.front());

    std::vector<scalar> s_center(n+1);

    for (size_t i = 1; i < n+1; ++i)
        s_center[i] = s_center[i-1] + norm(r_center[i] - r_center[i-1]);

    // (6) Transform the centerline to equally-spaced points
    std::vector<scalar> s_center_equispaced = linspace(0.0,s_center.back(),n+1);
    Polynomial<sVector3d> track_center(s_center, r_center, 1); 
    std::vector<sVector3d> r_center_equispaced(n_points);

    for (size_t i = 0; i < n_points; ++i)
        r_center_equispaced[i] = track_center(s_center_equispaced[i]);

    // (7) Remove the closing point from the arc-length (so that it has the same dimension as r_center_equispaced, n)
    if constexpr (closed)
        s_center_equispaced.pop_back();

    return {s_center_equispaced, r_center_equispaced};
}


template<bool closed>
void Circuit_preprocessor::FG<closed>::operator()(ADvector& fg, const ADvector& x)
{
    assert(x.size() == _n_variables);
    assert(fg.size() == (1 + _n_constraints));

    // (1) Load the state and control vectors
    size_t k = 0;

    // (1a) First variable is the arclength 
    CppAD::AD<scalar> ds = x[k++];

    // (2a) Load state and controls
    for (size_t i = 0; i < _n_points; ++i)
    {
        // Load state 
        for (size_t j = 0; j < NSTATE; ++j)
            _q[i][j] = x[k++];

        // Load control
        for (size_t j = 0; j < NCONTROLS; ++j)
            _u[i][j] = x[k++];
    }

    // Check that all variables in x were used
    assert(k == FG::_n_variables);

    // (3) Initialize fitness function
    fg[0] = 0.0;

    // (4) Compute the equations for the first node
    _dqds[0] = equations(_q[0],_u[0]);
    _dist2_left[0]   = find_closest_point<CppAD::AD<scalar>>(_r_left, Vector3d<CppAD::AD<scalar>>  (_q[0][IX] - sin(_q[0][ITHETA])*_q[0][INL], _q[0][IY] + cos(_q[0][ITHETA])*_q[0][INL], 0.0), true).second;
    _dist2_right[0]  = find_closest_point<CppAD::AD<scalar>>(_r_right, Vector3d<CppAD::AD<scalar>> (_q[0][IX] + sin(_q[0][ITHETA])*_q[0][INR], _q[0][IY] - cos(_q[0][ITHETA])*_q[0][INR], 0.0), true).second;
    _dist2_center[0] = find_closest_point<CppAD::AD<scalar>>(_r_center, Vector3d<CppAD::AD<scalar>>(_q[0][IX], _q[0][IY], 0.0), true).second;

    // (5) Compute the equations for the i-th node, and append the scheme equations for the i-th element
    k = 1;  // Reset the counter
    for (size_t i = 1; i < _n_points; ++i)
    {
        _dqds[i] = equations(_q[i],_u[i]);
        _dist2_left[i]   = find_closest_point<CppAD::AD<scalar>>(_r_left, Vector3d<CppAD::AD<scalar>>  (_q[i][IX] - sin(_q[i][ITHETA])*_q[i][INL], _q[i][IY] + cos(_q[i][ITHETA])*_q[i][INL], 0.0), true).second;
        _dist2_right[i]  = find_closest_point<CppAD::AD<scalar>>(_r_right, Vector3d<CppAD::AD<scalar>> (_q[i][IX] + sin(_q[i][ITHETA])*_q[i][INR], _q[i][IY] - cos(_q[i][ITHETA])*_q[i][INR], 0.0), true).second;
        _dist2_center[i] = find_closest_point<CppAD::AD<scalar>>(_r_center, Vector3d<CppAD::AD<scalar>>(_q[i][IX], _q[i][IY], 0.0), true).second;

        // Fitness function: minimize the square of the distance to the boundaries and centerline, and control powers
        fg[0] += 0.5*ds*_options.eps_d*(_dist2_left[i] + _dist2_left[i-1]);
        fg[0] += 0.5*ds*_options.eps_d*(_dist2_right[i]  + _dist2_right[i-1]  );
        fg[0] += 0.5*ds*_options.eps_c*(_dist2_center[i] + _dist2_center[i-1] );
        fg[0] += 0.5*ds*_options.eps_k*(_u[i][IDKAPPA]*_u[i][IDKAPPA] + _u[i-1][IDKAPPA]*_u[i-1][IDKAPPA]);
        fg[0] += 0.5*ds*_options.eps_n*(_u[i][IDNL]*_u[i][IDNL] + _u[i-1][IDNL]*_u[i-1][IDNL]);
        fg[0] += 0.5*ds*_options.eps_n*(_u[i][IDNR]*_u[i][IDNR] + _u[i-1][IDNR]*_u[i-1][IDNR]);

        // Equality constraints:  q^{i} = q^{i-1} + 0.5.ds.[dqds^{i} + dqds^{i-1}]
        for (size_t j = 0; j < NSTATE; ++j)
            fg[k++] = _q[i][j] - _q[i-1][j] - 0.5*ds*(_dqds[i-1][j] + _dqds[i][j]);
    }


    // (6) Append the scheme equations of the periodic element if the circuit is closed
    if constexpr (closed)
    {
        // Add the periodic element
        // Fitness function: minimize the square of the distance to the boundaries and centerline, and control powers
        fg[0] += 0.5*ds*_options.eps_d*(_dist2_left[0]    + _dist2_left[_n-1] );
        fg[0] += 0.5*ds*_options.eps_d*(_dist2_right[0]   + _dist2_right[_n-1]  );
        fg[0] += 0.5*ds*_options.eps_c*(_dist2_center[0]  + _dist2_center[_n-1]);
        fg[0] += 0.5*ds*_options.eps_k*(_u[0][IDKAPPA]*_u[0][IDKAPPA] + _u[_n-1][IDKAPPA]*_u[_n-1][IDKAPPA]);
        fg[0] += 0.5*ds*_options.eps_n*(_u[0][IDNL]*_u[0][IDNL] + _u[_n-1][IDNL]*_u[_n-1][IDNL]);
        fg[0] += 0.5*ds*_options.eps_n*(_u[0][IDNR]*_u[0][IDNR] + _u[_n-1][IDNR]*_u[_n-1][IDNR]);
    
        // Equality constraints:  q^{i} = q^{i-1} + 0.5.ds.[dqds^{i} + dqds^{i-1}]
        // except for theta, where q^{i} = q^{i-1} + 0.5.ds.[dqds^{i} + dqds^{i-1}] - 2.pi
        for (size_t j = 0; j < NSTATE; ++j)
            fg[k++] = _q[0][j] - _q[_n-1][j] - 0.5*ds*(_dqds[_n-1][j] + _dqds[0][j]) + (j==ITHETA ? 2.0*pi*_direction : 0.0);
    }

    // (7) Add a last constraint: the first point should be in the start line
    const sVector3d p = _r_left.front() - _r_right.front();
    fg[k++] = cross(p,Vector3d<CppAD::AD<scalar>>(_q.front()[IX], _q.front()[IY], 0.0)).z()/dot(p,p);

    if constexpr (!closed)
    {
        // If track is open, the last point should be in the finish line
        const sVector3d p = _r_left.back() - _r_right.back();
        fg[k++] = cross(p,Vector3d<CppAD::AD<scalar>>(_q.back()[IX], _q.back()[IY], 0.0)).z()/dot(p,p);
    }

    assert(k == FG::_n_constraints+1);
}

#endif
