#ifndef CIRCUIT_PREPROCESSOR_HPP
#define CIRCUIT_PREPROCESSOR_HPP

#include "lion/foundation/utils.hpp"
#include "lion/math/polynomial.h"
#include "lion/math/vector3d.h"
#include "lion/math/matrix_extensions.h"
#include "lion/math/ipopt_cppad_handler.hpp"
#include "src/core/foundation/fastest_lap_exception.h"
#include "src/core/applications/minimum_curvature_path.h"

inline std::pair<std::vector<Circuit_preprocessor::Coordinates>,std::vector<Circuit_preprocessor::Coordinates>>
    Circuit_preprocessor::read_kml(Xml_document& coord_left_kml, Xml_document& coord_right_kml)
{
    // Get child with data for the left boundary 
    std::vector<scalar> coord_left_raw, coord_right_raw;
    if (coord_left_kml.has_element("kml/Document/Placemark"))
        coord_left_raw = coord_left_kml.get_element("kml/Document/Placemark/LineString/coordinates").get_value(std::vector<scalar>());
    else if (coord_left_kml.has_element("kml/Document/Folder/Placemark"))
        coord_left_raw = coord_left_kml.get_element("kml/Document/Folder/Placemark/LineString/coordinates").get_value(std::vector<scalar>());
    else
        throw fastest_lap_exception("[ERROR] Circuit_preprocessor::read_kml -> KML format was not properly parsed");

    if (coord_right_kml.has_element("kml/Document/Placemark"))
        coord_right_raw = coord_right_kml.get_element("kml/Document/Placemark/LineString/coordinates").get_value(std::vector<scalar>());
    else if (coord_right_kml.has_element("kml/Document/Folder/Placemark"))
        coord_right_raw = coord_right_kml.get_element("kml/Document/Folder/Placemark/LineString/coordinates").get_value(std::vector<scalar>());
    else
        throw fastest_lap_exception("[ERROR] Circuit_preprocessor::read_kml -> KML format was not properly parsed");

    if ( coord_left_raw.size() % 3 != 0 )
        throw fastest_lap_exception("Error processing google-earth placemark: size must be multiple of 3");

    if ( coord_right_raw.size() % 3 != 0 )
        throw fastest_lap_exception("Error processing google-earth placemark: size must be multiple of 3");

    const size_t n_left = coord_left_raw.size()/3;
    std::vector<Coordinates> coord_left(n_left);

    const size_t n_right = coord_right_raw.size()/3;
    std::vector<Coordinates> coord_right(n_right);

    if (options.with_elevation)
    {
        for (size_t i = 0; i < n_left; ++i)
            coord_left[i] = { coord_left_raw[3 * i], coord_left_raw[3 * i + 1], coord_left_raw[3*i + 2]};

        for (size_t i = 0; i < n_right; ++i)
            coord_right[i] = { coord_right_raw[3 * i], coord_right_raw[3 * i + 1], coord_right_raw[3*i + 2]};
    }
    else
    {
        for (size_t i = 0; i < n_left; ++i)
            coord_left[i] = { coord_left_raw[3 * i], coord_left_raw[3 * i + 1], 0.0 };

        for (size_t i = 0; i < n_right; ++i)
            coord_right[i] = { coord_right_raw[3 * i], coord_right_raw[3 * i + 1], 0.0 };
    }

    return {coord_left, coord_right};
}


inline Circuit_preprocessor::Circuit_preprocessor(Xml_document& doc)
{
    Xml_element root = doc.get_root_element();

    if ( root.get_attribute("format") != "discrete" )
        throw fastest_lap_exception("Track should be of type \"discrete\"");

    if ( root.get_attribute("type") == "closed" )
        is_closed = true;
    else if ( root.get_attribute("type") == "open" )
        is_closed = false;
    else
        throw fastest_lap_exception("Incorrect track type, should be \"open\" or \"closed\"");

    if (root.has_attribute("dimensions"))
    {
        if (root.get_attribute("dimensions") == "2")
        {
            options.with_elevation = false;
        }
        else if (root.get_attribute("dimensions") == "3")
        {
            options.with_elevation = true;
        }
        else
        {
            throw fastest_lap_exception("[ERROR] Circuit_preprocessor -> \"dimensions\" attribute value \""
                                         + root.get_attribute("dimensions") + "\" not valid. Options are \"2\" or \"3\"");
        }
    }
    else 
    {
        throw fastest_lap_exception("[ERROR] Circuit_preprocessor -> Track XML file must have a dimensions attribute");
    }

    // Get a header with the errors 
    auto header = root.get_child("header");

    track_length             = header.get_child("track_length").get_value(scalar());
    left_boundary_L2_error   = header.get_child("L2_error_left").get_value(scalar());
    right_boundary_L2_error  = header.get_child("L2_error_right").get_value(scalar());
    left_boundary_max_error  = header.get_child("max_error_left").get_value(scalar());
    right_boundary_max_error = header.get_child("max_error_right").get_value(scalar());
    

    // Get optimization section
    auto opt               = root.get_child("optimization");
    options.eps_k          = opt.get_child("cost_curvature").get_value(scalar());
    options.eps_n          = opt.get_child("cost_track_limits_smoothness").get_value(scalar());
    options.eps_d          = opt.get_child("cost_track_limits_errors").get_value(scalar());
    options.eps_c          = opt.get_child("cost_centerline").get_value(scalar());
    options.maximum_yaw_dot  = opt.get_child("maximum_yaw_dot").get_value(scalar());
    options.maximum_dyaw_dot = opt.get_child("maximum_dyaw_dot").get_value(scalar());

    if (options.with_elevation)
    {
        options.eps_pitch = opt.get_child("cost_pitch").get_value(scalar());
        options.eps_roll = opt.get_child("cost_roll").get_value(scalar());
    }

    // Get the GPS coordinates conversion used
    auto gps_param = root.get_child("GPS_parameters");
    yaw0         = gps_param.get_child("origin_longitude").get_value(scalar())*DEG;
    roll0           = gps_param.get_child("origin_latitude").get_value(scalar())*DEG;
    R_earth        = gps_param.get_child("earth_radius").get_value(scalar());
    roll_ref        = gps_param.get_child("reference_latitude").get_value(scalar())*DEG;

    // Get the data
    auto data = root.get_child("data");
    n_points = std::stoi(data.get_attribute("number_of_points"));
    n_elements = (is_closed ? n_points : n_points - 1);

    // Arc-length
    s = data.get_child("arclength").get_value(std::vector<scalar>());

    // Centerline
    std::vector<scalar> x = data.get_child("centerline/x").get_value(std::vector<scalar>());
    std::vector<scalar> y = data.get_child("centerline/y").get_value(std::vector<scalar>());

    std::vector<scalar> z(n_points, 0.0);
    if (options.with_elevation)
        z = data.get_child("centerline/z").get_value(std::vector<scalar>());

    r_centerline = std::vector<sVector3d>(n_points);
    for (size_t i = 0; i < n_points; ++i)
        r_centerline[i] = sVector3d(x[i], y[i], z[i]);

    // Left boundary
    x = data.get_child("left_boundary/x").get_value(std::vector<scalar>());
    y = data.get_child("left_boundary/y").get_value(std::vector<scalar>());

    if (options.with_elevation)
        z = data.get_child("left_boundary/z").get_value(std::vector<scalar>());

    r_left = std::vector<sVector3d>(n_points);
    for (size_t i = 0; i < n_points; ++i)
        r_left[i] = sVector3d(x[i], y[i], z[i]);
    
    // Right boundary
    x = data.get_child("right_boundary/x").get_value(std::vector<scalar>());
    y = data.get_child("right_boundary/y").get_value(std::vector<scalar>());

    if (options.with_elevation)
        z = data.get_child("right_boundary/z").get_value(std::vector<scalar>());

    r_right = std::vector<sVector3d>(n_points);
    for (size_t i = 0; i < n_points; ++i)
        r_right[i] = sVector3d(x[i], y[i], z[i]);
    
    // Left measured boundary
    x = data.get_child("left_measured_boundary/x").get_value(std::vector<scalar>());
    y = data.get_child("left_measured_boundary/y").get_value(std::vector<scalar>());

    z = std::vector<scalar>(x.size(), 0.0);
    if (options.with_elevation)
        z = data.get_child("left_measured_boundary/z").get_value(std::vector<scalar>());

    assert(x.size() == y.size());
    assert(z.size() == x.size());

    r_left_measured = std::vector<sVector3d>(x.size());
    for (size_t i = 0; i < x.size(); ++i)
        r_left_measured[i] = sVector3d(x[i], y[i], z[i]);
    
    // Right measured boundary
    x = data.get_child("right_measured_boundary/x").get_value(std::vector<scalar>());
    y = data.get_child("right_measured_boundary/y").get_value(std::vector<scalar>());

    z = std::vector<scalar>(x.size(), 0.0);
    if (options.with_elevation)
        z = data.get_child("right_measured_boundary/z").get_value(std::vector<scalar>());

    assert(x.size() == y.size());
    assert(z.size() == x.size());

    r_right_measured = std::vector<sVector3d>(x.size());
    for (size_t i = 0; i < x.size(); ++i)
        r_right_measured[i] = sVector3d(x[i], y[i], z[i]);
    
    // Theta
    yaw = data.get_child("yaw").get_value(std::vector<scalar>());

    if (options.with_elevation)
    {
        pitch = data.get_child("pitch").get_value(std::vector<scalar>());
        roll = data.get_child("roll").get_value(std::vector<scalar>());
    }

    // Kappa
    yaw_dot = data.get_child("yaw_dot").get_value(std::vector<scalar>());

    if (options.with_elevation)
    {
        pitch_dot = data.get_child("pitch_dot").get_value(std::vector<scalar>());
        roll_dot = data.get_child("roll_dot").get_value(std::vector<scalar>());
    }

    // nl
    nl = data.get_child("nl").get_value(std::vector<scalar>());

    // nr
    nr = data.get_child("nr").get_value(std::vector<scalar>());

    // dyaw_dot
    dyaw_dot = data.get_child("dyaw_dot").get_value(std::vector<scalar>());

    if (options.with_elevation)
    {
        dpitch_dot = data.get_child("dpitch_dot").get_value(std::vector<scalar>());
        droll_dot = data.get_child("droll_dot").get_value(std::vector<scalar>());
    }

    // dnl
    dnl = data.get_child("dnl").get_value(std::vector<scalar>());

    // dnr
    dnr = data.get_child("dnr").get_value(std::vector<scalar>());

    // Get direction
    if (is_closed)
        direction = ( yaw.back() > yaw.front() ? COUNTERCLOCKWISE : CLOCKWISE);

    // Get kerbs
    if (root.has_child("kerbs"))
    {
        options.compute_kerbs = true;

        auto kerbs = root.get_child("kerbs");
        auto left_kerb_xml = kerbs.get_child("left");
        left_kerb = Kerb(left_kerb_xml);

        auto right_kerb_xml = kerbs.get_child("right");
        right_kerb = Kerb(right_kerb_xml);
    }
}

template<bool closed>
inline void Circuit_preprocessor::transform_coordinates(const std::vector<Coordinates>& coord_left, const std::vector<Coordinates>& coord_right)
{
    // (1) Get reference latitude (arbitrarily, that of the first right curve)
    roll_ref = coord_right.front().latitude*DEG;
    roll0    = coord_right.front().latitude*DEG;
    yaw0  = coord_right.front().longitude*DEG;

    x0   = yaw0*R_earth*cos(roll_ref);
    y0   = roll0*R_earth;

    // (2) Transform coordinates to cartesian (x,y,z) -> Flip y and z
    r_left_measured = std::vector<sVector3d>(coord_left.size(),{0.0,0.0,0.0});
    for (size_t i = 0; i < coord_left.size(); ++i)
        r_left_measured[i] = sVector3d((coord_left[i].longitude*DEG-yaw0)*R_earth*cos(roll_ref), - (coord_left[i].latitude*DEG-roll0)*R_earth, - coord_left[i].altitude);

    r_right_measured = std::vector<sVector3d>(coord_right.size(),{0.0,0.0,0.0});
    for (size_t i = 0; i < coord_right.size(); ++i)
        r_right_measured[i] = sVector3d((coord_right[i].longitude*DEG-yaw0)*R_earth*cos(roll_ref), - (coord_right[i].latitude*DEG-roll0)*R_earth, - coord_right[i].altitude);
}


template<bool closed, typename computation_type>
inline void Circuit_preprocessor::compute(const std::vector<scalar>& s_center, const std::vector<sVector3d>& r_center, 
    const std::vector<sVector3d>& r_center_to_right, const scalar track_length_estimate)
{
    using fg_type       = FG<closed, computation_type>;
    using state_names   = typename fg_type::state_names;
    using control_names = typename fg_type::control_names;

    // (1) Compute the initial condition via finite differences
    std::vector<scalar> x_init(n_points,0.0);
    std::vector<scalar> y_init(n_points,0.0);
    std::vector<scalar> z_init(n_points,0.0);
    std::vector<scalar> yaw_init(n_points,0.0);
    std::vector<scalar> pitch_init(n_points,0.0);
    std::vector<scalar> roll_init(n_points,0.0);
    std::vector<scalar> yaw_dot_init(n_points,0.0);
    std::vector<scalar> pitch_dot_init(n_points,0.0);
    std::vector<scalar> roll_dot_init(n_points,0.0);
    std::vector<scalar> nl_init(n_points,0.0);
    std::vector<scalar> nr_init(n_points,0.0);

    std::vector<scalar> dyaw_dot_init(n_points,0.0);
    std::vector<scalar> dpitch_dot_init(n_points,0.0);
    std::vector<scalar> droll_dot_init(n_points,0.0);
    std::vector<scalar> dnl_init(n_points,0.0);
    std::vector<scalar> dnr_init(n_points,0.0);

    std::vector<sVector3d> tangent_dir_init(n_points, {0.0,0.0,0.0});
    std::vector<sVector3d> normal_dir_init(n_points, {0.0,0.0,0.0});
    std::vector<sVector3d> bi_normal_dir_init(n_points, {0.0,0.0,0.0});

    // (1.1) Extract coordinates
    for (size_t i = 0; i < n_points; ++i)
    {
        x_init[i] = r_center[i][0];
        y_init[i] = r_center[i][1];

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
            z_init[i] = r_center[i][2];
    }

    if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
    {
        // (1.2) Compute Frenet frame

        // (1.2.1) Tangent vector
        std::transform(r_center.cbegin(), r_center.cend() - 1, r_center.cbegin() + 1, tangent_dir_init.begin(), [](const auto& r0, const auto& r1) { return (r1 - r0).normalize(); });

        if (closed)
            tangent_dir_init.back() = (r_center.front() - r_center.back()).normalize();
        else
            tangent_dir_init.back() = tangent_dir_init[n_elements - 1];

        // (1.2.2) Bi-normal vector
        std::transform(tangent_dir_init.cbegin(), tangent_dir_init.cend(), r_center_to_right.cbegin(), bi_normal_dir_init.begin(), [](const auto& t, const auto& n) { return cross(t, n).normalize(); });

        // (1.2.3) Normal vector
        std::transform(tangent_dir_init.cbegin(), tangent_dir_init.cend(), bi_normal_dir_init.cbegin(), normal_dir_init.begin(), [](const auto& t, const auto& b) { return cross(b, t).normalize(); });

        // (1.3) Compute Euler angles
        for (size_t i = 0; i < n_points; ++i)
        {
            const auto [yaw, pitch, roll] = rotmat2ea(transpose(Matrix3x3(tangent_dir_init[i], normal_dir_init[i], bi_normal_dir_init[i])));

            yaw_init[i] = yaw;
            pitch_init[i] = pitch;
            roll_init[i] = roll;
        }
    }
    else
    {
        static_assert(std::is_same_v<computation_type, flat_computation_names>);

        // yaw
        for (size_t i = 0; i < n_points-1; ++i)
            yaw_init[i] = atan2(y_init[i+1]-y_init[i],x_init[i+1]-x_init[i]);
        
        if constexpr (closed)
            yaw_init[n_elements-1] = atan2(y_init[0]-y_init[n_elements-1],x_init[0]-x_init[n_elements-1]);

        else
            yaw_init[n_elements] = yaw_init[n_elements-1];
    }

    // (1.3.1) Make sure that yaw is continuous
    for (size_t i = 1; i < n_points; ++i)
        yaw_init[i] = yaw_init[i - 1] + wrap_to_pi(yaw_init[i] - yaw_init[i - 1]);

    // (1.4) Compute track direction
    if constexpr (closed)
        direction = ( yaw_init[n_elements-1] > yaw_init[0] ? COUNTERCLOCKWISE : CLOCKWISE );

    // (1.5) Compute euler angles derivatives
    for (size_t i = 0; i < n_points - 1; ++i)
    {
        yaw_dot_init[i] = (yaw_init[i + 1] - yaw_init[i]) / (s_center[i + 1] - s_center[i]);

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            pitch_dot_init[i] = (pitch_init[i + 1] - pitch_init[i]) / (s_center[i + 1] - s_center[i]);
            roll_dot_init[i] = (roll_init[i + 1] - roll_init[i]) / (s_center[i + 1] - s_center[i]);
        }
    }

    if (closed)
    {
        yaw_dot_init[n_elements - 1] = (yaw_init[0] + 2.0 * pi * direction - yaw_init[n_elements - 1]) / norm(r_center.front() - r_center.back());

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            pitch_dot_init[n_elements - 1] = (pitch_init[0] - pitch_init[n_elements - 1]) / norm(r_center.front() - r_center.back());
            roll_dot_init[n_elements - 1] = (roll_init[0] - roll_init[n_elements - 1]) / norm(r_center.front() - r_center.back());
        }
    }
    else
    {
        yaw_dot_init[n_elements] = yaw_dot_init[n_elements - 1];

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            pitch_dot_init[n_elements] = pitch_dot_init[n_elements - 1];
            roll_dot_init[n_elements] = roll_dot_init[n_elements - 1];
        }
    }

    // nl and nr
    if constexpr (std::is_same_v<computation_type, flat_computation_names>)
    {
        // nl and nr
        std::array<size_t,2> i_l = {0,0};
        std::array<size_t,2> i_r = {0,0};
        for (size_t i = 0; i < n_points; ++i)
        {
            std::tie(std::ignore,nl_init[i],i_l) = find_closest_point<scalar>(r_left_measured, r_center[i], closed, min(i_l[0],i_l[1]), options.maximum_distance_find);
            std::tie(std::ignore,nr_init[i],i_r) = find_closest_point<scalar>(r_right_measured, r_center[i], closed, min(i_r[0],i_r[1]), options.maximum_distance_find);
            nl_init[i] = sqrt(nl_init[i]);
            nr_init[i] = sqrt(nr_init[i]);
        }
    }
    else
    {
        static_assert(std::is_same_v<computation_type, elevation_computation_names>);
        std::transform(r_center_to_right.cbegin(), r_center_to_right.cend(), nl_init.begin(), [](const auto& n_vector) { return norm(n_vector); });
        nr_init = nl_init;
    }

    // dyaw_dot, dnl, and dnr
    for (size_t i = 0; i < n_points-1; ++i)
    {
        dyaw_dot_init[i] = (yaw_dot_init[i+1]-yaw_dot_init[i])/(s_center[i+1]-s_center[i]);
        dnl_init[i] = (nl_init[i+1]-nl_init[i])/(s_center[i+1]-s_center[i]);
        dnr_init[i] = (nr_init[i+1]-nr_init[i])/(s_center[i+1]-s_center[i]);

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            dpitch_dot_init[i] = (pitch_dot_init[i + 1] - pitch_dot_init[i]) / (s_center[i + 1] - s_center[i]);
            droll_dot_init[i] = (roll_dot_init[i + 1] - roll_dot_init[i]) / (s_center[i + 1] - s_center[i]);
        }
    }

    if (closed)
    {
        dyaw_dot_init[n_elements-1] = (yaw_dot_init[0] - yaw_dot_init[n_elements-1])/norm(r_center.front()-r_center.back());
        dnl_init[n_elements-1] = (nl_init[0] - nl_init[n_elements-1])/norm(r_center.front()-r_center.back());
        dnr_init[n_elements-1] = (nr_init[0] - nr_init[n_elements-1])/norm(r_center.front()-r_center.back());

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            dpitch_dot_init[n_elements - 1] = (pitch_dot_init[0] - pitch_dot_init[n_elements - 1]) / norm(r_center.front() - r_center.back());
            droll_dot_init[n_elements - 1] = (roll_dot_init[0] - roll_dot_init[n_elements - 1]) / norm(r_center.front() - r_center.back());
        }
    }
    else
    {
        dyaw_dot_init[n_elements] = dyaw_dot_init[n_elements-1];
        dnl_init[n_elements] = dnl_init[n_elements-1];
        dnr_init[n_elements] = dnr_init[n_elements-1];

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            dpitch_dot_init[n_elements] = dpitch_dot_init[n_elements - 1];
            droll_dot_init[n_elements] = droll_dot_init[n_elements - 1];
        }
    }

    // compute the ds candidate
    std::vector<scalar> element_ds(n_elements);

    for (size_t i = 1; i < n_points; ++i)
        element_ds[i-1] = s_center[i] - s_center[i-1];

    if (closed)
        element_ds[n_elements-1] = track_length_estimate - s_center.back();

    // (2) Create the FG object
    fg_type fg(n_elements, n_points, element_ds, r_left_measured, r_right_measured, r_center, direction, options);

    // load them into an x vector
    std::vector<scalar> x(fg.get_n_variables());
    std::vector<scalar> x_lb(fg.get_n_variables());
    std::vector<scalar> x_ub(fg.get_n_variables());
    size_t k = 0;
    size_t k_ub = 0;
    size_t k_lb = 0;
    x[k++] = 1.0;
    x_lb[k_lb++] = 0.9;
    x_ub[k_ub++] = 1.11;
    for (size_t i = 0; i < n_points; ++i)
    {
        x[k++] = x_init[i];
        x[k++] = y_init[i];

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
            x[k++] = z_init[i];

        x[k++] = yaw_init[i];

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            x[k++] = pitch_init[i];
            x[k++] = roll_init[i];
        }

        x[k++] = yaw_dot_init[i];

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            x[k++] = pitch_dot_init[i];
            x[k++] = roll_dot_init[i];
        }

        x[k++] = nl_init[i];
        x[k++] = nr_init[i];
        x[k++] = dyaw_dot_init[i];

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            x[k++] = dpitch_dot_init[i];
            x[k++] = droll_dot_init[i];
        }

        x[k++] = dnl_init[i];
        x[k++] = dnr_init[i];

        x_lb[k_lb++] = x_init[i]-100.0;
        x_lb[k_lb++] = y_init[i]-100.0;

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
            x_lb[k_lb++] = z_init[i] - 10.0;

        x_lb[k_lb++] = yaw_init[i]-30.0*DEG;

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            x_lb[k_lb++] = pitch_init[i] - 10.0 * DEG;
            x_lb[k_lb++] = roll_init[i] - 10.0 * DEG;
        }

        x_lb[k_lb++] = -options.maximum_yaw_dot;

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            x_lb[k_lb++] = -options.maximum_yaw_dot;
            x_lb[k_lb++] = -options.maximum_yaw_dot;
        }

        x_lb[k_lb++] = 1.0;
        x_lb[k_lb++] = 1.0;
        x_lb[k_lb++] = -options.maximum_dyaw_dot;

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            x_lb[k_lb++] = -options.maximum_dyaw_dot;
            x_lb[k_lb++] = -options.maximum_dyaw_dot;
        }

        x_lb[k_lb++] = -options.maximum_dn;
        x_lb[k_lb++] = -options.maximum_dn;

        x_ub[k_ub++] = x_init[i]+100.0;
        x_ub[k_ub++] = y_init[i]+100.0;

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
            x_ub[k_ub++] = z_init[i]+10.0;

        x_ub[k_ub++] = yaw_init[i]+30.0*DEG;

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            x_ub[k_ub++] = pitch_init[i] + 10.0 * DEG;
            x_ub[k_ub++] = roll_init[i] + 10.0 * DEG;
        }

        x_ub[k_ub++] = options.maximum_yaw_dot;

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            x_ub[k_ub++] = options.maximum_yaw_dot;
            x_ub[k_ub++] = options.maximum_yaw_dot;
        }

        x_ub[k_ub++] = nl_init[i]+nr_init[i];
        x_ub[k_ub++] = nl_init[i]+nr_init[i];
        x_ub[k_ub++] = options.maximum_dyaw_dot;

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            x_ub[k_ub++] = options.maximum_dyaw_dot;
            x_ub[k_ub++] = options.maximum_dyaw_dot;
        }

        x_ub[k_ub++] = options.maximum_dn;
        x_ub[k_ub++] = options.maximum_dn;
    }

    assert(k == fg.get_n_variables());
    assert(k_lb == fg.get_n_variables());
    assert(k_ub == fg.get_n_variables());

    // (7) Run the optimization
    std::string ipoptoptions;
    ipoptoptions += "Integer print_level  ";
    ipoptoptions += std::to_string(options.print_level);
    ipoptoptions += "\n";
    ipoptoptions += "String  sb           yes\n";
    ipoptoptions += "Sparse true forward\n";
    ipoptoptions += "Retape true\n";
    ipoptoptions += "Numeric tol          1e-10\n";
    ipoptoptions += "Numeric constr_viol_tol  1e-10\n";
    ipoptoptions += "Numeric acceptable_tol  1e-8\n";

    // place to return solution
    CppAD::ipopt_cppad_result<std::vector<scalar>> result;

    // solve the problem
    CppAD::ipopt_cppad_solve(ipoptoptions, x, x_lb, x_ub, std::vector<scalar>(fg.get_n_constraints(),0.0), std::vector<scalar>(fg.get_n_constraints(),0.0), fg, result);

    if ( result.status != CppAD::ipopt_cppad_result<std::vector<scalar>>::success )
    {
        throw fastest_lap_exception("Optimization did not succeed");
    }

    // Load the solution
    s            = std::vector<scalar>(n_points,0.0);
    r_left       = std::vector<sVector3d>(n_points, { 0.0, 0.0, 0.0 });
    r_right      = std::vector<sVector3d>(n_points, { 0.0, 0.0, 0.0 });
    r_centerline = std::vector<sVector3d>(n_points, { 0.0, 0.0, 0.0 });
    yaw        = std::vector<scalar>(n_points, 0.0);
    pitch           = std::vector<scalar>(n_points, 0.0);
    roll          = std::vector<scalar>(n_points, 0.0);
    yaw_dot        = std::vector<scalar>(n_points, 0.0);
    pitch_dot       = std::vector<scalar>(n_points, 0.0);
    roll_dot      = std::vector<scalar>(n_points, 0.0);
    nl           = std::vector<scalar>(n_points, 0.0);
    nr           = std::vector<scalar>(n_points, 0.0);
    dyaw_dot       = std::vector<scalar>(n_points, 0.0);
    dpitch_dot      = std::vector<scalar>(n_points, 0.0);
    droll_dot     = std::vector<scalar>(n_points, 0.0);
    dnl          = std::vector<scalar>(n_points, 0.0);
    dnr          = std::vector<scalar>(n_points, 0.0);

    for (size_t i = 1; i < n_points; ++i)
        s[i] = s[i-1] + element_ds[i-1]*result.x[0];

    track_length = s.back() + (closed ? element_ds.back()*result.x[0] : 0.0);
    
    for (size_t i = 0; i < n_points; ++i)
    {
        std::array<scalar, state_names::end> states;
        std::array<scalar, control_names::end> controls;

        std::copy_n(result.x.cbegin() + 1 + (state_names::end + control_names::end) * i, state_names::end, states.begin());
        std::copy_n(result.x.cbegin() + 1 + (state_names::end + control_names::end) * i + state_names::end, control_names::end, controls.begin());

        // Get indexes
        r_left[i]  = fg_type::get_coordinates(states, -states[state_names::nl]);
        r_right[i] = fg_type::get_coordinates(states, states[state_names::nr]);
        r_centerline[i] = fg_type::get_coordinates(states, 0.0);

        yaw[i]  = states[state_names::yaw];
        yaw_dot[i]  = states[state_names::yaw_dot];
        nl[i]     = states[state_names::nl];
        nr[i]     = states[state_names::nr];
        dyaw_dot[i] = controls[control_names::dyaw_dot];
        dnl[i]    = controls[control_names::dnl];
        dnr[i]    = controls[control_names::dnr];

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            pitch[i]       = states[state_names::pitch];
            roll[i]      = states[state_names::roll];
            pitch_dot[i]   = states[state_names::pitch_dot];
            roll_dot[i]  = states[state_names::roll_dot];
            dpitch_dot[i]  = controls[control_names::dpitch_dot];
            droll_dot[i] = controls[control_names::droll_dot];
        }
    }

    // Compute the errors
    left_boundary_max_error   = sqrt(std::get<1>(find_closest_point<scalar>(r_left_measured,r_left.front(), closed, 0, 1.0e18)));
    right_boundary_max_error  = sqrt(std::get<1>(find_closest_point<scalar>(r_right_measured,r_right.front(), closed, 0, 1.0e18)));
    left_boundary_L2_error   = 0.0;
    right_boundary_L2_error  = 0.0;

    scalar prev_left_error  = left_boundary_max_error;
    scalar prev_right_error = right_boundary_max_error;
    std::array<size_t,2> i_l = {0,0};
    std::array<size_t,2> i_r = {0,0};

    for (size_t i = 1; i < n_points; ++i)
    {
        const scalar ds = s[i]-s[i-1];
        // Compute current error
        scalar current_left_error_2; 
        scalar current_right_error_2; 
        std::tie(std::ignore,current_left_error_2,i_l) = find_closest_point<scalar>(r_left_measured, r_left[i], closed, min(i_l[0],i_l[1]), options.maximum_distance_find);
        std::tie(std::ignore,current_right_error_2,i_r) = find_closest_point<scalar>(r_right_measured, r_right[i], closed, min(i_r[0],i_r[1]), options.maximum_distance_find);

        const scalar current_left_error = sqrt(current_left_error_2);
        const scalar current_right_error = sqrt(current_right_error_2);

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
        const scalar ds = track_length - s.back();
        scalar current_left_error = sqrt(std::get<1>(find_closest_point<scalar>(r_left_measured,r_left.front(), closed, 0, 1.0e18)));
        scalar current_right_error = sqrt(std::get<1>(find_closest_point<scalar>(r_right_measured,r_right.front(), closed, 0, 1.0e18)));

        // Compute L2 error
        left_boundary_L2_error  += 0.5*ds*(prev_left_error*prev_left_error + current_left_error*current_left_error);
        right_boundary_L2_error += 0.5*ds*(prev_right_error*prev_right_error + current_right_error*current_right_error);
    }

    left_boundary_L2_error = sqrt(left_boundary_L2_error/track_length);
    right_boundary_L2_error = sqrt(right_boundary_L2_error/track_length);

    // (7) Compute kerbs
    if (options.compute_kerbs)
    {
        // (7.1) Run a minimum curvature problem
        const Minimum_curvature_path minimum_curvature(*this, s, is_closed, {});

        minimum_curvature.xml();

        // (7.2) Compute left kerb
        auto lateral_displacement_percentage = (minimum_curvature.n + nl);
        for (size_t i_s = 0; i_s < s.size(); ++i_s)
            lateral_displacement_percentage[i_s] *= 1.0 / (nr[i_s] + nl[i_s]);

        left_kerb = Kerb(s, 
                         std::vector<scalar>(s.size(),1.0) - lateral_displacement_percentage, 
                         yaw_dot, 
                         Kerb::Side::left, 
                         options.kerb_width, 
                         options.use_kerbs, 
                         options.exterior_kerbs_direction);

        right_kerb = Kerb(s, 
                          lateral_displacement_percentage, 
                          yaw_dot, 
                          Kerb::Side::right, 
                          options.kerb_width, 
                          options.use_kerbs, 
                          options.exterior_kerbs_direction);
    }
}


inline std::unique_ptr<Xml_document> Circuit_preprocessor::xml() const
{
    std::ostringstream s_out;
    s_out.precision(17);
    std::unique_ptr<Xml_document> doc_ptr(std::make_unique<Xml_document>());

    doc_ptr->create_root_element("circuit");

    Xml_element root = doc_ptr->get_root_element();

    root.set_attribute("format","discrete");

    if (is_closed)
        root.set_attribute("type","closed");
    else
        root.set_attribute("type","open");

    root.set_attribute("dimensions", (options.with_elevation ? "3" : "2"));
    
    // Add a header with the errors 
    auto header = root.add_child("header");

    s_out << track_length;
    header.add_child("track_length").set_value(s_out.str()).set_attribute("units","m");
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
    opt.add_child("cost_curvature").set_value(std::to_string(options.eps_k));
    opt.add_child("cost_track_limits_smoothness").set_value(std::to_string(options.eps_n));
    opt.add_child("cost_track_limits_errors").set_value(std::to_string(options.eps_d));
    opt.add_child("cost_centerline").set_value(std::to_string(options.eps_c));
    opt.add_child("cost_pitch").set_value(std::to_string(options.eps_pitch));
    opt.add_child("cost_roll").set_value(std::to_string(options.eps_roll));
    opt.add_child("maximum_yaw_dot").set_value(std::to_string(options.maximum_yaw_dot));
    opt.add_child("maximum_dyaw_dot").set_value(std::to_string(options.maximum_dyaw_dot));


    // Add the GPS coordinates conversion used
    auto gps_param = root.add_child("GPS_parameters");
    gps_param.add_comment(" x =  earth_radius.cos(reference_latitude).(longitude - origin_longitude) ");
    gps_param.add_comment(" y = -earth_radius.(latitude - origin_latitude) ");

    s_out << yaw0*RAD;
    gps_param.add_child("origin_longitude").set_value(s_out.str()).set_attribute("units","deg");
    s_out.str(""); s_out.clear();

    s_out << roll0*RAD;
    gps_param.add_child("origin_latitude").set_value(s_out.str()).set_attribute("units","deg");
    s_out.str(""); s_out.clear();

    s_out << R_earth;
    gps_param.add_child("earth_radius").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    s_out << roll_ref*RAD;
    gps_param.add_child("reference_latitude").set_value(s_out.str()).set_attribute("units","deg");
    s_out.str(""); s_out.clear();

    // Add the data
    auto data = root.add_child("data");
    data.set_attribute("number_of_points",std::to_string(n_points));

    // Arc-length
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << s[i] << ", " ;
    s_out << s.back();

    data.add_child("arclength").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    // Centerline
    auto centerline = data.add_child("centerline");

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_centerline[i].x() << ", " ;
    s_out << r_centerline.back().x();

    centerline.add_child("x").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_centerline[i].y() << ", " ;
    s_out << r_centerline.back().y();

    centerline.add_child("y").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_centerline[i].z() << ", " ;
    s_out << r_centerline.back().z();

    centerline.add_child("z").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    // Left boundary
    auto left = data.add_child("left_boundary");

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_left[i].x() << ", " ;
    s_out << r_left.back().x();

    left.add_child("x").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_left[i].y() << ", " ;
    s_out << r_left.back().y();

    left.add_child("y").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_left[i].z() << ", " ;
    s_out << r_left.back().z();

    left.add_child("z").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    
    // Right boundary
    auto right = data.add_child("right_boundary");

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_right[i].x() << ", " ;
    s_out << r_right.back().x();

    right.add_child("x").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_right[i].y() << ", " ;
    s_out << r_right.back().y();

    right.add_child("y").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();
    
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << r_right[i].z() << ", " ;
    s_out << r_right.back().z();

    right.add_child("z").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    // Left measured boundary
    auto left_measured = data.add_child("left_measured_boundary");

    for (size_t i = 0; i < r_left_measured.size()-1; ++i)
        s_out << r_left_measured[i].x() << ", " ;
    s_out << r_left_measured.back().x();

    left_measured.add_child("x").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < r_left_measured.size()-1; ++i)
        s_out << r_left_measured[i].y() << ", " ;
    s_out << r_left_measured.back().y();

    left_measured.add_child("y").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();
    
    for (size_t i = 0; i < r_left_measured.size()-1; ++i)
        s_out << r_left_measured[i].z() << ", " ;
    s_out << r_left_measured.back().z();

    left_measured.add_child("z").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();
    

    // Left measured boundary
    auto right_measured = data.add_child("right_measured_boundary");

    for (size_t i = 0; i < r_right_measured.size()-1; ++i)
        s_out << r_right_measured[i].x() << ", " ;
    s_out << r_right_measured.back().x();

    right_measured.add_child("x").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < r_right_measured.size()-1; ++i)
        s_out << r_right_measured[i].y() << ", " ;
    s_out << r_right_measured.back().y();

    right_measured.add_child("y").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    for (size_t i = 0; i < r_right_measured.size()-1; ++i)
        s_out << r_right_measured[i].z() << ", " ;
    s_out << r_right_measured.back().z();

    right_measured.add_child("z").set_value(s_out.str()).set_attribute("units","m");
    s_out.str(""); s_out.clear();

    // Theta
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << yaw[i] << ", " ;
    s_out << yaw.back();

    data.add_child("yaw").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // Mu
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << pitch[i] << ", " ;
    s_out << pitch.back();

    data.add_child("pitch").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // Phi
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << roll[i] << ", " ;
    s_out << roll.back();

    data.add_child("roll").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // Kappa
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << yaw_dot[i] << ", " ;
    s_out << yaw_dot.back();

    data.add_child("yaw_dot").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // pitch_dot
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << pitch_dot[i] << ", " ;
    s_out << pitch_dot.back();

    data.add_child("pitch_dot").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // roll_dot
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << roll_dot[i] << ", " ;
    s_out << roll_dot.back();

    data.add_child("roll_dot").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // nl
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << nl[i] << ", " ;
    s_out << nl.back();

    data.add_child("nl").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // nr
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << nr[i] << ", " ;
    s_out << nr.back();

    data.add_child("nr").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // dyaw_dot
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << dyaw_dot[i] << ", " ;
    s_out << dyaw_dot.back();

    data.add_child("dyaw_dot").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // dpitch_dot
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << dpitch_dot[i] << ", " ;
    s_out << dpitch_dot.back();

    data.add_child("dpitch_dot").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // droll_dot
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << droll_dot[i] << ", " ;
    s_out << droll_dot.back();

    data.add_child("droll_dot").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // dnl
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << dnl[i] << ", " ;
    s_out << dnl.back();

    data.add_child("dnl").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    // dnr
    for (size_t i = 0; i < n_points-1; ++i)
        s_out << dnr[i] << ", " ;
    s_out << dnr.back();

    data.add_child("dnr").set_value(s_out.str()).set_attribute("units","rad");
    s_out.str(""); s_out.clear();

    if (options.compute_kerbs)
    {
        auto kerbs = root.add_child("kerbs");
        auto left_kerb_xml = kerbs.add_child("left");
        left_kerb.xml(left_kerb_xml);

        auto right_kerb_xml = kerbs.add_child("right");
        right_kerb.xml(right_kerb_xml);
    }

    return doc_ptr;
}

template<bool closed>
inline auto Circuit_preprocessor::compute_averaged_centerline
    (std::vector<sVector3d> r_left, std::vector<sVector3d> r_right, const size_t n_elements, const size_t n_points,
     const Options& options) -> Centerline
{
    // (2) If closed, add the first point as last point to close the track (needed to construct a polynomial version)
    if constexpr (closed)
        r_right.push_back(r_right.front());

    // (3) Compute the approximated arclength
    std::vector<scalar> s_right(r_right.size());

    for (size_t i = 1; i < r_right.size(); ++i)
        s_right[i] = s_right[i-1] + norm(r_right[i]-r_right[i-1]);

    // (4) Project the right boundary into a set of nodes as close as possible to the ds_distribution
    std::vector<scalar> s_right_equispaced = linspace(0.0,s_right.back(),n_elements+1);
    if constexpr (closed)
        s_right_equispaced.pop_back();

    Polynomial<sVector3d> track_right(s_right, r_right, 1); 
    std::vector<sVector3d> r_right_equispaced(n_points);
    
    for (size_t i = 0; i < n_points; ++i)
        r_right_equispaced[i] = track_right(s_right_equispaced[i]);

    // (5) Get the closest point in the left boundary to each point of the right boundary
    std::vector<sVector3d> r_left_equispaced(n_points);
    std::array<size_t,2> i_left = {0, 0};
    for (size_t i = 0; i < n_points; ++i)
        std::tie(r_left_equispaced[i],std::ignore,i_left) 
            = find_closest_point<scalar>(r_left,r_right_equispaced[i], closed, min(i_left[0],i_left[1]), options.maximum_distance_find);

    // (6) Compute the centerline estimation, and close it
    std::vector<sVector3d> r_center = 0.5*(r_left_equispaced + r_right_equispaced);
    std::vector<sVector3d> r_center_to_right = 0.5 * (r_right_equispaced - r_left_equispaced);
    if constexpr (closed)
    {
        r_center.push_back(r_center.front());
        r_center_to_right.push_back(r_center_to_right.front());
    }

    std::vector<scalar> s_center(n_elements+1);

    for (size_t i = 1; i < n_elements+1; ++i)
        s_center[i] = s_center[i-1] + norm(r_center[i] - r_center[i-1]);

    const scalar track_length_estimate = s_center.back();

    // (6) Transform the centerline to equally-spaced points
    std::vector<scalar> s_center_equispaced = linspace(0.0,s_center.back(),n_elements+1);
    Polynomial<sVector3d> r_center_polynomial(s_center, r_center, 1), n_right_polynomial(s_center, r_center_to_right, 1);
    std::vector<sVector3d> r_center_equispaced(n_points), n_right_equispaced(n_points);

    for (size_t i = 0; i < n_points; ++i)
    {
        r_center_equispaced[i] = r_center_polynomial(s_center_equispaced[i]);
        n_right_equispaced[i] = n_right_polynomial(s_center_equispaced[i]);
    }

    // (7) Remove the closing point from the arc-length (so that it has the same dimension as r_center_equispaced, n_elements)
    if constexpr (closed)
        s_center_equispaced.pop_back();

    return Centerline { .s = s_center_equispaced, .r_center = r_center_equispaced, .r_center_to_right = n_right_equispaced, .track_length = track_length_estimate };
}


template<bool closed>
inline auto Circuit_preprocessor::compute_averaged_centerline
    (std::vector<sVector3d> r_left, std::vector<sVector3d> r_right, const std::vector<std::pair<sVector3d,scalar>>& ds_breakpoints, 
     const Options& options) -> Centerline
{
    // (2) If closed, add the first point as last point to close the track (needed to construct a polynomial version)
    if constexpr (closed)
        r_right.push_back(r_right.front());

    // (3) Compute the approximated arclength
    std::vector<scalar> s_right(r_right.size());

    for (size_t i = 1; i < r_right.size(); ++i)
        s_right[i] = s_right[i-1] + norm(r_right[i]-r_right[i-1]);

    // (4) Project the right boundary into elements with the ds_breakpoints
    Polynomial<sVector3d> track_right(s_right, r_right, 1); 
    std::vector<scalar> s_right_mesh = {0.0, ds_breakpoints.front().second};
    std::vector<sVector3d> r_right_mesh = { track_right(s_right_mesh[0]), track_right(s_right_mesh[1]) };

    scalar ds_prev = ds_breakpoints.front().second;
    while ( s_right_mesh.back() < s_right.back() )
    {
        scalar ds = compute_ds_for_coordinates<closed>(r_right_mesh.back(), r_right, ds_breakpoints);

        // Restrict the maximum aspect ratio of adjacent cells
        if ( ds > options.adaption_aspect_ratio_max*ds_prev )
            ds = options.adaption_aspect_ratio_max*ds_prev;
        else if ( ds < ds_prev/options.adaption_aspect_ratio_max )
            ds = ds_prev/options.adaption_aspect_ratio_max;

        s_right_mesh.push_back(s_right_mesh.back() + ds);
        r_right_mesh.push_back(track_right(min(s_right_mesh.back(),s_right.back())));

        ds_prev = ds;
    }

    // Squash the last 6 elements to equally-spaced nodes
    scalar ds = (s_right.back() - s_right_mesh[s_right_mesh.size() - 7])/6.0;

    for (size_t i = 0; i < 6; ++i)
    {
        s_right_mesh[s_right_mesh.size()-6+i] = s_right_mesh[s_right_mesh.size()-7] + (i+1)*ds;
        r_right_mesh[s_right_mesh.size()-6+i] = track_right(s_right_mesh[s_right_mesh.size()-6+i]);
    }

    if constexpr (closed)
    {
        s_right_mesh.pop_back();
        r_right_mesh.pop_back();
    }

    const size_t n_points = s_right_mesh.size();
    const size_t n_elements = (closed ? n_points : n_points - 1);

    // (5) Get the closest point in the left boundary to each point of the right boundary
    std::vector<sVector3d> r_left_mesh(n_points);
    std::array<size_t,2> i_left = {0,0};
    for (size_t i = 0; i < n_points; ++i)
        std::tie(r_left_mesh[i],std::ignore,i_left) 
            = find_closest_point<scalar>(r_left,r_right_mesh[i], closed, min(i_left[0],i_left[1]), options.maximum_distance_find);

    // (6) Compute the centerline estimation, and close it
    std::vector<sVector3d> r_center = 0.5*(r_left_mesh + r_right_mesh);
    std::vector<sVector3d> r_center_to_right = 0.5 * (r_right_mesh - r_left_mesh);
    if constexpr (closed)
    {
        r_center.push_back(r_center.front());
        r_center_to_right.push_back(r_center_to_right.front());
    }

    std::vector<scalar> s_center(n_elements+1);

    for (size_t i = 1; i < n_elements+1; ++i)
        s_center[i] = s_center[i-1] + norm(r_center[i] - r_center[i-1]);

    // (6) Transform the centerline to mesh points
    Polynomial<sVector3d> track_center(s_center, r_center, 1); 
    Polynomial<sVector3d> r_center_to_right_poly(s_center, r_center_to_right, 1);
    std::vector<scalar> s_center_mesh = {0.0, ds_breakpoints.front().second};
    std::vector<sVector3d> r_center_mesh = { track_center(s_center_mesh[0]), track_center(s_center_mesh[1]) };
    std::vector<sVector3d> r_center_to_right_mesh = { r_center_to_right_poly(s_center_mesh[0]), r_center_to_right_poly(s_center_mesh[1]) };

    ds_prev = ds_breakpoints.front().second;
    while ( s_center_mesh.back() < s_center.back() )
    {
        scalar ds = compute_ds_for_coordinates<closed>(r_center_mesh.back(), r_center, ds_breakpoints);

        // Restrict the maximum aspect ratio of adjacent cells
        if ( ds > options.adaption_aspect_ratio_max*ds_prev )
            ds = options.adaption_aspect_ratio_max*ds_prev;
        else if ( ds < ds_prev/options.adaption_aspect_ratio_max )
            ds = ds_prev/options.adaption_aspect_ratio_max;

        s_center_mesh.push_back(s_center_mesh.back() + ds);
        r_center_mesh.push_back(track_center(min(s_center_mesh.back(),s_center.back())));
        r_center_to_right_mesh.push_back(r_center_to_right_poly(min(s_center_mesh.back(),s_center.back())));

        ds_prev = ds;
    }

    // Squash the last 6 elements to equally-spaced nodes
    ds = (s_center.back() - s_center_mesh[s_center_mesh.size() - 7])/6.0;

    for (size_t i = 0; i < 6; ++i)
    {
        s_center_mesh[s_center_mesh.size()-6+i] = s_center_mesh[s_center_mesh.size()-7] + (i+1)*ds;
        r_center_mesh[s_center_mesh.size()-6+i] = track_center(s_center_mesh[s_center_mesh.size()-6+i]);
        r_center_to_right_mesh[s_center_mesh.size()-6+i] = r_center_to_right_poly(s_center_mesh[s_center_mesh.size()-6+i]);
    }

    const scalar track_length_estimate = s_center_mesh.back();

    // (7) Remove the closing point
    if constexpr (closed)
    {
        s_center_mesh.pop_back();
        r_center_mesh.pop_back();
        r_center_to_right_mesh.pop_back();
    }

    return Centerline { .s = s_center_mesh, .r_center = r_center_mesh, .r_center_to_right = r_center_to_right_mesh, .track_length = track_length_estimate };
}


template<bool closed>
inline auto Circuit_preprocessor::compute_averaged_centerline
    (std::vector<sVector3d> r_left, std::vector<sVector3d> r_right, const std::vector<scalar>& s_distribution, const std::vector<scalar>& ds_distribution,
     const Options& options) -> Centerline
{
    // (1) Construct a polynomial with the ds = f(s)
    sPolynomial f_ds(s_distribution,ds_distribution,1,false);

    // (2) If closed, add the first point as last point to close the track (needed to construct a polynomial version)
    if constexpr (closed)
        r_right.push_back(r_right.front());

    // (3) Compute the approximated arclength
    std::vector<scalar> s_right(r_right.size());


    for (size_t i = 1; i < r_right.size(); ++i)
        s_right[i] = s_right[i-1] + norm(r_right[i]-r_right[i-1]);

    // (4) Project the right boundary into elements with the ds_breakpoints
    Polynomial<sVector3d> track_right(s_right, r_right, 1); 
    std::vector<scalar> s_right_mesh = {0.0};
    std::vector<sVector3d> r_right_mesh = { track_right(s_right_mesh[0]) };

    scalar ds_prev = f_ds(0.0);
    while ( s_right_mesh.back() < s_right.back() )
    {
        scalar ds = (s_right_mesh.back() < s_distribution.back() ? f_ds(s_right_mesh.back()) : ds_distribution.back());

        // Restrict the maximum aspect ratio of adjacent cells
        if ( ds > options.adaption_aspect_ratio_max*ds_prev )
            ds = options.adaption_aspect_ratio_max*ds_prev;
        else if ( ds < ds_prev/options.adaption_aspect_ratio_max )
            ds = ds_prev/options.adaption_aspect_ratio_max;

        s_right_mesh.push_back(s_right_mesh.back() + ds);
        r_right_mesh.push_back(track_right(min(s_right_mesh.back(),s_right.back())));

        ds_prev = ds;
    }

    // Squash the last 6 elements to equally-spaced nodes
    scalar ds = (s_right.back() - s_right_mesh[s_right_mesh.size() - 7])/6.0;

    for (size_t i = 0; i < 6; ++i)
    {
        s_right_mesh[s_right_mesh.size()-6+i] = s_right_mesh[s_right_mesh.size()-7] + (i+1)*ds;
        r_right_mesh[s_right_mesh.size()-6+i] = track_right(s_right_mesh[s_right_mesh.size()-6+i]);
    }

    if constexpr (closed)
    {
        s_right_mesh.pop_back();
        r_right_mesh.pop_back();
    }

    const size_t n_points = s_right_mesh.size();
    const size_t n_elements = (closed ? n_points : n_points - 1);

    // (5) Get the closest point in the left boundary to each point of the right boundary
    std::vector<sVector3d> r_left_mesh(n_points);
    std::array<size_t,2> i_left = {0,0};
    for (size_t i = 0; i < n_points; ++i)
        std::tie(r_left_mesh[i],std::ignore,i_left) 
            = find_closest_point<scalar>(r_left,r_right_mesh[i], closed, min(i_left[0],i_left[1]), options.maximum_distance_find);

    // (6) Compute the centerline estimation, and close it
    std::vector<sVector3d> r_center = 0.5*(r_left_mesh + r_right_mesh);
    std::vector<sVector3d> r_center_to_right = 0.5*(r_right_mesh - r_left_mesh);
    if constexpr (closed)
    {
        r_center.push_back(r_center.front());
        r_center_to_right.push_back(r_center_to_right.front());
    }

    std::vector<scalar> s_center(n_elements+1);

    for (size_t i = 1; i < n_elements+1; ++i)
        s_center[i] = s_center[i-1] + norm(r_center[i] - r_center[i-1]);

    // (6) Transform the centerline to mesh points
    Polynomial<sVector3d> track_center(s_center, r_center, 1); 
    Polynomial<sVector3d> r_center_to_right_poly(s_center, r_center_to_right, 1); 
    std::vector<scalar> s_center_mesh = {0.0};
    std::vector<sVector3d> r_center_mesh = {track_center(s_center_mesh[0])};
    std::vector<sVector3d> r_center_to_right_mesh = {r_center_to_right_poly(s_center_mesh[0])};

    ds_prev = f_ds(0.0);
    while ( s_center_mesh.back() < s_center.back() )
    {
        scalar ds = (s_center_mesh.back() < s_distribution.back() ? f_ds(s_center_mesh.back()) : ds_distribution.back());

        // Restrict the maximum aspect ratio of adjacent cells
        if ( ds > options.adaption_aspect_ratio_max*ds_prev )
            ds = options.adaption_aspect_ratio_max*ds_prev;
        else if ( ds < ds_prev/options.adaption_aspect_ratio_max )
            ds = ds_prev/options.adaption_aspect_ratio_max;

        s_center_mesh.push_back(s_center_mesh.back() + ds);
        r_center_mesh.push_back(track_center(min(s_center_mesh.back(),s_center.back())));
        r_center_to_right_mesh.push_back(r_center_to_right_poly(min(s_center_mesh.back(),s_center.back())));

        ds_prev = ds;
    }

    // Squash the last 6 elements to equally-spaced nodes
    ds = (s_center.back() - s_center_mesh[s_center_mesh.size() - 7])/6.0;

    for (size_t i = 0; i < 6; ++i)
    {
        s_center_mesh[s_center_mesh.size()-6+i] = s_center_mesh[s_center_mesh.size()-7] + (i+1)*ds;
        r_center_mesh[s_center_mesh.size()-6+i] = track_center(s_center_mesh[s_center_mesh.size()-6+i]);
        r_center_to_right_mesh[s_center_mesh.size()-6+i] = r_center_to_right_poly(s_center_mesh[s_center_mesh.size()-6+i]);
    }

    const scalar track_length_estimate = s_center_mesh.back();

    // (7) Remove the closing point
    if constexpr (closed)
    {
        s_center_mesh.pop_back();
        r_center_mesh.pop_back();
        r_center_to_right_mesh.pop_back();
    }

    return Centerline { .s = s_center_mesh, .r_center = r_center_mesh, .r_center_to_right = r_center_to_right_mesh, .track_length = track_length_estimate };
}


inline std::pair<std::vector<Circuit_preprocessor::Coordinates>, std::vector<Circuit_preprocessor::Coordinates>> 
    Circuit_preprocessor::trim_coordinates
    (const std::vector<Coordinates>& coord_left, const std::vector<Coordinates>& coord_right, Coordinates start, Coordinates finish) 
{
    // (1) Convert vector of coordinates to vector of sVector3d
    std::vector<sVector3d> v_coord_left(coord_left.size());
    std::vector<sVector3d> v_coord_right(coord_right.size());

    for (size_t i = 0; i < coord_left.size(); ++i)
        v_coord_left[i] = sVector3d(coord_left[i].longitude, coord_left[i].latitude, 0.0);

    for (size_t i = 0; i < coord_right.size(); ++i)
        v_coord_right[i] = sVector3d(coord_right[i].longitude, coord_right[i].latitude, 0.0);

    sVector3d v_start  = {start.longitude, start.latitude, 0.0};
    sVector3d v_finish = {finish.longitude, finish.latitude, 0.0};

    // (2) Find the closest point to the start/finish point in the two track limits
    auto [v_left_start,d2_left_start,i_left_start] = find_closest_point<scalar>(v_coord_left, v_start, false, 0, 1.0e18);
    auto [v_right_start,d2_right_start,i_right_start] = find_closest_point<scalar>(v_coord_right, v_start, false, 0, 1.0e18);

    auto [v_left_finish,d2_left_finish,i_left_finish] = find_closest_point<scalar>(v_coord_left, v_finish, false, 0, 1.0e18);
    auto [v_right_finish,d2_right_finish,i_right_finish] = find_closest_point<scalar>(v_coord_right, v_finish, false, 0, 1.0e18);

    // (3) Sanity checks

    // Check that the provided starting point is inside the track
    if ( dot(v_start-v_left_start,v_start-v_right_start) > -0.8*norm(v_start-v_left_start)*norm(v_start-v_right_start) )
        throw fastest_lap_exception("The provided starting point does not lie within the track");

    // Check that the provided finish point is inside the track
    if ( dot(v_finish-v_left_finish,v_finish-v_right_finish) > -0.8*norm(v_finish-v_left_finish)*norm(v_finish-v_right_finish) )
        throw fastest_lap_exception("The provided finishing point does not lie within the track");

    // Check that i_start < i_finish
    if ( i_left_finish[0] < i_left_start[0] )
        throw fastest_lap_exception("The provided starting point is ahead of the provided finish point");

    if ( i_right_finish[0] < i_right_start[0] )
        throw fastest_lap_exception("The provided starting point is ahead of the provided finish point");
    
    // (4) Trim the left boundary
    std::vector<Coordinates> coord_left_trim = { {v_left_start.x(), v_left_start.y()} };

    size_t i_start = (i_left_start[0] == i_left_start[1] ? i_left_start[1] + 1 : i_left_start[1]);
    size_t i_finish   = (i_left_finish[0] == i_left_finish[1] ? i_left_finish[0] - 1 : i_left_finish[0] );
    for (size_t i = i_start; i <= i_finish; ++i)
        coord_left_trim.push_back(coord_left[i]);

    coord_left_trim.push_back({v_left_finish.x(), v_left_finish.y()});

    // (4) Trim the right boundary
    std::vector<Coordinates> coord_right_trim = { {v_right_start.x(), v_right_start.y()} };

    i_start = (i_right_start[0] == i_right_start[1] ? i_right_start[1] + 1 : i_right_start[1]);
    i_finish   = (i_right_finish[0] == i_right_finish[1] ? i_right_finish[0] - 1 : i_right_finish[0] );
    for (size_t i = i_start; i <= i_finish; ++i)
        coord_right_trim.push_back(coord_right[i]);

    coord_right_trim.push_back({v_right_finish.x(), v_right_finish.y()});

    return {coord_left_trim, coord_right_trim};
}


template<bool closed, typename computation_type>
void Circuit_preprocessor::FG<closed, computation_type>::operator()(ADvector& fg, const ADvector& x)
{
    assert(x.size() == _n_variables);
    assert(fg.size() == (1 + _n_constraints));

    // (1) Load the state and control vectors
    size_t k = 0;

    // (1a) First variable is the arclength 
    CppAD::AD<scalar> ds_factor = x[k++];

    // (2a) Load state and controls
    for (size_t i = 0; i < _n_points; ++i)
    {
        // Load state 
        for (size_t j = 0; j < state_names::end; ++j)
            _q[i][j] = x[k++];

        // Load control
        for (size_t j = 0; j < control_names::end; ++j)
            _u[i][j] = x[k++];
    }

    // Check that all variables in x were used
    assert(k == FG::_n_variables);

    // (3) Initialize fitness function
    fg[0] = 0.0;

    // (4) Compute the equations for the first node
    _dqds[0] = equations(_q[0],_u[0]);
    std::array<size_t,2> i_left = {0,0}, i_right = {0,0}, i_center = {0,0};

    const Vector3d<CppAD::AD<scalar>> r_left_0   = get_coordinates(_q[0], -_q[0][state_names::nl]);
    const Vector3d<CppAD::AD<scalar>> r_right_0  = get_coordinates(_q[0], _q[0][state_names::nr]);
    const Vector3d<CppAD::AD<scalar>> r_center_0 = get_coordinates(_q[0], 0.0);

    std::tie(std::ignore, _dist2_left[0], i_left)     = find_closest_point<CppAD::AD<scalar>>(_r_left, r_left_0, true, 0, options.maximum_distance_find);
    std::tie(std::ignore, _dist2_right[0], i_right)   = find_closest_point<CppAD::AD<scalar>>(_r_right, r_right_0, true, 0, options.maximum_distance_find);
    std::tie(std::ignore, _dist2_center[0], i_center) = find_closest_point<CppAD::AD<scalar>>(_r_center, r_center_0, true, 0, options.maximum_distance_find);

    // (5) Compute the equations for the i-th node, and append the scheme equations for the i-th element
    k = 1;  // Reset the counter
    for (size_t i = 1; i < _n_points; ++i)
    {
        _dqds[i] = equations(_q[i],_u[i]);
        const Vector3d<CppAD::AD<scalar>> r_left_i   = get_coordinates(_q[i], -_q[i][state_names::nl]);
        const Vector3d<CppAD::AD<scalar>> r_right_i  = get_coordinates(_q[i], _q[i][state_names::nr]);
        const Vector3d<CppAD::AD<scalar>> r_center_i = get_coordinates(_q[i], 0.0);

        std::tie(std::ignore,_dist2_left[i],i_left) = find_closest_point<CppAD::AD<scalar>>(_r_left, r_left_i, true, min(i_left[0],i_left[1]), options.maximum_distance_find);
        std::tie(std::ignore,_dist2_right[i],i_right) = find_closest_point<CppAD::AD<scalar>>(_r_right, r_right_i, true, min(i_right[0],i_right[1]), options.maximum_distance_find);
        std::tie(std::ignore,_dist2_center[i],i_center) = find_closest_point<CppAD::AD<scalar>>(_r_center, r_center_i, true, min(i_center[0],i_center[1]), options.maximum_distance_find);

        // Fitness function: minimize the square of the distance to the boundaries and centerline, and control powers
        const auto ds = ds_factor*_ds[i-1];
        fg[0] += 0.5*ds*options.eps_d*(_dist2_left[i] + _dist2_left[i-1]);
        fg[0] += 0.5*ds*options.eps_d*(_dist2_right[i]  + _dist2_right[i-1]  );
        fg[0] += 0.5*ds*options.eps_c*(_dist2_center[i] + _dist2_center[i-1] );
        fg[0] += 0.5*ds*options.eps_k*(_u[i][control_names::dyaw_dot]*_u[i][control_names::dyaw_dot] + _u[i-1][control_names::dyaw_dot]*_u[i-1][control_names::dyaw_dot]);
        fg[0] += 0.5*ds*options.eps_n*(_u[i][control_names::dnl]*_u[i][control_names::dnl] + _u[i-1][control_names::dnl]*_u[i-1][control_names::dnl]);
        fg[0] += 0.5*ds*options.eps_n*(_u[i][control_names::dnr]*_u[i][control_names::dnr] + _u[i-1][control_names::dnr]*_u[i-1][control_names::dnr]);

        if constexpr (std::is_same_v<computation_type, elevation_computation_names>)
        {
            fg[0] += 0.5*ds*options.eps_pitch*(_u[i][control_names::dpitch_dot]*_u[i][control_names::dpitch_dot] + _u[i-1][control_names::dpitch_dot]*_u[i-1][control_names::dpitch_dot]);
            fg[0] += 0.5*ds*options.eps_roll*(_u[i][control_names::droll_dot]*_u[i][control_names::droll_dot] + _u[i-1][control_names::droll_dot]*_u[i-1][control_names::droll_dot]);
        }

        // Equality constraints:  q^{i} = q^{i-1} + 0.5.ds.[dqds^{i} + dqds^{i-1}]
        for (size_t j = 0; j < state_names::end; ++j)
            fg[k++] = _q[i][j] - _q[i-1][j] - 0.5*ds*(_dqds[i-1][j] + _dqds[i][j]);
    }

    // (6) Append the scheme equations of the periodic element if the circuit is closed
    if constexpr (closed)
    {
        const auto ds = ds_factor*_ds.back();
        // Add the periodic element
        // Fitness function: minimize the square of the distance to the boundaries and centerline, and control powers
        fg[0] += 0.5*ds*options.eps_d*(_dist2_left[0]    + _dist2_left[_n_elements-1] );
        fg[0] += 0.5*ds*options.eps_d*(_dist2_right[0]   + _dist2_right[_n_elements-1]  );
        fg[0] += 0.5*ds*options.eps_c*(_dist2_center[0]  + _dist2_center[_n_elements-1]);
        fg[0] += 0.5*ds*options.eps_k*(_u[0][control_names::dyaw_dot]*_u[0][control_names::dyaw_dot] + _u[_n_elements-1][control_names::dyaw_dot]*_u[_n_elements-1][control_names::dyaw_dot]);
        fg[0] += 0.5*ds*options.eps_n*(_u[0][control_names::dnl]*_u[0][control_names::dnl] + _u[_n_elements-1][control_names::dnl]*_u[_n_elements-1][control_names::dnl]);
        fg[0] += 0.5*ds*options.eps_n*(_u[0][control_names::dnr]*_u[0][control_names::dnr] + _u[_n_elements-1][control_names::dnr]*_u[_n_elements-1][control_names::dnr]);
    
        // Equality constraints:  q^{i} = q^{i-1} + 0.5.ds.[dqds^{i} + dqds^{i-1}]
        // except for yaw, where q^{i} = q^{i-1} + 0.5.ds.[dqds^{i} + dqds^{i-1}] - 2.pi
        for (size_t j = 0; j < state_names::end; ++j)
            fg[k++] = _q[0][j] - _q[_n_elements-1][j] - 0.5*ds*(_dqds[_n_elements-1][j] + _dqds[0][j]) + (j==state_names::yaw ? 2.0*pi*_direction : 0.0);
    }

    // (7) Add a last constraint: the first point should be in the start line
    const sVector3d p = _r_left.front() - _r_right.front();
    const Vector3d<CppAD::AD<scalar>> first_point_coordinates = get_coordinates(_q.front(), 0.0);
    fg[k++] = cross(p,first_point_coordinates - _r_right.front()).z()/dot(p,p);

    if constexpr (!closed)
    {
        // If track is open, the last point should be in the finish line
        const sVector3d p = _r_left.back() - _r_right.back();
        const Vector3d<CppAD::AD<scalar>> last_point_coordinates = get_coordinates(_q.back(), 0.0);
        fg[k++] = cross(p,last_point_coordinates - _r_right.back()).z()/dot(p,p);
    }

    assert(k == FG::_n_constraints+1);
}


template<bool closed>
inline scalar Circuit_preprocessor::compute_ds_for_coordinates(const sVector3d point, const std::vector<sVector3d>& r_curve,
    const std::vector<std::pair<sVector3d,scalar>>& ds_breakpoints)
{
    // (1) Find the closest point to the requested point in the curve
    std::array<size_t,2> i_point; 
    sVector3d v_closest_point;
    std::tie(v_closest_point,std::ignore,i_point) = find_closest_point<scalar>(r_curve, point, closed, 0, 1.0e18);

    // (2) Loop on the ds breakpoints until we find a point ahead of the requested
    size_t i_closest_break = ds_breakpoints.size()-1;
    for (size_t i = 1; i < ds_breakpoints.size(); ++i)
    {
        std::array<size_t,2> i_break; 
        sVector3d v_closest_break;
        std::tie(v_closest_break,std::ignore,i_break) = find_closest_point<scalar>(r_curve, ds_breakpoints[i].first, closed, 0, 1.0e18);

        if ( who_is_ahead(i_break, i_point, v_closest_break, v_closest_point, ds_breakpoints[i].first) == 1 )
        {
            i_closest_break = i - 1;
            break;
        }
    }

    return ds_breakpoints[i_closest_break].second;
}

inline size_t Circuit_preprocessor::who_is_ahead(std::array<size_t,2>& i_p1, std::array<size_t,2>& i_p2, const sVector3d& p1, 
    const sVector3d& p2, const sVector3d& p_ref)
{
    if ((i_p1.front() == i_p2.front()) && (i_p1.back() == i_p2.back()) )
    {
        if ( i_p1.front() == i_p1.back() )
            // Impossible to know with this information, we just arbitrarily say number 1
            return 1;

        else
            // Choose the one farthest to the reference point
            return (norm(p1-p_ref) > norm(p2-p_ref) ? 1 : 2);
    }
    else 
        return ((i_p1.front() < i_p2.front()) ? 2 : 1); 
}

#endif
