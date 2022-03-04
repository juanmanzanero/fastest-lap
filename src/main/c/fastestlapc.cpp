#include "fastestlapc.h"
#include<iostream>
#include<unordered_map>

#include "src/core/vehicles/lot2016kart.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "src/core/applications/steady_state.h"
#include "src/core/applications/optimal_laptime.h"

std::unordered_map<std::string,lot2016kart_all> vehicles_lot2016kart;
std::unordered_map<std::string,limebeer2014f1_all> vehicles_limebeer2014f1;

std::unordered_map<std::string,Track_by_arcs> tracks_by_arcs;
std::unordered_map<std::string,Track_by_polynomial> tracks_by_polynomial;

void create_vehicle(struct c_Vehicle* vehicle, const char* name, const char* database_file)
{
    const std::string s_database = database_file;

    // Copy the vehicle name
    vehicle->name = new char[strlen(name)+1];
    memcpy(vehicle->name, name, strlen(name));
    vehicle->name[strlen(name)] = '\0';
 
    // Copy the database file path
    vehicle->database_file = new char[strlen(database_file)+1];
    memcpy(vehicle->database_file, database_file, strlen(database_file));
    vehicle->database_file[strlen(database_file)] = '\0';

    // Open the database as Xml
    Xml_document database = { database_file, true }; 

    // Get vehicle type from the database file
    const std::string vehicle_type = database.get_root_element().get_attribute("type");

    if ( vehicle_type == "roberto-lot-kart-2016" )
    {
        vehicle->type = LOT2016KART;
        auto out = vehicles_lot2016kart.insert({name,{database}});
        if (out.second==false) 
        {
            throw std::runtime_error("Vehicle already exists");
        }
    }
    else if ( vehicle_type == "limebeer-2014-f1" )
    {
        vehicle->type = LIMEBEER2014F1;
        auto out = vehicles_limebeer2014f1.insert({name,{database}});
        if (out.second==false) 
        {
            throw std::runtime_error("Vehicle already exists");
        }
    }
    else
    {
        throw std::runtime_error("Vehicle type not recognized");
    }
}

void create_track(struct c_Track* track, const char* name, const char* track_file, const double scale)
{
    const std::string s_track_file = track_file;

    // Copy the track name
    track->name = new char[strlen(name)+1];
    memcpy(track->name, name, strlen(name));
    track->name[strlen(name)] = '\0';
 
    // Copy the track file path
    track->track_file = new char[strlen(track_file)+1];
    memcpy(track->track_file, track_file, strlen(track_file));
    track->track_file[strlen(track_file)] = '\0';

    // Copy the track scale
    track->scale = scale;

    // Open the track as Xml
    Xml_document track_xml = { track_file, true }; 

    // Read type: open or closed
    const std::string track_type = track_xml.get_root_element().get_attribute("type");
    bool is_closed;

    if ( track_type == "closed" )
        is_closed = true;

    else if ( track_type == "open" )
        is_closed = false;

    else
        throw std::runtime_error("Track attribute type \"" + track_type + "\" shall be \"open\" or \"closed\"");

    // Read format: by-arcs or by-polynomial
    const std::string track_format = track_xml.get_root_element().get_attribute("format");

    track->is_closed = is_closed;
    
    if ( track_format == "by-arcs" )
    {
        track->format = BY_ARCS;

        auto out = tracks_by_arcs.insert({name,{track_xml,scale,is_closed}});

        if (out.second==false) 
        {
            throw std::runtime_error("Track already exists");
        }
    }
    else if ( track_format == "by-polynomial" || track_format == "discrete")
    {
        track->format = BY_POLYNOMIAL;

        auto out = tracks_by_polynomial.insert({name,{track_xml}});
        
        if (out.second==false) 
        {
            throw std::runtime_error("Track already exists");
        }
    }
}

void set_parameter(struct c_Vehicle* c_vehicle, const char* parameter, const double value)
{
    if ( c_vehicle->type == LIMEBEER2014F1 )
    {
        vehicles_limebeer2014f1.at(c_vehicle->name).set_parameter(parameter, value);
    }
}


void vehicle_equations(double* dqdt, double* dqa, double** jac_dqdt, double** jac_dqa, double*** h_dqdt, double*** h_dqa, struct c_Vehicle* vehicle, double* q, double* qa, double* u, double s)
{


}



template<typename vehicle_t>
void compute_gg_diagram(vehicle_t& car, double* ay, double* ax_max, double* ax_min, double v, const int n_points)
{
    Steady_state ss(car);
    auto [sol_max, sol_min] = ss.gg_diagram(v,n_points);

    for (int i = 0; i < n_points; ++i)
    {
        ay[i] = sol_max[i].ay;
        ax_max[i] = sol_max[i].ax;
        ax_min[i] = sol_min[i].ax;
    }
}

void gg_diagram(double* ay, double* ax_max, double* ax_min, struct c_Vehicle* c_vehicle, double v, const int n_points)
{
    if ( c_vehicle->type == LOT2016KART )
        compute_gg_diagram(vehicles_lot2016kart.at(c_vehicle->name).cartesian_ad, ay, ax_max, ax_min, v, n_points);

    else if ( c_vehicle->type == LIMEBEER2014F1 )
        compute_gg_diagram(vehicles_limebeer2014f1.at(c_vehicle->name).cartesian_ad, ay, ax_max, ax_min, v, n_points);
}


void track_coordinates(double* x_center, double* y_center, double* x_left, double* y_left, double* x_right, double* y_right, double* theta, struct c_Track* c_track, const int n_points)
{
    if ( c_track->format == BY_ARCS )
    {
        auto& track = tracks_by_arcs.at(c_track->name);
    
        const scalar& L = track.get_total_length();
        const scalar ds = L/((scalar)(n_points-1));

        
        for (int i = 0; i < n_points; ++i)
        {
            const scalar s = ((double)i)*ds;
    
            // Compute centerline
            auto [r_c,v_c,a_c] = track(s);
    
            x_center[i] = r_c[0];
            y_center[i] = r_c[1];
    
            // Heading angle (theta)
            theta[i] = atan2(v_c[1],v_c[0]);
    
            // Compute left boundary
            auto [r_l,v_l,a_l] = track.position_at(s,track.get_right_track_limit(s),0.0,0.0);
    
            x_left[i] = r_l[0];
            y_left[i] = r_l[1];
    
            // Compute right boundary
            auto [r_r,v_r,a_r] = track.position_at(s,-track.get_left_track_limit(s),0.0,0.0);
    
            x_right[i] = r_r[0];
            y_right[i] = r_r[1];
    
            
        }
    }
    else if ( c_track->format == BY_POLYNOMIAL )
    {
        std::string name = c_track->name;
        auto& track = tracks_by_polynomial.at(name);
    
        const scalar& L = track.get_total_length();
        const scalar ds = L/((scalar)(n_points-1));
        
        PRINTVARIABLE(JMT,L);
        for (int i = 0; i < n_points; ++i)
        {
            const scalar s = ((double)i)*ds;
    
            // Compute centerline
            auto [r_c,v_c,a_c] = track(s);
    
            x_center[i] = r_c[0];
            y_center[i] = r_c[1];
    
            // Heading angle (theta)
            theta[i] = atan2(v_c[1],v_c[0]);
    
            // Compute left boundary
            auto [r_l] = track.position_at(s,track.get_right_track_limit(s));
    
            x_left[i] = r_l[0];
            y_left[i] = r_l[1];
    
            // Compute right boundary
            auto [r_r] = track.position_at(s,-track.get_left_track_limit(s));
    
            x_right[i] = r_r[0];
            y_right[i] = r_r[1];
        }
    }
    
    return;
}


template<typename vehicle_t, typename track_t>
void compute_optimal_laptime(vehicle_t& vehicle, track_t& track, double** channels_data, struct c_Vehicle* c_vehicle, const int n_points, const int n_channels, const char** channels_name)
{
    auto& car_curv = vehicle.template get_curvilinear_ad_car<track_t>();
    auto& car_curv_sc = vehicle.template get_curvilinear_scalar_car<track_t>();

    auto& car_cart = vehicle.cartesian_ad;
    auto& car_cart_sc = vehicle.cartesian_scalar;

    // Set the track into the curvilinear car dynamic model
    car_curv.get_road().change_track(track);
    car_curv_sc.get_road().change_track(track);

    // Start from the steady-state values at 50km/h-0g    
    scalar v = 50.0*KMH;

    auto ss = Steady_state(car_cart).solve(v,0.0,0.0); 

    if ( c_vehicle->type == LOT2016KART )
        ss.u[1] = 0.0;

    bool is_direct = false;
    std::array<scalar,2> dissipations = {1.0e-2, 200*200*1.0e-10};

    if ( c_vehicle->type == LIMEBEER2014F1 )
    {
        is_direct = true;
        dissipations[0] = 10.0;
        dissipations[1] = 1.0e-3;
    }
    
    std::tie(ss.dqdt, std::ignore) = car_cart_sc(ss.q, ss.qa, ss.u, 0.0);

    Optimal_laptime opt_laptime(n_points, true, is_direct, car_curv, ss.q, ss.qa, ss.u, dissipations, {});

    // Set outputs
    for (int i = 0; i < n_points; ++i)
    {
        const scalar& L = car_curv.get_road().track_length();
        car_curv_sc(opt_laptime.q[i], opt_laptime.qa[i], opt_laptime.u[i], ((double)i)*L/((double)n_points));

        for (int j = 0; j < n_channels; ++j)
        {
            if ( std::string(channels_name[j]) == "x" ) 
                channels_data[j][i] = car_curv_sc.get_road().get_x();
    
            else if ( std::string(channels_name[j]) == "y" )
                channels_data[j][i] = car_curv_sc.get_road().get_y();

            else if ( std::string(channels_name[j]) == "s" )
                channels_data[j][i] = ((double)i)*L/((double)n_points);

            else if ( std::string(channels_name[j]) == "u" )
                channels_data[j][i] = opt_laptime.q[i][vehicle_t::vehicle_scalar_curvilinear_a::Chassis_type::IU];

            else if ( std::string(channels_name[j]) == "v" )
                channels_data[j][i] = opt_laptime.q[i][vehicle_t::vehicle_scalar_curvilinear_a::Chassis_type::IV];

            else if ( std::string(channels_name[j]) == "time" )
                channels_data[j][i] = opt_laptime.q[i][vehicle_t::vehicle_scalar_curvilinear_a::Road_type::ITIME];

            else if ( std::string(channels_name[j]) == "delta" )
                channels_data[j][i] = opt_laptime.u[i][vehicle_t::vehicle_scalar_curvilinear_a::Chassis_type::Front_axle_type::ISTEERING];

            else if ( std::string(channels_name[j]) == "psi" )
                channels_data[j][i] = car_curv_sc.get_road().get_psi();

            else if ( std::string(channels_name[j]) == "omega" )
                channels_data[j][i] = opt_laptime.q[i][vehicle_t::vehicle_scalar_curvilinear_a::Chassis_type::IOMEGA];

            else if ( std::string(channels_name[j]) == "throttle" )
            {
                if constexpr (std::is_same<vehicle_t, lot2016kart_all>::value)
                {
                    channels_data[j][i] = opt_laptime.u[i][vehicle_t::vehicle_scalar_curvilinear_a::Chassis_type::Rear_axle_type::ITORQUE];
                }

                else if constexpr (std::is_same<vehicle_t, limebeer2014f1_all>::value)
                {
                    channels_data[j][i] = opt_laptime.u[i][vehicle_t::vehicle_scalar_curvilinear_a::Chassis_type::ITHROTTLE];
                }
            }
            else if ( std::string(channels_name[j]) == "rear_axle/left_tire/x" )
                channels_data[j][i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<0>().get_position().at(0);

            else if ( std::string(channels_name[j]) == "rear_axle/left_tire/y" )
                channels_data[j][i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<0>().get_position().at(1);

            else if ( std::string(channels_name[j]) == "rear_axle/right_tire/x" )
                channels_data[j][i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<1>().get_position().at(0);

            else if ( std::string(channels_name[j]) == "rear_axle/right_tire/y" )
                channels_data[j][i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<1>().get_position().at(1);

            else if ( std::string(channels_name[j]) == "front_axle/left_tire/x" )
                channels_data[j][i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<0>().get_position().at(0);

            else if ( std::string(channels_name[j]) == "front_axle/left_tire/y" )
                channels_data[j][i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<0>().get_position().at(1);

            else if ( std::string(channels_name[j]) == "front_axle/right_tire/x" )
                channels_data[j][i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<1>().get_position().at(0);

            else if ( std::string(channels_name[j]) == "front_axle/right_tire/y" )
                channels_data[j][i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<1>().get_position().at(1);
        }
    }
}


void optimal_laptime(double** channels_data, struct c_Vehicle* c_vehicle, const struct c_Track* c_track, const int n_points, const int n_channels, const char** channels_name) 
{
    if ( c_vehicle->type == LOT2016KART )
    {
        if ( c_track->format == BY_ARCS )
        {
            compute_optimal_laptime(vehicles_lot2016kart.at(c_vehicle->name), tracks_by_arcs.at(c_track->name), 
                                    channels_data, c_vehicle, n_points, n_channels, channels_name);
        }
        else if ( c_track->format == BY_POLYNOMIAL )
        {
            compute_optimal_laptime(vehicles_lot2016kart.at(c_vehicle->name), tracks_by_polynomial.at(c_track->name), 
                                    channels_data, c_vehicle, n_points, n_channels, channels_name);
        }
    }
    else if ( c_vehicle->type == LIMEBEER2014F1 )
    {
        if ( c_track->format == BY_ARCS )
        {
            compute_optimal_laptime(vehicles_limebeer2014f1.at(c_vehicle->name), tracks_by_arcs.at(c_track->name), 
                                    channels_data, c_vehicle, n_points, n_channels, channels_name);
        }
        else if ( c_track->format == BY_POLYNOMIAL )
        {
            compute_optimal_laptime(vehicles_limebeer2014f1.at(c_vehicle->name), tracks_by_polynomial.at(c_track->name), 
                                    channels_data, c_vehicle, n_points, n_channels, channels_name);
        }
    }
}
