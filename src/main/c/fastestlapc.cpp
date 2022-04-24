#include "fastestlapc.h"
#include <iostream>
#include <unordered_map>
#include <algorithm>

#include "src/core/vehicles/lot2016kart.h"
#include "src/core/vehicles/limebeer2014f1.h"
#include "src/core/applications/steady_state.h"
#include "src/core/applications/optimal_laptime.h"
#include "lion/propagators/crank_nicolson.h"

// Persistent vehicles
std::unordered_map<std::string,lot2016kart_all> vehicles_lot2016kart;
std::unordered_map<std::string,limebeer2014f1_all> vehicles_limebeer2014f1;

// Persistent tracks
std::unordered_map<std::string,Track_by_polynomial> table_track;

// Persistent scalars
std::unordered_map<std::string,scalar> table_scalar;

// Persistent vectors
std::unordered_map<std::string,std::vector<scalar>> table_vector;

// Persistent warm start 
template<typename vehicle_t>
Optimal_laptime<typename vehicle_t::vehicle_ad_curvilinear>& get_warm_start()
{
    static Optimal_laptime<typename limebeer2014f1_all::vehicle_ad_curvilinear> warm_start_limebeer2014f1;
    static Optimal_laptime<typename lot2016kart_all::vehicle_ad_curvilinear> warm_start_lot2016kart;

    if constexpr (std::is_same_v<vehicle_t,limebeer2014f1_all>)
        return warm_start_limebeer2014f1;
    else if constexpr (std::is_same_v<vehicle_t,lot2016kart_all>)
        return warm_start_lot2016kart;
    else
        throw std::runtime_error("[ERROR] get_warm_start() -> vehicle_t is not supported");
}


void create_vehicle(struct c_Vehicle* vehicle, const char* name, const char* vehicle_type, const char* database_file)
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

    if ( std::string(vehicle_type) == "roberto-lot-kart-2016" )
    {
        // Check if the vehicle exists
        if ( vehicles_lot2016kart.count(name) != 0 )
            throw std::runtime_error(std::string("Vehicle of type roberto-lot-kart-2016 with name \"") + name + "\" already exists"); 

        vehicle->type = LOT2016KART;

        // Open the database as Xml
        Xml_document database = { database_file, true }; 

        // Get vehicle type from the database file
        const std::string vehicle_type_db = database.get_root_element().get_attribute("type");

        if ( vehicle_type_db != std::string(vehicle_type) )
            throw std::runtime_error("vehicle type read from the database is not \"roberto-lot-kart-2016\"");

        auto out = vehicles_lot2016kart.insert({name,{database}});
        if (out.second==false) 
        {
            throw std::runtime_error("The insertion to the map failed");
        }
    }
    else if ( std::string(vehicle_type) == "limebeer-2014-f1" )
    {
        // Check if the vehicle exists
        if ( vehicles_limebeer2014f1.count(name) != 0 )
            throw std::runtime_error(std::string("Vehicle of type limebeer-2014-f1 with name \"") + name + "\" already exists"); 

        vehicle->type = LIMEBEER2014F1;
        
        if ( strlen(database_file) > 0 )
        {
            // Open the database as Xml
            Xml_document database = { database_file, true }; 

            // Get vehicle type from the database file
            const std::string vehicle_type_db = database.get_root_element().get_attribute("type");

            if ( vehicle_type_db != std::string(vehicle_type) )
                throw std::runtime_error("vehicle type read from the database is not \"limebeer-2014-f1\"");

            auto out = vehicles_limebeer2014f1.insert({name,{database}});

            if (out.second==false) 
            {
                throw std::runtime_error("Vehicle already exists");
            }        
        }
        else
        {
            // Construct a default car
            auto out = vehicles_limebeer2014f1.insert({name,{}});

            if (out.second==false) 
            {
                throw std::runtime_error("The insertion to the map failed");
            }
        }

    }
    else
    {
        throw std::runtime_error("Vehicle type not recognized");
    }
}

void create_track(struct c_Track* track, const char* name, const char* track_file, const char* options)
{
    // (1) Check that the track does not exists in the map
    if ( table_track.count(name) != 0 )
        throw std::runtime_error(std::string("Track with name \"") + name + "\" already exists"); 

    // (2) Process options
    std::string save_variables_prefix;
    std::vector<std::string> variables_to_save;
    if ( strlen(options) > 0 )
    {
        // Parse the options in XML format
        // Example:
        //      <options>
        //          <save_variables>
        //              <prefix>
        //              <variables>
        //                  <s/>
        //                  <theta/>
        //                  ...
        //              </variables>
        //          </save_variables>
        //      </options>
        //
        std::string s_options(options);
        Xml_document doc;
        doc.parse(s_options);

        // Save variables
        if ( doc.has_element("options/save_variables") )
        {
            save_variables_prefix = doc.get_element("options/save_variables/prefix").get_value();
    
            for (auto& variables : doc.get_element("options/save_variables/variables").get_children() )
                variables_to_save.push_back(variables.get_name());
        }
    }

    // (3) Open the track
    // Copy the track name
    const std::string s_track_file = track_file;
    track->name = new char[strlen(name)+1];
    memcpy(track->name, name, strlen(name));
    track->name[strlen(name)] = '\0';
 
    // Copy the track file path
    track->track_file = new char[strlen(track_file)+1];
    memcpy(track->track_file, track_file, strlen(track_file));
    track->track_file[strlen(track_file)] = '\0';

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

    // Read format: only discrete tracks are supported by the C API
    const std::string track_format = track_xml.get_root_element().get_attribute("format");

    track->is_closed = is_closed;
    
    if ( track_format != "discrete")
        throw std::runtime_error(std::string("Track format \"") + track_format + "\" is not supported");

    table_track.insert({name,{track_xml}});

    // (4) Save variables

    // Get alias to the track preprocessor
    const auto& preprocessor = table_track[name].get_preprocessor();

    for (const auto& variable_name : variables_to_save )
    {
        // Check that the variable does not exist in any of the tables
        if ( table_scalar.count(save_variables_prefix + variable_name) != 0 )
            throw std::runtime_error(std::string("Variable \"") + save_variables_prefix + variable_name + "\" already exists in the scalar table");

        if ( table_vector.count(save_variables_prefix + variable_name) != 0 )
            throw std::runtime_error(std::string("Variable \"") + save_variables_prefix + variable_name + "\" already exists in the vector table");


        if ( variable_name == "s" )
            table_vector.insert({save_variables_prefix+variable_name, preprocessor.s});
        else
            throw std::runtime_error(std::string("Variable \"") + variable_name + "\" is not implemented");
    }
}

template<typename Vehicle_t>
double get_vehicle_property_generic(Vehicle_t& vehicle, const double* c_q, const double* c_qa, const double* c_u, const double s, const char* c_property_name)
{
    // (1) Construct Cpp version of the C inputs
    std::array<scalar,Vehicle_t::NSTATE> q;
    std::array<scalar,Vehicle_t::NALGEBRAIC> qa;
    std::array<scalar,Vehicle_t::NCONTROL> u;

    std::copy_n(c_q, Vehicle_t::NSTATE, q.begin());
    std::copy_n(c_qa, Vehicle_t::NALGEBRAIC, qa.begin());
    std::copy_n(c_u, Vehicle_t::NCONTROL, u.begin());

    std::string property_name(c_property_name);

    vehicle(q, qa, u, s);

    if ( property_name == "x" ) 
        return vehicle.get_road().get_x();

    else if ( property_name == "y" )
        return vehicle.get_road().get_y();

    else if ( property_name == "s" )
        return s;

    else if ( property_name == "n" )
        return q[Vehicle_t::Road_type::IN];

    else if ( property_name == "alpha" )
        return q[Vehicle_t::Road_type::IALPHA];

    else if ( property_name == "u" )
        return q[Vehicle_t::Chassis_type::IU];

    else if ( property_name == "v" )
        return q[Vehicle_t::Chassis_type::IV];

    else if ( property_name == "time" )
        return q[Vehicle_t::Road_type::ITIME];

    else if ( property_name == "delta" )
        return u[Vehicle_t::Chassis_type::Front_axle_type::ISTEERING];

    else if ( property_name == "psi" )
        return vehicle.get_road().get_psi();

    else if ( property_name == "omega" )
        return q[Vehicle_t::Chassis_type::IOMEGA];

    else if ( property_name == "throttle" )
    {
        if constexpr (std::is_same<Vehicle_t, lot2016kart_all>::value)
        {
            return u[Vehicle_t::Chassis_type::Rear_axle_type::ITORQUE];
        }

        else if constexpr (std::is_same<Vehicle_t, limebeer2014f1_all>::value)
        {
            return u[Vehicle_t::Chassis_type::ITHROTTLE];
        }
        else
        {
            throw std::runtime_error("[ERROR] Vehicle type is not defined");
        }
    }

    else if ( property_name == "brake-bias" )
    {
        if constexpr (std::is_same<Vehicle_t, lot2016kart_all>::value)
        {
            throw std::runtime_error("[ERROR] brake-bias is not available for vehicles of type lot2016kart");
        }

        else if constexpr (std::is_same<Vehicle_t, limebeer2014f1_all>::value)
        {
            return u[Vehicle_t::Chassis_type::IBRAKE_BIAS];
        }
        else
        {
            throw std::runtime_error("[ERROR] Vehicle type is not defined");
        }
    }
    else if ( property_name == "rear_axle.left_tire.x" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<0>().get_position().at(0);

    else if ( property_name == "rear_axle.left_tire.y" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<0>().get_position().at(1);

    else if ( property_name == "rear_axle.right_tire.x" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<1>().get_position().at(0);

    else if ( property_name == "rear_axle.right_tire.y" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<1>().get_position().at(1);

    else if ( property_name == "front_axle.left_tire.x" )
        return vehicle.get_chassis().get_front_axle().template get_tire<0>().get_position().at(0);

    else if ( property_name == "front_axle.left_tire.y" )
        return vehicle.get_chassis().get_front_axle().template get_tire<0>().get_position().at(1);

    else if ( property_name == "front_axle.right_tire.x" )
        return vehicle.get_chassis().get_front_axle().template get_tire<1>().get_position().at(0);

    else if ( property_name == "front_axle.right_tire.y" )
        return vehicle.get_chassis().get_front_axle().template get_tire<1>().get_position().at(1);

    else if ( property_name == "front_axle.left_tire.kappa" )
        return vehicle.get_chassis().get_front_axle().template get_tire<0>().get_kappa();

    else if ( property_name == "front_axle.right_tire.kappa" )
        return vehicle.get_chassis().get_front_axle().template get_tire<1>().get_kappa();

    else if ( property_name == "rear_axle.left_tire.kappa" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();

    else if ( property_name == "rear_axle.right_tire.kappa" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();

    else if ( property_name == "front_axle.left_tire.lambda" )
        return vehicle.get_chassis().get_front_axle().template get_tire<0>().get_lambda();

    else if ( property_name == "front_axle.right_tire.lambda" )
        return vehicle.get_chassis().get_front_axle().template get_tire<1>().get_lambda();

    else if ( property_name == "rear_axle.left_tire.lambda" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<0>().get_lambda();

    else if ( property_name == "rear_axle.right_tire.lambda" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<1>().get_lambda();

    else if ( property_name == "front_axle.left_tire.Fx" )
        return vehicle.get_chassis().get_front_axle().template get_tire<0>().get_force().x();

    else if ( property_name == "front_axle.right_tire.Fx" )
        return vehicle.get_chassis().get_front_axle().template get_tire<1>().get_force().x();

    else if ( property_name == "rear_axle.left_tire.Fx" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<0>().get_force().x();

    else if ( property_name == "rear_axle.right_tire.Fx" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<1>().get_force().x();

    else if ( property_name == "front_axle.left_tire.Fy" )
        return vehicle.get_chassis().get_front_axle().template get_tire<0>().get_force().y();

    else if ( property_name == "front_axle.right_tire.Fy" )
        return vehicle.get_chassis().get_front_axle().template get_tire<1>().get_force().y();

    else if ( property_name == "rear_axle.left_tire.Fy" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<0>().get_force().y();

    else if ( property_name == "rear_axle.right_tire.Fy" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<1>().get_force().y();

    else if ( property_name == "front_axle.left_tire.dissipation" )
        return vehicle.get_chassis().get_front_axle().template get_tire<0>().get_dissipation();

    else if ( property_name == "front_axle.right_tire.dissipation" )
        return vehicle.get_chassis().get_front_axle().template get_tire<1>().get_dissipation();

    else if ( property_name == "rear_axle.left_tire.dissipation" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<0>().get_dissipation();

    else if ( property_name == "rear_axle.right_tire.dissipation" )
        return vehicle.get_chassis().get_rear_axle().template get_tire<1>().get_dissipation();

    else if ( property_name == "Fz_fl" )
    {
        if constexpr (std::is_same<Vehicle_t, limebeer2014f1_all>::value)
        {
            return qa[Vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IFZFL];
        }
        else 
        {
            throw std::runtime_error("Fz_fl is only defined for limebeer2014f1 models");
        }
    }

    else if ( property_name == "Fz_fr" )
    {
        if constexpr (std::is_same<Vehicle_t, limebeer2014f1_all>::value)
        {
            return qa[Vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IFZFR];
        }
        else 
        {
            throw std::runtime_error("Fz_fr is only defined for limebeer2014f1 models");
        }
    }

    else if ( property_name == "Fz_rl" )
    {
        if constexpr (std::is_same<Vehicle_t, limebeer2014f1_all>::value)
        {
            return qa[Vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IFZRL];
        }
        else 
        {
            throw std::runtime_error("Fz_rl is only defined for limebeer2014f1 models");
        }
    }

    else if ( property_name == "Fz_rr" )
    {
        if constexpr (std::is_same<Vehicle_t, limebeer2014f1_all>::value)
        {
            return qa[Vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IFZRR];
        }
        else 
        {
            throw std::runtime_error("Fz_rr is only defined for limebeer2014f1 models");
        }
    }

    else if ( property_name == "ax" )
    {
        sVector3d velocity = {vehicle.get_chassis().get_u(), vehicle.get_chassis().get_v(), 0.0};

        sVector3d acceleration = {vehicle.get_chassis().get_du() - velocity.y()*vehicle.get_chassis().get_omega(), 
                                  vehicle.get_chassis().get_dv() + velocity.x()*vehicle.get_chassis().get_omega(),
                                  0.0
                                 };

        return dot(velocity,acceleration)/norm(velocity);
    }

    else if ( property_name == "ay" )
    {
        sVector3d velocity = {vehicle.get_chassis().get_u(), vehicle.get_chassis().get_v(), 0.0};

        sVector3d acceleration = {vehicle.get_chassis().get_du() - velocity.y()*vehicle.get_chassis().get_omega(), 
                                  vehicle.get_chassis().get_dv() + velocity.x()*vehicle.get_chassis().get_omega(),
                                  0.0
                                 };

        return cross(velocity,acceleration).z()/norm(velocity);
    }

    else
    {
        throw std::runtime_error("Variable \"" + property_name + "\" is not defined");
    }
}


double get_vehicle_property(struct c_Vehicle* c_vehicle, const double* q, const double* qa, const double* u, const double s, const char* property_name)
{
    if ( c_vehicle->type == LOT2016KART )
    {
        return get_vehicle_property_generic(vehicles_lot2016kart.at(c_vehicle->name).curvilinear_scalar, q, qa, u, s, property_name);
    }
    else if ( c_vehicle->type == LIMEBEER2014F1 )
    {
        return get_vehicle_property_generic(vehicles_limebeer2014f1.at(c_vehicle->name).curvilinear_scalar, q, qa, u, s, property_name);
    }
    else
    {
        throw std::runtime_error("[ERROR] libfastestlapc::get_vehicle_property -> vehicle type is not defined");
    }
}


void save_vehicle_as_xml(struct c_Vehicle* c_vehicle, const char* file_name)
{
    if ( c_vehicle->type == LOT2016KART )
    {
        vehicles_lot2016kart.at(c_vehicle->name).curvilinear_scalar.xml()->save(std::string(file_name));
    }
    else if ( c_vehicle->type == LIMEBEER2014F1 )
    {
        vehicles_limebeer2014f1.at(c_vehicle->name).curvilinear_scalar.xml()->save(std::string(file_name));
    }
    else
    {
        throw std::runtime_error("[ERROR] libfastestlapc::get_vehicle_property -> vehicle type is not defined");
    }

}


int download_vector_table_variable_size(const char* name_c)
{
    std::string name(name_c);

    // Look for the item in the table
    const auto& item = table_vector.find(name);

    // Check that it was found
    if ( item == table_vector.end() )
        throw std::runtime_error(std::string("Variable \"") + name + "\" does not exists in the vector table");

    const auto& table_data = item->second;
    
    return table_data.size();
}


double download_scalar_table_variable(const char* name_c)
{
    std::string name(name_c);

    // Look for the item in the table
    const auto& item = table_scalar.find(name);

    // Check that it was found
    if ( item == table_scalar.end() )
        throw std::runtime_error(std::string("Variable \"") + name + "\" does not exists in the scalar table");

    // Check input consistency
    return item->second;
}


void download_vector_table_variable(double* data, const int n, const char* name_c)
{
    std::string name(name_c);

    // Look for the item in the table
    const auto& item = table_vector.find(name);

    // Check that it was found
    if ( item == table_vector.end() )
        throw std::runtime_error(std::string("Variable \"") + name + "\" does not exists in the vector table");

    // Check input consistency
    const auto& table_data = item->second;

    if ( table_data.size() != static_cast<size_t>(n) )
        throw std::runtime_error(std::string("Incorrect input size for variable \"") + name + "\". Input: " 
            + std::to_string(n) + ", should be " + std::to_string(table_data.size()));

    // Copy the data into the provided pointer
    std::copy(table_data.cbegin(), table_data.cend(), data);

    return;
}


void load_vector_table_variable(double* data, const int n, const char* name_c)
{
    std::string name(name_c);

    // Check that the variable does not exist
    if ( table_vector.count(name) != 0 )
        throw std::runtime_error(std::string("Variable \"") + name + "\" already exists in the vector table");

    table_vector.insert({name,{data,data+n}});
}


void clear_tables()
{
    table_scalar.clear();
    table_vector.clear();
}


void clear_tables_by_prefix(const char* prefix_c)
{
    std::string prefix(prefix_c);

    // Scalar map
    for (auto it = table_scalar.cbegin(); it != table_scalar.cend(); )
    {
        if ( it->first.find(prefix) == 0 ) 
        {
            it = table_scalar.erase(it); 
        }
        else
        {
            ++it;
        }
    }


    // Vector map
    for (auto it = table_vector.cbegin(); it != table_vector.cend(); )
    {
        if ( it->first.find(prefix) == 0 ) 
        {
            it = table_vector.erase(it); 
        }
        else
        {
            ++it;
        }
    }
}


void delete_vehicle(struct c_Vehicle* c_vehicle)
{
    if ( c_vehicle->type == LIMEBEER2014F1 )
        vehicles_limebeer2014f1.erase(c_vehicle->name);

    else if ( c_vehicle->type == LOT2016KART )
        vehicles_lot2016kart.erase(c_vehicle->name);
    
    else
        throw std::runtime_error("Vehicle type is not recognized");
}

void set_scalar_parameter(struct c_Vehicle* c_vehicle, const char* parameter, const double value)
{
    if ( c_vehicle->type == LIMEBEER2014F1 )
    {
        vehicles_limebeer2014f1.at(c_vehicle->name).set_parameter(parameter, value);
    }
}


void set_vector_parameter(struct c_Vehicle* c_vehicle, const char* parameter, const double value[3])
{
    if ( c_vehicle->type == LIMEBEER2014F1 )
    {
        sVector3d v_value = { value[0], value[1], value[2] };
        vehicles_limebeer2014f1.at(c_vehicle->name).set_parameter(parameter, v_value);
    }
}


void set_matrix_parameter(struct c_Vehicle* c_vehicle, const char* parameter, const double value[9])
{
    if ( c_vehicle->type == LIMEBEER2014F1 )
    {
        sMatrix3x3 m_value = { value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7], value[8] };
        vehicles_limebeer2014f1.at(c_vehicle->name).set_parameter(parameter, m_value);
    }
}


void add_variable_parameter(struct c_Vehicle* c_vehicle, const char* parameter_name, const int n, const double* s, const double* values)
{
    std::vector<scalar> v_s(s,s+n);
    std::vector<scalar> v_values(values,values+n);

    sPolynomial p(v_s,v_values,1,true);
    
    if ( c_vehicle->type == LIMEBEER2014F1 )
        vehicles_limebeer2014f1.at(c_vehicle->name).add_variable_parameter(std::string(parameter_name), p);

    else if ( c_vehicle->type == LOT2016KART )
        vehicles_lot2016kart.at(c_vehicle->name).add_variable_parameter(std::string(parameter_name), p);

    else
        throw std::runtime_error("Vehicle type not recognized");
}


void vehicle_equations(double* dqdt, double* dqa, double** jac_dqdt, double** jac_dqa, double*** h_dqdt, double*** h_dqa, struct c_Vehicle* vehicle, double* q, double* qa, double* u, double s)
{


}


template<typename Vehicle_t>
void compute_propagation(Vehicle_t car, double* c_q, double* c_qa, double* c_u, double s, double ds, double* c_u_next, const char* c_options)
{
    // (1) Construct Cpp version of the C inputs
    std::array<scalar,Vehicle_t::NSTATE> q;
    std::array<scalar,Vehicle_t::NALGEBRAIC> qa;
    std::array<scalar,Vehicle_t::NCONTROL> u;
    std::array<scalar,Vehicle_t::NCONTROL> u_next;

    std::copy_n(c_q, Vehicle_t::NSTATE, q.begin());
    std::copy_n(c_qa, Vehicle_t::NALGEBRAIC, qa.begin());
    std::copy_n(c_u, Vehicle_t::NCONTROL, u.begin());
    std::copy_n(c_u_next, Vehicle_t::NCONTROL, u_next.begin());

    // (2) Parse options
    typename Crank_nicolson<Vehicle_t,Vehicle_t::NSTATE,Vehicle_t::NALGEBRAIC,Vehicle_t::NCONTROL>::Options opts;
    if ( strlen(c_options) > 0 )
    {
        std::string options = c_options;
        Xml_document doc;
        doc.parse(options);
    
        if ( doc.has_element("options/sigma") )             opts.sigma = doc.get_element("options/sigma").get_value(scalar());
        if ( doc.has_element("options/max_iter") )          opts.max_iter = doc.get_element("options/max_iter").get_value(scalar());
        if ( doc.has_element("options/error_tolerance") )   opts.error_tolerance = doc.get_element("options/error_tolerance").get_value(scalar());
        if ( doc.has_element("options/relaxation_factor") ) opts.relaxation_factor = doc.get_element("options/relaxation_factor").get_value(scalar());
    }

    // (3) Take step
    Crank_nicolson<Vehicle_t,Vehicle_t::NSTATE,Vehicle_t::NALGEBRAIC,Vehicle_t::NCONTROL>::take_step(car, u, u_next, q, qa, s, ds, opts);

    // (4) Return the variables to the c version
    std::copy_n(q.begin(), Vehicle_t::NSTATE, c_q);
    std::copy_n(qa.begin(), Vehicle_t::NALGEBRAIC, c_qa);
}

void propagate(double* q, double* qa, double* u, struct c_Vehicle* c_vehicle, struct c_Track* c_track, double s, double ds, double* u_next, bool use_circuit, const char* options)
{
    if ( c_vehicle->type == LOT2016KART )
    {
        if ( use_circuit )
        {
            vehicles_lot2016kart.at(c_vehicle->name).curvilinear_ad.get_road().change_track(table_track.at(c_track->name));
            vehicles_lot2016kart.at(c_vehicle->name).curvilinear_scalar.get_road().change_track(table_track.at(c_track->name));
            compute_propagation(vehicles_lot2016kart.at(c_vehicle->name).curvilinear_ad, q, qa, u, s, ds, u_next, options);
        }
        else
        {
            compute_propagation(vehicles_lot2016kart.at(c_vehicle->name).cartesian_ad, q, qa, u, s, ds, u_next, options);
        }
    }
    else
    {
        if ( use_circuit )
        {
            vehicles_limebeer2014f1.at(c_vehicle->name).curvilinear_ad.get_road().change_track(table_track.at(c_track->name));
            vehicles_limebeer2014f1.at(c_vehicle->name).curvilinear_scalar.get_road().change_track(table_track.at(c_track->name));
            compute_propagation(vehicles_limebeer2014f1.at(c_vehicle->name).curvilinear_ad, q, qa, u, s, ds, u_next, options);
        }
        else
        {
            compute_propagation(vehicles_limebeer2014f1.at(c_vehicle->name).cartesian_ad, q, qa, u, s, ds, u_next, options);
        }
    }
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


void track_coordinates(double* x_center, double* y_center, double* x_left, double* y_left, double* x_right, double* y_right, double* theta, struct c_Track* c_track, const int n_points, const double* s)
{
    std::string name = c_track->name;
    auto& track = table_track.at(name);

    for (int i = 0; i < n_points; ++i)
    {
        // Compute centerline
        auto [r_c,v_c,a_c] = track(s[i]);

        x_center[i] = r_c[0];
        y_center[i] = r_c[1];

        // Heading angle (theta)
        theta[i] = atan2(v_c[1],v_c[0]);

        // Compute left boundary
        auto [r_l] = track.position_at(s[i],track.get_right_track_limit(s[i]));

        x_left[i] = r_l[0];
        y_left[i] = r_l[1];

        // Compute right boundary
        auto [r_r] = track.position_at(s[i],-track.get_left_track_limit(s[i]));

        x_right[i] = r_r[0];
        y_right[i] = r_r[1];
    }
    
    return;
}


template<typename vehicle_t>
struct Optimal_laptime_configuration
{
    // Parse the options in XML format
    // Example:
    //      <options>
    //          <warm_start> false </warm_start>
    //          <save_warm_start> true </save_warm_start>
    //          <write_xml> true </write_xml>
    //          <xml_file_name> run.xml </xml_file_name>
    //          <print_level> 5 </print_level>
    //          <initial_speed> 50.0 </initial_speed>
    //          <sigma> 0.5 </sigma>
    //          <save_variables>
    //              <prefix> run/ </prefix>
    //              <variables>
    //                  <u/>
    //                  <v/>
    //                  ...
    //              </variables>
    //          </save_variables>
    //          <closed_simulation> true </closed_simulation>
    //          <initial_condition>
    //              <q/>
    //              <qa/>
    //              <u/>
    //          </initial_condition>
    //          <control_variables>
    //              <delta optimal_control_type="full-mesh">
    //                  <dissipation/>
    //              </delta>
    //              <throttle optimal_control_type="full-mesh">
    //                  <dissipation/>
    //              </throttle>
    //              <brake-bias optimal_control_type="dont optimize"/>
    //          </control_variables>
    //      </options>
    //
    Optimal_laptime_configuration(const char* options)
    {
        if ( strlen(options) == 0 ) return;

        std::string s_options(options);
        Xml_document doc;
        doc.parse(s_options);

        // Use warm start
        if ( doc.has_element("options/warm_start") ) warm_start = doc.get_element("options/warm_start").get_value(bool());

        // Save new warm start data
        if ( doc.has_element("options/save_warm_start") ) save_warm_start = doc.get_element("options/save_warm_start").get_value(bool());

        // Write xml file
        if ( doc.has_element("options/write_xml") ) write_xml = doc.get_element("options/write_xml").get_value(bool());

        // XML file name
        if ( write_xml )
            xml_file_name = doc.get_element("options/xml_file_name").get_value();

        // Print level
        if ( doc.has_element("options/print_level") ) print_level = doc.get_element("options/print_level").get_value(int());

        // Output variables
        if ( doc.has_element("options/save_variables") )
        {
            save_variables_prefix = doc.get_element("options/save_variables/prefix").get_value();

            auto variables_node = doc.get_element("options/save_variables/variables");

            variables_to_save = {};
            for (auto& variable : variables_node.get_children())
                variables_to_save.push_back(variable.get_name());
        }

        if ( doc.has_element("options/steady_state_speed") ) steady_state_speed = doc.get_element("options/initial_speed").get_value(scalar());
    
        if ( doc.has_element("options/closed_simulation") ) is_closed = doc.get_element("options/closed_simulation").get_value(bool());

        // If open simulation, provide initial condition
        if ( !is_closed )
        {
            if ( !doc.has_element("options/initial_condition") ) throw std::runtime_error("For open simulations, the initial condition must be provided"
                                                                                          "in 'options/initial_condition'");
            set_initial_condition = true;
            auto v_q_start  = table_vector.at(doc.get_element("options/initial_condition/q").get_attribute("from_table"));
            auto v_qa_start = table_vector.at(doc.get_element("options/initial_condition/qa").get_attribute("from_table"));
            auto v_u_start  = table_vector.at(doc.get_element("options/initial_condition/u").get_attribute("from_table"));

            std::copy(v_q_start.cbegin() , v_q_start.cend() , q_start.begin());
            std::copy(v_qa_start.cbegin(), v_qa_start.cend(), qa_start.begin());
            std::copy(v_u_start.cbegin() , v_u_start.cend() , u_start.begin());
        }

        if ( doc.has_element("options/sigma") ) sigma = doc.get_element("options/sigma").get_value(scalar());

        // Prepare control variables
        if ( doc.has_element("options/control_variables") )
        {
            auto [key_name, q_names, qa_names, u_names] = vehicle_t::vehicle_ad_curvilinear::get_state_and_control_names();
            (void) key_name;
            (void) q_names;
            (void) qa_names;

            for (auto variable : doc.get_element("options/control_variables").get_children() )
            {
                // Find position in the control variables array
                const auto it_variable = std::find(u_names.cbegin(), u_names.cend(), variable.get_name());

                if ( it_variable == u_names.cend() ) throw std::runtime_error("[ERROR] Control variable \"" + variable.get_name() + "\" is not recognized");

                const auto i_control = std::distance(u_names.cbegin(), it_variable);

                // Get type
                control_type[i_control]  = variable.get_attribute("type");

                // Switch the different cases
                if ( control_type[i_control] == "dont optimize" )
                {
                    // No extra parameters are needed
                }
                else if ( control_type[i_control] == "constant" )
                {
                    throw std::runtime_error("[ERROR] To be implemented"); 
                }
                else if ( control_type[i_control] == "hypermesh" )
                {
                    // Read the hypermesh
                    hypermeshes[i_control] = variable.get_child("hypermesh").get_value(std::vector<scalar>());
                }
                else if ( control_type[i_control] == "full-mesh" )
                {
                    // Check if the dissipation is present
                    if ( variable.has_child("dissipation") ) dissipations[i_control] = variable.get_child("dissipation").get_value(scalar());
                }
                else
                {   
                    throw std::runtime_error("[ERROR] Optimal control type \"" + control_type[i_control] + "\" not recognized");
                }
            } 

        }
    } 
 
    bool warm_start                   = false;                  // Use warm start
    bool save_warm_start              = false;                  // Save simulation for warm start
    bool write_xml                    = false;                  // Write output as xml
    std::string xml_file_name         = "optimal_laptime.xml";  // Name of the output xml file
    size_t print_level                = 0;                      // Print level
    scalar steady_state_speed         = 50.0;                   // Speed used for the steady state calculation [kmh]
    bool is_direct                    = get_default_is_direct();// Compute direct simulation
    bool is_closed                    = true;                   // Compute closed simulation
    bool set_initial_condition        = false;                  // If an initial condition has to be set
    scalar sigma                      = 0.5;                    // Scheme used (0.5:Crank Nicolson, 1.0:Implicit Euler)
    std::string save_variables_prefix = "run/";                 // Prefix used to save the variables in the table
    std::vector<std::string> variables_to_save{};               // Variables chosen to be saved

    // Control variables definition
    std::array<std::string,vehicle_t::vehicle_ad_curvilinear::NCONTROL> control_type = get_default_control_types();
    std::array<scalar,vehicle_t::vehicle_ad_curvilinear::NCONTROL> dissipations = get_default_dissipations();
    std::array<std::vector<scalar>,vehicle_t::vehicle_ad_curvilinear::NCONTROL> hypermeshes;

    // For open simulations: define starting point state and controls
    std::array<scalar,vehicle_t::vehicle_ad_curvilinear::NSTATE>     q_start;       // Starting states (only open simulations)
    std::array<scalar,vehicle_t::vehicle_ad_curvilinear::NALGEBRAIC> qa_start;      // Starting algebraic states (only open simulations)
    std::array<scalar,vehicle_t::vehicle_ad_curvilinear::NCONTROL> u_start;         // Starting controls (only open simulations)

// Helper functions ----------------------------------:-
 
 private:
    static bool get_default_is_direct() 
    {
        if constexpr (std::is_same_v<vehicle_t,limebeer2014f1_all>)
            return true;
        else if constexpr (std::is_same_v<vehicle_t,lot2016kart_all>)
            return false;
        else
            throw std::runtime_error("[ERROR] get_default_control_types() not defined for this vehicle_t");
    }
    
    static std::array<std::string,vehicle_t::vehicle_ad_curvilinear::NCONTROL> get_default_control_types() 
    {
        if constexpr (std::is_same_v<vehicle_t,limebeer2014f1_all>)
            return {"full-mesh", "full-mesh", "dont optimize"};
        else if constexpr (std::is_same_v<vehicle_t,lot2016kart_all>)
            return {"full-mesh", "full-mesh"};
        else
            throw std::runtime_error("[ERROR] get_default_control_types() not defined for this vehicle_t");
    }
    
    static std::array<scalar,vehicle_t::vehicle_ad_curvilinear::NCONTROL> get_default_dissipations()
    {
        if constexpr (std::is_same_v<vehicle_t,limebeer2014f1_all>)
            return {5.0, 8.0e-4, 0.0};
        else if constexpr (std::is_same_v<vehicle_t,lot2016kart_all>)
            return {1.0e-2, 200*200*1.0e-10};
        else
            throw std::runtime_error("[ERROR] get_default_control_types() not defined for this vehicle_t");
    }

};

template<typename vehicle_t>
typename Optimal_laptime<typename vehicle_t::vehicle_ad_curvilinear>::template Control_variables<>
    construct_control_variables(const Optimal_laptime_configuration<vehicle_t>& conf, 
    const size_t n_points, const std::array<scalar, vehicle_t::vehicle_ad_curvilinear::NCONTROL>& u_steady_state)
{
    // (1) Define control variables 
    auto control_variables = typename Optimal_laptime<typename vehicle_t::vehicle_ad_curvilinear>::template Control_variables<>{};

    // (2) Construct each control variable from the information in the configuration 
    for (size_t j = 0; j < vehicle_t::vehicle_ad_curvilinear::NCONTROL; ++j)
    {
        if ( conf.control_type[j] == "dont optimize" )
        {
            // (2.1) Don't optimize: take the value from the output of the steady state calculation
            control_variables[j] = Optimal_laptime<typename vehicle_t::vehicle_ad_curvilinear>::create_dont_optimize();
        }
        else if ( conf.control_type[j] == "constant" )
        {
            throw std::runtime_error("[ERROR] Not implemented yet");
        }
        else if ( conf.control_type[j] == "hypermesh" )
        {
            control_variables[j] = Optimal_laptime<typename vehicle_t::vehicle_ad_curvilinear>::create_hypermesh(conf.hypermeshes[j],
                                                                                                                 std::vector<scalar>(conf.hypermeshes[j].size(),u_steady_state[j]));
        }
        else if ( conf.control_type[j] == "full-mesh" )
        {
            // (2.4) Create a full mesh control variable from the steady state values
            if ( conf.is_direct )
            {
                control_variables[j] = Optimal_laptime<typename vehicle_t::vehicle_ad_curvilinear>::create_full_mesh(std::vector<scalar>(n_points,u_steady_state[j]),
                                                                                                    conf.dissipations[j]);
            }
            else 
            {
                control_variables[j] = Optimal_laptime<typename vehicle_t::vehicle_ad_curvilinear>::create_full_mesh(std::vector<scalar>(n_points,u_steady_state[j]),
                                                                                                    std::vector<scalar>(n_points,0.0),
                                                                                                    conf.dissipations[j]);
            }
        }
    }
    
    // (3) Return
    return control_variables;
} 


template<typename vehicle_t>
void compute_optimal_laptime(vehicle_t& vehicle, Track_by_polynomial& track, struct c_Vehicle* c_vehicle, const int n_points, const double* s, const char* options)
{
    // (1) Get aliases to cars
    auto& car_curv = vehicle.get_curvilinear_ad_car();
    auto& car_curv_sc = vehicle.get_curvilinear_scalar_car();

    auto& car_cart = vehicle.cartesian_ad;

    // (2) Process options
    auto conf = Optimal_laptime_configuration<vehicle_t>(options);
   
    // (3) Set the track into the curvilinear car dynamic model
    car_curv.get_road().change_track(track);
    car_curv_sc.get_road().change_track(track);

    // (4) Start from the steady-state values at 0g    
    scalar v = conf.steady_state_speed*KMH;

    auto ss = Steady_state(car_cart).solve(v,0.0,0.0); 

    if ( c_vehicle->type == LOT2016KART )
        ss.u[1] = 0.0;

    // (5) Compute optimal laptime

    // (5.1) Get arclength
    std::vector<scalar> arclength(s,s+n_points);
    Optimal_laptime<typename vehicle_t::vehicle_ad_curvilinear> opt_laptime;
    typename Optimal_laptime<typename vehicle_t::vehicle_ad_curvilinear>::Options opts;
    opts.print_level = conf.print_level;
    opts.sigma       = conf.sigma;

    // (5.2.a) Start from steady-state
    if ( !conf.warm_start )
    {
        std::vector<std::array<scalar,vehicle_t::vehicle_ad_curvilinear::NSTATE>> q0  = {static_cast<size_t>(n_points),ss.q};
        std::vector<std::array<scalar,vehicle_t::vehicle_ad_curvilinear::NALGEBRAIC>> qa0 = {static_cast<size_t>(n_points),ss.qa};
        auto control_variables = construct_control_variables(conf, static_cast<size_t>(n_points), ss.u); 

        if ( conf.set_initial_condition )
        {
            q0.front()  = conf.q_start;
            qa0.front() = conf.qa_start;

            // Set only full-mesh variables
            for (size_t j = 0; j < vehicle_t::vehicle_ad_curvilinear::NCONTROL; ++j)
            {   
                if ( control_variables[j].optimal_control_type == Optimal_laptime<typename vehicle_t::vehicle_ad_curvilinear>::FULL_MESH)
                {
                    control_variables[j].u.front() = conf.u_start[j];
                }
            }
        }
        opt_laptime = Optimal_laptime(arclength, conf.is_closed, conf.is_direct, car_curv, q0, qa0, control_variables, opts);
    }
    // (5.2.b) Warm start
    else
    {
        opt_laptime = Optimal_laptime(get_warm_start<vehicle_t>().s, get_warm_start<vehicle_t>().is_closed, get_warm_start<vehicle_t>().is_direct, 
                        car_curv, get_warm_start<vehicle_t>().q, 
                        get_warm_start<vehicle_t>().qa, get_warm_start<vehicle_t>().control_variables, 
                        get_warm_start<vehicle_t>().optimization_data.zl, get_warm_start<vehicle_t>().optimization_data.zu, 
                        get_warm_start<vehicle_t>().optimization_data.lambda, opts);
    }

    // (6) Save results -----------------------------------------------------------------------

    // (6.1) Save xml file
    if ( conf.write_xml )
        opt_laptime.xml()->save(conf.xml_file_name);

    // (6.2) Save outputs
    for (const auto& variable_name : conf.variables_to_save)
    {
        // Check if the variable_name exists in any of the tables
        if ( table_scalar.count(conf.save_variables_prefix + variable_name) != 0 )
            throw std::runtime_error(std::string("Variable \"") + conf.save_variables_prefix + variable_name + "\" already exists in the scalar table");

        if ( table_vector.count(conf.save_variables_prefix + variable_name) != 0 )
            throw std::runtime_error(std::string("Variable \"") + conf.save_variables_prefix + variable_name + "\" already exists in the vector table");

        bool is_vector = true;

        // Scalar variables
        if ( variable_name == "laptime" )
        {
            table_scalar.insert({conf.save_variables_prefix+variable_name, opt_laptime.laptime});
            is_vector = false;
        }

        // Vector variables
        if ( is_vector ) 
        {
            std::vector<scalar> data(n_points);
            for (int i = 0; i < n_points; ++i)
            {
                car_curv_sc(opt_laptime.q[i], opt_laptime.qa[i], opt_laptime.control_variables.control_array_at_s(car_curv,i,s[i]), s[i]);
    
                if ( variable_name == "x" ) 
                    data[i] = car_curv_sc.get_road().get_x();
        
                else if ( variable_name == "y" )
                    data[i] = car_curv_sc.get_road().get_y();
    
                else if ( variable_name == "s" )
                    data[i] = s[i];

                else if ( variable_name == "n" )
                    data[i] = opt_laptime.q[i][vehicle_t::vehicle_scalar_curvilinear::Road_type::IN];
    
                else if ( variable_name == "alpha" )
                    data[i] = opt_laptime.q[i][vehicle_t::vehicle_scalar_curvilinear::Road_type::IALPHA];
    
                else if ( variable_name == "u" )
                    data[i] = opt_laptime.q[i][vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IU];
    
                else if ( variable_name == "v" )
                    data[i] = opt_laptime.q[i][vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IV];
    
                else if ( variable_name == "time" )
                    data[i] = opt_laptime.q[i][vehicle_t::vehicle_scalar_curvilinear::Road_type::ITIME];
    
                else if ( variable_name == "delta" )
                    data[i] = opt_laptime.control_variables[vehicle_t::vehicle_scalar_curvilinear::Chassis_type::Front_axle_type::ISTEERING].u[i];
    
                else if ( variable_name == "psi" )
                    data[i] = car_curv_sc.get_road().get_psi();
    
                else if ( variable_name == "omega" )
                    data[i] = opt_laptime.q[i][vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IOMEGA];
    
                else if ( variable_name == "throttle" )
                {
                    if constexpr (std::is_same<vehicle_t, lot2016kart_all>::value)
                    {
                        data[i] = opt_laptime.control_variables[vehicle_t::vehicle_scalar_curvilinear::Chassis_type::Rear_axle_type::ITORQUE].u[i];
                    }
    
                    else if constexpr (std::is_same<vehicle_t, limebeer2014f1_all>::value)
                    {
                        data[i] = opt_laptime.control_variables[vehicle_t::vehicle_scalar_curvilinear::Chassis_type::ITHROTTLE].u[i];
                    }
                }
                else if ( variable_name == "brake-bias" )
                {
                    if constexpr (std::is_same<vehicle_t, lot2016kart_all>::value)
                    {
                        throw std::runtime_error("[ERROR] brake-bias is not available for vehicles of type lot2016kart");
                    }
            
                    else if constexpr (std::is_same<vehicle_t, limebeer2014f1_all>::value)
                    {
                        data[i] = car_curv_sc.get_chassis().get_brake_bias();
                    }
                    else
                    {
                        throw std::runtime_error("[ERROR] Vehicle type is not defined");
                    }
                }
                else if ( variable_name == "rear_axle.left_tire.x" )
                    data[i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<0>().get_position().at(0);
    
                else if ( variable_name == "rear_axle.left_tire.y" )
                    data[i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<0>().get_position().at(1);
    
                else if ( variable_name == "rear_axle.right_tire.x" )
                    data[i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<1>().get_position().at(0);
    
                else if ( variable_name == "rear_axle.right_tire.y" )
                    data[i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<1>().get_position().at(1);
    
                else if ( variable_name == "front_axle.left_tire.x" )
                    data[i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<0>().get_position().at(0);
    
                else if ( variable_name == "front_axle.left_tire.y" )
                    data[i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<0>().get_position().at(1);
    
                else if ( variable_name == "front_axle.right_tire.x" )
                    data[i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<1>().get_position().at(0);
    
                else if ( variable_name == "front_axle.right_tire.y" )
                    data[i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<1>().get_position().at(1);

                else if ( variable_name == "front_axle.left_tire.kappa" )
                    data[i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<0>().get_kappa();
    
                else if ( variable_name == "front_axle.right_tire.kappa" )
                    data[i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<1>().get_kappa();
    
                else if ( variable_name == "rear_axle.left_tire.kappa" )
                    data[i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<0>().get_kappa();
    
                else if ( variable_name == "rear_axle.right_tire.kappa" )
                    data[i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<1>().get_kappa();

                else if ( variable_name == "front_axle.left_tire.dissipation" )
                    data[i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<0>().get_dissipation();
            
                else if ( variable_name == "front_axle.right_tire.dissipation" )
                    data[i] = car_curv_sc.get_chassis().get_front_axle().template get_tire<1>().get_dissipation();
            
                else if ( variable_name == "rear_axle.left_tire.dissipation" )
                    data[i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<0>().get_dissipation();
            
                else if ( variable_name == "rear_axle.right_tire.dissipation" )
                    data[i] = car_curv_sc.get_chassis().get_rear_axle().template get_tire<1>().get_dissipation();
    
                else if ( variable_name == "Fz_fl" )
                {
                    if constexpr (std::is_same<vehicle_t, limebeer2014f1_all>::value)
                    {
                        data[i] = opt_laptime.qa[i][vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IFZFL];
                    }
                    else 
                    {
                        throw std::runtime_error("Fz_fl is only defined for limebeer2014f1 models");
                    }
                }
    
                else if ( variable_name == "Fz_fr" )
                {
                    if constexpr (std::is_same<vehicle_t, limebeer2014f1_all>::value)
                    {
                        data[i] = opt_laptime.qa[i][vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IFZFR];
                    }
                    else 
                    {
                        throw std::runtime_error("Fz_fr is only defined for limebeer2014f1 models");
                    }
                }
    
                else if ( variable_name == "Fz_rl" )
                {
                    if constexpr (std::is_same<vehicle_t, limebeer2014f1_all>::value)
                    {
                        data[i] = opt_laptime.qa[i][vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IFZRL];
                    }
                    else 
                    {
                        throw std::runtime_error("Fz_rl is only defined for limebeer2014f1 models");
                    }
                }
    
                else if ( variable_name == "Fz_rr" )
                {
                    if constexpr (std::is_same<vehicle_t, limebeer2014f1_all>::value)
                    {
                        data[i] = opt_laptime.qa[i][vehicle_t::vehicle_scalar_curvilinear::Chassis_type::IFZRR];
                    }
                    else 
                    {
                        throw std::runtime_error("Fz_rr is only defined for limebeer2014f1 models");
                    }
                }

                else
                {
                    throw std::runtime_error("Variable \"" + variable_name + "\" is not defined");
                }
    
            }
    
            // Insert in the vector table
            table_vector.insert({conf.save_variables_prefix + variable_name, data});
        }
    }

    // (6.3) Save warm start for next runs
    if (conf.save_warm_start)
        get_warm_start<vehicle_t>() = opt_laptime;
}


void optimal_laptime(struct c_Vehicle* c_vehicle, const struct c_Track* c_track, const int n_points, const double* s, const char* options) 
{
    if ( c_vehicle->type == LOT2016KART )
    {
        compute_optimal_laptime(vehicles_lot2016kart.at(c_vehicle->name), table_track.at(c_track->name), 
                                c_vehicle, n_points, s, options);
    }
    else if ( c_vehicle->type == LIMEBEER2014F1 )
    {
        compute_optimal_laptime(vehicles_limebeer2014f1.at(c_vehicle->name), table_track.at(c_track->name), 
                                c_vehicle, n_points, s, options);
    }
}


void change_track(struct c_Vehicle* c_vehicle, const struct c_Track* c_track)
{
    if ( c_vehicle->type == LOT2016KART )
    {
        vehicles_lot2016kart.at(c_vehicle->name).get_curvilinear_ad_car().get_road().change_track(table_track.at(c_track->name));
        vehicles_lot2016kart.at(c_vehicle->name).get_curvilinear_scalar_car().get_road().change_track(table_track.at(c_track->name));
    }
    else if ( c_vehicle->type == LIMEBEER2014F1 )
    {
        vehicles_limebeer2014f1.at(c_vehicle->name).get_curvilinear_ad_car().get_road().change_track(table_track.at(c_track->name));
        vehicles_limebeer2014f1.at(c_vehicle->name).get_curvilinear_scalar_car().get_road().change_track(table_track.at(c_track->name));
    }
}
