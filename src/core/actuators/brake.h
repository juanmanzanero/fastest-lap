#ifndef __BRAKE_H__
#define __BRAKE_H__

#include "lion/foundation/types.h"
#include "lion/io/Xml_document.h"

template<typename Timeseries_t>
class Brake
{
 public:
    using Timeseries_type = Timeseries_t;

    //! Default constructor
    Brake() = default;

    //! Constructor: brake from maximum decceleration, wheel radius, and vehicle mass
    Brake(const std::string& path) :
        _path(path), _Tmax(0.0) 
    {}  

    //! Constructor: brake from maximum decceleration, wheel radius, and vehicle mass
    Brake(Xml_document& database, const std::string& path) :
        _path(path), _Tmax(0.0) 
    {
        read_parameters(database, path, get_parameters());
    }  

    // Set parameter    
    template<typename T>
    void set_parameter(const std::string& parameter, const T value)
    {
        // Find the parameter in the database
        const auto found = ::set_parameter(get_parameters(), parameter, _path, value); 

        // If not found, throw an exception
        if ( !found )
            throw std::runtime_error("Parameter \"" + parameter + "\" was not found in brake");
    }

    // Fill xml
    void fill_xml(Xml_document& doc) const
    {
        ::write_parameters(doc, _path, get_parameters());
    }

    //! Operator(): Apply a scaling of the maximum braking torque with the brake percentage applied
    Timeseries_t operator()(const Timeseries_t brake_percentage) { return brake_percentage*_Tmax; }

 private:
    std::string _path;
    scalar _Tmax;

    DECLARE_PARAMS({ "max_torque", _Tmax }); 
};

#endif 
