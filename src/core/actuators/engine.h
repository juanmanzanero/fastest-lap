#ifndef __ENGINE_H__
#define __ENGINE_H__

#include<map>
#include "lion/math/polynomial.h"
#include "lion/io/Xml_document.h"
#include "lion/io/database_parameters.h"

template<typename Timeseries_t>
class Engine
{
 public:
    using Timeseries_type = Timeseries_t;
    //! A default constructor if no engine is needed
    Engine() = default;

    //! Proper constructor from parameters + engine path
    Engine(Xml_document& database, const std::string& path, const bool only_max_power);

    // Set parameter    
    template<typename T>
    void set_parameter(const std::string& parameter, const T value)
    {
        // Find the parameter in the database
        const auto found = ::set_parameter(get_parameters(), parameter, _path, value); 

        // If not found, throw an exception
        if ( !found )
            throw std::runtime_error("Parameter \"" + parameter + "\" was not found");
    }

    constexpr const sPolynomial& get_polynomial() const { return _p; }

    Timeseries_t operator()(const Timeseries_t throttle_percentage, const Timeseries_t rpm);

    constexpr const bool& direct_torque() const { return _direct_torque; }
    constexpr       bool& direct_torque()       { return _direct_torque; }

    constexpr const scalar& gear_ratio() const { return _gear_ratio; }

 private:
    std::string _path;

    sPolynomial _p;

    scalar _gear_ratio;
    bool _direct_torque = true;
    bool _only_max_power;

    scalar _maximum_power;      

    std::vector<Database_parameter> get_parameters() { return 
    { 
        { "maximum-power", _maximum_power } 
    };}
};

#include "engine.hpp"

#endif
