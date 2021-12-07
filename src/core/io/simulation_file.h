#ifndef __SIMULATION_FILE_H__
#define __SIMULATION_FILE_H__

#include "lion/io/xml_document.h"

template<typename Dynamic_model_t>
class Simulation_file
{
 public:

    //! Constructor: creates the Xml document and fills it with the simulation data
    Simulation_file(const std::string& name, 
                    Dynamic_model_t& car,
                    const std::vector<scalar> s,
                    const std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>& q,
                    const std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>>& u 
                   );

    void save_to_disk(const std::string& file_name) { _file.save(file_name); }

    Xml_document& get_file() { return _file; }

 private:
    std::string  _name;
    Xml_document _file;
};

#include "simulation_file.hpp"

#endif
