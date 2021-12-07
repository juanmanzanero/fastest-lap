#ifndef __SIMULATION_FILE_HPP__
#define __SIMULATION_FILE_HPP__

template<typename Dynamic_model_t>
Simulation_file<Dynamic_model_t>::Simulation_file(const std::string& name, Dynamic_model_t& car, const std::vector<scalar> s, 
    const std::vector<std::array<scalar,Dynamic_model_t::NSTATE>>& q, const std::vector<std::array<scalar,Dynamic_model_t::NCONTROL>>& u)
: _name(name),
  _file()
{
    // Check inputs
    const size_t n = q.size();

    if ( s.size() != n)
    {
        std::ostringstream sOut;
        sOut << "Size of arclength vectors is different to size of state vectors" << std::endl;
        sOut << "   size(s): " << s.size() << std::endl;
        sOut << "   size(q): " << q.size() << std::endl;
        throw std::runtime_error(sOut.str());
    }
    
    if ( u.size() != n)
        throw std::runtime_error("Size of control vectors is different to size of state vectors");

    // Add root node
    _file.create_root_element("simulation");

    Xml_element root_node = _file.get_root_element();

    root_node.add_attribute("name", _name);

    const auto [key_name, state_names, control_names] = car.get_state_and_control_names();

    // Add records
    for (size_t i = 0; i < n; ++i)
    {
        car(q[i],u[i],s[i]);
        std::ostringstream s_out;
        s_out << std::setprecision(16);
        Xml_element record = root_node.add_child("record");
        record.add_attribute("id", int(i));

        s_out.str(std::string()); s_out << s[i];
        record.add_child(key_name).set_value(s_out.str());

        for (size_t j = 0; j < Dynamic_model_t::NSTATE; ++j)
        {
            s_out.str(std::string()); s_out << q[i][j];
            record.add_child(state_names[j]).set_value(s_out.str());
        }
        
        for (size_t j = 0; j < Dynamic_model_t::NCONTROL; ++j)
        {
            s_out.str(std::string()); s_out << u[i][j];
            record.add_child(control_names[j]).set_value(s_out.str());
        }

        if ( std::find(state_names.begin(), state_names.end(), "x") == state_names.end() )
        {
            s_out.str(std::string()); s_out << car.get_road().get_x();
            record.add_child("x").set_value(s_out.str());
        }

        if ( std::find(state_names.begin(), state_names.end(), "y") == state_names.end() )
        {
            s_out.str(std::string()); s_out << car.get_road().get_y();
            record.add_child("y").set_value(s_out.str());
        }

        if ( std::find(state_names.begin(), state_names.end(), "psi") == state_names.end() )
        {
            s_out.str(std::string()); s_out << car.get_road().get_psi();
            record.add_child("psi").set_value(s_out.str());
        }
    }
}

#endif
