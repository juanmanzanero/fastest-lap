#ifndef OPTIMAL_LAPTIME_DATA_H
#define OPTIMAL_LAPTIME_DATA_H

#include <vector>

#include "src/core/applications/detail/optimal_laptime_control_variables.h"

namespace fastest_lap::optimal_laptime::detail
{
    template<size_t number_of_inputs, size_t number_of_controls>
    struct Optimal_laptime_data
    {
        template<typename Dynamic_model_t>
        void construct_from_xml(Xml_document& doc);

        template<typename Dynamic_model_t>
        std::unique_ptr<Xml_document> xml() const;

        template<bool is_direct, typename Control_variables_t>
        constexpr static const size_t n_variables_per_point(const Control_variables_t& control_variables)
        {
            return number_of_inputs - 1 + (is_direct ? 1 : 2)*control_variables.number_of_full_optimizations;
        }
    
        template<bool is_direct, typename Control_variables_t>
        constexpr static const size_t n_constraints_per_element(const Control_variables_t& control_variables, const size_t number_of_extra_constraints)
        {
            return number_of_inputs - 1 + number_of_extra_constraints
                + (is_direct ? 0 : control_variables.number_of_full_optimizations);
        }

        bool success;
        bool is_closed;
        bool is_direct;
        bool warm_start;
        size_t n_elements;
        size_t n_points;
        std::vector<scalar> s;                                    //! Arclengths
        std::vector<std::array<scalar,number_of_inputs>> inputs;  //! All state vectors
        Control_variables<number_of_controls, scalar> controls;   //! All control variables pre and post optimization
        std::vector<scalar> x_coord;                              //! x-coordinate of the trajectory
        std::vector<scalar> y_coord;                              //! y-coordinate of the trajectory
        std::vector<scalar> psi;                                  //! heading angle
    
        size_t iter_count;  //! Number of iterations spent in IPOPT
    
        struct 
        {
            std::vector<scalar> x;
            std::vector<scalar> x_lb;
            std::vector<scalar> x_ub;
            std::vector<scalar> c_lb;
            std::vector<scalar> c_ub;
            std::vector<scalar> zl;     //! Lagrange multipliers of the variable lower bounds
            std::vector<scalar> zu;     //! Lagrange multipliers of the variable upper bounds
            std::vector<scalar> lambda; //! Lagrange multipliers of the optimization constraints
            std::vector<scalar> s;
            std::vector<scalar> vl;
            std::vector<scalar> vu;
        } optimization_data;            //! Auxiliary class to store the optimization data
    
        scalar laptime;
    };


    template<size_t number_of_inputs, size_t number_of_controls>
    template<typename Dynamic_model_t>
    inline void Optimal_laptime_data<number_of_inputs, number_of_controls>::construct_from_xml(Xml_document& doc)
    {
        Xml_element root = doc.get_root_element();
    
        if ( root.get_attribute("type") == "closed" )
            is_closed = true;
        else if ( root.get_attribute("type") == "open" )
            is_closed = false;
        else
            throw fastest_lap_exception("Incorrect track type, should be \"open\" or \"closed\"");
    
        if ( root.get_attribute("is_direct") == "true" )
            is_direct = true;
        else if ( root.get_attribute("is_direct") == "false" )
            is_direct = false;
        else
            throw fastest_lap_exception("Incorrect \"is_direct\" attribute, should be \"true\" or \"false\"");
    
        auto children = doc.get_root_element().get_children();
    
        const auto [key_name, q_names, u_names] = Dynamic_model_t{}.get_state_and_control_names();
    
        // Get the data
        n_points = std::stoi(root.get_attribute("n_points"));
        n_elements = (is_closed ? n_points : n_points - 1);
    
        laptime = root.get_child("laptime").get_value(scalar());
    
        s = root.get_child(key_name).get_value(std::vector<scalar>());
    
        // Get state
        inputs = std::vector<std::array<scalar,Dynamic_model_t::number_of_inputs>>(n_points);
        for (size_t i = 0; i < Dynamic_model_t::number_of_inputs; ++i)
        {
            std::vector<scalar> data_in = root.get_child(q_names[i]).get_value(std::vector<scalar>());
            for (size_t j = 0; j < n_points; ++j)
                inputs[j][i] = data_in[j];
        }
    
        // Get controls
        controls = Control_variables<number_of_controls>{};
        for (size_t i = 0; i < Dynamic_model_t::number_of_controls; ++i)
        {
            auto control_var_element = root.get_child("control_variables/" + u_names[i]);
            
            // Get optimal control type attribute
            auto optimal_control_type_str = control_var_element.get_attribute("optimal_control_type");
    
            if ( optimal_control_type_str == "dont optimize" )
                controls[i].optimal_control_type = Optimal_control_type::DONT_OPTIMIZE;
            else if ( optimal_control_type_str == "constant" )
                controls[i].optimal_control_type = Optimal_control_type::CONSTANT;
            else if ( optimal_control_type_str == "hypermesh" )
                controls[i].optimal_control_type = Optimal_control_type::HYPERMESH;
            else if ( optimal_control_type_str == "full-mesh" )
                controls[i].optimal_control_type = Optimal_control_type::FULL_MESH;
            else
                throw fastest_lap_exception("optimal_control_type attribute not recognize. Options are: 'dont optimize', 'constant', 'hypermesh', 'full-mesh'");
    
            // Get value
            controls[i].controls = root.get_child("control_variables/" + u_names[i] + "/values").get_value(std::vector<scalar>());
    
            // Get hypermesh
            if ( controls[i].optimal_control_type == Optimal_control_type::HYPERMESH )
                controls[i].s_hypermesh = root.get_child("control_variables/" + u_names[i] + "/hypermesh").get_value(std::vector<scalar>());
    
            // Get derivative value if present
            if ( !is_direct && root.has_child("control_variables/" + u_names[i] + "/derivatives") )
                controls[i].dcontrols_dt = root.get_child("control_variables/" + u_names[i] + "/derivatives").get_value(std::vector<scalar>());
    
            // Get hypermesh 
            if ( controls[i].optimal_control_type == Optimal_control_type::HYPERMESH )
                controls[i].s_hypermesh = root.get_child("control_variables/" + u_names[i] + "/hypermesh").get_value(std::vector<scalar>());
        }
    
        // check them and compute its statistics
        controls.check();
    
        // Get x
        x_coord = root.get_child("x").get_value(std::vector<scalar>());
        y_coord = root.get_child("y").get_value(std::vector<scalar>());
        psi     = root.get_child("psi").get_value(std::vector<scalar>());
    
        // Get optimization data
        optimization_data.zl     = root.get_child("optimization_data").get_child("zl").get_value(std::vector<scalar>());
        optimization_data.zu     = root.get_child("optimization_data").get_child("zu").get_value(std::vector<scalar>());
        optimization_data.lambda = root.get_child("optimization_data").get_child("lambda").get_value(std::vector<scalar>());
    }

    
    template<size_t number_of_inputs, size_t number_of_controls>
    template<typename Dynamic_model_t>
    inline std::unique_ptr<Xml_document> Optimal_laptime_data<number_of_inputs, number_of_controls>::xml() const
    {
        std::unique_ptr<Xml_document> doc_ptr(std::make_unique<Xml_document>());
    
        std::ostringstream s_out;
        s_out.precision(17);
    
        doc_ptr->create_root_element("optimal_laptime");
    
        auto root = doc_ptr->get_root_element();
        
        root.set_attribute("n_points",std::to_string(n_points));
        
        if ( is_closed )
            root.set_attribute("type", "closed");
        else
            root.set_attribute("type", "open");
    
        if ( is_direct )
            root.set_attribute("is_direct","true");
        else
            root.set_attribute("is_direct","false");
        
    
        // Save arclength
        for (size_t j = 0; j < s.size()-1; ++j)
            s_out << s[j] << ", ";
    
        s_out << s.back();
    
        root.add_child("laptime").set_value(std::to_string(laptime));
    
        const auto [key_name, q_names, u_names] = Dynamic_model_t{}.get_state_and_control_names();
    
        root.add_child(key_name).set_value(s_out.str());
        s_out.str(""); s_out.clear();
    
    
        // Save state
        for (size_t i = 0; i < Dynamic_model_t::number_of_inputs; ++i)
        {
            for (size_t j = 0; j < inputs.size()-1; ++j)
                s_out << inputs[j][i] << ", " ;
    
            s_out << inputs.back()[i];
    
            root.add_child(q_names[i]).set_value(s_out.str());
            s_out.str(""); s_out.clear();
        }
    
        // Save controls
        auto node_control_variables = root.add_child("control_variables");
        for (size_t i = 0; i < Dynamic_model_t::number_of_controls; ++i)
        {
            auto node_variable = node_control_variables.add_child(u_names[i]);
            if ( controls[i].optimal_control_type == Optimal_control_type::DONT_OPTIMIZE )
            {
                node_variable.set_attribute("optimal_control_type","dont optimize");
                node_variable.add_child("values");
            }
            else if ( controls[i].optimal_control_type == Optimal_control_type::CONSTANT )
            {   
                node_variable.set_attribute("optimal_control_type","constant");
                s_out << controls[i].controls.front();
                node_variable.add_child("values").set_value(s_out.str());
                s_out.str(""); s_out.clear();
            }
            else if ( controls[i].optimal_control_type == Optimal_control_type::HYPERMESH )
            {
                node_variable.set_attribute("optimal_control_type","hypermesh");
                for (size_t j = 0; j < controls[i].controls.size()-1; ++j)
                    s_out << controls[i].controls[j] << ", " ;
    
                s_out << controls[i].controls.back();
    
                node_variable.add_child("values").set_value(s_out.str());
                s_out.str(""); s_out.clear();
    
                for (size_t j = 0; j < controls[i].s_hypermesh.size()-1; ++j)
                    s_out << controls[i].s_hypermesh[j] << ", " ;
    
                s_out << controls[i].s_hypermesh.back();
    
                node_variable.add_child("hypermesh").set_value(s_out.str());
                s_out.str(""); s_out.clear();
            }
            else if ( controls[i].optimal_control_type == Optimal_control_type::FULL_MESH )
            {
                node_variable.set_attribute("optimal_control_type", "full-mesh");
                for (size_t j = 0; j < controls[i].controls.size()-1; ++j)
                    s_out << controls[i].controls[j] << ", " ;
    
                s_out << controls[i].controls.back();
    
                node_variable.add_child("values").set_value(s_out.str());
                s_out.str(""); s_out.clear();
    
                // Save controls derivatives if not direct
                if ( !is_direct )
                {
                    for (size_t j = 0; j < controls[i].dcontrols_dt.size()-1; ++j)
                        s_out << controls[i].dcontrols_dt[j] << ", " ;
            
                    s_out << controls[i].dcontrols_dt.back();
    
                    node_variable.add_child("derivatives").set_value(s_out.str());
                    s_out.str(""); s_out.clear();
                }
            }
        }
    
        // Save x, y, and psi if they are not contained
        for (size_t j = 0; j < inputs.size()-1; ++j)
            s_out << x_coord[j] << ", " ;
    
        s_out << x_coord.back();
        root.add_child("x").set_value(s_out.str());
        s_out.str(""); s_out.clear();
    
        for (size_t j = 0; j < inputs.size()-1; ++j)
            s_out << y_coord[j] << ", " ;
    
        s_out << y_coord.back();
        root.add_child("y").set_value(s_out.str());
        s_out.str(""); s_out.clear();
    
        for (size_t j = 0; j < inputs.size()-1; ++j)
            s_out << psi[j] << ", " ;
    
        s_out << psi.back();
        root.add_child("psi").set_value(s_out.str());
        s_out.str(""); s_out.clear();
    
        // Save optimization data
        auto opt_data = root.add_child("optimization_data");
        size_t n_vars(0);
        size_t n_cons(0);
    
        if ( is_direct )
        {
            n_vars = n_variables_per_point<true>(controls);
            n_cons = n_constraints_per_element<true>(controls, Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS);
        }
        else
        {
            n_vars = n_variables_per_point<false>(controls);
            n_cons = n_constraints_per_element<false>(controls, Dynamic_model_t::N_OL_EXTRA_CONSTRAINTS);
        }
    
        opt_data.set_attribute("n_variables_per_point", std::to_string(n_vars));
        opt_data.set_attribute("n_constraints_per_element", std::to_string(n_cons));
    
        opt_data.add_child("number_of_iterations").set_value(std::to_string(iter_count));
    
        for (size_t j = 0; j < optimization_data.zl.size()-1; ++j)
            s_out << optimization_data.zl[j] << ", " ;
    
        s_out << optimization_data.zl.back();
    
        opt_data.add_child("zl").set_value(s_out.str());
        s_out.str(""); s_out.clear();
    
        for (size_t j = 0; j < optimization_data.zu.size()-1; ++j)
            s_out << optimization_data.zu[j] << ", " ;
    
        s_out << optimization_data.zu.back();
    
        opt_data.add_child("zu").set_value(s_out.str());
        s_out.str(""); s_out.clear();
    
        for (size_t j = 0; j < optimization_data.lambda.size()-1; ++j)
            s_out << optimization_data.lambda[j] << ", " ;
    
        s_out << optimization_data.lambda.back();
    
        opt_data.add_child("lambda").set_value(s_out.str());
        s_out.str(""); s_out.clear();
    
        return doc_ptr;
    } 
}

#endif
