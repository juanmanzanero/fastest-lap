#ifndef DYNAMIC_MODEL_H
#define DYNAMIC_MODEL_H

template<typename Timeseries_t>
class Dynamic_model
{
 public:
    using T = Timeseries_t;

    //! Handling of parameters ------------------------------------------:-
    template<typename ... Args> 
    void add_parameter(const std::string& parameter_path, Args&& ... args)
    {
        // Check that the parameter was not already loaded
        for (const auto& p : _parameters)
        {
            if ( p.get_path() == parameter_path )
                throw std::runtime_error("[ERROR] Parameter::add_parameter -> parameter already exists");
        }

        _parameters.emplace_back(parameter_path, std::forward<Args>(args)...);
    }

    // Set all the parameters from a hyper-vector
    void set_all_parameters(const std::vector<Timeseries_t>& inputs) { _parameters.set_all_parameters(inputs); }

 private:

    class Parameter
    {
     public:
        //! Construct a constant parameter
        Parameter(const std::string& path, const std::string& alias, const scalar& value) 
        : _path(path), _aliases{alias}, _values{value}, _mesh{} 
        {}

        //! Construct a varying parameter
        Parameter(const std::string& path, const std::vector<std::string>& aliases, const std::vector<scalar>& values, const std::vector<std::pair<scalar,size_t>>& mesh) 
        : _path(path), _aliases(aliases), _values(values.size()), _mesh(mesh)
        {
            // (1) Copy the parameters, which will transform their values from scalar to Timeseries_t
            std::copy(values.cbegin(), values.cend(), _values.begin());

            // (2) Check that there are at least two parameters
            if ( _values.size() < 2 )
                throw std::runtime_error("[ERROR] A spatially-varying parameter should be provided with at least two parameters");

            // (3) Check that the same number of aliases and values were given
            if ( _aliases.size() != _values.size() )
                throw std::runtime_error("[ERROR] The number of aliases must match the number of values"); 

            // (3) Check that the mesh is sorted
            if ( !std::is_sorted(_mesh.cbegin(), _mesh.cend(), [](const auto& lhs, const auto& rhs) -> auto { return lhs.first <= rhs.first; }) )
                throw std::runtime_error("[ERROR] Mesh checkpoints should be sorted in ascending order");
    
            // (4) Check that all parameters are used
            std::vector<bool> used_parameters(_values.size(), false);

            for (const auto& [s_i, i_par] : _mesh)
            {
                if ( i_par >= _values.size() )
                    throw std::runtime_error("[ERROR] Error creating varying parameter, i_par is out of bounds");

                used_parameters[i_par] = true;
            }

            if ( std::count(used_parameters.cbegin(), used_parameters.cend(), false) > 0 )
                throw std::runtime_error("[ERROR] There are unused parameters");
        }

        //! Get the total number of parameters
        size_t n_parameters() const { return _values.size(); }

        //! Evaluate the parameter at a given arclenght
        Timeseries_t operator()(const scalar& s) const
        {
            if ( _values.size() == 1 )            
            {
                return _values.front();
            }
            else
            {
                // (1) Get an iterator to the first point in the hypermesh grid that *it > s
                auto it = std::upper_bound(_mesh.cbegin(), _mesh.cend(), s, [](const auto& lhs, const auto& rhs) -> auto { return lhs < rhs.first; });

                // (2) Compute the value
                if ( it == _mesh.cbegin() )   
                    // (2.1) If s < mesh[0], return the first element
                    return _values.front();
                else if ( it == _mesh.cend() )
                    // (2.2) If s > mesh[end], return the last element 
                    return _values.back();
                else
                {
                    // (2.3) Blend using a linear piecewise interpolation: 
                    //       xi=0 for left checkpoint, xi=1 for right checkpoint
                    const scalar xi      = (s - (it-1)->first)/(it->first - (it-1)->first);
                    return _values[(it-1)->second] + xi*(_values[it->second] - _values[(it-1)->second]);
                }
            }
        }
        
        //! Get the values
        const std::vector<Timeseries_t>& get_values() const { return _values; }
              std::vector<Timeseries_t>& get_values()       { return _values; }

        //! Get the parameter path
        const std::string& get_path() const { return _path; }

        //! Get the parameter aliases
        const std::vector<std::string>& get_aliases() const { return _aliases; }

     private:
        std::string _path;                           // Path to the parameter. To compute gradients w.r.t. it, make sure it points to a CppAD variable
        std::vector<std::string>  _aliases;          // Aliases 
        std::vector<Timeseries_t> _values;           // Values of the parameter
        std::vector<std::pair<scalar,size_t>> _mesh; // (Optional) mesh for varying parameters: (arclength, parameter_index)
    };

    struct Parameters : public std::vector<Parameter>
    {
        using base_type =  std::vector<Parameter>;
        size_t get_number_of_parameters() const { return std::accumulate(base_type::cbegin(), base_type::cend(), 0, [](const auto& lhs, const auto& rhs) -> auto { return lhs + rhs.get_values().size(); }); }

        std::vector<scalar> get_all_parameters_as_scalar() const
        {
            std::vector<scalar> p(get_number_of_parameters());
    
            auto it_p = p.begin();
            for (const auto& parameter : *this)
            {
                std::transform(parameter.get_values().cbegin(), parameter.get_values().cend(), it_p, [](const auto& p) -> auto { return Value(p); } );
                it_p += parameter.get_values().size();
            }

            assert(it_p == p.cend());

            return p;
        }

        std::vector<std::string> get_all_parameters_aliases() const
        {
            std::vector<std::string> p(get_number_of_parameters());
    
            auto it_p = p.begin();
            for (const auto& parameter : *this)
            {
                std::copy(parameter.get_aliases().cbegin(), parameter.get_aliases().cend(), it_p);
                it_p += parameter.get_values().size();
            }

            assert(it_p == p.cend());

            return p;
        }


        void set_all_parameters(const std::vector<Timeseries_t>& inputs)
        {
            assert(inputs.size() == get_number_of_parameters());

            auto it_p = inputs.cbegin();
            for (auto& parameter : *this)
            {
                std::copy(it_p, it_p+parameter.get_values().size(), parameter.get_values().begin());
                it_p += parameter.get_values().size();
            }

            assert(it_p == inputs.cend());
        }
    };

    Parameters _parameters; //! Parameters that can be used to have spatially varying parameters, and/or to compute sensitivities

 public:

    // Get a constant reference to the parameters
    const Parameters& get_parameters() const { return _parameters; }

 protected:
    Parameters& get_parameters() { return _parameters; }

    
};

#endif
