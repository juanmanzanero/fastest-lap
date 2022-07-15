#ifndef FASTEST_LAP_CLIB_H__
#define FASTEST_LAP_CLIB_H__


#ifdef _MSC_VER
#ifdef fastestlapc_EXPORTS
#define fastestlapc_API __declspec(dllexport)
#else
#define fastestlapc_API __declspec(dllimport)
#endif

#elif (defined (__GNUC__) || defined (__GNUG__))
#ifdef fastestlapc_EXPORTS
#define fastestlapc_API __attribute__ ((visibility ("default")))
#else
#define fastestlapc_API
#endif

#else
#define fastestlapc_API
#endif

#ifndef __cplusplus
#include <stdbool.h>
#endif


#ifdef __cplusplus
extern "C" {
#endif

// Printing ------------------------------------------------------------------------------------------------------------

extern fastestlapc_API void set_print_level(int print_level);

extern fastestlapc_API void print_variables(); 

extern fastestlapc_API void print_variable(const char* variable_name);

extern fastestlapc_API void print_variable_to_string(char* str_out, const int n_char, const char* variable_name);

// Factories -----------------------------------------------------------------------------------------------------------

extern fastestlapc_API void create_vehicle_from_xml(const char* vehicle_name, const char* database_file);

extern fastestlapc_API void create_vehicle_empty(const char* vehicle_name, const char* vehicle_type);

extern fastestlapc_API void create_track_from_xml(const char* name, const char* track_file); // [TEST OK]

extern fastestlapc_API void create_vector(const char* name, const int n, double* data);

extern fastestlapc_API void create_scalar(const char* name, double value);

extern fastestlapc_API void copy_variable(const char* old_name, const char* new_name);

extern fastestlapc_API void move_variable(const char* old_name, const char* new_name);

// Destructors ---------------------------------------------------------------------------------------------------------

extern fastestlapc_API void delete_variables();

extern fastestlapc_API void delete_variable(const char* prefix_c);

extern fastestlapc_API void delete_variables_by_prefix(const char* prefix_c);

// Getters -------------------------------------------------------------------------------------------------------------

extern fastestlapc_API void variable_type(char* variable_type, const int str_len_max, const char* variable_name);

extern fastestlapc_API double download_scalar(const char* name_c);

extern fastestlapc_API int download_vector_size(const char* name_c);

extern fastestlapc_API void download_vector(double* data, const int n, const char* name_c);

extern fastestlapc_API void vehicle_type_get_sizes(int* n_state, int* n_algebraic, int* n_control, int* n_outputs, const char* c_vehicle_type_name);

extern fastestlapc_API void vehicle_type_get_names(char* key_name, char* state_names[], char* algebraic_state_names[], char* control_names[], char* output_names[], const int n_char, const char* vehicle_type_name);

extern fastestlapc_API double vehicle_get_output(const char* vehicle_name, const double* q, const double* qa, const double* u, const double s, const char* property_name);

extern fastestlapc_API void vehicle_save_as_xml(const char* vehicle_name, const char* file_name);

extern fastestlapc_API int track_download_number_of_points(const char* track_name); // [TEST OK]

extern fastestlapc_API void track_download_data(double* data, const char* track_name, const int n, const char* variable_name_c); // [TEST OK]

extern fastestlapc_API double track_download_length(const char* track_name);

// Modifyers -----------------------------------------------------------------------------------------------------------

extern fastestlapc_API void vehicle_set_parameter(const char* vehicle_name, const char* parameter, const double value);      // [TEST OK]

extern fastestlapc_API void vehicle_declare_new_constant_parameter(const char* c_vehicle_name, const char* parameter_path, 
    const char* parameter_alias, const double parameter_value);

extern fastestlapc_API void vehicle_declare_new_variable_parameter(const char* c_vehicle_name, const char* parameter_path, 
    const char* parameter_alias, const int n_parameters, const double* parameter_values,
    const int mesh_size, const int* mesh_parameter_indexes, const double* mesh_points);

extern fastestlapc_API void vehicle_change_track(const char* c_vehicle, const char* c_track);       // [TEST OK]

// Applications --------------------------------------------------------------------------------------------------------

extern fastestlapc_API void propagate_vehicle(double* q, double* qa, double* u, const char* vehicle_name, const char* track_name, double s, double ds, double* u_next, bool use_circuit, const char* options);

extern fastestlapc_API void gg_diagram(double* ay, double* ax_max, double* ax_min, const char* vehicle_name, double v, const int n_points);

extern fastestlapc_API void optimal_laptime(const char* c_vehicle, const char* c_track_name, const int n_points, const double* s, const char* options);

extern fastestlapc_API void circuit_preprocessor(const char* options);

//void vehicle_equations(double* dqdt, double* dqa, const char* vehicle_name, double* q, double* qa, double* u, double s);


#ifdef __cplusplus
}
#endif

#endif
