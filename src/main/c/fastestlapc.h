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

#ifdef __cplusplus
#define STRUCT struct
#define ENUM enum
#else
#define STRUCT typedef struct
#define ENUM typedef enum
#include <stdbool.h>
#endif


#ifdef __cplusplus
extern "C" {
#endif

ENUM c_Vehicle_type { LOT2016KART, LIMEBEER2014F1 };

STRUCT c_Track
{
    char* name;
    char* track_file;
    bool is_closed;
};

extern fastestlapc_API void set_print_level(int print_level);

// Factories -----------------------------------------------------------------------------------------------------------
extern fastestlapc_API void create_vehicle(const char* vehicle_name, const char* vehicle_type, const char* database_file);

extern fastestlapc_API void create_track(struct c_Track* track, const char* name, const char* track_file, const char* options);

extern fastestlapc_API void delete_vehicle(const char* vehicle_name);

extern fastestlapc_API void clear_tables();

extern fastestlapc_API void clear_tables_by_prefix(const char* prefix_c);

// Getters -------------------------------------------------------------------------------------------------------------
extern fastestlapc_API double download_scalar_table_variable(const char* name_c);

extern fastestlapc_API int download_vector_table_variable_size(const char* name_c);

extern fastestlapc_API void download_vector_table_variable(double* data, const int n, const char* name_c);

extern fastestlapc_API void load_vector_table_variable(double* data, const int n, const char* name_c);

extern fastestlapc_API double get_vehicle_property(const char* vehicle_name, const double* q, const double* qa, const double* u, const double s, const char* property_name);

extern fastestlapc_API void save_vehicle_as_xml(const char* vehicle_name, const char* file_name);

// Modifyers -----------------------------------------------------------------------------------------------------------
extern fastestlapc_API void set_scalar_parameter(const char* vehicle_name, const char* parameter, const double value);

extern fastestlapc_API void set_vector_parameter(const char* vehicle_name, const char* parameter, const double value[3]);

extern fastestlapc_API void set_matrix_parameter(const char* vehicle_name, const char* parameter, const double value[9]);

extern fastestlapc_API void add_variable_parameter(const char* c_vehicle, const char* parameter_name, const int n, const double* s, const double* values);

extern fastestlapc_API void change_track(const char* c_vehicle, const struct c_Track* c_track);

// Applications --------------------------------------------------------------------------------------------------------
//void vehicle_equations(double* dqdt, double* dqa, const char* vehicle_name, double* q, double* qa, double* u, double s);

extern fastestlapc_API void propagate(double* q, double* qa, double* u, const char* vehicle_name, struct c_Track* track, double s, double ds, double* u_next, bool use_circuit, const char* options);

extern fastestlapc_API void gg_diagram(double* ay, double* ax_max, double* ax_min, const char* vehicle_name, double v, const int n_points);

extern fastestlapc_API void optimal_laptime(const char* c_vehicle, const struct c_Track* c_track, const int n_points, const double* s, const char* options);

extern fastestlapc_API void track_coordinates(double* x_center, double* y_center, double* x_left, double* y_left, double* x_right, double* y_right, double* theta, struct c_Track* c_track, const int n_points, const double* s);


#ifdef __cplusplus
}
#endif

#endif
