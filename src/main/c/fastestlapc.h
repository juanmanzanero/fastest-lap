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

STRUCT c_Vehicle
{
    char* name;
    int type;
    char* database_file;
};


STRUCT c_Track
{
    char* name;
    char* track_file;
    bool is_closed;
};


// Factories -----------------------------------------------------------------------------------------------------------
void create_vehicle(struct c_Vehicle* vehicle, const char* name, const char* vehicle_type, const char* database_file);

void create_track(struct c_Track* track, const char* name, const char* track_file, const char* options);

void delete_vehicle(struct c_Vehicle* vehicle);

void clear_tables();

void clear_tables_by_prefix(const char* prefix_c);

// Getters -------------------------------------------------------------------------------------------------------------
double download_scalar_table_variable(const char* name_c);

int download_vector_table_variable_size(const char* name_c);

void download_vector_table_variable(double* data, const int n, const char* name_c);

void load_vector_table_variable(double* data, const int n, const char* name_c);

double get_vehicle_property(struct c_Vehicle* vehicle, const double* q, const double* qa, const double* u, const double s, const char* property_name);

void save_vehicle_as_xml(struct c_Vehicle* vehicle, const char* file_name);

// Modifyers -----------------------------------------------------------------------------------------------------------
void set_scalar_parameter(struct c_Vehicle* vehicle, const char* parameter, const double value);

void set_vector_parameter(struct c_Vehicle* vehicle, const char* parameter, const double value[3]);

void set_matrix_parameter(struct c_Vehicle* vehicle, const char* parameter, const double value[9]);

void add_variable_parameter(struct c_Vehicle* c_vehicle, const char* parameter_name, const int n, const double* s, const double* values);

void change_track(struct c_Vehicle* c_vehicle, const struct c_Track* c_track);

// Applications --------------------------------------------------------------------------------------------------------
//void vehicle_equations(double* dqdt, double* dqa, struct c_Vehicle* vehicle, double* q, double* qa, double* u, double s);

void propagate(double* q, double* qa, double* u, struct c_Vehicle* vehicle, struct c_Track* track, double s, double ds, double* u_next, bool use_circuit, const char* options);

void gg_diagram(double* ay, double* ax_max, double* ax_min, struct c_Vehicle* vehicle, double v, const int n_points);

void optimal_laptime(struct c_Vehicle* c_vehicle, const struct c_Track* c_track, const int n_points, const double* s, const char* options);

void track_coordinates(double* x_center, double* y_center, double* x_left, double* y_left, double* x_right, double* y_right, double* theta, struct c_Track* c_track, const int n_points);

#ifdef __cplusplus
}
#endif
