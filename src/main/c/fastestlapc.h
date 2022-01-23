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
ENUM c_Track_type {BY_ARCS, BY_POLYNOMIAL};

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
    int format;
    double scale;
    bool is_closed;
};

STRUCT c_Channel
{
    double* data;
    char*   name;
};


// Factories -----------------------------------------------------------------------------------------------------------
void create_vehicle(struct c_Vehicle* vehicle, const char* name, const char* database_file);

void create_track(struct c_Track* track, const char* name, const char* track_file, const double scale);

// Applications --------------------------------------------------------------------------------------------------------
void vehicle_equations(double* dqdt, double* dqa, struct c_Vehicle* vehicle, double* q, double* qa, double* u, double s);

void gg_diagram(double* ay, double* ax_max, double* ax_min, struct c_Vehicle* vehicle, double v, const int n_points);

void optimal_laptime(struct c_Channel* channels, struct c_Vehicle* c_vehicle, const struct c_Track* c_track, const double width, const int n_points, const int n_channels);

void track_coordinates(double* x_center, double* y_center, double* x_left, double* y_left, double* x_right, double* y_right, double* theta, struct c_Track* c_track, const double width, const int n_points);

#ifdef __cplusplus
}
#endif
