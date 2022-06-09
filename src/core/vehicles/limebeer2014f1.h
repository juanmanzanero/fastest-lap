#ifndef __LOT2013KART_H__
#define __LOT2013KART_H__

#include "src/core/tire/tire_pacejka.h"
#include "src/core/chassis/axle_car_3dof.h"
#include "src/core/chassis/chassis_car_3dof.h"
#include "src/core/vehicles/track_by_polynomial.h"
#include "src/core/vehicles/track_by_arcs.h"
#include "src/core/vehicles/road_cartesian.h"
#include "src/core/vehicles/road_curvilinear.h"
#include "src/core/vehicles/dynamic_model_car.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"

template<typename Timeseries_t>
class limebeer2014f1
{
 public:
    limebeer2014f1() = delete;

    using Front_left_tire_type  = Tire_pacejka_simple<Timeseries_t,0,0>;
    using Front_right_tire_type = Tire_pacejka_simple<Timeseries_t,Front_left_tire_type::STATE_END,Front_left_tire_type::CONTROL_END>;
    using Rear_left_tire_type   = Tire_pacejka_simple<Timeseries_t,Front_right_tire_type::STATE_END,Front_right_tire_type::CONTROL_END>;
    using Rear_right_tire_type  = Tire_pacejka_simple<Timeseries_t,Rear_left_tire_type::STATE_END,Rear_left_tire_type::CONTROL_END>;

    using Front_axle_t          = Axle_car_3dof<Timeseries_t,Front_left_tire_type,Front_right_tire_type,STEERING,Rear_right_tire_type::STATE_END,Rear_right_tire_type::CONTROL_END>;
    using Rear_axle_t           = Axle_car_3dof<Timeseries_t,Rear_left_tire_type,Rear_right_tire_type,POWERED,Front_axle_t::STATE_END,Front_axle_t::CONTROL_END>;
    using Chassis_t             = Chassis_car_3dof<Timeseries_t,Front_axle_t,Rear_axle_t,Rear_axle_t::STATE_END,Rear_axle_t::CONTROL_END>;

    using Road_cartesian_t   = Road_cartesian<Timeseries_t,Chassis_t::STATE_END,Chassis_t::CONTROL_END>;

    template<typename Track_t>
    using Road_curvilinear_t = Road_curvilinear<Timeseries_t,Track_t,Chassis_t::STATE_END,Chassis_t::CONTROL_END>;
 
 private:
      
    template<typename Road_type>
    class Dynamic_model : public Dynamic_model_car<Timeseries_t,Chassis_t,Road_type,Road_type::STATE_END,Road_type::CONTROL_END>
    {
     public:
        using Road_t          = Road_type;
        using Dynamic_model_t = Dynamic_model_car<Timeseries_t,Chassis_t, Road_type, Road_type::STATE_END, Road_type::CONTROL_END>;

        Dynamic_model(const Road_t& road = Road_t()) : Dynamic_model_t(road) {}
        Dynamic_model(Xml_document& database, const Road_t& road = Road_t()) : Dynamic_model_t(database,road) {}

        // Steady-state computation
        static constexpr const size_t N_SS_VARS = 11;
        static constexpr const size_t N_SS_EQNS = 14;

        // Factor to scale the acceleration on the fitness function
        static constexpr const scalar acceleration_scaling = g0;

        // The content of x is: x = [kappa_fl, kappa_fr, kappa_rl, kappa_rr, psi, Fz_fl, Fz_fr, Fz_rl, Fz_rr, delta, throttle]
        static std::vector<scalar> steady_state_initial_guess()
        {
            return {0.0, 0.0, 0.0, 0.0, 0.0, -0.25, -0.25, -0.25, -0.25, 0.0, 0.0};
        }

        static std::pair<std::vector<scalar>,std::vector<scalar>> steady_state_variable_bounds() 
        {
            return { { -0.085, -0.085, -0.085, -0.085, -10.0*DEG, -3.0, -3.0, -3.0, -3.0, -10.0*DEG, -1.0},
                     {  0.085,  0.085,  0.085,  0.085,  10.0*DEG,  1.0,  1.0,  1.0,  1.0,  10.0*DEG,  1.0} };
        }

        static std::pair<std::vector<scalar>,std::vector<scalar>> steady_state_variable_bounds_accelerate() 
        {
            return { { -0.04, -0.04, -0.08, -0.08, -10.0*DEG, -3.0, -3.0, -3.0, -3.0, -10.0*DEG, -0.1},
                     {  0.04,  0.04,  0.085,  0.085,  10.0*DEG,  1.0,  1.0,  1.0,  1.0,  10.0*DEG,  1.0} };
        }

        static std::pair<std::vector<scalar>,std::vector<scalar>> steady_state_variable_bounds_brake() 
        {
            return { { -0.170, -0.170, -0.170, -0.170, -1.0e-2*DEG, -3.0, -3.0, -3.0, -3.0, -4.0*DEG, -1.0},
                     {  0.04,  0.04,  0.04,  0.04,  10.0*DEG,  1.0,  1.0,  1.0,  1.0,  10.0*DEG,  0.5} };
        }


        static std::pair<std::vector<scalar>,std::vector<scalar>> steady_state_constraint_bounds() 
        {
            return { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.090, -0.090, -0.090, -0.090},
                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.090,  0.090,  0.090,  0.090} };
        }

        template<typename T>
        static std::vector<T> get_x(const std::array<T,Dynamic_model_t::NSTATE>& q,
                                         const std::array<T,Dynamic_model_t::NALGEBRAIC>& qa,
                                         const std::array<T,Dynamic_model_t::NCONTROL>& u,
                                         scalar v) 
        {
            return { q[Dynamic_model_t::Chassis_type::front_axle_type::IKAPPA_LEFT],
                     q[Dynamic_model_t::Chassis_type::front_axle_type::IKAPPA_RIGHT],
                     q[Dynamic_model_t::Chassis_type::rear_axle_type::IKAPPA_LEFT],
                     q[Dynamic_model_t::Chassis_type::rear_axle_type::IKAPPA_RIGHT],
                     q[Dynamic_model_t::Road_type::IPSI],
                     qa[Dynamic_model_t::Chassis_type::IFZFL],
                     qa[Dynamic_model_t::Chassis_type::IFZFR],
                     qa[Dynamic_model_t::Chassis_type::IFZRL],
                     qa[Dynamic_model_t::Chassis_type::IFZRR],
                     u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING],
                     u[Dynamic_model_t::Chassis_type::ITHROTTLE]
                    };
        }

        std::tuple<std::array<Timeseries_t,N_SS_EQNS>,
                   std::array<Timeseries_t,Dynamic_model_t::NSTATE>,
                   std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC>,
                   std::array<Timeseries_t,Dynamic_model_t::NCONTROL>>
            steady_state_equations(const std::array<Timeseries_t,N_SS_VARS>& x, 
                                   const Timeseries_t& ax, 
                                   const Timeseries_t& ay, 
                                   const Timeseries_t& v)
        {
            // The content of x is: x = [kappa_fl, kappa_fr, kappa_rl, kappa_rr, psi, Fz_fl, Fz_fr, Fz_rl, Fz_rr, delta, throttle]
            // Construct state and controls
            const Timeseries_t& psi = x[4];
            const Timeseries_t omega = ay/v;
             
            // Construct the state
            std::array<Timeseries_t,Dynamic_model_t::NSTATE> q;
            q[Dynamic_model_t::Chassis_type::front_axle_type::IKAPPA_LEFT]  = x[0];
            q[Dynamic_model_t::Chassis_type::front_axle_type::IKAPPA_RIGHT] = x[1];
            q[Dynamic_model_t::Chassis_type::rear_axle_type::IKAPPA_LEFT]   = x[2];
            q[Dynamic_model_t::Chassis_type::rear_axle_type::IKAPPA_RIGHT]  = x[3];
            q[Dynamic_model_t::Chassis_type::IU]                            = v*cos(psi);
            q[Dynamic_model_t::Chassis_type::IV]                            = -v*sin(psi);
            q[Dynamic_model_t::Chassis_type::IOMEGA]                        = omega;
            q[Dynamic_model_t::Road_type::IX]                               = 0.0;
            q[Dynamic_model_t::Road_type::IY]                               = 0.0;
            q[Dynamic_model_t::Road_type::IPSI]                             = x[4];

            // Constract the algebraic variables
            std::array<Timeseries_t,Dynamic_model_t::NALGEBRAIC> qa;
            qa[Dynamic_model_t::Chassis_type::IFZFL] = x[5];
            qa[Dynamic_model_t::Chassis_type::IFZFR] = x[6];
            qa[Dynamic_model_t::Chassis_type::IFZRL] = x[7];
            qa[Dynamic_model_t::Chassis_type::IFZRR] = x[8];
        
            // Construct the controls
            std::array<scalar,Dynamic_model_t::NCONTROL> u_def = this->get_state_and_control_upper_lower_and_default_values().u_def;
            std::array<Timeseries_t,Dynamic_model_t::NCONTROL> u;
            std::copy(u_def.cbegin(), u_def.cend(), u.begin());
    
            u[Dynamic_model_t::Chassis_type::front_axle_type::ISTEERING] = x[9];
            u[Dynamic_model_t::Chassis_type::ITHROTTLE]    = x[10];
        
            // Compute time derivative
            auto [dqdt,dqa] = (*this)(q,qa,u,0.0);
        
            // Compute constraints
            std::array<Timeseries_t,N_SS_EQNS> constraints;
        
            constraints[0] = dqa[0];
            constraints[1] = dqa[1];
            constraints[2] = dqa[2];
            constraints[3] = dqa[3];
            constraints[4] = (dqdt[Dynamic_model_t::Chassis_type::IIDU]*sin(psi)
                            + dqdt[Dynamic_model_t::Chassis_type::IIDV]*cos(psi))/g0;
            constraints[5] = (ax - dqdt[Dynamic_model_t::Chassis_type::IIDU]*cos(psi)
                                 + dqdt[Dynamic_model_t::Chassis_type::IIDV]*sin(psi))/g0;

            constraints[6] = this->get_chassis().get_front_axle().get_kappa_left_derivative()/9.81*660.0;
            constraints[7] = this->get_chassis().get_front_axle().get_kappa_right_derivative()/9.81*660.0;
            constraints[8] = this->get_chassis().get_rear_axle().get_kappa_left_derivative()/9.81*660.0;
            constraints[9] = this->get_chassis().get_rear_axle().get_kappa_right_derivative()/9.81*660.0;
        
            constraints[10] = this->get_chassis().get_front_axle().template get_tire<0>().get_lambda();
            constraints[11] = this->get_chassis().get_front_axle().template get_tire<1>().get_lambda();
            constraints[12] = this->get_chassis().get_rear_axle().template get_tire<0>().get_lambda();
            constraints[13] = this->get_chassis().get_rear_axle().template get_tire<1>().get_lambda();

            return {constraints,q,qa,u};
        }

        // Optimal lap-time --------------------------------------------------------
        static constexpr const size_t N_OL_EXTRA_CONSTRAINTS = 6;    //! The number of tire constraints: lambda_fl, lambda_fr, lambda_rl, lambda_rr
                                                                     //! + the real track limits: -wL < n + sign(n).t.cos(alpha) < wR

        std::tuple<std::vector<scalar>,std::vector<scalar>> optimal_laptime_derivative_control_bounds() const
        {
            return {{-20.0*DEG,-10.0}, {20.0*DEG, 10.0}};
        }

        std::pair<std::vector<scalar>,std::vector<scalar>> optimal_laptime_extra_constraints_bounds(const scalar s) const
        {
            const auto wl = this->get_road().get_left_track_limit(s);
            const auto wr = this->get_road().get_right_track_limit(s);
            return 
            {
                {-0.11,-0.11,-0.11,-0.11,-wl,-wl},
                {0.11,0.11,0.11,0.11,wr,wr}
            };
        }

        std::array<Timeseries_t,N_OL_EXTRA_CONSTRAINTS> optimal_laptime_extra_constraints() const
        {
            const auto& n = this->get_road().get_n();
            const auto& alpha = this->get_road().get_alpha();
            const auto& track = this->get_chassis().get_front_axle().get_track();
    
            return 
            {
                this->get_chassis().get_front_axle().template get_tire<0>().get_lambda(),
                this->get_chassis().get_front_axle().template get_tire<1>().get_lambda(),
                this->get_chassis().get_rear_axle().template get_tire<0>().get_lambda(),
                this->get_chassis().get_rear_axle().template get_tire<1>().get_lambda(),
                n + 0.5*track*cos(alpha),
                n - 0.5*track*cos(alpha),
            };
        }

        // Integral quantities
        struct Integral_quantities
        {
            enum { IENGINE_POWER, IFRONT_LEFT_TIRE_ENERGY, IFRONT_RIGHT_TIRE_ENERGY,
                                  IREAR_LEFT_TIRE_ENERGY, IREAR_RIGHT_TIRE_ENERGY, N_INTEGRAL_QUANTITIES };

            inline const static std::vector<std::string> names = {"engine-energy","tire-fl-energy","tire-fr-energy","tire-rl-energy","tire-rr-energy"};

        };

        std::array<Timeseries_t,Integral_quantities::N_INTEGRAL_QUANTITIES> compute_integral_quantities() const
        {
            std::array<Timeseries_t,Integral_quantities::N_INTEGRAL_QUANTITIES> outputs;

            outputs[Integral_quantities::IENGINE_POWER] = this->get_chassis().get_rear_axle().get_engine().get_power()*1.0e-6;
            
            outputs[Integral_quantities::IFRONT_LEFT_TIRE_ENERGY]  = -this->get_chassis().get_front_axle().template get_tire<0>().get_dissipation()*1.0e-6;

            outputs[Integral_quantities::IFRONT_RIGHT_TIRE_ENERGY] = -this->get_chassis().get_front_axle().template get_tire<1>().get_dissipation()*1.0e-6;

            outputs[Integral_quantities::IREAR_LEFT_TIRE_ENERGY]   = -this->get_chassis().get_rear_axle().template get_tire<0>().get_dissipation()*1.0e-6;

            outputs[Integral_quantities::IREAR_RIGHT_TIRE_ENERGY]  = -this->get_chassis().get_rear_axle().template get_tire<1>().get_dissipation()*1.0e-6;

            return outputs;
        }

    };

 public:
    
    using cartesian = Dynamic_model<Road_cartesian_t>;

    template<typename Track_t>
    using curvilinear = Dynamic_model<Road_curvilinear_t<Track_t>>;

    using curvilinear_p = curvilinear<Track_by_polynomial>;
    using curvilinear_a = curvilinear<Track_by_arcs>;
};

struct limebeer2014f1_all
{
    limebeer2014f1_all(Xml_document& database_xml)
    : cartesian_scalar(database_xml), 
      curvilinear_scalar(database_xml),
      cartesian_ad(database_xml),
      curvilinear_ad(database_xml)  
    {}

    limebeer2014f1_all()
    : cartesian_scalar(), 
      curvilinear_scalar(),
      cartesian_ad(),
      curvilinear_ad()  
    {}

    using vehicle_scalar_curvilinear = limebeer2014f1<scalar>::curvilinear_p;
    using vehicle_ad_curvilinear = limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p;

    // Get curvilinear AD car for the polynomial track
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p& get_curvilinear_ad_car() { return curvilinear_ad; }

    // Get curvilinear scalar car for the polynomial track
    limebeer2014f1<scalar>::curvilinear_p& get_curvilinear_scalar_car() { return curvilinear_scalar; }

    template<typename T>
    void set_parameter(const std::string& parameter, const T value)
    {
        cartesian_scalar.set_parameter(parameter, value);
        curvilinear_scalar.set_parameter(parameter, value);

        cartesian_ad.set_parameter(parameter, value);
        curvilinear_ad.set_parameter(parameter, value);
    }

    template<typename ... Args> 
    void add_parameter(const std::string& parameter_name, Args&& ... args)
    {
        cartesian_scalar.add_parameter(parameter_name, std::forward<Args>(args)...);
        curvilinear_scalar.add_parameter(parameter_name, std::forward<Args>(args)...);

        cartesian_ad.add_parameter(parameter_name, std::forward<Args>(args)...);
        curvilinear_ad.add_parameter(parameter_name, std::forward<Args>(args)...);
    }


    limebeer2014f1<scalar>::cartesian                 cartesian_scalar;
    limebeer2014f1<scalar>::curvilinear_p             curvilinear_scalar;
    limebeer2014f1<CppAD::AD<scalar>>::cartesian      cartesian_ad;
    limebeer2014f1<CppAD::AD<scalar>>::curvilinear_p  curvilinear_ad;
};

#endif
