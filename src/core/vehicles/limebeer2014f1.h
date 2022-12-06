#ifndef LIMEBEER2014F1_H
#define LIMEBEER2014F1_H

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

    using Front_left_tire_type  = Tire_pacejka_simple<Timeseries_t,0,0,0>;
    using Front_right_tire_type = Tire_pacejka_simple<Timeseries_t,Front_left_tire_type::state_names::end,Front_left_tire_type::algebraic_state_names::end, Front_left_tire_type::control_names::end>;
    using Rear_left_tire_type   = Tire_pacejka_simple<Timeseries_t,Front_right_tire_type::state_names::end,Front_right_tire_type::algebraic_state_names::end, Front_right_tire_type::control_names::end>;
    using Rear_right_tire_type  = Tire_pacejka_simple<Timeseries_t,Rear_left_tire_type::state_names::end,Rear_left_tire_type::algebraic_state_names::end, Rear_left_tire_type::control_names::end>;

    using Front_axle_t          = Axle_car_3dof<Timeseries_t,Front_left_tire_type,Front_right_tire_type,STEERING,Rear_right_tire_type::state_names::end,Rear_right_tire_type::algebraic_state_names::end,Rear_right_tire_type::control_names::end>;
    using Rear_axle_t           = Axle_car_3dof<Timeseries_t,Rear_left_tire_type,Rear_right_tire_type,POWERED,Front_axle_t::Axle_type::state_names::end,Front_axle_t::Axle_type::algebraic_state_names::end,Front_axle_t::Axle_type::control_names::end>;
    using Chassis_t             = Chassis_car_3dof<Timeseries_t,Front_axle_t,Rear_axle_t,Rear_axle_t::Axle_type::state_names::end,Rear_axle_t::Axle_type::algebraic_state_names::end,Rear_axle_t::Axle_type::control_names::end>;

    using Road_cartesian_t   = Road_cartesian<Timeseries_t,Chassis_t::state_names::end,Chassis_t::algebraic_state_names::end,Chassis_t::control_names::end>;

    template<typename Track_t>
    using Road_curvilinear_t = Road_curvilinear<Timeseries_t,Track_t,Chassis_t::state_names::end,Chassis_t::algebraic_state_names::end,Chassis_t::control_names::end>;
 
 private:
      
    template<typename Road_type>
    class Dynamic_model : public Dynamic_model_car<Timeseries_t,Chassis_t,Road_type>
    {
     public:
        using Road_t          = Road_type;
        using Dynamic_model_t = Dynamic_model_car<Timeseries_t,Chassis_t, Road_type>;

        Dynamic_model(const Road_t& road = Road_t()) : Dynamic_model_t(road) {}
        Dynamic_model(Xml_document& database, const Road_t& road = Road_t()) : Dynamic_model_t(database,road) {}

        // Steady-state computation
        static constexpr const size_t number_of_steady_state_variables = 11;
        static constexpr const size_t number_of_steady_state_equations = 15;

        // Factor to scale the acceleration on the fitness function
        static constexpr const scalar acceleration_units = g0;

        static constexpr const scalar maximum_yaw = 10.0*DEG;
        static constexpr const scalar maximum_steering = 10.0*DEG;

        // The content of x is: x = [kappa_fl, kappa_fr, kappa_rl, kappa_rr, psi, Fz_fl, Fz_fr, Fz_rl, Fz_rr, delta, throttle]
        static std::vector<scalar> steady_state_initial_guess()
        {
            return {0.0, 0.0, 0.0, 0.0, 0.0, -0.25, -0.25, -0.25, -0.25, 0.0, 0.0};
        }

        static std::pair<std::vector<scalar>,std::vector<scalar>> steady_state_variable_bounds() 
        {
            return { { -1.15, -1.15, -1.15, -1.15, 0.0, -2.0, -2.0, -2.0, -2.0, 0.0, -1.0},
                     {  0.25,  0.25,  1.00,  1.00, 1.5,  0.1,  0.1,  0.1,  0.1, 1.5,  1.0} };
        }

        static std::pair<std::vector<scalar>,std::vector<scalar>> steady_state_variable_bounds_accelerate() 
        {
            return steady_state_variable_bounds();
        }

        static std::pair<std::vector<scalar>,std::vector<scalar>> steady_state_variable_bounds_brake() 
        {
            return { { -1.15, -1.15, -1.30, -1.30, 0.0, -2.0, -2.0, -2.0, -2.0, 0.0, -1.0},
                     {  0.25,  0.25,  0.25,  0.25, 1.0,  0.1,  0.1,  0.1,  0.1, 1.0,  0.5} };
        }

        static std::pair<std::vector<scalar>,std::vector<scalar>> steady_state_constraint_bounds() 
        {
            return { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0, -1.0, -1.0},
                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  1.2,  1.2,  1.2,  1.2}};
        }

        std::vector<scalar> get_x(const std::array<scalar,Dynamic_model_t::number_of_inputs>& q,
                                  const std::array<scalar,Dynamic_model_t::number_of_controls>& u,
                                  scalar v) const
        {
            return { q[Dynamic_model_t::Chassis_type::front_axle_type::input_names::KAPPA_LEFT],
                     q[Dynamic_model_t::Chassis_type::front_axle_type::input_names::KAPPA_RIGHT],
                     q[Dynamic_model_t::Chassis_type::rear_axle_type::input_names::KAPPA_LEFT],
                     q[Dynamic_model_t::Chassis_type::rear_axle_type::input_names::KAPPA_RIGHT],
                     q[Dynamic_model_t::Road_type::input_names::PSI]/maximum_yaw,
                     q[Dynamic_model_t::Chassis_type::input_names::force_z_fl_g],
                     q[Dynamic_model_t::Chassis_type::input_names::force_z_fr_g],
                     q[Dynamic_model_t::Chassis_type::input_names::force_z_rl_g],
                     q[Dynamic_model_t::Chassis_type::input_names::force_z_rr_g],
                     u[Dynamic_model_t::Chassis_type::front_axle_type::control_names::STEERING]/maximum_steering,
                     u[Dynamic_model_t::Chassis_type::control_names::throttle] 
                    };
        }

        std::tuple<std::array<Timeseries_t,number_of_steady_state_equations>,
                   std::array<Timeseries_t,Dynamic_model_t::number_of_inputs>,
                   std::array<Timeseries_t,Dynamic_model_t::number_of_controls>>
            steady_state_equations(const std::array<Timeseries_t,number_of_steady_state_variables>& x, 
                                   const Timeseries_t& longitudinal_acceleration, 
                                   const Timeseries_t& lateral_acceleration, 
                                   const Timeseries_t& velocity_mps)
        {
            // The content of x is: x = [kappa_fl, kappa_fr, kappa_rl, kappa_rr, psi, Fz_fl, Fz_fr, Fz_rl, Fz_rr, delta, throttle]

            // Construct state and controls
            const Timeseries_t& psi = maximum_yaw*x[4];
            const Timeseries_t omega = (lateral_acceleration*acceleration_units)/velocity_mps;

            // Constract the algebraic variables
            std::array<Timeseries_t,Dynamic_model_t::number_of_inputs> q;
            q[Dynamic_model_t::Chassis_type::input_names::force_z_fl_g] = x[5];
            q[Dynamic_model_t::Chassis_type::input_names::force_z_fr_g] = x[6];
            q[Dynamic_model_t::Chassis_type::input_names::force_z_rl_g] = x[7];
            q[Dynamic_model_t::Chassis_type::input_names::force_z_rr_g] = x[8];

            const auto& m = this->get_chassis().get_mass();
             
            const auto lambda_max_fl = this->get_chassis().get_front_axle().template get_tire<0>().get_model().maximum_lambda(-q[Dynamic_model_t::Chassis_type::input_names::force_z_fl_g]*m*g0);
            const auto lambda_max_fr = this->get_chassis().get_front_axle().template get_tire<1>().get_model().maximum_lambda(-q[Dynamic_model_t::Chassis_type::input_names::force_z_fr_g]*m*g0);
            const auto lambda_max_rl = this->get_chassis().get_rear_axle().template get_tire<0>().get_model().maximum_lambda(-q[Dynamic_model_t::Chassis_type::input_names::force_z_rl_g]*m*g0);
            const auto lambda_max_rr = this->get_chassis().get_rear_axle().template get_tire<1>().get_model().maximum_lambda(-q[Dynamic_model_t::Chassis_type::input_names::force_z_rr_g]*m*g0);

            // Construct the state
            q[Dynamic_model_t::Chassis_type::front_axle_type::input_names::KAPPA_LEFT]  = x[0];
            q[Dynamic_model_t::Chassis_type::front_axle_type::input_names::KAPPA_RIGHT] = x[1];
            q[Dynamic_model_t::Chassis_type::rear_axle_type::input_names::KAPPA_LEFT]   = x[2];
            q[Dynamic_model_t::Chassis_type::rear_axle_type::input_names::KAPPA_RIGHT]  = x[3];
            q[Dynamic_model_t::Chassis_type::input_names::com_velocity_x_mps]           = velocity_mps*cos(psi);
            q[Dynamic_model_t::Chassis_type::input_names::com_velocity_y_mps]           = -velocity_mps*sin(psi);
            q[Dynamic_model_t::Chassis_type::input_names::yaw_rate_radps]               = omega;
            q[Dynamic_model_t::Road_type::input_names::X]                               = 0.0;
            q[Dynamic_model_t::Road_type::input_names::Y]                               = 0.0;
            q[Dynamic_model_t::Road_type::input_names::PSI]                             = psi;

            // Construct the controls
            std::array<scalar,Dynamic_model_t::number_of_controls> u_def = this->get_state_and_control_upper_lower_and_default_values().controls_def;
            std::array<Timeseries_t,Dynamic_model_t::number_of_controls> u;
            std::copy(u_def.cbegin(), u_def.cend(), u.begin());
    
            u[Dynamic_model_t::Chassis_type::front_axle_type::control_names::STEERING] = x[9]*maximum_steering;
            u[Dynamic_model_t::Chassis_type::control_names::throttle]  = x[10];
        
            // Compute time derivative
            auto [states,dstates_dt,algebraic_equations] = (*this)(q,u,0.0);

            std::array<Timeseries_t,number_of_steady_state_equations> constraints;
        
            constraints[0] = algebraic_equations[0];
            constraints[1] = algebraic_equations[1];
            constraints[2] = algebraic_equations[2];
            constraints[3] = algebraic_equations[3];
            constraints[4] = (dstates_dt[Dynamic_model_t::Chassis_type::state_names::com_velocity_x_mps]*sin(psi)
                            + dstates_dt[Dynamic_model_t::Chassis_type::state_names::com_velocity_y_mps]*cos(psi))/(g0);
            constraints[5] = longitudinal_acceleration*acceleration_units/g0 + (- dstates_dt[Dynamic_model_t::Chassis_type::state_names::com_velocity_x_mps]*cos(psi)
                                 + dstates_dt[Dynamic_model_t::Chassis_type::state_names::com_velocity_y_m]*sin(psi))/(g0);

            constraints[6] = dstates_dt[Dynamic_model_t::Chassis_type::state_names::yaw_rate_radps]/g0;

            constraints[7] = this->get_chassis().get_front_axle().get_dangular_momentum_dt_left()/(g0*m);
            constraints[8] = this->get_chassis().get_front_axle().get_dangular_momentum_dt_right()/(g0*m);
            constraints[9] = this->get_chassis().get_rear_axle().get_dangular_momentum_dt_left()/(g0*m);
            constraints[10] = this->get_chassis().get_rear_axle().get_dangular_momentum_dt_right()/(g0*m);

            constraints[11] = this->get_chassis().get_front_axle().template get_tire<0>().get_lambda()/lambda_max_fl;
            constraints[12] = this->get_chassis().get_front_axle().template get_tire<1>().get_lambda()/lambda_max_fr;
            constraints[13] = this->get_chassis().get_rear_axle().template get_tire<0>().get_lambda()/lambda_max_rl;
            constraints[14] = this->get_chassis().get_rear_axle().template get_tire<1>().get_lambda()/lambda_max_rr;

            return {constraints,q,u};
        }

        // Optimal lap-time --------------------------------------------------------
        static constexpr const size_t N_OL_EXTRA_CONSTRAINTS = 6;    //! The number of tire constraints: lambda_fl, lambda_fr, lambda_rl, lambda_rr
                                                                     //! + the real track limits: -wL < n + sign(n).t.cos(alpha) < wR

        std::tuple<std::vector<scalar>,std::vector<scalar>> optimal_laptime_derivative_control_bounds() const
        {
            return {{-20.0*DEG,-10.0,-10.0}, {20.0*DEG, 10.0, 10.0}};
        }

        std::pair<std::vector<scalar>,std::vector<scalar>> optimal_laptime_extra_constraints_bounds(const scalar s) const
        {
            const auto wl = this->get_road().get_left_track_limit(s);
            const auto wr = this->get_road().get_right_track_limit(s);

            const auto& m = this->get_chassis().get_mass();

            const auto& tire_fl = this->get_chassis().get_front_axle().template get_tire<0>();
            const auto& tire_fr = this->get_chassis().get_front_axle().template get_tire<1>();
            const auto& tire_rl = this->get_chassis().get_rear_axle().template get_tire<0>();
            const auto& tire_rr = this->get_chassis().get_rear_axle().template get_tire<1>();

            const auto& lambda_max_fl_0g = Value(tire_fl.get_model().maximum_lambda(0.0));
            const auto& lambda_max_fr_0g = Value(tire_fr.get_model().maximum_lambda(0.0));
            const auto& lambda_max_rl_0g = Value(tire_rl.get_model().maximum_lambda(0.0));
            const auto& lambda_max_rr_0g = Value(tire_rr.get_model().maximum_lambda(0.0));

            const auto& lambda_max_fl_1g = Value(tire_fl.get_model().maximum_lambda(m*g0));
            const auto& lambda_max_fr_1g = Value(tire_fr.get_model().maximum_lambda(m*g0));
            const auto& lambda_max_rl_1g = Value(tire_rl.get_model().maximum_lambda(m*g0));
            const auto& lambda_max_rr_1g = Value(tire_rr.get_model().maximum_lambda(m*g0));

            const auto& lambda_max_fl = max(lambda_max_fl_0g,lambda_max_fl_1g);
            const auto& lambda_max_fr = max(lambda_max_fr_0g,lambda_max_fr_1g);
            const auto& lambda_max_rl = max(lambda_max_rl_0g,lambda_max_rl_1g);
            const auto& lambda_max_rr = max(lambda_max_rr_0g,lambda_max_rr_1g);
    
            return 
            {
                {-lambda_max_fl,-lambda_max_fr,-lambda_max_rl,-lambda_max_rr,-wl,-wl},
                { lambda_max_fl, lambda_max_fr, lambda_max_rl, lambda_max_rr, wr, wr}
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
                                  IREAR_LEFT_TIRE_ENERGY, IREAR_RIGHT_TIRE_ENERGY, IBOOST_TIME, N_INTEGRAL_QUANTITIES };

            inline const static std::vector<std::string> names = {"engine-energy","tire-fl-energy","tire-fr-energy","tire-rl-energy","tire-rr-energy","boost-time"};

        };

        std::array<Timeseries_t,Integral_quantities::N_INTEGRAL_QUANTITIES> compute_integral_quantities() const
        {
            std::array<Timeseries_t,Integral_quantities::N_INTEGRAL_QUANTITIES> outputs;

            outputs[Integral_quantities::IENGINE_POWER]            = this->get_chassis().get_rear_axle().get_engine().get_power()*1.0e-6;
            
            outputs[Integral_quantities::IFRONT_LEFT_TIRE_ENERGY]  = -this->get_chassis().get_front_axle().template get_tire<0>().get_dissipation()*1.0e-6;

            outputs[Integral_quantities::IFRONT_RIGHT_TIRE_ENERGY] = -this->get_chassis().get_front_axle().template get_tire<1>().get_dissipation()*1.0e-6;

            outputs[Integral_quantities::IREAR_LEFT_TIRE_ENERGY]   = -this->get_chassis().get_rear_axle().template get_tire<0>().get_dissipation()*1.0e-6;

            outputs[Integral_quantities::IREAR_RIGHT_TIRE_ENERGY]  = -this->get_chassis().get_rear_axle().template get_tire<1>().get_dissipation()*1.0e-6;

            outputs[Integral_quantities::IBOOST_TIME]              = this->get_chassis().get_rear_axle().get_boost();

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
