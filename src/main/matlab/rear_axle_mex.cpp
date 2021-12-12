// To be compiled from MEX
// mex -v -I/Users/juanmanzanero/Documents/software/fastest-lap/ CXXFLAGS='$CXXFLAGS -std=c++17' rear_axle_mex.cpp
//#undef DLL_EXPORT_SYM
#include "mex.hpp"
#include "mexAdapter.hpp"
#include "MatlabDataArray.hpp"
#include <vector>
#include <map>

#include "src/core/tire/tire_pacejka.h"
#include "src/core/chassis/axle_car.h"
#include "src/core/vehicles/dynamic_model_powered_axle.h"
#include "src/core/vehicles/track_run.h"
#include "src/core/chassis/chassis_car_6dof.h"
#include "lion/math/polynomial.h"

// mex types
using namespace matlab::data;
using matlab::mex::ArgumentList;


// fastest-lap types
using Rear_left_tire = Tire_pacejka<0,0>;
using Rear_right_tire = Tire_pacejka<Rear_left_tire::STATE_END,Rear_left_tire::CONTROL_END>;
using Axle_type = Axle_car<Rear_left_tire,Rear_right_tire,POWERED_WITHOUT_DIFFERENTIAL,Rear_right_tire::STATE_END,Rear_right_tire::CONTROL_END>;
using Dynamic_model_t = Dynamic_model_powered_axle<Axle_type,Axle_type::STATE_END,Axle_type::CONTROL_END>;
using Run_t           = Track_run<Dynamic_model_t>;

const static std::map<std::string,scalar> Lot2016kart_rear_axle =
{
     std::make_pair("rear-axle/track"               , 1.200      ), 
     std::make_pair("rear-axle/chassis_stiffness"   , 60.0e3     ), 
     std::make_pair("rear-axle/antiroll_stiffness"  , 0.0        ), 
     std::make_pair("rear-axle/inertia"             , 0.2        ), 
     std::make_pair("chassis/mass"                  , 165.0      ), 
     std::make_pair("chassis/rho_air"               , 1.2        ), 
     std::make_pair("chassis/CdA"                   , 0.7        ), 
     std::make_pair("rear-tire/R0"                  , 0.139      ), 
     std::make_pair("rear-tire/kt"                  , 61.3e3     ), 
     std::make_pair("rear-tire/ct"                  , 1.0e3      ), 
     std::make_pair("rear-tire/Fz0"                 , 560        ), 
     std::make_pair("rear-tire/pCx1"                , 2.3        ), 
     std::make_pair("rear-tire/pDx1"                , 0.9        ), 
     std::make_pair("rear-tire/pEx1"                , 0.95       ), 
     std::make_pair("rear-tire/pKx1"                , 20.0       ), 
     std::make_pair("rear-tire/pKx2"                , 1.0        ), 
     std::make_pair("rear-tire/pKx3"                , -0.5       ), 
     std::make_pair("rear-tire/pCy1"                , 2.3        ), 
     std::make_pair("rear-tire/pDy1"                , 1.5        ), 
     std::make_pair("rear-tire/pEy1"                , 0.9        ), 
     std::make_pair("rear-tire/pKy1"                , 37.6       ), // I have changed its sign
     std::make_pair("rear-tire/pKy2"                , 1.6        ), 
     std::make_pair("rear-tire/pKy4"                , 2.0        ), 
     std::make_pair("rear-tire/rBx1"                , 14.0       ), 
     std::make_pair("rear-tire/rCx1"                , 1.0        ), // This one I am not 100% sure
     std::make_pair("rear-tire/rBy1"                , 12.0       ), 
     std::make_pair("rear-tire/rCy1"                , 0.6        ), 
     std::make_pair("rear-tire/lambdaFz0"           , 1.6        )
};





class MexFunction : public matlab::mex::Function
{
 public:
    MexFunction()
    {
        run.set_number_of_blocks(Axle_type::ITORQUE,2);
        run.set_polynomial_order(10);
    }
    void operator()(ArgumentList outputs, ArgumentList inputs)
    {
        const int iCase = inputs[0][0]; 
        switch (iCase)
        {
         case(1): // Evaluate fitness function              
            case_fitnessfunction(outputs, inputs);
            break;
         case(2): // Evaluate torque polynomial
            case_evaltorque(outputs, inputs);
            break;
         case(3): // Run simulation and get output
            case_runsimulation(outputs, inputs);
            break;
        }
    }

    void case_fitnessfunction(ArgumentList outputs, ArgumentList inputs)
    {
            matlab::data::ArrayFactory factory;
            TypedArray<double> torque_vals_matlab = std::move(inputs[1]);

            std::vector<double> torque_vals(torque_vals_matlab.begin(), torque_vals_matlab.end());
            double laptime = run(torque_vals,0.0,10.0,0.001);
            outputs[0] = factory.createArray<double>({1,1}, {laptime});
    }

    void case_evaltorque(ArgumentList outputs, ArgumentList inputs)
    {
            matlab::data::ArrayFactory factory;
            TypedArray<double> torque_vals_matlab = std::move(inputs[1]);

            std::vector<double> torque_vals(torque_vals_matlab.begin(), torque_vals_matlab.end());

            sPolynomial poly(0.0, {1.0,1.0,1.0,1.0}, {torque_vals});

            std::vector<double> t(1001,0.0);
            std::vector<double> torq(1001,0.0);

            for (size_t i = 0; i < 1001; ++i)
            {
                t[i] = 10.0/1000.0*i;
                torq[i] = poly[t[i]];
            }

            outputs[0] = factory.createArray<double>({1,t.size()}, t.data(), t.data()+t.size());
            outputs[1] = factory.createArray<double>({1,torq.size()}, torq.data(), torq.data()+torq.size());
    }

    void case_runsimulation(ArgumentList outputs, ArgumentList inputs)
    {
            matlab::data::ArrayFactory factory;
            TypedArray<double> torque_vals_matlab = std::move(inputs[1]);

            std::vector<double> torque_vals(torque_vals_matlab.begin(), torque_vals_matlab.end());
            auto [t,q] = run.simulate_and_return(torque_vals,0.0,10.0,0.001);
            const size_t NROW = q.at(0).size();
            const size_t NCOL = q.size();
            Array qout = factory.createArray<double>({NROW,NCOL});
            Array tout = factory.createArray<double>({1,t.size()}, t.data(), t.data()+t.size());
    
            for (size_t j = 0; j < NCOL; ++j)
                for (size_t i = 0; i < NROW; ++i)
                {
                    qout[i][j] = q[j][i];
                }

            outputs[0] = tout;
            outputs[1] = qout;
    }



 private:
    Xml_document database = {std::string("./database/lot2016kart_rear_axle"), true};
    Dynamic_model_t dyn_model = {560.0, database};
    std::array<timeseries,Dynamic_model_t::NSTATE> _q0 = {5.0/0.139,0.0,0.0,0.0,5.0};
    
    Run_t run = {dyn_model, _q0, Dynamic_model_t::NCONTROL};

};
