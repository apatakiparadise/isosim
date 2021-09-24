/**************************************************************
* ISOSIM Main header
* Handles real-time inverse dynamics and forward dynamics
*       with communication via rosbridge
*
*
* Author: Joshua Rolls
* Date: 08-September-2021
**************************************************************/


#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"
#include <rosbridge_ws_client.hpp>

#include "IsometricExternalForce.h"

#define ISOSIM_STANDBY 0
#define ISOSIM_RUN 1
#define ISOSIM_END_EXPERIMENT 4

namespace isosim {




class IsosimROS {


    public:
        void init(void);

        int subscriber(void);
        int publisher(void);

        SimTK::Vec3 get_latest_force(void); //THREADSAFE? no

        
    private:
        
        void unimplemented(void);
        

};

class IsosimEngine {

    public:
        void init(void);

        void start(void);

        void loop(void);

        IsosimROS commsClient;

        int get_state(void);
        void set_state(int state);

        void testPointActuator(void); //delete this //TODO

        ~IsosimEngine();
    
    private:

        int programState; //running/stopping
        // static SimTK::Vec<3,double> latestForce;
        bool generateIDModel(void); //imports/configures inverse dynamics model
        OpenSim::Model IDModel;
        OpenSim::Manager* IDmanager;
        OpenSim::InverseDynamicsSolver* idSolver;
        //point force used to represent force exterted on end effector
        OpenSim::PointActuator endEffector;
        double IDtimestep;


        bool generateFDModel(void); //imports/configures forward dynamics model
        OpenSim::Model FDModel;
        OpenSim::Manager* FDmanager;
        double FDtimestep; //should be same as IDtimestep
        
        //actuators to apply input from ID in FD model
        OpenSim::TorqueActuator FDshoulderTorque;
        OpenSim::TorqueActuator FDelbowTorque;

        struct ID_Output {

            time_t timestamp;
            SimTK::Vector residualMobilityForces;
            bool valid;
        };

        struct FD_Output {

            time_t timestamp;
            SimTK::Vector q;
            SimTK::Vector u;
            SimTK::Vector uDot;
            bool valid;

        };

        //performs one iteration of forward dynamics
        IsosimEngine::FD_Output forwardD(IsosimEngine::ID_Output input);

        //performs one iteration of inverse dynamics
        ID_Output inverseD(void);

        void step(void);

        struct ID_Input {

            time_t timestamp;
            double forceMag;
            SimTK::Vec3 forceDirection;
        };

        ID_Input forceVecToInput (SimTK::Vec3 forceVector);

        


        
};









} //namespace isosim