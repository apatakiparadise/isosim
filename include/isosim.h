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


#include <fstream>

#define ISOSIM_STANDBY 0
#define ISOSIM_RUN 1
#define ISOSIM_END_EXPERIMENT 4


#define OPTIMAL_TORQUE 100 //N m

namespace isosim {




class IsosimROS {


    public:
        void init(void);

        int subscriber(void);
        int publisher(void);

        SimTK::Vec3 get_latest_force(void); //THREADSAFE? no

        struct IsosimData {

            double timestamp; 
            
            SimTK::Vec3 wristPos;
            SimTK::Vec3 elbowPos;
            bool valid;
        };

        bool publishState(IsosimData stateData);

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

        struct FD_Output {

            double timestamp; //should this be a time_t or not?

            SimTK::Vec3 wristPos;
            SimTK::Vec3 elbowPos;
            bool valid;

        };
    
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
        double currentSimTime;
        
        //actuators to apply input from ID in FD model
        OpenSim::TorqueActuator FDshoulderTorque1;
        OpenSim::TorqueActuator FDshoulderTorque2;
        OpenSim::TorqueActuator FDelbowTorque;

        struct ID_Output {

            double timestamp;
            SimTK::Vector residualMobilityForces;
            bool valid;
        };

        

        //performs one iteration of forward dynamics
        IsosimEngine::FD_Output forwardD(IsosimEngine::ID_Output input);

        //performs one iteration of inverse dynamics
        ID_Output inverseD(void);

        void step(void);

        struct ID_Input {

            double timestamp;
            double forceMag;
            SimTK::Vec3 forceDirection;
        };

        ID_Input forceVecToInput (SimTK::Vec3 forceVector);

        
        //COMBINED FD/ID - performs forward dynamics on a non-static model while still calculating joint torques
        IsosimEngine::FD_Output forwardInverseD(void);

        std::ofstream logger;

        IsosimROS::IsosimData FDoutputToIsosimData(IsosimEngine::FD_Output calculatedState);
        
};









} //namespace isosim