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

#include <mutex>
#include <future>
#include <thread>

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

        struct ForceInput {
            SimTK::Vec3 force;
            double time;
        };

        ForceInput get_latest_force(void); //THREADSAFE

        bool set_latest_force_time(SimTK::Vec3 forceIn, double tim); //THREADSAFE

        struct IsosimData {

            double timestamp; 
            
            SimTK::Vec3 wristPos;
            SimTK::Vec3 elbowPos;
            bool valid;
        };

        bool setPositionToPublish(IsosimData stateData);


        ~IsosimROS();

    private:

        //publishes state (use inside publisher thread)
        // bool publishState(IsosimData stateData);
        
        
        ForceInput _latestInput;

        //repeatedly publishes position, until pubExitSignal is received
        // void positionPublisherThread(RosbridgeWsClient& client, const std::future<void>& futureObj);
        std::promise<void> pubExitSignal;
        std::thread* pubTh;
        std::future<void> futureObj;

        std::promise<void> subExitSignal;
        std::thread* subTh;
        std::future<void> subFutureObj;
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
        SimTK::TimeStepper* FDstepper;
        std::unique_ptr<SimTK::TimeStepper> _FDstepper;
        std::unique_ptr<SimTK::Integrator> _FDintegr;
        double FDtimestep; //should be same as IDtimestep
        double prevSimTime;
        
        //actuators to apply input from ID in FD model
        OpenSim::TorqueActuator FDshoulderTorque1; // shoulder elevation
        OpenSim::TorqueActuator FDshoulderTorque2; // inward rotation (positive is adduction, neg abduction)
        OpenSim::TorqueActuator FDelbowTorque; // elbow flexion (pos is flexion, neg is extension)

        struct ID_Output {

            double timestamp;
            SimTK::Vector residualMobilityForces;
            bool valid;
        };

        
        //performs one iteration of inverse dynamics
        ID_Output inverseD(void);

        //performs one iteration of forward dynamics
        IsosimEngine::FD_Output forwardD(IsosimEngine::ID_Output input);
        SimTK::Vec3 FDshoulderPosG;
        //applies spring to joint when it approaches its limit
        double torqueSpring(double q, double u, double udot, double torque, const OpenSim::Coordinate* coord);
        double Kspring = 20; //K constant for joint springs (N.m/rad)
        double Bspring = 4*3;//*0; //Beta constant for joint springs (damping) (N.m.s/rad)
        double Bfree = 0.1;//*0; //Beta constant for damping within range of motion (N.m.s/rad)

        void step(void);

        struct ID_Input {

            double timestamp;
            double forceMag;
            SimTK::Vec3 forceDirection;
        };

        ID_Input forceVecToInput (IsosimROS::ForceInput forceInput);

        
        //COMBINED FD/ID - performs forward dynamics on a non-static model while still calculating joint torques
        IsosimEngine::FD_Output forwardInverseD(void);

        std::ofstream logger;

        IsosimROS::IsosimData FDoutputToIsosimData(IsosimEngine::FD_Output calculatedState);
        
};









} //namespace isosim