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
    
    private:

        int programState; //running/stopping
        // static SimTK::Vec<3,double> latestForce;
        bool generateIDModel(void); //imports/configures inverse dynamics model
        OpenSim::Model IDModel;
        OpenSim::ExternalForce IDForceFromROS;

        bool generateFDModel(void); //imports/configures forward dynamics model
        OpenSim::Model FDModel;

        void forwardD(void);

        void inverseD(void);

        void step(void);
};









} //namespace isosim