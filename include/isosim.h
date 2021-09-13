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
#include <rosbridge_ws_client.hpp>


#define ISOSIM_STANDBY 0
#define ISOSIM_RUN 1
#define ISOSIM_END_EXPERIMENT 4

namespace isosim {

class IsosimROS {


    public:
        void init(void);

        int subscriber(void);
        int publisher(void);
        
    private:
        

        static void advertiserCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);
        static void forceSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);

        Eigen::Vector3d latestForce;
};

class IsosimEngine {

    public:
        void init(void);

        void start(void);

        void loop(void);

        IsosimROS commsClient;
    
    private:

        int programState; //running/stopping

        bool generateIDModel(void); //imports/configures inverse dynamics model

        bool generateFDModel(void); //imports/configures forward dynamics model

        void forwardD(void);

        void inverseD(void);

        void step(void);
}









} //namespace isosim