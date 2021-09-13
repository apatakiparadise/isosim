/**************************************************************
* ISOSIM Main file
* Handles real-time inverse dynamics and forward dynamics
*       with communication via rosbridge
*
*
* Author: Joshua Rolls
* Date: 08-September-2021
**************************************************************/



#include "isosim.h"

using namespace isosim;
// namespace isosim{




//public within namespace
RosbridgeWsClient RBcppClient("localhost:9090");


int main(void) {

    //do stuff

    std::cout << "main has been called" << std::endl;
    // IsosimROS rosclient;

    // rosclient.init();

    IsosimEngine engine;

    engine.init();

    while (engine::protocolState != ISOSIM_END_EXPERIMENT) {

        switch(engine::protocolState) {

            case(ISOSIM_RUN):
                std::cout << "starting loop" << std::endl;
                engine::loop(); //shouldn't return unless control signal received
                break;

            case(ISOSIM_STANDBY):
                //do nothing
                break;
            case(ISOSIM_END_EXPERIMENT):
                break; //next while iteration will end this
        }

    }//while not ended

    std::cout << "EXPERIMENT OVER> ISOSIM OUT" << std::endl;

    return true;
}


/***************************************************************
* IsosimROS FUNCTIONS
***************************************************************/

void IsosimROS::init(void) {

    std::cout << "isosim init" << std::endl;
    //do nothing
    // RosbridgeWsClient RBcppClient("localhost:9090");
    // RBcppClientptr = &RBcppClient;
    // RBcppClient = RosbridgeWsClient("localhost:9090");

    RBcppClient.addClient("service_advertiser");
    RBcppClient.advertiseService("service_advertiser", "/isosimservice", "std_srvs/SetBool", IsosimROS::advertiserCallback);

    RBcppClient.addClient("topic_advertiser");
    RBcppClient.advertise("topic_advertiser", "/isosimtopic", "std_msgs/String");


    RBcppClient.addClient("topic_subscriber");
    RBcppClient.subscribe("topic_subscriber", "/twistfromCMD",IsosimROS::forceSubscriberCallback);

    //publish some data
    RBcppClient.addClient("test_publisher");
    rapidjson::Document d;
    d.SetObject();
    d.AddMember("data", "Test message from /isosimtopic", d.GetAllocator());
    while(1) {
        RBcppClient.publish("/isosimtopic",d);
        std::this_thread::sleep_for(std::chrono::seconds(30));
    }
    return;
}





// rostopic pub -r 10 /twistfromCMD geometry_msgs/Twist  "{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.0}}"




//private definitions
void IsosimROS::advertiserCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message)
{
    // message->string() is destructive, so we have to buffer it first
    std::string messagebuf = in_message->string();
    std::cout << "advertiseServiceCallback(): Message Received: " << messagebuf << std::endl;

    rapidjson::Document document;
    if (document.Parse(messagebuf.c_str()).HasParseError())
    {
    std::cerr << "advertiseServiceCallback(): Error in parsing service request message: " << messagebuf << std::endl;
    return;
    }

    rapidjson::Document values(rapidjson::kObjectType);
    rapidjson::Document::AllocatorType& allocator = values.GetAllocator();
    values.AddMember("success", document["args"]["data"].GetBool(), allocator);
    values.AddMember("message", "from advertiseServiceCallback", allocator);

    // RBcppClientptr->addClient("testclient");
    RBcppClient.serviceResponse(document["service"].GetString(), document["id"].GetString(), true, values);


}


void IsosimROS::forceSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message)
{
    #ifdef DEBUG
    // std::cout << "subscriberCallback(): Message Received: " << in_message->string() << std::endl;
    #endif
    rapidjson::Document forceD;

    // char *cstr = new char[in_message->string().length() + 1];
    // strcpy(cstr, in_message->string().c_str());
    // std::cout << "msg length : " << in_message->string().length() << std::endl;
    // std::cout << "actual string received : " << cstr << std::endl;
    if (forceD.Parse(in_message->string().c_str()).HasParseError() ) {
        std::cerr << "\n\nparse error\n" << std::endl;
    };

    #ifdef DEBUG
        rapidjson::StringBuffer buffer;
        buffer.Clear();
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        forceD.Accept(writer);
        std::cout << std::endl << "msg received:   " << buffer.GetString() << std::endl << std::endl;
    #endif

    std::cout << "\nAccess values in document:\n";
    assert(forceD.IsObject());    // Document is a JSON value represents the root of DOM. Root can be either an object or array.

    

    assert(forceD.HasMember("msg"));
    assert(forceD["msg"].HasMember("linear"));
    
    assert(forceD["msg"]["linear"].HasMember("x"));
    assert(forceD["msg"]["linear"]["x"].IsDouble());
    std::cout << "seems to work\n";
    std::cout << forceD["msg"]["linear"]["x"].GetDouble() << std::endl;

    double x = forceD["msg"]["linear"]["x"].GetDouble();
    double y = forceD["msg"]["linear"]["y"].GetDouble();
    double z = forceD["msg"]["linear"]["z"].GetDouble();

    latestForce = {x,y,z};


    

    // printf("linear = %d\n", document["linear"].GetString());
}



/***************************************************************
* IsosimEngine FUNCTIONS
***************************************************************/

/*************PUBLIC*******************/
void IsosimEngine::init(void) {

    programState = ISOSIM_STANDBY;

    //initialise communication
    commsClient.init();

    generateIDModel(); //import/configure model

    generateFDModel(); //




}


/*******PRIVATE***************/

void IsosimEngine::generateIDModel(void) {

    //import model

    //setup everything

    //get integrator and save it to class variable
}

void IsosimEngine::generateFDModel(void) {


    //import model

    //setup everything

    //get integrator and save it to class variable


}




void IsosimEngine::step(void) {

    //step id first using IsosimEngine::inverseD

    //get output torques in a form that is usable to the FD engine

    //step fd using IsosimEngine::forwardD

    //publish using comms::publisher
    // (maybe this last one can be done by a callback function)
    // (so we just set the latest state and that gets transmitted to commshub)

}

// } //namespace isosim