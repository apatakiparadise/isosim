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

#include <ctime>



using namespace isosim;
using namespace SimTK;

// namespace isosim{


//callbacks (will need more)
static void advertiserCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);
static void forceSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);
static SimTK::Vec3 latestForce;

//public within namespace
RosbridgeWsClient RBcppClient("localhost:9090");


int main(void) {

    //do stuff

    std::cout << "main has been called" << std::endl; std::cout << get_current_dir_name() << std::endl;
    // IsosimROS rosclient;

    // rosclient.init();

    IsosimEngine engine;

    engine.init();

    engine.set_state(ISOSIM_RUN); //TODO: delete this once control comms is implemented

    while (engine.get_state() != ISOSIM_END_EXPERIMENT) {
        
        switch(engine.get_state()) {

            case(ISOSIM_RUN):
                std::cout << "starting loop" << std::endl;
                engine.loop(); //shouldn't return unless control signal received
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


/************************************************************************************************************************************
 * **********************************************************************************************************************************
 * IsosimROS FUNCTIONS
 * **********************************************************************************************************************************
*************************************************************************************************************************************/

void IsosimROS::init(void) {

    std::cout << "isosim init" << std::endl;
    //do nothing
    // RosbridgeWsClient RBcppClient("localhost:9090");
    // RBcppClientptr = &RBcppClient;
    // RBcppClient = RosbridgeWsClient("localhost:9090");

    // static SimTK::Vec3 ForceVar(0,0,0);
    // latestForce = &ForceVar;
    SimTK::Vec3 latestForce;
    latestForce = {0,0,0};

    RBcppClient.addClient("service_advertiser");
    RBcppClient.advertiseService("service_advertiser", "/isosimservice", "std_srvs/SetBool", advertiserCallback);

    RBcppClient.addClient("topic_advertiser");
    RBcppClient.advertise("topic_advertiser", "/isosimtopic", "std_msgs/String");


    RBcppClient.addClient("topic_subscriber");
    RBcppClient.subscribe("topic_subscriber", "/twistfromCMD",forceSubscriberCallback);

    //publish some dataroslaunch rosbridge_server rosbridge_websocket.launch

    RBcppClient.addClient("test_publisher");
    rapidjson::Document d;
    d.SetObject();
    d.AddMember("data", "Test message from /isosimtopic", d.GetAllocator());
    // while(1) {
        // RBcppClient.publish("/isosimtopic",d);
        // std::this_thread::sleep_for(std::chrono::seconds(30));
    // }
    return;
}

SimTK::Vec3 IsosimROS::get_latest_force(void) {

    // SimTK::Vec3 latestForce;

    //threadsafe...... not
    return latestForce;
}



// rostopic pub -r 10 /twistfromCMD geometry_msgs/Twist  "{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.0}}"
// roslaunch rosbridge_server rosbridge_websocket.launch




//private definitions
void advertiserCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message)
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


void forceSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message)
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

    // static SimTK::Vec3 forceFromROS = Vec3(x,y,z);
    
    
    // SimTK::Vec3 latestForce;
    // IsosimROS::latestForce = {x,y,z};
    latestForce = {x,y,z};
    // std::cout <<"_latestforce" << latestForce.get(0) << std::endl;
    // latestForce.set(2,z);
    

    // printf("linear = %d\n", document["linear"].GetString());
}



/*****************************************************************************************************************************
 ******************************************************************************************************************************
 * IsosimEngine FUNCTIONS
 ******************************************************************************************************************************
/******************************************************************************************************************************/

/*************PUBLIC*******************/
void IsosimEngine::init(void) {

    programState = ISOSIM_STANDBY;

    //initialise communication
    commsClient.init();

    generateIDModel(); //import/configure model

    generateFDModel(); //




}

int IsosimEngine::get_state(void) {

    return programState;
}

//NOT THREADSAFE (yet)
void IsosimEngine::set_state(int state) {

    programState = state;
}


void IsosimEngine::loop(void) {

    while(1) {
        // std::cout << "forcefromcomms" << commsClient.get_latest_force() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

    }
;
}

/*******PRIVATE***************/

bool IsosimEngine::generateIDModel(void) {

    // Define the initial and final simulation times //SHOULD BECOME OBSELETE IN REALTIME
    double initialTime = 0.0;
    double finalTime = 30.00 / 30;
    const double timestep = 1e-3;

    //import model
    IDModel =  OpenSim::Model("Models/arm26_ID.osim"); std::cout << "loaded model from arm26" << std::endl;
    //setup everything

    //get joints
    const OpenSim::JointSet& IDjointset = IDModel.get_JointSet();
    const OpenSim::Joint& IDshoulderJoint = IDjointset.get("r_shoulder");
    const OpenSim::Coordinate& IDshoulderelevcoord = IDshoulderJoint.getCoordinate();
 
    const OpenSim::Joint& IDelbowJoint = IDjointset.get("r_elbow");
    const OpenSim::Coordinate& IDelbowflex = IDelbowJoint.getCoordinate();

    std::cout << IDelbowflex.getName() << "<----- elbowflex name \n";


    //get bods
    const OpenSim::BodySet& IDbodyset = IDModel.get_BodySet();
    const OpenSim::Body& humerusbod = IDbodyset.get("r_humerus");
    std::cout << humerusbod.getName() << "<---name of humerusbod\n";

    //attach force to marker
    IDForceFromROS.setName("forceFromROS");
    IDForceFromROS.setAppliedToBodyName("r_humerus");
    IDForceFromROS.setForceExpressedInBodyName("r_humerus");
    IDForceFromROS.setPointExpressedInBodyName("r_humerus");
    IDForceFromROS.setForceIdentifier("Force1");
    IDForceFromROS.set_appliesForce(true);
    std::cout << IDForceFromROS.getAppliedToBodyName() << "<----force applied to body\n";

    OpenSim::Storage IDForceStorage((int) (finalTime - initialTime)*timestep,"IDStorage");
    // IDForceStorage.setName("IDStorage");
    OpenSim::Array<std::string> IDForceStorageColumns;
    IDForceStorageColumns.append("Force1.x");
    IDForceStorageColumns.append("Force1.y");
    IDForceStorageColumns.append("Force1.z");
    IDForceStorage.setColumnLabels(IDForceStorageColumns); //TODO
    // IDForceStorage.setMax
    for (double tim = initialTime; tim <= finalTime; tim+=timestep) {
        IDForceStorage.append(tim,SimTK::Vec3(100,100,100)*tim);
        
    }
    std::cout << "finished storage appending\n";

    IDForceFromROS.setDataSource(IDForceStorage);
    // IDModel.getForceSet()

    std::cout << "is this our error point?\n";
    // IDModel.addForce(&IDForceFromROS);
    IDModel.addComponent(&IDForceFromROS);

    OpenSim::InverseDynamicsSolver idSolver(IDModel);
    /////////////////////////////////////////////
    // DEFINE CONSTRAINTS IMPOSED ON THE MODEL //
    /////////////////////////////////////////////
    


    //delete below


    // set use visualizer to true to visualize the simulation live
    IDModel.setUseVisualizer(true);

    //reporters
    OpenSim::ForceReporter* forceRep = new OpenSim::ForceReporter(&IDModel);  
    IDModel.updAnalysisSet().adoptAndAppend(forceRep);

    Vec3 grav = IDModel.get_gravity()*1;
    IDModel.set_gravity(grav);
    // Initialize the system and get the default state
    SimTK::State& si = IDModel.initSystem();
    OpenSim::Manager manager(IDModel);
    manager.setIntegratorAccuracy(timestep);

    //lock joints
    IDshoulderJoint.getCoordinate().setLocked(si,true);
    IDelbowJoint.getCoordinate().setValue(si,convertDegreesToRadians(90));
    IDelbowJoint.getCoordinate().setLocked(si, true);

    IDForceFromROS.setAppliesForce(si, true);    
    SimTK::Vec3 wristPoint = IDModel.getMarkerSet().get("r_radius_styloid").get_location();
    // SimTK::Vec3 wristPoint = IDModel.get("r_radius_styloid").getOffset();
    std::cout << wristPoint << "<----- styloid offset\n";
    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    // IDForceFromROS.applyForceToPoint(si, humerusbod, wristPoint,SimTK::Vec3(10,10,10),bodyForces);
    // IDForceFromROS.computeForce(si,bodyForces,SimTK::Vec3(10,10,20));

    //TODO: figure out how to set the value of the ExternalForce (do I need to create a Storage object and then update that?)
    //TODO: I guess that would work but in real-time you would need to change both the time array and the force values regularly...
    // TODO: Otherwise if I can access the functions that actually put the force into the model, then it could be okay...



    // Print out details of the model
    IDModel.printDetailedInfo(si, std::cout);

    //REALTIME STUFF BELOW
    si.setTime(initialTime);
    manager.initialize(si);
    //get engine properties
    SimTK::Integrator* integrator_ = &manager.getIntegrator();
    std::cout << "we have an integrator    \n";
    SimTK::State state_ = integrator_->getAdvancedState();

    IDModel.getVisualizer().show(state_);

    //perform ID

    // double mobsize = IDModel.getMultibodySystem().getMobilityForces(state_, Stage::Dynamics).size();
    SimTK::Vector residualMobilityForces;
    double simTime = 0;
    while (simTime < finalTime) {
        // state_.setTime(simTime);
        // std::cout << "running  " << simTime << std::endl; 
        integrator_->stepBy(timestep);
        IDModel.getMultibodySystem().realize(state_, Stage::Dynamics);
        if (simTime < initialTime+(timestep*3)) {
            simTime += timestep;
            continue;
        }
        // std::cout << "stepped  " << simTime << std::endl; 
        const Vector& appliedMobilityForces = 
                IDModel.getMultibodySystem().getMobilityForces(state_, Stage::Dynamics);
        const Vector_<SpatialVec>& appliedBodyForces = 
                IDModel.getMultibodySystem().getRigidBodyForces(state_, Stage::Dynamics);
        // Vector& appliedMobilityForces = appliedMobilityForcesC;
        // Vector_<SpatialVec>& appliedBodyForces = appliedBodyForcesC; 
        const SimTK::Vector testUdot = Test::randVector(state_.getNU())*0;
        // std::cout << "uddoted  " << simTime << std::endl; 
        // SimTK::Vector_<SimTK::SpatialVec> appliedBodyForces(IDModel.getMultibodySystem().getRigidBodyForces(state_, SimTK::Stage::Dynamics));
        // IDModel.getMultibodySystem().getMatterSubsystem().calcResidualForceIgnoringConstraints(
                    // state_,appliedMobilityForces,appliedBodyForces,testUdot, residualMobilityForces);
        residualMobilityForces = idSolver.solve(state_,testUdot,appliedMobilityForces,appliedBodyForces);
        simTime+= timestep;
        std::cout << residualMobilityForces << std::endl;
    }

    // Integrate from initial time to final time
    // si.setTime(initialTime);
    // manager.initialize(si);
    // std::cout<<"\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
    // manager.integrate(finalTime);

    auto forcesTable = forceRep->getForcesTable();
    // std::string forceFilename
    OpenSim::STOFileAdapter::write(forcesTable, "forces.sto");
    
    //delete above




    
    //get integrator and save it to class variable


    return true;
}

bool IsosimEngine::generateFDModel(void) {


    //import model

    //setup everything

    //get integrator and save it to class variable

    return true;
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