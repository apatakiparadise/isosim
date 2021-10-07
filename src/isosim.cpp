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


#define LOGGING

#define IDFD
// #undef IDFD //not doing both currently
#define EXTRA_ROT 1

#undef DEBUG

using namespace isosim;
using namespace SimTK;

// namespace isosim{


//callbacks (will need more)
static void advertiserCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);
static void forceSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);
static SimTK::Vec3 latestForce;

//public within namespace
RosbridgeWsClient RBcppClient("localhost:9090");


//order of joints in system state Q/Qdot/Udot vectors
#define SHOULDER_NUM 0
#define ELBOW_NUM 1

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
    RBcppClient.advertise("topic_advertiser", "/isosimtopic", "franka_panda_controller_swc/IsosimOutput");


    RBcppClient.addClient("topic_subscriber");
    RBcppClient.subscribe("topic_subscriber", "/twistfromCMD",forceSubscriberCallback);

    //publish some data     roslaunch rosbridge_server rosbridge_websocket.launch

    RBcppClient.addClient("test_publisher");  //TODO: what does this publisher client actually do? and where???
    // rapidjson::Document d;
    // d.SetObject();
    // d.AddMember("data", "Test message from /isosimtopic", d.GetAllocator());
    // while(1) {
    //     RBcppClient.publish("/isosimtopic",d);
    //     std::this_thread::sleep_for(std::chrono::seconds(100));
    // }
    return;
}

SimTK::Vec3 IsosimROS::get_latest_force(void) {

    // SimTK::Vec3 latestForce;

    //threadsafe...... not
    return latestForce;
}

bool IsosimROS::publishState(IsosimROS::IsosimData stateData) {

    if (stateData.valid == false) {

        std::cout << "INVALID DATA\n";
        return false;
    }

    rapidjson::Document d;
    d.SetObject();

    rapidjson::Value msg(rapidjson::kObjectType);


    //create wrist values
    rapidjson::Value wristVec(rapidjson::kObjectType);
    rapidjson::Value wX;
    rapidjson::Value wY;
    rapidjson::Value wZ;
    wX.SetDouble(stateData.wristPos[0]);
    wY.SetDouble(stateData.wristPos[1]);
    wZ.SetDouble(stateData.wristPos[2]);
    wristVec.AddMember("x",wX,d.GetAllocator());
    wristVec.AddMember("y",wY,d.GetAllocator());
    wristVec.AddMember("z",wZ,d.GetAllocator());

    //create elbow values
    rapidjson::Value elbowVec(rapidjson::kObjectType);
    rapidjson::Value eX;
    rapidjson::Value eY;
    rapidjson::Value eZ;
    eX.SetDouble(stateData.elbowPos[0]);
    eY.SetDouble(stateData.elbowPos[1]);
    eZ.SetDouble(stateData.elbowPos[2]);
    elbowVec.AddMember("x",eX,d.GetAllocator());
    elbowVec.AddMember("y",eY,d.GetAllocator());
    elbowVec.AddMember("z",eZ,d.GetAllocator());

    //create time values
    rapidjson::Value timestamp;
    timestamp.SetDouble(stateData.timestamp);

    //add values to d
    d.AddMember("wristPos",wristVec,d.GetAllocator());
    d.AddMember("elbowPos",elbowVec,d.GetAllocator());
    d.AddMember("time",timestamp,d.GetAllocator());
    


    RBcppClient.publish("/isosimtopic",d);

    return true; //when should we return false? //TODO
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
    std::cout << "subscriberCallback(): Message Received: " << in_message->string() << std::endl;
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

    
    assert(forceD.IsObject());    // Document is a JSON value represents the root of DOM. Root can be either an object or array.

    

    assert(forceD.HasMember("msg"));
    assert(forceD["msg"].HasMember("linear"));
    
    assert(forceD["msg"]["linear"].HasMember("x"));
    assert(forceD["msg"]["linear"]["x"].IsDouble());

    // std::cout << forceD["msg"]["linear"]["x"].GetDouble() << std::endl;

    double x = forceD["msg"]["linear"]["x"].GetDouble();
    double y = forceD["msg"]["linear"]["y"].GetDouble();
    double z = forceD["msg"]["linear"]["z"].GetDouble();

    
    //save to global variable
    latestForce = {x,y,z};
    



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


    #ifdef LOGGING
        //setup logger
        time_t _tm =time(NULL );
        struct tm * curtime = localtime ( &_tm );
        std::cout<<"The current date/time is:"<<asctime(curtime);

        logger.open("Output/position.log", std::ios_base::app);
    #endif

    
    //give an initial force
    latestForce = {0,-3,0};
    clock_t timeAtStart = clock();
    for (double i = 0; i < 20; i+= IDtimestep) {
        step();
    }
    clock_t midTime = clock();
    latestForce = {100,0,0};
    for (double i = 0; i < 200; i+= IDtimestep) {
        step();
    }
    latestForce = {0,1,0};
    for (double i = 0; i < 20; i+= IDtimestep) {
        step();
    }
    
    clock_t timeAtEnd = clock();

    std::cout << "Completed forward test in " << (midTime - timeAtStart)*1000/CLOCKS_PER_SEC << " seconds\n";
    std::cout << "Completed backward test in " << ((timeAtEnd - midTime)*1000/CLOCKS_PER_SEC) << " seconds\n";
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
    const double timestep = 1e-3 * 50; //TODO fix this
    IDtimestep = timestep; //this should be received from control input
    currentSimTime = 0; //can be overriden by control

    //import model
    IDModel =  OpenSim::Model("Models/arm26-mod.osim"); std::cout << "loaded model from arm26-mod" << std::endl;
    std::cout << IDModel.getName() << "<----------------model name\n\n";
    //setup everything
    Vec3 grav = IDModel.get_gravity()*0; //end effector force is the net force (so includes gravity)
    IDModel.set_gravity(grav); //redundant unless we change grav above

    //get joints
    const OpenSim::JointSet& IDjointset = IDModel.get_JointSet();
    const OpenSim::Joint& IDshoulderJoint = IDjointset.get("r_shoulder");


    // if (EXTRA_ROT) {
        
    //     const OpenSim::Coordinate& IDshoulderelevcoord = IDshoulderJoint.get_coordinates(0);
    //     const OpenSim::Coordinate& IDshoulderRot2Coord = IDshoulderJoint.get_coordinates(1);
        
    // } else {
    //     const OpenSim::Coordinate& IDshoulderelevcoord = IDshoulderJoint.getCoordinate();
    //     // shoulderElevRange = {IDshoulderelevcoord.getRangeMin(), IDshoulderelevcoord.getRangeMax()};
    // }
    

    const OpenSim::Joint& IDelbowJoint = IDjointset.get("r_elbow");
    const OpenSim::Coordinate& IDelbowflex = IDelbowJoint.getCoordinate();

    // elbowFlexRange = {IDshoulderelevcoord.getRangeMin(), IDshoulderelevcoord.getRangeMax()};

    std::cout << IDelbowflex.getName() << "<----- elbowflex name \n";



    //get bods
    const OpenSim::BodySet& IDbodyset = IDModel.get_BodySet();
    const OpenSim::Body& humerusbod = IDbodyset.get("r_humerus");
    const OpenSim::Body& radiusbod = IDbodyset.get("r_ulna_radius_hand");
    std::cout << humerusbod.getName() << "<---name of humerusbod\n";
    std::cout << radiusbod.getName() << "<---name of radiusbod\n";
    IDbodyset.print("bodyset.bods");

    //get muscles and disable
    const OpenSim::Set<OpenSim::Muscle>& muscleSet =  IDModel.getMuscles();
    for (int i=0; i < muscleSet.getSize(); i++) {
        muscleSet.get(i).set_appliesForce(false);
    }


    //get wrist point
    SimTK::Vec3 wristPointLoc = IDModel.getMarkerSet().get("r_radius_styloid").get_location();

    //init system (for end effector's sake)
    /*SimTK::State& si = */IDModel.initSystem();

    //IsosimEngine::endEffector
    endEffector.set_body(radiusbod.getName());
    endEffector.setName("end_effector");
    std::cout << endEffector.get_body() << "  <-- name of body for end effector \n";
    endEffector.set_point(wristPointLoc);
    endEffector.set_point_is_global(false); //point coordinates are relative to radius
    endEffector.set_force_is_global(true); //force coordinates will be in ground frame
    endEffector.set_direction(SimTK::Vec3(1,0,0));   //x is front, y is up, 
    std::cout << endEffector.get_direction() << "<-- eE dir\n";//endEffector.set_direction(SimTK::Vec3(1,1,1));
    double optimalEndForce = 100; //Newtons (should be maximum force)
    endEffector.setOptimalForce(optimalEndForce);
    std::cout << endEffector.getOptimalForce() << "  <-- optimal force from endeffector\n";

    IDModel.addForce(&endEffector);
    IDModel.finalizeConnections();
    IDModel.setUseVisualizer(true);
    SimTK::State& si = IDModel.initSystem(); std::cout << "hello there\n";
    std::cout << endEffector.get_appliesForce() << "<-- that end effector is applying force---------------------\n";

    Vector endEffectorControls(1);
    endEffectorControls(0) = 1*0;// 0 for realtime (controls will be added back later)
    
    
    //add control values
    Vector modelControls = IDModel.getDefaultControls();
    endEffector.addInControls(endEffectorControls,modelControls);
    IDModel.setDefaultControls(modelControls);

    //print initial accelerations to test
    IDModel.computeStateVariableDerivatives(si);
    Vector udotActuatorsCombination = si.getUDot();


    std::cout << "ground control to major tom\n";
    
    
    /////////////////////////////////////////////
    // DEFINE CONSTRAINTS IMPOSED ON THE MODEL //
    /////////////////////////////////////////////
    


    //reporters
    OpenSim::ForceReporter* forceRep = new OpenSim::ForceReporter(&IDModel);
    IDModel.updAnalysisSet().adoptAndAppend(forceRep);

    
    IDmanager = new OpenSim::Manager(IDModel);
    
    IDmanager->setIntegratorAccuracy(timestep);

    //lock joints - Only if we're performing ID first, not necessary for the direct FD method
    IDelbowJoint.getCoordinate().setValue(si,convertDegreesToRadians(90));
    #ifdef IDFD
    IDelbowJoint.getCoordinate().setLocked(si, true);
    std::cout << "ground control to Major Tom\n";
    
    std::cout << IDshoulderJoint.numCoordinates() << "<-- number of state var\n";
    for (int i = 0; i < IDshoulderJoint.numCoordinates(); i++) { //locks all coordinates of the joint
        IDshoulderJoint.get_coordinates(i).setLocked(si,true);
    }
    std::cout << "standby for takeoff, and put your seatbelt on\n";
    
    // IDshoulderJoint.getCoordinate().setLocked(si,true);
    #else
    IDelbowJoint.getCoordinate().setClamped(si,true);
    IDshoulderJoint.getCoordinate().setClamped(si,true);
    std::cout << "shoulder coord free to satisfy constraints" << IDshoulderJoint.getCoordinate().get_is_free_to_satisfy_constraints() << std::endl;
    std::cout << "shoulder coord prescribed function" << IDshoulderJoint.getCoordinate().get_prescribed() << std::endl;
    std::cout << "shouldercoord clamped in si " << IDshoulderJoint.getCoordinate().getClamped(si) << std::endl;
    #endif

    IDModel.print("isosimModel.osim");
    // Print out details of the model
    IDModel.printDetailedInfo(si, std::cout);

    #define TEST_ID
    #ifdef TEST_ID 
    #define REALTIME_TEST
    #ifdef REALTIME_TEST
    //REALTIME STUFF BELOW
    std::cout << "--------------------------ENTER REALTIME TEST MODE--------------------\n";
    si.setTime(initialTime);
    IDmanager->initialize(si);
    //get engine properties (just for our while loop here)
    SimTK::Integrator* integrator_ = &IDmanager->getIntegrator();

    SimTK::State state_ =  integrator_->getAdvancedState();

    IDModel.getVisualizer().show(state_);
    IDModel.getMultibodySystem().realize(state_, Stage::Dynamics);
    std::cout << "----------------IMPORTANT-------------------\n";
    std::cout << "how many nq are there?\n";
    std::cout << "and what coordinates do they represent?\n";
    std::cout << "we shall find out\n";
    std::cout << "there are ....  " << state_.getNQ() << " NQ in state_\n";
    std::cout << "the current Q are " << state_.getQ() << " at initialisation\n";
    std::cout << "but there are more subsystems\n";
    std::cout << state_.getNumSubsystems() << " subsystems actually\n";
    std::cout << "so let's try something different\n";
    for (SimTK::SubsystemIndex subsysI(0); subsysI < state_.getNumSubsystems(); subsysI++) {
        std::cout << state_.getNQ(subsysI) << " NQ in subsys " << subsysI << "\n";
           
    }
    std::cout << "what does this mean?\n";
    // return false;

    SimTK::Vector residualMobilityForces;
    double simTime = 0;

    SimTK::Vec3 reverseDirection = endEffector.get_direction() * -1;
    bool reversed = false;

    OpenSim::InverseDynamicsSolver* solver = new OpenSim::InverseDynamicsSolver(IDModel);
    idSolver = solver;
    idSolver->setName("id_solver");

    /*
    clock_t timeAtStart = clock();
    clock_t currentTime = timeAtStart;
    while (simTime < finalTime) {
        currentTime = clock();
        
        if ( (reversed == false) && (simTime > finalTime / 2) ) {
            endEffector.set_direction(reverseDirection);
            reversed==true;
        }
        endEffectorControls(0) = simTime / finalTime;
        Vector modelControls = IDModel.getDefaultControls();
        // modelControls.dump("old mod controls");
        endEffector.addInControls(endEffectorControls,modelControls);
        // modelControls.dump("new mod controls");
        IDModel.setControls(state_, modelControls);


        IDModel.getMultibodySystem().realize(state_, Stage::Dynamics);
        if (simTime < initialTime+(timestep*3)) {
            simTime += timestep;
            continue;
        }
        
        const Vector& appliedMobilityForces = 
                IDModel.getMultibodySystem().getMobilityForces(state_, Stage::Dynamics);

        const SimTK::Vector testUdot = Test::randVector(state_.getNU())*0;

        SimTK::Vector_<SimTK::SpatialVec> appliedBodyForces(IDModel.getMultibodySystem().getRigidBodyForces(state_, SimTK::Stage::Dynamics));

       
        residualMobilityForces = idSolver->solve(state_,testUdot,appliedMobilityForces,appliedBodyForces);


        std::cout << "residualmobforces: " << residualMobilityForces << " at time: " << simTime << std::endl;
        

        simTime+= timestep;
        //below function calls only necessary for visualising over time (and they don't work)
        integrator_->stepBy(timestep);
        state_ = integrator_->getAdvancedState();
        IDModel.getVisualizer().show(state_);
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    
    } //end while loop
    

    clock_t timeAtEnd = clock();   
    std::cout << "integrated from " << initialTime << " to " << finalTime << " seconds  in " << (timeAtEnd - timeAtStart)*1000/CLOCKS_PER_SEC << " milliseconds\n";
    */
    #else //NOT REALTIME
    // Integrate from initial time to final time
    std::cout << "commencing non-realtime simulation\n";
    si.setTime(initialTime);
    std::cout << "time is set, let's initialise the manager\n";
    IDmanager->initialize(si);

    std::cout<<"\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
    IDmanager->integrate(finalTime);
     
    auto forcesTable = forceRep->getForcesTable();
    forceRep->getForceStorage().print("forcestorage.mot");

    // std::string forceFilename
    OpenSim::STOFileAdapter::write(forcesTable, "forces.sto");


    std::cout << IDmanager->getIntegrator().getAdvancedState().getQ() << "<--Q??\n";
    while(1) {
        IDModel.getVisualizer().show(IDmanager->getIntegrator().getAdvancedState());
    }
    #endif


    #endif //TEST_ID
    


    return true;
}

bool IsosimEngine::generateFDModel(void) {

    std::cout << "initialising FD model\n";
    //import model
    FDModel =  OpenSim::Model("Models/arm26-mod.osim");
    //setup everything
    try {
        FDtimestep = IDtimestep;
    } catch (...) {
        std::cerr << "ID model not initialised\n";
        std::cerr << "Exiting\n";
        return false;
    }
    double initialTime = 0;

    //set gravity (for debugging)
    Vec3 grav = FDModel.get_gravity()*0;
    FDModel.set_gravity(grav); //redundant unless we change grav above


    //get bodies
    const OpenSim::BodySet& FDbodyset = FDModel.get_BodySet();
    const OpenSim::Body& FDbasebod = FDbodyset.get("base");
    const OpenSim::Body& FDhumerusbod = FDbodyset.get("r_humerus");
    const OpenSim::Body& FDradiusbod = FDbodyset.get("r_ulna_radius_hand"); //not used here

    //get joints

    const OpenSim::JointSet& FDjointset = FDModel.get_JointSet();

    FDModel.initSystem(); //just so we can get the right component
    const OpenSim::CustomJoint& FDshoulderCustomJoint =  FDModel.getComponent<OpenSim::CustomJoint>("/jointset/r_shoulder");
    const OpenSim::CustomJoint& FDelbowCustomJoint =  FDModel.getComponent<OpenSim::CustomJoint>("/jointset/r_elbow");
    // const OpenSim::Joint& FDelbowJoint = FDjointset.get("r_elbow");
    
    //get muscles and disable
    const OpenSim::Set<OpenSim::Muscle>& muscleSet =  FDModel.getMuscles();
    for (int i=0; i < muscleSet.getSize(); i++) {
        muscleSet.get(i).set_appliesForce(false);
    }

    //add torque actuators (to be controlled by ID input)
    SimTK::Vec3 FDshoulderAxis1 = FDshoulderCustomJoint.getSpatialTransform().get_rotation1().getAxis();
    SimTK::Vec3 FDshoulderAxis2 = FDshoulderCustomJoint.getSpatialTransform().get_rotation2().getAxis();
    std::cout << FDshoulderAxis1 << "<-- this is the shoulder axis rot1\n";
    std::cout << FDshoulderAxis2 << "<-- this is the shoulder axis rot2\n";
    FDshoulderTorque1.setName("shoulder_elev_T");
    FDshoulderTorque1.set_bodyA("base");
    FDshoulderTorque1.set_bodyB("r_humerus");
    FDshoulderTorque1.set_torque_is_global(false);
    FDshoulderTorque1.setAxis(FDshoulderAxis1);
    FDshoulderTorque1.setOptimalForce(OPTIMAL_TORQUE) ; // N.m (maximum torque supposedly)

    FDshoulderTorque2.setName("shoulder_rot2_T");
    FDshoulderTorque2.set_bodyA("base");
    FDshoulderTorque2.set_bodyB("r_humerus");
    FDshoulderTorque2.set_torque_is_global(false);
    FDshoulderTorque2.setAxis(FDshoulderAxis2);
    FDshoulderTorque2.setOptimalForce(OPTIMAL_TORQUE) ; // N.m (maximum torque supposedly)


    SimTK::Vec3 FDelbowFlexAxis = FDelbowCustomJoint.getSpatialTransform().get_rotation1().getAxis();
    FDelbowTorque.set_bodyA("r_humerus");
    FDelbowTorque.set_bodyB("r_ulna_radius_hand");
    FDelbowTorque.set_torque_is_global(false);
    FDelbowTorque.setAxis(FDelbowFlexAxis);
    FDelbowTorque.setOptimalForce(OPTIMAL_TORQUE);

    FDModel.addForce(&FDshoulderTorque1);
    FDModel.addForce(&FDshoulderTorque2);
    FDModel.addForce(&FDelbowTorque);
    FDModel.finalizeConnections();
    
    //set default shoulder torque to zero
    SimTK::Vector shoulderControls1(1);
    shoulderControls1(0) = 0;
    SimTK::Vector shoulderControls2(1);
    shoulderControls2(0) = 0;

    SimTK::Vector modelControls = FDModel.getDefaultControls();
    FDshoulderTorque1.addInControls(shoulderControls1,modelControls);
    FDshoulderTorque2.addInControls(shoulderControls2,modelControls);

    //set default elbow torque
    SimTK::Vector elbowControls(1);
    elbowControls(0) = 0;

    modelControls = FDModel.getDefaultControls();
    FDelbowTorque.addInControls(elbowControls, modelControls);

    //initialise model and get state
    FDModel.setUseVisualizer(true);
    SimTK::State& si = FDModel.initSystem();

    
    //create manager and save it to class variable
    FDmanager = new OpenSim::Manager(FDModel);

    FDmanager->setIntegratorAccuracy(FDtimestep);

    FDelbowCustomJoint.getCoordinate().setValue(si,convertDegreesToRadians(90));

    //clamp joints to within limits
    FDelbowCustomJoint.getCoordinate().setClamped(si,true);
    for (int i=0; i<FDshoulderCustomJoint.numCoordinates(); i++) {
        FDshoulderCustomJoint.get_coordinates(i).setClamped(si,true);
    }
    // FDshoulderCustomJoint.getCoordinate().setClamped(si,true);
    
    
    FDModel.print("FDisosimModel.osim");
    FDModel.printDetailedInfo(si, std::cout);
   
   
    si.setTime(initialTime);
    FDmanager->initialize(si);


    
    std::cout << "FDmodel generated\n";

    return true;
}




void IsosimEngine::step(void) {

    #ifdef IDFD
    //step id first using IsosimEngine::inverseD
    IsosimEngine::ID_Output IDout = inverseD();
    

    //get output torques in a form that is usable to the FD engine

    //step fd using IsosimEngine::forwardD
    IsosimEngine::FD_Output FDout = forwardD(IDout);
    #else
    //do the FD without bothwering with inverseD
    // IsosimEngine::FD_Output FDout = forwardInverseD();
    #endif
    //publish using comms::publisher
    commsClient.publishState(FDoutputToIsosimData(FDout));
    
    // (maybe this last one can be done by a callback function)
    // (so we just set the latest state and that gets transmitted to commshub)

    #ifdef LOGGING
        logger << FDout.timestamp << "   " << FDout.wristPos << std::endl;
    #endif

    currentSimTime += IDtimestep; //shouldn't be necessary with control input //TODO this
}


IsosimEngine::ID_Output IsosimEngine::inverseD(void) {

    IsosimEngine::ID_Input input(forceVecToInput(commsClient.get_latest_force())); //need to make this threadsafe eventually
    IsosimEngine::ID_Output output;
    output.valid = false; //change to true later if data
    output.timestamp = input.timestamp;




    SimTK::Integrator* integrator_ = &IDmanager->getIntegrator();

    SimTK::State state_ = integrator_->getAdvancedState();
    IDModel.getMultibodySystem().realize(state_, Stage::Dynamics); //just for adding in controls (will do this again)

    Vector controls(1);
    controls(0) = input.forceMag / endEffector.get_optimal_force();

    SimTK::Vec3 direction = input.forceDirection;
    endEffector.set_direction(direction);

    Vector modelControls = IDModel.getDefaultControls();
    endEffector.addInControls(controls, modelControls);

    IDModel.setControls(state_,modelControls);



    IDModel.getMultibodySystem().realize(state_, Stage::Dynamics);

    const Vector& appliedMobilityForces = 
                IDModel.getMultibodySystem().getMobilityForces(state_, Stage::Dynamics);

    const SimTK::Vector Udot = Test::randVector(state_.getNU())*0; //zero for isometric

    SimTK::Vector_<SimTK::SpatialVec> appliedBodyForces(IDModel.getMultibodySystem().getRigidBodyForces(state_, SimTK::Stage::Dynamics));

    output.residualMobilityForces = idSolver->solve(state_,Udot,appliedMobilityForces,appliedBodyForces);


    // std::cout << "residualmob: " << output.residualMobilityForces << " from forcevec: " <<  latestForce << "\n magnitude: " << input.forceMag << " and direction: " << input.forceDirection << std::endl;
    output.valid = true;
    return output;

}


IsosimEngine::ID_Input IsosimEngine::forceVecToInput (SimTK::Vec3 forceVector) {

    IsosimEngine::ID_Input input;
    input.timestamp = currentSimTime;

    input.forceMag = sqrt(~forceVector * forceVector);
    if (input.forceMag == 0) {
        input.forceDirection = {1,1,1}; //no force anyway
    } else {
        input.forceDirection = forceVector / input.forceMag;
    }

    return input;

}



IsosimEngine::FD_Output IsosimEngine::forwardD(IsosimEngine::ID_Output input) {

    IsosimEngine::FD_Output output;
    output.valid = false; //we'll change this to true once we have data
    output.timestamp = input.timestamp;

    if (input.valid == false) {
        std::cout << "invalid ID output provided as input to FD\n";
        return output;
    }

    
 

    // std::cout << "INPUT PROPERTIES\n resmob: " << input.residualMobilityForces << "timestamp: " << input.timestamp << "valid?: " << input.valid << std::endl;

    SimTK::Integrator* integrator_ = &FDmanager->getIntegrator();

    

    const SimTK::State& state_ = integrator_->getAdvancedState(); //this needs to be a pointer, since we plan on doing fun things to it
    
    const OpenSim::CustomJoint& FDshoulderCustomJoint =  FDModel.getComponent<OpenSim::CustomJoint>("/jointset/r_shoulder");
    const OpenSim::CustomJoint& FDelbowCustomJoint =  FDModel.getComponent<OpenSim::CustomJoint>("/jointset/r_elbow");
    
    
    // double shoulderMaxElev =  FDshoulderCustomJoint.getCoordinate().getRangeMax();
    // double shoulderMinElev =  FDshoulderCustomJoint.getCoordinate().getRangeMin();
    double elbowMaxFlex = FDelbowCustomJoint.getCoordinate().getRangeMax();
    double elbowMinFlex = FDelbowCustomJoint.getCoordinate().getRangeMin();


    //possible need to realize to dynamics here. But how will we fd after?
    // FDModel.getMultibodySystem().realize(state_, Stage::Velocity);


    Vector shoulderControls1_(1);
    Vector shoulderControls2_(1);
    Vector elbowControls_(1);
    
    shoulderControls1_(0) = input.residualMobilityForces(
        FDshoulderCustomJoint.get_coordinates(0).getMobilizerQIndex()) / FDshoulderTorque1.getOptimalForce();
    shoulderControls2_(0) = input.residualMobilityForces(
        FDshoulderCustomJoint.get_coordinates(1).getMobilizerQIndex()) / FDshoulderTorque2.getOptimalForce();
    elbowControls_(0) = input.residualMobilityForces(
        FDelbowCustomJoint.getCoordinate().getMobilizerQIndex()) / FDelbowTorque.getOptimalForce();

    //if controls are taking us past the limit, set the control to zero
    // if ((shoulderControls1_(0) > 0 && state_.getQ()[SHOULDER_NUM] > shoulderMaxElev) ||
    //         (shoulderControls1_(0) < 0 && state_.getQ()[SHOULDER_NUM] < shoulderMinElev)) {
    //     shoulderControls1_(0) = 0;
    //     std::cout << "MAXXXXXXXED"; 
    //     // while(1) {std::cout << "hi?" << state_.getQ();}
    // }
    if ((elbowControls_(0) > 0 && state_.getQ()[ELBOW_NUM] > elbowMaxFlex) ||
            (elbowControls_(0) < 0 && state_.getQ()[ELBOW_NUM] < elbowMinFlex)) {
        elbowControls_(0) = 0;
        // std::cout << "MAXXXXXXXED"; while(1) {std::cout << "hey? ";}
    }

    Vector modelControls_ = FDModel.getDefaultControls();
    FDshoulderTorque1.addInControls(shoulderControls1_,modelControls_);
    FDshoulderTorque2.addInControls(shoulderControls2_,modelControls_);

    FDelbowTorque.addInControls(elbowControls_,modelControls_); //TODO: does this work or do I need to split in two steps?

    FDModel.setControls(state_,modelControls_);

    FDModel.getMultibodySystem().realizeTopology();

    //step simulation
    integrator_->stepBy(FDtimestep); //eventually will need stepTo() because we might miss timesteps

    SimTK::State newState_ = integrator_->getAdvancedState(); //TODO: should this work with the State& state_ now that I changed it to a pointer?
                                                            //TODO: actually, didn't I change it to a pointer??? I thought I called updAdvancedState().... not sure what's happening here

    FDModel.getVisualizer().show(newState_);

    //realize again so we can get state variables (can reduce the stage later if we decide we don't need U/Udot -- //TODO)
    // FDModel.getMultibodySystem().realize(newState_, Stage::Acceleration);


    //find positions in ground frame
        // SimTK::Vec3 humerusPos = FDModel.getBodySet().get("r_humerus").getPositionInGround(newState_); //humerus pos doesn't change (shoulder is fixed location)
    output.elbowPos = FDModel.getBodySet().get("r_ulna_radius_hand").getPositionInGround(newState_);
    output.wristPos = FDModel.getMarkerSet().get("r_radius_styloid").getLocationInGround(newState_);
    if (output.elbowPos.size() > 0) {
        output.valid = true;
    }
    return output;

    



}

/*
IsosimEngine::FD_Output IsosimEngine::forwardInverseD(void) {

    IsosimEngine::ID_Input input(forceVecToInput(latestForce)); //need to make this threadsafe eventually

    IsosimEngine::FD_Output output;
    output.timestamp = input.timestamp;

    SimTK::Integrator* integrator_ = &IDmanager->getIntegrator();
    SimTK::State& state_ = integrator_->updAdvancedState();

    
    IDModel.getMultibodySystem().realize(state_,Stage::Dynamics);
    
    Vector controls(1);
    controls(0) = input.forceMag / endEffector.get_optimal_force();

    SimTK::Vec3 direction = input.forceDirection;
    endEffector.set_direction(direction);

    Vector modelControls = IDModel.getDefaultControls();
    endEffector.addInControls(controls, modelControls);

    IDModel.setControls(state_,modelControls);
    IDModel.getMultibodySystem().realize(state_, Stage::Acceleration);//del

    //step FD
    integrator_->stepBy(FDtimestep);

    state_.getQ();
    
    // SimTK::clampInPlace( shoulderElevRange(0), state_.updQ()(0),shoulderElevRange(1));
    // SimTK::clampInPlace(elbowFlexRange(0),state_.updQ()(1),elbowFlexRange(1));
    // SimTK::State newState_ = integrator_->getAdvancedState();
    // IDModel.getMultibodySystem().realize(newState_, Stage::Acceleration);

    // IDModel.getControls(newState_).dump("new state controls");

    IDModel.getVisualizer().show(state_);
    output.q = state_.getQ();
    output.u = state_.getU();
    output.uDot = state_.getUDot();

    std::cout << latestForce << "<-- force at time -->" << output.timestamp << std::endl;
    std::cout << output.q << "<-- Q at time -->" << output.timestamp << std::endl;
    std::cout << output.uDot << "<-- uDot at time -->" << output.timestamp << std::endl; 


    output.valid = true;

    return output;
}
*/


IsosimROS::IsosimData IsosimEngine::FDoutputToIsosimData(IsosimEngine::FD_Output calculatedState) {


    IsosimROS::IsosimData outputToRos;
    if (calculatedState.valid == true) {
        outputToRos.timestamp = calculatedState.timestamp;
        // outputToRos.q = calculatedState.q;
        // outputToRos.u = calculatedState.u;
        // outputToRos.uDot = calculatedState.uDot;
        outputToRos.wristPos = calculatedState.wristPos;
        outputToRos.elbowPos = calculatedState.elbowPos;
        outputToRos.valid = true;
    } else {
        std::cout << "INVALID OUTPUT FROM FD";
        outputToRos.valid = false;
    }
    return outputToRos;
}


IsosimEngine::~IsosimEngine() {

    delete[] idSolver;
    delete[] IDmanager;

}



/////TEST FUNCTION (OBSOLETE)
/*
void IsosimEngine::testPointActuator(void) {

    try {

        OpenSim::Model model =  OpenSim::Model("Models/arm26.osim"); std::cout << "loaded model from arm26" << std::endl;
        const OpenSim::BodySet& IDbodyset = model.get_BodySet();
        const OpenSim::Body& humerusbod = IDbodyset.get("r_humerus");
        const OpenSim::Body& radiusbod = IDbodyset.get("r_ulna_radius_hand");
        std::cout << humerusbod.getName() << "<---name of humerusbod\n";
        std::cout << radiusbod.getName() << "<---name of radiusbod\n";
        IDbodyset.print("bodyset.bods");
        model.setUseVisualizer(true);
        model.initSystem();

        OpenSim::PointActuator* pAct = new OpenSim::PointActuator(radiusbod.getName());
        pAct->setName("pact");
        double optforce = 1;
        pAct->setOptimalForce(optforce);
        pAct->set_force_is_global(true);
        pAct->set_point(SimTK::Vec3(1,1,1));
        pAct->set_point_is_global(false);
        model.addForce(pAct);
        model.finalizeConnections();

        SimTK::State& state1 = model.initSystem();

        Vector pointActuatorControls(1); // input to addInControl should be a Vector
        pointActuatorControls(0) = optforce; // axis already defined when initializing
        SimTK::Vec3 forceInG(1,1,1);
        Vector pointActuatorVector(3); // to print out the whole force vector
        for (int i = 0; i < 3; i++){
            pointActuatorVector(i) = forceInG(i);
        }
        pointActuatorVector.dump("Forces applied by the point Actuator:");

        // Add control values and set their values
        Vector modelControls = model.getDefaultControls();
        pAct->addInControls(pointActuatorControls, modelControls);
        modelControls.dump("model controls post adding:");
        model.setDefaultControls(modelControls);
        std::cout << "hey hey, no segfault\n";
        model.computeStateVariableDerivatives(state1);
        Vector udotActuatorsCombination = state1.getUDot();
        udotActuatorsCombination.dump("Accelerations due to actuator");


        OpenSim::Manager manager(model);
        manager.setIntegratorAccuracy(1e-3);
        state1.setTime(0);
        manager.initialize(state1);
        double endt = 1;
        manager.integrate(endt);




    }
    catch(const std::exception& ex) {
        std::cout << "Exception here : " << ex.what() <<  std::endl;
        return;
    }
};

*/





// } //namespace isosim