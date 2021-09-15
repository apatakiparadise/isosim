/*******************************************************
 * Demonstration of forward dynamics using OpenSim API
 * Calculated in real-time
 * Author: Joshua Rolls
 * Date: 01.09.21
 ******************************************************/ 
#include <OpenSim/OpenSim.h>

#include <ctime>    // for clock()


using namespace OpenSim;
using namespace SimTK;

int createPendulumModel(void);
void step(double timestep);

int main()
{
    try {
        // Create an OpenSim model and set its name
        int res = createPendulumModel();    

    }
    catch (const OpenSim::Exception& ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    

    std::cout << "OpenSim example completed successfully.\n";
    std::cin.get();
    return 0;
}


int createPendulumModel(void) {

    //////////////////////
    // MODEL PARAMETERS //
    //////////////////////

    //BodyMass
    double headMass = 20.0, headRadius = 0.1;

    //Arm length 1m
    double armLength = 1.0;

    //Clearance height
    double clearance = 0.5;

    double startAngle = SimTK::Pi/2;
    
    ///////////////////////////////////////////
    // DEFINE BODIES AND JOINTS OF THE MODEL //
    ///////////////////////////////////////////

    Model osimModel;
    osimModel.setName("PendulumModel");
    std::cout << "Model name: " << osimModel.getName() << '\n';

    //get ground frame
    Ground& ground = osimModel.updGround();

    //Create Frames to attach geometry to
    //x,y,z with y as vertical
    OpenSim::PhysicalFrame* anchorFrame = new PhysicalOffsetFrame(ground, Transform(Vec3(0, 0, 0)));
    anchorFrame->setName("anchorframe");
    osimModel.addComponent(anchorFrame);


    OpenSim::PhysicalFrame* xFrame = new PhysicalOffsetFrame(*anchorFrame, Transform(Vec3(armLength+clearance, 0, 0)));
    xFrame->setName("xframe");
    osimModel.addComponent(xFrame);

    OpenSim::PhysicalFrame* yFrame = new PhysicalOffsetFrame(*anchorFrame, Transform(Vec3(0, armLength+clearance, 0)));
    yFrame->setName("yframe");
    osimModel.addComponent(yFrame);

    OpenSim::PhysicalFrame* zFrame = new PhysicalOffsetFrame(*anchorFrame, Transform(Vec3(0, 0, armLength+clearance)));
    zFrame->setName("zframe");
    osimModel.addComponent(zFrame);


    //add display geometry
    ground.attachGeometry(new Mesh("checkered_floor.vtp"));

    //add anchor to fix pendulum to
    Brick* anchorGeometry = new Brick(SimTK::Vec3(0.05, 0.05, 0.05));
    anchorGeometry->upd_Appearance().set_color(SimTK::Vec3(1.0, 0.5, 1.0));
    
    anchorFrame->attachGeometry(anchorGeometry);

    Brick* xptrGeometry = new Brick(SimTK::Vec3(0.05, 0.05, 0.05));
    xptrGeometry->upd_Appearance().set_color(SimTK::Vec3(1.0, 0.0, 0.0));
    xFrame->attachGeometry(xptrGeometry);


    Brick* yptrGeometry = new Brick(SimTK::Vec3(0.05, 0.05, 0.05));
    yptrGeometry->upd_Appearance().set_color(SimTK::Vec3(0.0, 1.0, 0.0));
    yFrame->attachGeometry(yptrGeometry);


    Brick* zptrGeometry = new Brick(SimTK::Vec3(0.05, 0.05, 0.05));
    zptrGeometry->upd_Appearance().set_color(SimTK::Vec3(0.0, 0.0, 1.0));
    zFrame->attachGeometry(zptrGeometry);

    Brick* gndGeometry = new Brick(SimTK::Vec3(.01, .01, .01));
    gndGeometry->upd_Appearance().set_color(SimTK::Vec3(1.0, 1.0, 1.0));
    ground.attachGeometry(gndGeometry);



    //add pendulum
    SimTK::Vec3 pendLoc(0,0,clearance);
    // ContactSphere* pendSphereGeometry = new ContactSphere(headRadius,pendLoc,*anchorFrame);
    Sphere* pendSphereGeometry = new Sphere(headRadius);
    pendSphereGeometry->upd_Appearance().set_color(SimTK::Vec3(0.5, 1.0, 0.0));

    // pendFrame->attachGeometry(pendSphereGeometry);


    // BLOCK BODY
    Vec3 headMassCenter(1,1,1);
    Inertia headInertia = headMass*Inertia::sphere(headRadius);

    // Create a new block body with the specified properties
    OpenSim::Body *head = new OpenSim::Body("sphere", headMass, headMassCenter, headInertia);

    // Add display geometry to the block to visualize in the GUI
    head->attachGeometry(new Mesh("block.vtp"));
    
    // Use attachGeometry to set frame name & addGeometry
    // head->attachGeometry(new Sphere(headRadius*100));
    head->attachGeometry(pendSphereGeometry);

    // FREE JOINT

    // Create a new free joint with 6 degrees-of-freedom (coordinates) between the head and anchor frames
    Vec3 locationInParent(0, headRadius, 0), orientationInParent(0), locationInBody(0), orientationInBody(0);
    FreeJoint *headToAnchor = new FreeJoint("headToAnchor", *anchorFrame, locationInParent,orientationInParent,
            *head, locationInBody, orientationInBody);


    headToAnchor->connectSocket_parent_frame(*anchorFrame); //important
    
    //...
    // Set the angle and position ranges for the free (6-degree-of-freedom)
    // joint between the block and ground frames.
    double angleRange[2] = {-SimTK::Pi, SimTK::Pi};
    double positionRange[2] = {-armLength, armLength};
    headToAnchor->updCoordinate(FreeJoint::Coord::Rotation1X).setRange(angleRange);
    headToAnchor->updCoordinate(FreeJoint::Coord::Rotation2Y).setRange(angleRange);
    headToAnchor->updCoordinate(FreeJoint::Coord::Rotation3Z).setRange(angleRange);
    headToAnchor->updCoordinate(FreeJoint::Coord::TranslationX).setRange(positionRange);
    headToAnchor->updCoordinate(FreeJoint::Coord::TranslationY).setRange(positionRange);
    headToAnchor->updCoordinate(FreeJoint::Coord::TranslationZ).setRange(positionRange);


    // GRAVITY
    // Obtain the default acceleration due to gravity
    Vec3 gravity = osimModel.getGravity();
    std::cout << gravity;
    gravity = gravity * 0.5;
    osimModel.setGravity(gravity);
    // Define non-zero default states for the free joint
    // headToAnchor->updCoordinate(FreeJoint::Coord::TranslationX)
                    // .setDefaultValue(armLength/2);
    // double h_start = headMass*gravity[1] /
                        // (stiffness*blockSideLength*blockSideLength);
    std::cout << "so far, so good!\n";
    // headToAnchor->updCoordinate(FreeJoint::Coord::Rotation3Z)
                    // .setDefaultValue(startAngle*0); 

    headToAnchor->updCoordinate(FreeJoint::Coord::TranslationX)
                    .setDefaultValue(armLength); 
    headToAnchor->updCoordinate(FreeJoint::Coord::TranslationY)
                    .setDefaultValue(0); 
    headToAnchor->updCoordinate(FreeJoint::Coord::TranslationZ)
                    .setDefaultValue(0); 



    // Add the block and joint to the model
    osimModel.addBody(head);
    osimModel.addJoint(headToAnchor);


    ///////////////////////////////////////////////
    // DEFINE THE SIMULATION START AND END TIMES //
    ///////////////////////////////////////////////

    // Define the initial and final simulation times
    double initialTime = 0.0;
    double finalTime = 30.00;//*0.01;    



    /////////////////////////////////////////////
    // DEFINE CONSTRAINTS IMPOSED ON THE MODEL //
    /////////////////////////////////////////////
    Vec3 pointOnAnchor(0, armLength+clearance ,0);
    Vec3 pointOnHead(0, 0, 0);

    // Create a new constant distance constraint
    ConstantDistanceConstraint *constDist = 
        new ConstantDistanceConstraint(*anchorFrame, 
            pointOnAnchor, *head, pointOnHead, armLength);

    // Add the new point on a line constraint to the model
    osimModel.addConstraint(constDist);


    std::cout << "body1 loc: " << constDist->get_location_body_1();
    std::cout << "\nso far, so bad\n";
    std::cout << "body1:" << constDist->getBody1() << "\nbody2: " << constDist->getBody2() << std::endl;
    //////////////////////////
    // PERFORM A SIMULATION //
    //////////////////////////

    // set use visualizer to true to visualize the simulation live
    osimModel.setUseVisualizer(true);

    // Initialize the system and get the default state
    SimTK::State& si = osimModel.initSystem();
    // Enable constraint consistent with current configuration of the model
    constDist->setIsEnforced(si, true);

    osimModel.getMultibodySystem().realize(si, Stage::Velocity);



    std::cout << "anchorframe transform: " << anchorFrame->getTransformInGround(si)<< "\n";

    std::cout << "so far, so worse\n";
    // Create the manager managing the forward integration and its outputs
    Manager manager(osimModel);
    manager.setIntegratorAccuracy(1.0e-6);

    // Print out details of the model
    osimModel.printDetailedInfo(si, std::cout);

    // Integrate from initial time to final time
    si.setTime(initialTime);
    manager.initialize(si);
    std::cout<<"\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
    // manager.integrate(finalTime);

    SimTK::Integrator* integrator_ = &manager.getIntegrator();
    std::cout << "we have an integrator    ";
    SimTK::State state_ = integrator_->getAdvancedState();
    // OpenSim::Model model_ = osimModel;

    osimModel.getVisualizer().show(state_);

    double timestep = integrator_->getAccuracyInUse();

    double simtime = 0;

    clock_t startTime = clock();
    std::cout << startTime/CLOCKS_PER_SEC << "<----clock\n";
    // clock_t currTime = clock();
    while (simtime < finalTime) {
        // if ((clock()-startTime)/CLOCKS_PER_SEC > simtime      ) {
            integrator_->stepBy(timestep);
            break;
            state_ = integrator_->getAdvancedState();
            osimModel.getVisualizer().show(state_);
            simtime+=timestep;
        // }
    }
    time_t simulationTime = clock() - startTime;


    std::cout << "time taken to run simulation: " << 1.e3*(simulationTime)/CLOCKS_PER_SEC << "ms" << std::endl;

    




}


void step(double timestep) {

    //do nothing
}