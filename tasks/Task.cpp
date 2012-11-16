/**
 * @author  Christian Rauch <Christian.Rauch@dfki.de>
 * @version 1.0
 * @date 16.11.2012 (dd/mm/yyyy)
 */
 
#include "Task.hpp"
#include<base/logging.h>
#include<boost/tokenizer.hpp>

using namespace mr_control;

//Task::Task(std::string const& name, TaskCore::TaskState initial_state)
//    : TaskBase(name, initial_state)
Task::Task(std::string const& name) //needs_configuration
    : TaskBase(name), wheel_pos({0,0})
{
}

//Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
//    : TaskBase(name, engine, initial_state)
Task::Task(std::string const& name, RTT::ExecutionEngine* engine) //needs_configuration
    : TaskBase(name, engine), wheel_pos({0,0})
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    // Read number of wheels
    nwheels = _wheels.get();
    
    ArArgumentBuilder *MRarguments = new ArArgumentBuilder();
    
    MRarguments->add("-robotPort");
    MRarguments->add(_serial_port.get().c_str());
    
    // Use boost tokenizer to read the ports that should turned on by default on
    // boot. 'poweron_boot' is of type std::string where all values 
    // (portnumbers) are separated by comma or space (e.g., "3 6 7 9 10").
    boost::tokenizer<> tokPortList(_poweron_boot.get());
    std::string nrstr;
    for(boost::tokenizer<>::iterator tokit=tokPortList.begin();
    	tokit!=tokPortList.end(); ++tokit){
    	nrstr = *tokit;
    	PowerPortsON.push_back(atoi(nrstr.c_str()));
    }
    
    LOG_INFO("Aria: List of Parameters: %s", MRarguments->getFullString());
    
    MRparser = new ArArgumentParser(MRarguments);
    MRparser->loadDefaultArguments();
    //MRparser->log(); //print unprocessed arguments
    
    if(!MRparser->checkHelpAndWarnUnparsed())
    	LOG_WARN("Aria: Unparsed Arguments found");
	
    LOG_INFO("Aria: Used Parameters: %s",*MRparser->getArgv());
    
    delete MRarguments;
    MRarguments = 0;
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    
    // Reset index of updateHook
    index = 0;
    
    // Initialise Aria
    Aria::init();
    
    LOG_INFO("Aria: Initialised.")

    //ArLog::init(ArLog::None, ArLog::Normal);
    // Verbose Aria-Logging into Logfile "MrControl_AriaLog.log"
    //ArLog::init(ArLog::File, ArLog::Verbose, "MrControl_AriaLog.log", false, true);
    
    LOG_DEBUG_S<<"Aria: Creating new ArRobot.";
    MRrobot = new ArRobot("", true, false);
    LOG_DEBUG_S<<"Aria: Creating new ArRobotConnector.";
    MRconnector = new ArRobotConnector(MRparser, MRrobot);
    
    // Connect to Robot or Simulator
    LOG_DEBUG_S<<"Aria: Connecting Robot.";
    bool connectsuccess = MRconnector->connectRobot(MRrobot);
    LOG_DEBUG_S<<"Aria: Robot connected? "<<connectsuccess;
    
    if (!connectsuccess){
        LOG_ERROR("Aria: Could not connect!");
        ArLog::log(ArLog::Normal, "Error, could not connect to robot.");
        //Aria::logOptions(); // show parameters
        //Aria::exit(1);
        return false;
    }
    else{
        LOG_INFO("Aria: Robot connected.")
    }
    
    LOG_DEBUG_S<<"Aria: Initialising asynchronous Thread.";
    
    // Open new thread for processing cycle
    MRrobot->runAsync(false);
    LOG_INFO("Aria: Thread started.")
    
    // Turn ON default Power-Ports
    for(std::vector<int>::iterator portsit=PowerPortsON.begin(); portsit!=PowerPortsON.end(); ++portsit){
        controlPDB(*portsit, 1);
    }
    
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
    // Count updateHook calls
    index++;
    
    // Write Commands to Robot
    base::MotionCommand2D MRmotion;
    double MRtransVel, MRrotVel;
    
    // Process Motion Commands
    // MotionCommand2D
    if (_transrot_vel.read(MRmotion) != RTT::NoData){
        LOG_DEBUG("Aria: TranslVel %.3f m/s, RotVel %.3f rad/s", MRmotion.translation, MRmotion.rotation);
        
        MRrobot->lock();
        
        MRrobot->setVel(MRmotion.translation * 1000);
        MRrobot->setRotVel(MRmotion.rotation * 180 / M_PI);
        
        MRrobot->unlock();
    }
    
    // AA: goForward/goBackward
    if (_aa_transl_vel.read(MRtransVel) != RTT::NoData){
        MRrobot->lock();
        MRrobot->setVel(MRtransVel * 1000);
        MRrobot->unlock();
    }
    
    // AA: turnLeft/turnRight
    if (_aa_rot_vel.read(MRrotVel) != RTT::NoData){
        MRrobot->lock();
        MRrobot->setRotVel(MRrotVel * 180 / M_PI);        
        MRrobot->unlock();
    }
    
    // Read Sensor Data from Robot
    base::samples::RigidBodyState MRpose;
    base::actuators::Status MRmotorstatus;
    // Resize Motor States to number of wheels
    MRmotorstatus.resize(nwheels);
    
    AriaTypes::samples::Velocity MRvel;
    AriaTypes::samples::Velocity2 MRvel2;
    AriaTypes::samples::BatteryLevel MRbatteryLevel;
    AriaTypes::samples::Temperature MRtemperature;
    AriaTypes::samples::CompassHeading MRcompass;
    
    AriaTypes::samples::Odometer MRodom;
    AriaTypes::samples::Encoder MRenc;
    
    AriaTypes::samples::Bumpers MRbumpers;
    
    // Lock Robot for Reading
    MRrobot->lock();
    
    double diffconvfactor = MRrobot->getRobotParams()->getDiffConvFactor();
    
    // Position
    MRpose.time = base::Time::now();
    MRpose.position = Eigen::Vector3d(MRrobot->getX() / 1000, MRrobot->getY() / 1000, 0); // in meters
    MRpose.orientation = Eigen::AngleAxis<double>(MRrobot->getTh() * M_PI/180, Eigen::Vector3d::UnitZ()); // rad
    
    MRpose.velocity = Eigen::Vector3d(MRrobot->getVel() / 1000, 0, 0); // m/s
    MRpose.angular_velocity = Eigen::Vector3d(MRrobot->getRotVel() * M_PI/180, 0, 0); // rad/s
    
    // Velocity
    MRvel.time = base::Time::now();
    MRvel.velTransRot.translation = MRrobot->getVel() / 1000; // in m/s
    MRvel.velTransRot.rotation = MRrobot->getRotVel() * M_PI/180; // in rad/s
    
    // Velocity2 (left, right)
    MRvel2.time = base::Time::now();
    MRvel2.velLeft = MRrobot->getLeftVel() / 1000; // in m/s
    MRvel2.velRight = MRrobot->getRightVel() / 1000; // in m/s
    
    // Battery
    MRbatteryLevel.time = base::Time::now();
    MRbatteryLevel.battery = MRrobot->getStateOfCharge();
    
    // Temperature
    MRtemperature.time = base::Time::now();
    MRtemperature.temp = MRrobot->getTemperature();
    
    // Compass
    MRcompass.time = base::Time::now();
    MRcompass.heading = MRrobot->getCompass();
    
    // Odomerty
    MRodom.time = base::Time::now();
    MRodom.odomDistance = MRrobot->getTripOdometerDistance() / 1000; // in m
    MRodom.odomAngle = MRrobot->getTripOdometerDegrees() * M_PI/180; // in rad
    
    // Motor State
    MRmotorstatus.time = base::Time::now();
    MRmotorstatus.index = index;
    
    base::Time dt;
    if(index <= 1){
        // first cycle, there is not time difference to previous cycle
        dt = base::Time::fromMilliseconds(0);
    }
    else{
        dt = base::Time::now() - t_prev;
    }
    
    // get some properties from the robot specific parameter file (aria/params/*.p)
//    ArRobotParams params = MRrobot->getRobotParams();
//    getVel2Divisor(); // multiplier for VEL2 commands.
//    getVelConvFactor(); // velocity conversion factor.
    //diffconvfactor = angular_vel / wheel_vel = 0.0056 ?
    
    // get rotation of wheels by traveled distance per side (through velovcity and delta time)
    wheel_pos[0] += MRrobot->getLeftVel() * diffconvfactor * dt.toSeconds();
    wheel_pos[1] += MRrobot->getRightVel() * diffconvfactor * dt.toSeconds();
    
    // remove full turns (2*pi)
    //wheel_pos[0] = fmod(wheel_pos[0], 2*M_PI);
    //wheel_pos[1] = fmod(wheel_pos[1], 2*M_PI);
    
    MRmotorstatus.states[odometry::FRONT_LEFT].position = wheel_pos[0]; // front left
    MRmotorstatus.states[odometry::REAR_LEFT].position = wheel_pos[0]; // rear left
    MRmotorstatus.states[odometry::FRONT_RIGHT].position = wheel_pos[1]; // front right
    MRmotorstatus.states[odometry::REAR_RIGHT].position = wheel_pos[1]; // rear right
    
    
    // Raw Data from left and right Encoders
    MRenc.time = base::Time::now();
    MRrobot->requestEncoderPackets();
    MRenc.encLeft = MRrobot->getLeftEncoder();
    MRenc.encRight = MRrobot->getRightEncoder();
    // Reading Encoder Values is not yet working
    //LOG_DEBUG_S<<"Aria: Encoder L: "<<MRrobot->getLeftEncoder()<<", Encoder R: "<<MRrobot->getRightEncoder();
    MRrobot->stopEncoderPackets();
    
    // Bumpers
    // See Aria Documentation and/or ArModes.cpp for an Example how to read out these values.
    MRbumpers.time = base::Time::now();
    MRbumpers.nrFront = MRrobot->getNumFrontBumpers();
    MRbumpers.nrRear = MRrobot->getNumRearBumpers();
    
    MRbumpers.front = ((MRrobot->getStallValue() & 0xff00) >> 8);
    MRbumpers.rear = ((MRrobot->getStallValue() & 0xff));
    
    int maskFront = ((MRrobot->getStallValue() & 0xff00) >> 8);
    int maskRear = ((MRrobot->getStallValue() & 0xff));
    
    // Sensor Reading finished
    MRrobot->unlock();
    
    // Write Bumper States per Bumper into Vector
    bool frbump[MRbumpers.nrFront];
    bool rebump[MRbumpers.nrRear];
    int bit = 0;
    unsigned int i = 0;
    
    for(i = 0, bit = 2; i < MRrobot->getNumFrontBumpers(); i++, bit *= 2){
    	frbump[i] = (MRbumpers.front & bit);
    	MRbumpers.bumpersFront.push_back( (maskFront & bit) );
    }
    
    for (i = 0, bit = 2; i < MRrobot->getNumRearBumpers(); i++, bit *= 2){
    	rebump[i] = (MRbumpers.rear & bit);
    	MRbumpers.bumpersRear.push_back( (maskRear & bit) );
    }
    
    
    // Log Debug Information of front and rear Bumpers
//    std::stringstream debugstream;
//    
//    debugstream<<"Aria: Front Bumpers: ";
//    for(int i=0; i<MRbumpers.nrFront; i++)
//    {
//    	debugstream<<frbump[i]<<" ";
//    }
//    
//    LOG_DEBUG_S<<debugstream.str();
//    debugstream.clear();
//    debugstream.str("");
//    
//    
//    debugstream<<"Aria: Rear Bumpers: ";
//    for(int i=0; i<MRbumpers.nrRear; i++)
//    {
//    	debugstream<<rebump[i]<<" ";
//    }
//    
//    LOG_DEBUG_S<<debugstream.str();
//    debugstream.clear();
//    debugstream.str("");
        
    // Distribute Messages
    _robot_pose.write(MRpose);
    _robot_motion.write(MRvel);
    _robot_motion2.write(MRvel2);
    _robot_battery.write(MRbatteryLevel);
    _robot_temp.write(MRtemperature);
    _robot_compass.write(MRcompass);
    _robot_odometer.write(MRodom);
    _robot_encoder.write(MRenc);
    _robot_bumpers.write(MRbumpers);
    _motor_states.write(MRmotorstatus);
    
    // start of measurement
    t_prev = base::Time::now();
}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }

void Task::stopHook()
{
    
    TaskBase::stopHook();
    
    MRrobot->stopRunning();
    MRrobot->waitForRunExit();
    
    // Stop Aria background threads
    Aria::shutdown();
    
    delete MRconnector;
    MRconnector = 0;
    
    delete MRrobot;
    MRrobot = 0;
    
    LOG_INFO("Aria: Stopped.");
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    
    delete MRparser;
    MRparser = 0;
    
    // Reset number of wheels
    nwheels = 0;
    
    LOG_INFO("Aria: Exit.");
}

// Operation Methods

// Set the translational and rotational Velocities
void Task::transrotVel(::base::MotionCommand2D const & velocities)
{
	LOG_DEBUG("Aria: TranslVel %.3f m/s, RotVel %.3f rad/s", velocities.translation, velocities.rotation);
        
        MRrobot->lock();
        MRrobot->setVel(velocities.translation * 1000);
        MRrobot->setRotVel(velocities.rotation * 180 / M_PI);
        MRrobot->unlock();
}

void Task::transrotVel2(double translational, double rotational)
{
	LOG_DEBUG("Aria: TranslVel %.3f m/s, RotVel %.3f rad/s", translational, rotational);
	
	MRrobot->lock();
        MRrobot->setVel(translational * 1000);
        MRrobot->setRotVel(rotational * 180 / M_PI);
        MRrobot->unlock();
}

// Set Velocities for left and right Wheels
void Task::lrVel(double left, double right)
{
	LOG_DEBUG("Aria: Velocity L: %.3f m/s, R: .3f m/s", left, right);
	
	MRrobot->lock();
	MRrobot->setVel2(left*1000, right*1000);
	MRrobot->unlock();
}

// Turn on/off the PDB
void Task::controlPDB(boost::int32_t portNr, bool onoff)
{	
	std::string onoffstr;
	onoff ? onoffstr="ON":onoffstr="OFF";
	
	LOG_INFO("Aria: Turning Port %i %s",portNr, onoffstr.c_str());
    	
	// Send command #116, parameter: port-number, onoff (1=on, 0=off)
	MRrobot->lock();
	MRrobot->com2Bytes(116, portNr, onoff);
	MRrobot->unlock();
}

// Send a direct serial Command to Robot
void Task::directCommand(::AriaTypes::commands::DirectCommand2Byte const & MRcmd2byte)
{
	LOG_INFO("Aria: Direct Command %i with HB: %i LB: %i", MRcmd2byte.cmdnr, MRcmd2byte.highbyte, MRcmd2byte.lowbyte);
	
	MRrobot->lock();
	MRrobot->com2Bytes(MRcmd2byte.cmdnr, MRcmd2byte.highbyte, MRcmd2byte.lowbyte);
	MRrobot->unlock();
}

// Reset the Odometer
void Task::resetOdometer()
{
    LOG_INFO("Aria: Resetting Odometer");

    MRrobot->lock();
    MRrobot->resetTripOdometer();
    MRrobot->unlock();
}

