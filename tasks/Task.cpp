
#include "Task.hpp"
#include<base/logging.h>
#include<boost/tokenizer.hpp>

using namespace aria;

base::Time fromArTime(const ArTime& t) {
    base::Time b_time;
    b_time.microseconds = t.getMSec() * 1000 + t.getSec() * 1e6;
    return b_time;
}

Task::Task(std::string const& name) //needs_configuration
    : TaskBase(name)
    , MRarguments(0)
    , MRparser(0)
    , wheel_pos({0,0})
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine) //needs_configuration
    : TaskBase(name, engine)
    , MRarguments(0)
    , MRparser(0)
    , wheel_pos({0,0})
{
}

Task::~Task()
{
}

void Task::motorsOff()
{
    MRrobot->lock();
    if (MRrobot) MRrobot->enableMotors();
    MRrobot->unlock();
}
void Task::motorsOn()
{
    MRrobot->lock();
    if (MRrobot) MRrobot->disableMotors();
    MRrobot->unlock();
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    // Read number of wheels
    nwheels = _wheels.get();

    mTimeout = base::Time::fromSeconds(_timeout.get());
    LOG_INFO_S<<"Timeout is: "<<mTimeout;
    
    // Initialise Aria
    Aria::init();
    LOG_INFO("Aria: Initialised.")

    // set path to aria
    const std::string ariapath = _ariapath.get();
    struct stat dirstat;
    stat(ariapath.c_str(), &dirstat);

    if(!ariapath.empty() && S_ISDIR(dirstat.st_mode)) { 
        LOG_DEBUG_S<<"Using Aria path: "<<ariapath;
        Aria::setDirectory(ariapath.c_str());
    }
    else {
        LOG_WARN_S<<"No Aria path given or directory does not exist: \""<<ariapath<<"\"";
        LOG_WARN_S<<"Using default aria directory at: "<<Aria::getDirectory();
    }


    MRarguments = new ArArgumentBuilder();
    MRarguments->add("-robotPort");
    MRarguments->add(_serial_port.get().c_str());
    MRarguments->add(_parameter.get().c_str());
    MRarguments->add("-rb");
    char bd[10];
    snprintf(bd,10,"%d",_baudrate.get());
    MRarguments->add(bd);
    
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
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    
    // Reset index of updateHook
    index = 0;
    
    //ArLog::init(ArLog::None, ArLog::Normal);
    // Verbose Aria-Logging into Logfile "MrControl_AriaLog.log"
    //ArLog::init(ArLog::File, ArLog::Verbose, "MrControl_AriaLog.log", false, true);
    
    LOG_DEBUG_S<<"Aria: Creating new ArRobot.";
    MRrobot = new ArRobot("", true, false);
    LOG_DEBUG_S<<"Aria: Creating new ArRobotConnector.";
    MRconnector = new ArRobotConnector(MRparser, MRrobot);
    
    // Connect to Robot or Simulator
    LOG_DEBUG_S<<"Aria: Connecting Robot.";
    bool connectsuccess = MRconnector->connectRobot();
    LOG_DEBUG_S<<"Aria: Robot connected? "<<connectsuccess;
    
    if (!connectsuccess && !MRrobot->isConnected()){
        LOG_ERROR("Aria: Could not connect!");
        return false;
    }
    else{
        LOG_INFO("Aria: Robot connected.")
    }
    
    LOG_DEBUG_S<<"Aria: Initialising asynchronous Thread.";
    
    // Open new thread for processing cycle
    MRrobot->runAsync(true);
    LOG_INFO("Aria: Thread started.");
    
    MRrobot->enableMotors();
    LOG_INFO_S<<"Motors enabled.";
    
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
    base::commands::Motion2D MRmotion;

    base::Time t_now = base::Time::now();
    
    // Process Motion Commands
    // commands::Motion2D
    bool export_mcmd = false;
    base::samples::Motion2D command_in;
    if (_transrot_vel.read(MRmotion) == RTT::NewData){
        //LOG_DEBUG("Aria: TranslVel %.3f m/s, RotVel %.3f rad/s", MRmotion.translation, MRmotion.rotation);

        mLastCommandReceived = base::Time::now();

        if(MRrobot->lock() != 0) {
            // see enum ArMutex::Status for further information
            LOG_ERROR_S<<"Failed to get robot lock!";
        }

        MRrobot->setVel(MRmotion.translation * 1000);
        MRrobot->setRotVel(MRmotion.rotation * 180 / M_PI);

        MRrobot->unlock();
        
        command_in.translation = MRmotion.translation;
        command_in.rotation = MRmotion.rotation;

        export_mcmd = true;
    }
    else if((base::Time::now() - mLastCommandReceived) > mTimeout ) {
        // send default values after not receiving commands for a certain period
        //LOG_DEBUG_S<<"Timeout at: "<<base::Time::now();
        if(MRrobot->lock() != 0) {
            // see enum ArMutex::Status for further information
            LOG_ERROR_S<<"Failed to get robot lock!";
        }
        MRrobot->setVel(0);
        MRrobot->setRotVel(0);
        MRrobot->unlock();
        
        command_in.translation = 0;
        command_in.rotation = 0;

        export_mcmd = true;
    }

    if(export_mcmd) {
        command_in.time = base::Time::now();
        _robot_command_in.write(command_in);
    }
    
    
    // Read Sensor Data from Robot
    base::samples::RigidBodyState MRpose;
    base::samples::RigidBodyState MRposeraw;
    base::actuators::Status MRmotorstatus;
    // Resize Motor States to number of wheels
    MRmotorstatus.resize(nwheels);
    
    samples::Velocity MRvel;
    samples::Velocity2 MRvel2;
    samples::BatteryLevel MRbatteryLevel;
    samples::Temperature MRtemperature;
    samples::CompassHeading MRcompass;
    
    samples::Odometer MRodom;
    samples::Encoder MRenc;
    
    samples::Bumpers MRbumpers;
    
    // Lock Robot for Reading
    MRrobot->lock();
    
    double diffconvfactor = MRrobot->getRobotParams()->getDiffConvFactor();

    // Position
    MRpose.time = t_now;
    MRpose.sourceFrame = _body_frame.get();
    MRpose.targetFrame = _odometry_frame.get();
    MRpose.position = Eigen::Vector3d(MRrobot->getX() / 1000, MRrobot->getY() / 1000, 0); // in meters
    MRpose.orientation = Eigen::AngleAxis<double>(MRrobot->getTh() * M_PI/180, Eigen::Vector3d::UnitZ()); // rad
    
    MRpose.velocity = Eigen::Vector3d(MRrobot->getVel() / 1000, 0, 0); // m/s
    MRpose.angular_velocity = Eigen::Vector3d(MRrobot->getRotVel() * M_PI/180, 0, 0); // rad/s
    
    // Raw Position (without corrections by gyro or software if available)
    ArPose pose_raw = MRrobot->getRawEncoderPose();
    MRposeraw.time = t_now;
    MRposeraw.sourceFrame = _body_frame.get();
    MRposeraw.targetFrame = _odometry_frame.get();
    MRposeraw.position = Eigen::Vector3d(pose_raw.getX() / 1000, pose_raw.getY() / 1000, 0); // in meters
    MRposeraw.orientation = Eigen::AngleAxis<double>(pose_raw.getThRad(), Eigen::Vector3d::UnitZ()); // rad
    
    // Velocity
    MRvel.time = t_now;
    MRvel.velTransRot.translation = MRrobot->getVel() / 1000; // in m/s
    MRvel.velTransRot.rotation = MRrobot->getRotVel() * M_PI/180; // in rad/s
    
    // Velocity2 (left, right)
    MRvel2.time = t_now;
    MRvel2.velLeft = MRrobot->getLeftVel() / 1000; // in m/s
    MRvel2.velRight = MRrobot->getRightVel() / 1000; // in m/s
    
    // Battery
    MRbatteryLevel.time = t_now;
    MRbatteryLevel.battery = MRrobot->getStateOfCharge();
    
    // Temperature
    MRtemperature.time = t_now;
    MRtemperature.temp = MRrobot->getTemperature();
    
    // Compass
    MRcompass.time = t_now;
    MRcompass.heading = MRrobot->getCompass();
    
    // Odomerty
    MRodom.time = t_now;
    MRodom.odomDistance = MRrobot->getTripOdometerDistance() / 1000; // in m
    MRodom.odomAngle = MRrobot->getTripOdometerDegrees() * M_PI/180; // in rad
    
    // Motor State
    MRmotorstatus.time = t_now;
    MRmotorstatus.index = index;

    // Status
    RobotStatus robot_status;
    robot_status.time = t_now;
    robot_status.lastPacketTime = fromArTime(MRrobot->getLastPacketTime());
    robot_status.lastOdometryTime = fromArTime(MRrobot->getLastOdometryTime());
    robot_status.lastIOPacketTime = fromArTime(MRrobot->getIOPacketTime());
    robot_status.cycleTime.microseconds = MRrobot->getCycleTime() * 1000;
    robot_status.batteryVoltage = MRrobot->getBatteryVoltage();
    robot_status.chargeState = MRbatteryLevel.battery;
    robot_status.temperatureValue = MRtemperature.temp;
    robot_status.count = MRrobot->getCounter();
    robot_status.motorsEnabled = MRrobot->areMotorsEnabled();

    
    base::Time dt;
    if(index <= 1){
        // first cycle, there is no time difference to previous cycle
        dt = base::Time::fromMilliseconds(0);
    }
    else{
        dt = t_now - t_prev;
    }
    
    // get rotation of wheels by traveled distance per side (through velocity and delta time)
    wheel_pos[0] += MRrobot->getLeftVel() * diffconvfactor * dt.toSeconds();
    wheel_pos[1] += MRrobot->getRightVel() * diffconvfactor * dt.toSeconds();
    
    MRmotorstatus.states[odometry::FRONT_LEFT].position = wheel_pos[0]; // front left
    MRmotorstatus.states[odometry::REAR_LEFT].position = wheel_pos[0]; // rear left
    MRmotorstatus.states[odometry::FRONT_RIGHT].position = wheel_pos[1]; // front right
    MRmotorstatus.states[odometry::REAR_RIGHT].position = wheel_pos[1]; // rear right
    MRmotorstatus.states[odometry::FRONT_LEFT].positionExtern = wheel_pos[0]; // front left
    MRmotorstatus.states[odometry::REAR_LEFT].positionExtern = wheel_pos[0]; // rear left
    MRmotorstatus.states[odometry::FRONT_RIGHT].positionExtern = wheel_pos[1]; // front right
    MRmotorstatus.states[odometry::REAR_RIGHT].positionExtern = wheel_pos[1]; // rear right
    
    
    // Raw Data from left and right Encoders
    MRenc.time = t_now;
    MRrobot->requestEncoderPackets();
    MRenc.encLeft = MRrobot->getLeftEncoder();
    MRenc.encRight = MRrobot->getRightEncoder();
    // Reading Encoder Values is not yet working
    //LOG_DEBUG_S<<"Aria: Encoder L: "<<MRrobot->getLeftEncoder()<<", Encoder R: "<<MRrobot->getRightEncoder();
    MRrobot->stopEncoderPackets();
    
    // Bumpers
    // See Aria Documentation and/or ArModes.cpp for an Example how to read out these values.
    MRbumpers.time = t_now;
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
    
    
    // Distribute Messages
    _robot_pose.write(MRpose);
    _robot_pose_raw.write(MRposeraw);
    _robot_motion.write(MRvel);
    _robot_motion2.write(MRvel2);
    _robot_battery.write(MRbatteryLevel);
    _robot_temp.write(MRtemperature);
    _robot_compass.write(MRcompass);
    _robot_odometer.write(MRodom);
    _robot_encoder.write(MRenc);
    _robot_bumpers.write(MRbumpers);
    _motor_states.write(MRmotorstatus);
    _robot_status.write(robot_status);
    
    // start of measurement
    t_prev = t_now;
}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }

void Task::stopHook()
{
    
    TaskBase::stopHook();
 
    MRrobot->lock();
    MRrobot->setVel(0);
    MRrobot->setRotVel(0);
    MRrobot->unlock();
    
    MRrobot->stopRunning();
    
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
    delete MRarguments;
    MRarguments = 0;
    
    // Reset number of wheels
    nwheels = 0;
    
    LOG_INFO("Aria: Exit.");
}

// Operation Methods

// Set the translational and rotational Velocities
void Task::transrotVel(base::commands::Motion2D const & velocities)
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
void Task::directCommand(commands::DirectCommand2Byte const & MRcmd2byte)
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

