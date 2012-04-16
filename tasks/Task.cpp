/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include<base/logging.h>
#include<boost/tokenizer.hpp>

using namespace mr_control;

//Task::Task(std::string const& name, TaskCore::TaskState initial_state)
//    : TaskBase(name, initial_state)
Task::Task(std::string const& name) //needs_configuration
    : TaskBase(name)
//    , MRconnector(0), MRrobot(0)
{
}

//Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
//    : TaskBase(name, engine, initial_state)
Task::Task(std::string const& name, RTT::ExecutionEngine* engine) //needs_configuration
    : TaskBase(name, engine)
//    , MRconnector(0), MRrobot(0)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    ArArgumentBuilder *MRarguments = new ArArgumentBuilder();
    
    MRarguments->add("-robotPort");
    MRarguments->add(_serial_port.get().c_str());
    
    // vector<int> PowerPortsON
    boost::tokenizer<> tokPortList(_poweron_boot.get());
    std::string nrstr;
    int portnr;
    for(boost::tokenizer<>::iterator tokit=tokPortList.begin();
    	tokit!=tokPortList.end(); ++tokit){
    	//cout<<"Port: "<<*tokit<<endl;
    	nrstr = *tokit;
    	portnr = atoi(nrstr.c_str());
    	//cout<<"Port: "<<portnr<<endl;
    	//PowerPortsON.push_back(portnr);
    	PowerPortsON.push_back(atoi(nrstr.c_str()));
    }
    
    //cout<<"Aria_Task: List of Parameters: "<<MRarguments->getFullString()<<endl;
    LOG_INFO("Aria: List of Parameters: %s", MRarguments->getFullString());
    
    MRparser = new ArArgumentParser(MRarguments);
    MRparser->loadDefaultArguments();
    MRparser->log();
    
    if(!MRparser->checkHelpAndWarnUnparsed())
    	LOG_WARN("Aria: Unparsed Arguments found");
        //cout<<"Aria_Task: Unparsed Arguments found"<<endl;
        
    //cout<<"Aria_Task: Used Parameters "<<*MRparser->getArgv()<<endl;
    LOG_INFO("Aria: Used Parameters: %s",*MRparser->getArgv());
    
    delete MRarguments;
    MRarguments = 0;
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    // Initialise Aria
    Aria::init();
    
    //cout<<"Aria_Task: Initialised"<<endl;
    LOG_INFO("Aria: Initialised.")

    //ArLog::init(ArLog::None, ArLog::Normal);
    ArLog::init(ArLog::File, ArLog::Normal, "MrControl_AriaLog.log", false, true);
    
    MRrobot = new ArRobot("", true, false);
    MRconnector = new ArRobotConnector(MRparser, MRrobot);
    
    //cout<<"Aria: Connector created"<<endl;
    
    // Connect to Robot or Simulator
    if (!MRconnector->connectRobot()){
        //cout<<"Aria_Task: Could not connect!"<<endl;
        LOG_ERROR("Aria: Could not connect!");
        ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
        Aria::logOptions();
        Aria::exit(1);
        return false;
    }
    else{
        //cout<<"Aria_Task: Robot connected"<<endl;
        LOG_INFO("Aria: Robot connected.")
    }
    
    // Open new thread for processing cycle
    MRrobot->runAsync(false);
    
    //cout<<"Aria_Task: Thread started"<<endl;
    LOG_INFO("Aria: Thread started.")
    
    //Turn ON default Power-Ports
    for(vector<int>::iterator portsit=PowerPortsON.begin(); portsit!=PowerPortsON.end(); ++portsit){
        //cout<<"Port: "<<*portsit<<endl;
        controlPDB(*portsit, 1);
    }
    
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    	
    base::MotionCommand2D MRmotion;
    bool MRdoResetOdometry = 0;
    double MRtransVel, MRrotVel;
    
    AriaTypes::commands::DevicePower MRdeviceOnOff;
    AriaTypes::commands::DirectCommand2Byte MRdirectCommand;
    
    // Process Motion Commands
    // MotionCommand2D
    if (_transrot_vel.read(MRmotion) != RTT::NoData){
    
        //cout<<"Aria_Task: Command received"<<endl;
        //cout<<"Aria_Task: TranslVel "<<MRmotion.translation<<" m/s, RotVel "<<MRmotion.rotation<<" rad/s"<<endl;
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
    
    // Fetch Motion- and Odometer-Data from Robot, as well as miscellaneous Data
    base::samples::RigidBodyState MRpose;
    AriaTypes::samples::Velocity MRvel;
    AriaTypes::samples::Velocity2 MRvel2;
    AriaTypes::samples::BatteryLevel MRbatteryLevel;
    AriaTypes::samples::Temperature MRtemperature;
    AriaTypes::samples::CompassHeading MRcompass;
    
    AriaTypes::samples::Odometer MRodom;
    AriaTypes::samples::Encoder MRenc;
    
    AriaTypes::samples::Bumpers MRbumpers;
    
    vector<bool> MRbumpersFront;
    vector<bool> MRbumpersRear;
    
    
    MRrobot->lock();
    
    // Position
    MRpose.position = Eigen::Vector3d(MRrobot->getX() / 1000, MRrobot->getY() / 1000, 0); // in meters
    MRpose.orientation = Eigen::AngleAxis<double>(MRrobot->getTh() * M_PI/180, Eigen::Vector3d::UnitZ()); // rad
    
    MRpose.velocity = Eigen::Vector3d(MRrobot->getVel(), 0, 0); // m/s
    MRpose.angular_velocity = Eigen::Vector3d(MRrobot->getRotVel() * M_PI/180, 0, 0); // rad/s
    
    //cout<<"Aria_Task: Theta: "<<MRrobot->getTh()<<"°"<<endl;
    //cout<<"Aria_Task: Yaw "<<MRpose.getYaw()<<", Pitch "<<MRpose.getPitch()<<", Roll "<<MRpose.getRoll()<<endl;
    
    // Velocity
    MRvel.time = base::Time::now();
    MRvel.velTransRot.translation = MRrobot->getVel(); // in mm/s
    MRvel.velTransRot.rotation = MRrobot->getRotVel(); // in deg/s
    
    // Velocity2 (left, right)
    MRvel2.time = base::Time::now();
    MRvel2.velLeft = MRrobot->getLeftVel(); // in mm/s
    MRvel2.velRight = MRrobot->getRightVel(); // in mm/s
    
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
    MRodom.odomDistance = MRrobot->getTripOdometerDistance();
    MRodom.odomDegrees = MRrobot->getTripOdometerDegrees();
    
    // Raw Data from left and right Encoders
    MRenc.time = base::Time::now();
    MRrobot->requestEncoderPackets();
    MRenc.encLeft = MRrobot->getLeftEncoder();
    MRenc.encRight = MRrobot->getRightEncoder();
    //cout<<"Aria_Task: Encoder L: "<<MRrobot->getLeftEncoder()<<", R: "<<MRrobot->getRightEncoder()<<endl;
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
    
    
    MRrobot->unlock();
    
    bool frbump[MRbumpers.nrFront];
    bool rebump[MRbumpers.nrRear];
    int bit = 0;
    int i = 0;
    
    for(i = 0, bit = 2; i < MRrobot->getNumFrontBumpers(); i++, bit *= 2){
    	frbump[i] = (MRbumpers.front & bit);
    	MRbumpers.bumpersFront.push_back( (maskFront & bit) );
    }
    
    for (i = 0, bit = 2; i < MRrobot->getNumRearBumpers(); i++, bit *= 2){
    	rebump[i] = (MRbumpers.rear & bit);
    	MRbumpers.bumpersRear.push_back( (maskRear & bit) );
    }
    
    
//    cout<<"Aria_Task: Front Bumpers: ";
//    for(int i=0; i<MRbumpers.nrFront; i++)
//    {
//    	cout<<frbump[i]<<" ";
//    }
//    cout<<endl;
    
//    cout<<"Aria_Task: Rear Bumpers: ";
//    for(int i=0; i<MRbumpers.nrRear; i++)
//    {
//    	cout<<rebump[i]<<" ";
//    }
//    cout<<endl;
    
    
    
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
    
    //cout<<"Aria_Task: Stopped"<<endl;
    LOG_INFO("Aria: Stopped.");
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    
    // Cleanup and exit Aria
    //Aria::exit(0);
    
    delete MRparser;
    MRparser = 0;
    
    //cout<<"Aria_Task: Exit"<<endl;
    LOG_INFO("Aria: Exit.");
}

// Operation Methods

// Set the translational and rotational Velocities
void Task::transrotVel(::base::MotionCommand2D const & velocities)
{
	//cout<<"Aria_Task: TranslVel "<<velocities.translation<<" m/s, RotVel "<<velocities.rotation<<" rad/s"<<endl;
	LOG_DEBUG("Aria: TranslVel %.3f m/s, RotVel %.3f rad/s", velocities.translation, velocities.rotation);
        
        MRrobot->lock();
        MRrobot->setVel(velocities.translation * 1000);
        MRrobot->setRotVel(velocities.rotation * 180 / M_PI);
        MRrobot->unlock();
}

void Task::transrotVel2(double translational, double rotational)
{
	//cout<<"Aria_Task: TranslVel "<<translational<<" m/s, RotVel "<<rotational<<" rad/s"<<endl;
	LOG_DEBUG("Aria: TranslVel %.3f m/s, RotVel %.3f rad/s", translational, rotational);
	
	MRrobot->lock();
        MRrobot->setVel(translational * 1000);
        MRrobot->setRotVel(rotational * 180 / M_PI);
        MRrobot->unlock();
}

// Set Velocities for left and right Wheels
void Task::lrVel(double left, double right)
{
	//cout<<"Aria_Task: Velocity L: "<<left<<" m/s, R: "<<right<<" m/s"<<endl;
	LOG_DEBUG("Aria: Velocity L: %.3f m/s, R: .3f m/s", left, right);
	
	MRrobot->lock();
	MRrobot->setVel2(left*1000, right*1000);
	MRrobot->unlock();
}

// Turn on/off the PDB
void Task::controlPDB(boost::int32_t portNr, bool onoff)
{
        //std::stringstream loginfostr;
	//cout<<"Aria_Task: Turning Port "<<portNr<<" ";
	//onoff ? cout<<"ON":cout<<"OFF";
	//cout<<endl;
	
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
	//cout<<"Aria_Task: Direct Command "<<MRcmd2byte.cmdnr<<" with HB: "<<MRcmd2byte.highbyte<<" LB: "<<MRcmd2byte.lowbyte<<endl;
	LOG_INFO("Aria: Direct Command %i with HB: %i LB: %i", MRcmd2byte.cmdnr, MRcmd2byte.highbyte, MRcmd2byte.lowbyte);
	
	MRrobot->lock();
	MRrobot->com2Bytes(MRcmd2byte.cmdnr, MRcmd2byte.highbyte, MRcmd2byte.lowbyte);
	MRrobot->unlock();
}

// Reset the Odometer
void Task::resetOdometer()
{
    //cout<<"Aria_Task: Resetting Odometer"<<endl;
    LOG_INFO("Aria: Resetting Odometer");

    MRrobot->lock();
    MRrobot->resetTripOdometer();
    MRrobot->unlock();
}

