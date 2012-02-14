/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace mr_control;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
//Task::Task(std::string const& name) //needs_configuration
//    : TaskBase(name)
    , MRconnector(0), MRrobot(0)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
//Task::Task(std::string const& name, RTT::ExecutionEngine* engine) //needs_configuration
//    : TaskBase(name, engine)
    , MRconnector(0), MRrobot(0)
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
    
    cout<<"Aria_Task: List of Parameters: "<<MRarguments->getFullString()<<endl;
    
    MRparser = new ArArgumentParser(MRarguments);
    MRparser->loadDefaultArguments();
    MRparser->log();
    
    if(!MRparser->checkHelpAndWarnUnparsed())
        cout<<"Aria_Task: UnparsMRed Arguments found"<<endl;
        
    cout<<"Aria_Task: params "<<*MRparser->getArgv()<<endl;
    
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
    
    cout<<"Aria_Task: Initialised"<<endl;
    
    //ArArgumentBuilder MRarguments(512); // largest number of arguments = 512
    //ArArgumentBuilder *MRarguments = new ArArgumentBuilder();
    
    //MRarguments->add("-robotPort");
    //MRarguments->add(_serial_port.get().c_str());
    
    //cout<<"Aria_Task: List of Parameters: "<<MRarguments->getFullString()<<endl;
    
    /*
    char const* argv[] = { "", "-robotPort", _serial_port.get().c_str() };
    int argc = 3;
    MRparser = new ArArgumentParser(&argc, const_cast<char**>(argv));
    MRparser->loadDefaultArguments();
    MRparser->log();
    
    
    if(!MRparser->checkHelpAndWarnUnparsed())
        cout<<"Aria_Task: UnparsMRed Arguments found"<<endl;
        
    cout<<"Aria_Task: params "<<*MRparser->getArgv()<<endl;
    */
    

    MRrobot = new ArRobot("", true, false);
    MRconnector = new ArRobotConnector(MRparser, MRrobot);
    //ArRobotConnector MRconnector(MRparser, MRrobot);
    
    //cout<<"Aria: Connector created"<<endl;
    
    // Connect to Robot or Simulator
    if (!MRconnector->connectRobot()){
        cout<<"Aria_Task: Could not connect!"<<endl;
        ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
        Aria::logOptions();
        Aria::exit(1);
        return false;
    }
    else
        cout<<"Aria_Task: Robot connected"<<endl;
    
    // Open new thread for processing cycle
    MRrobot->runAsync(false);
    
    cout<<"Aria_Task: Thread started"<<endl;
    
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    	
    base::MotionCommand2D MRmotion;
    bool MRdoResetOdometry = 0;
    
    // Process Motion Commands
    if (_transrot_vel.read(MRmotion) != RTT::NoData){
    	//return;
    
        //cout<<"Aria_Task: Command received"<<endl;
        cout<<"Aria_Task: TranslVel "<<MRmotion.translation<<" m/s, RotVel "<<MRmotion.rotation<<" rad/s"<<endl;
        
        MRrobot->lock();
        //cout<<"Aria_Task: Thread locked"<<endl;
    
        //MRrobot.setVel(MRmotion.translation);
        MRrobot->setVel(MRmotion.translation * 1000);
        //MRrobot.setRotVel(MRmotion.rotation);
        MRrobot->setRotVel(MRmotion.rotation * 180 / M_PI);
    
        //MRrobot.unlock();
        MRrobot->unlock();
        //cout<<"Aria_Task: Thread unlocked"<<endl;
    }
    
    // Process Reset of Odometer
    if(_reset_odometry.read(MRdoResetOdometry) != RTT::NoData){
    
        if( MRdoResetOdometry ){
            cout<<"Aria_Task: Resetting Odometer"<<endl;
        
            MRrobot->lock();
            MRrobot->resetTripOdometer();
            MRrobot->unlock();
        }
    }
    
    
    // Fetch Motion- and Odometer-Data from Robot, as well as miscellaneous Data
    //base::Pose MRpose;
    base::samples::RigidBodyState MRpose;
    //base::MotionCommand2D MRvel;
    AriaType::samples::Velocity MRvel;
    //double MRbatteryLevel = 0;
    AriaType::samples::BatteryLevel MRbatteryLevel;
    //double MRtemperature = 0;
    AriaType::samples::Temperature MRtemperature;
    //double MRcompass = 0;
    AriaType::samples::CompassHeading MRcompass;
    
    AriaType::samples::Odometer MRodom;
    AriaType::samples::Encoder MRenc;
    
    //double MRodomDist = 0, MRodomDegr = 0;
    //long int MRencL = 0, MRencR = 0;
    
    
    MRrobot->lock();
    
    // Position
    MRpose.position = Eigen::Vector3d(MRrobot->getX() / 1000, MRrobot->getY() / 1000, 0); // in meters
    //MRpose.orientation = Eigen::AngleAxis<double>(MRrobot->getTh(), Eigen::Vector3d::UnitZ());
    MRpose.orientation = Eigen::AngleAxis<double>(MRrobot->getTh() * M_PI/180, Eigen::Vector3d::UnitZ());
    
    cout<<"Aria_Task: Theta: "<<MRrobot->getTh()<<"°"<<endl;
    cout<<"Aria_Task: Yaw "<<MRpose.getYaw()<<", Pitch "<<MRpose.getPitch()<<", Roll "<<MRpose.getRoll()<<endl;
    
    // Velocity
    MRvel.velTransRot.translation = MRrobot->getVel(); // in mm/s
    MRvel.velTransRot.rotation = MRrobot->getRotVel(); // in deg/s
    
    MRvel.time = base::Time::now();
    
    // Battery, Temperature, Compass
    MRbatteryLevel.battery = MRrobot->getStateOfCharge();
    MRbatteryLevel.time = base::Time::now();
    MRtemperature.temp = MRrobot->getTemperature();
    MRtemperature.time = base::Time::now();
    MRcompass.heading = MRrobot->getCompass();
    MRcompass.time = base::Time::now();
    
    // Odomerty
    MRodom.odomDistance = MRrobot->getTripOdometerDistance();
    MRodom.odomDegrees = MRrobot->getTripOdometerDegrees();
    MRodom.time = base::Time::now();
    
    // Raw Data from left and right Encoders
    MRrobot->requestEncoderPackets();
    MRenc.encLeft = MRrobot->getLeftEncoder();
    MRenc.encRight = MRrobot->getRightEncoder();
    MRrobot->stopEncoderPackets();
    MRenc.time = base::Time::now();
    
    MRrobot->unlock();
    
    // Distribute Messages
    _robot_pose.write(MRpose);
    _robot_motion.write(MRvel);
    _robot_battery.write(MRbatteryLevel);
    _robot_temp.write(MRtemperature);
    _robot_compass.write(MRcompass);
    _robot_odometer.write(MRodom);
    _robot_encoder.write(MRenc);
    
    //_odom_dist.write(MRodomDist);
    //_odom_degr.write(MRodomDegr);
    //_enc_left.write(MRencL);
    //_enc_right.write(MRencR);
    
}
// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
void Task::stopHook()
{
    TaskBase::stopHook();
    
    //MRrobot.stopRunning();
    MRrobot->stopRunning();
    
    //MRrobot.waitForRunExit();
    MRrobot->waitForRunExit();
    
    // Stop Aria background threads
    Aria::shutdown();
    // Cleanup and exit Aria
    Aria::exit(0);
    
    delete MRconnector;
    MRconnector = 0;
    
    delete MRrobot;
    MRrobot = 0;
    
    cout<<"Aria_Task: Shutdown"<<endl;
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    
    delete MRparser;
    MRparser = 0;
}

