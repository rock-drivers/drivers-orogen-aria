/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
//#include "Aria.h"

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

    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    // Initialise Aria
    Aria::init();
    
    //ArArgumentBuilder MRarguments(512); // largest number of arguments = 512
    //ArArgumentBuilder *MRarguments = new ArArgumentBuilder();
    
    //MRarguments->add("-robotPort");
    //MRarguments->add(_serial_port.get().c_str());
    
    //cout<<"Aria_Task: List of Parameters: "<<MRarguments->getFullString()<<endl;
    
    char const* argv[] = { "", "-robotPort", _serial_port.get().c_str() };
    int argc = 3;
    MRparser = new ArArgumentParser(&argc, const_cast<char**>(argv));
    MRparser->loadDefaultArguments();
    MRparser->log();
    
    if(!MRparser->checkHelpAndWarnUnparsed())
        cout<<"Aria_Task: UnparsMRed Arguments found"<<endl;
        
    cout<<"Aria_Task: params "<<*MRparser->getArgv()<<endl;
    
    cout<<"Aria_Task: Initialised"<<endl;

    MRrobot = new ArRobot("", true, false);
    MRconnector = new ArRobotConnector(MRparser, MRrobot);
    //ArRobotConnector MRconnector(MRparser, MRrobot);
    
    //cout<<"Aria: Connector created"<<endl;
    cout << "connector robot: " << MRconnector->getRobot() << " from class: " << MRrobot << endl;
    
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
    //MRrobot->runAsync(false);
    
    cout<<"Aria_Task: Thread started"<<endl;
    
    //cout<<"Aria_Task: Waiting"<<endl;
    //ArUtil::sleep(3000);
    
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    /*
    if ( MRrobot->isRunning() )
    	cout << "Aria_Taks: aria is running!" << endl;
    else
    	cout << "not running!" << endl;
    */
    	
    base::MotionCommand2D MRmotion;
    
    if (_transrot_vel.read(MRmotion) == RTT::NoData)
    	return;
    
    cout<<"Aria_Task: Command received"<<endl;
    cout<<"Aria_Task: TranslVel "<<MRmotion.translation<<", RotVel "<<MRmotion.rotation<<endl;
    
    //cout<<"Aria_Task: Waiting 2s in updateHook"<<endl;
    //ArUtil::sleep(2000);
    
    MRrobot->lock();
    cout<<"Aria_Task: Thread locked"<<endl;
    
    //MRrobot.setVel(MRmotion.translation);
    MRrobot->setVel(MRmotion.translation * 1000);
    //MRrobot.setRotVel(MRmotion.rotation);
    MRrobot->setRotVel(MRmotion.rotation * 180 / M_PI);
    
    //MRrobot.unlock();
    MRrobot->unlock();
    cout<<"Aria_Task: Thread unlocked"<<endl;
    
    //ArUtil::sleep(2000);
    
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

    Aria::shutdown();
    delete MRconnector;
    MRconnector = 0;
    
    cout<<"Aria_Task: Shutdown"<<endl;
}
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

