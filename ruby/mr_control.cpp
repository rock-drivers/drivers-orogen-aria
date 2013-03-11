#include <rice/Class.hpp>
#include <rice/Constructor.hpp>
#include <rice/String.hpp>

#include <Aria.h>

using namespace Rice;

class MrControl 
{
private:
    ArArgumentBuilder* mrArgs;
    ArArgumentParser* mrParser;
    ArRobot* mrRobot;
    ArRobotConnector* mrConnector;

public:
    MrControl(const std::string& serial_port, bool logging) 
    {
        if(!logging)
            ArLog::init(ArLog::None, ArLog::Terse, "", false, false, false);
        else
            ArLog::init(ArLog::StdOut, ArLog::Normal, "", false, false, false);

        mrArgs = new ArArgumentBuilder;
        mrArgs->add("-robotPort");
        mrArgs->add(serial_port.c_str());

        mrParser = new ArArgumentParser(mrArgs);
        mrParser->loadDefaultArguments();

        mrRobot = new ArRobot("", true, false);
        mrConnector = new ArRobotConnector(mrParser, mrRobot);

        if(!mrConnector->connectRobot(mrRobot)) {
            // TODO
        }

        mrRobot->runAsync(false);
    }

    ~MrControl() 
    {
        Aria::shutdown();

        delete mrArgs;
        delete mrParser;
        delete mrRobot;
        delete mrConnector;
    }

    void power_on(int port)
    {
        mrRobot->lock();
        mrRobot->com2Bytes(116, port, true);
        mrRobot->unlock();
    }

    void power_off(int port)
    {
        mrRobot->lock();
        mrRobot->com2Bytes(116, port, false);
        mrRobot->unlock();
    }

    double charge_state() 
    {
        mrRobot->lock();

        double status = mrRobot->getStateOfCharge();
        
        mrRobot->unlock();

        return status;
    }

    double temperature_state()
    {
        mrRobot->lock();

        double status = mrRobot->getTemperature();
    
        mrRobot->unlock();

        return status;
    }

    double battery_state()
    {
        mrRobot->lock();

        double volt = mrRobot->getBatteryVoltageNow();

        mrRobot->unlock();

        return volt;
    }
};


Data_Type<MrControl> rb_mrcontrol;


extern "C"
void Init_mr_control()
{

    rb_mrcontrol = define_class<MrControl>("MrControlInterface")
        .define_constructor(Constructor<MrControl, std::string, bool>())
        .define_method("power_on", &MrControl::power_on, (Arg("port")))
        .define_method("power_off", &MrControl::power_off, (Arg("port")))
        .define_method("charge_state", &MrControl::charge_state)
        .define_method("battery_state", &MrControl::battery_state)
        .define_method("temperature_state", &MrControl::temperature_state);
}
