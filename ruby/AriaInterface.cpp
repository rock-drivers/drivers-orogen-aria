#include <rice/Class.hpp>
#include <rice/Constructor.hpp>
#include <rice/String.hpp>

#include <Aria.h>

using namespace Rice;

class AriaInterface
{
private:
    ArArgumentBuilder* mrArgs;
    ArArgumentParser* mrParser;
    ArRobot* mrRobot;
    ArRobotConnector* mrConnector;

public:
    AriaInterface(const std::string& serial_port, bool logging) 
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

    ~AriaInterface() 
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


Data_Type<AriaInterface> rb_aria_interface;


extern "C"
void Init_aria()
{
    rb_aria_interface = define_class<AriaInterface>("AriaInterface")
        .define_constructor(Constructor<AriaInterface, std::string, bool>())
        .define_method("power_on", &AriaInterface::power_on, (Arg("port")))
        .define_method("power_off", &AriaInterface::power_off, (Arg("port")))
        .define_method("charge_state", &AriaInterface::charge_state)
        .define_method("battery_state", &AriaInterface::battery_state)
        .define_method("temperature_state", &AriaInterface::temperature_state);
}
