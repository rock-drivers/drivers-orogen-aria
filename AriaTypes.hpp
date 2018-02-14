#ifndef _ARIATYPE_HPP_
#define _ARIATYPE_HPP_

#include <vector>

#include <base/Time.hpp>
#include <base/commands/Motion2D.hpp>

namespace aria
{

/** Contains informations from the robot that could be nice to know.*/
struct RobotStatus {

    base::Time time;
    base::Time lastPacketTime;
    base::Time lastOdometryTime;
    base::Time lastIOPacketTime;
    base::Time cycleTime;
    double batteryVoltage;
    double chargeState;
    int temperatureValue;
    unsigned int count;
    bool motorsEnabled;
};

// Commands to be send to Robot
namespace commands
{
	struct Velocity2
	{
		double velLeft;
		double velRight;
	};
	
	// De-/Activate Power-Port on PDB of SeekurJr (command 116)
	struct DevicePower
	{
		unsigned char portnr; // 0-12 accprding to PDB of SeekurJr
		bool onoff; // true=on, false=off
	};
	
	// Sending direct commands via ArRobot::com2Bytes(cmdnr, highbyte, lowbyte)
	struct DirectCommand2Byte
	{
		unsigned char cmdnr;
		char highbyte;
		char lowbyte;
	};
}

// Samples of Robot States
namespace samples
{
	struct Velocity
	{
		base::commands::Motion2D velTransRot;
		base::Time time;
	};
	
	struct Velocity2
	{
		double velLeft;
		double velRight;
		base::Time time;
	};
	
	struct BatteryLevel
	{
		double battery;
		base::Time time;
	};
	
	struct CompassHeading
	{
		double heading;
		base::Time time;
	};
	
	struct Temperature
	{
		double temp;
		base::Time time;
	};
	
	struct Odometer
	{
		double odomDistance;
		double odomAngle;
		base::Time time;
	};
	
	struct Encoder
	{
		long int encLeft;
		long int encRight;
		base::Time time;
	};
	
	struct Bumpers
	{
		// Number of Bumpers (specific per Robot)
		int nrFront;
		int nrRear;
		// States of Bumpers. See Aria Documentation and/or ArModes.cpp
		// for an Example how to read out these values.
		int front;
		int rear;
		
		//std::vector<bool> bumpersFront;
		std::vector<uint8_t> bumpersFront;
		//std::vector<bool> bumpersRear;
		std::vector<uint8_t> bumpersRear;
		
		base::Time time;
	};
	
} // end namespace samples
} // end namespace AriaType

#endif // _ARIATYPE_HPP_
