#ifndef _ARIATYPE_HPP_
#define _ARIATYPE_HPP_

#include <iostream>

#include <base/time.h>
#include <base/motion_command.h>

namespace AriaTypes
{
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
		base::MotionCommand2D velTransRot;
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
		double odomDegrees;
		base::Time time;
	};
	
	struct Encoder
	{
		long int encLeft;
		long int encRight;
		base::Time time;
	};
	
} // end namespace samples
} // end namespace AriaType

#endif // _ARIATYPE_HPP_
