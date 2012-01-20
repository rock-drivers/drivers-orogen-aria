#ifndef _ARIATYPE_HPP_
#define _ARIATYPE_HPP_

#include <iostream>

#include <base/time.h>
#include <base/motion_command.h>

namespace AriaType
{
// Commands to be send to Robot
namespace commands
{
	struct Velocity2
	{
		double velLeft;
		double velRight;
		base::Time time;
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
