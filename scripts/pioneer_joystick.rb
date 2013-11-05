#!/usr/bin/ruby

require 'orocos'
include Orocos

## Initialize orocos ##
Orocos.initialize

maxSpeed = 0.4 #SeekurJr=1.5 m/s
maxJoyRotSpeed = 45 # degree/s

Orocos.run 'controldev::JoystickTask' => 'joystick', 'aria::Task' => 'aria' do
  
    mrControl = TaskContext.get 'aria'
    joystick = TaskContext.get 'joystick'
    
    # joystick configuration
    joystick.device = "/dev/input/js0"
    joystick.maxSpeed = maxSpeed # m/s
    joystick.minSpeed = -maxSpeed # m/s
    joystick.maxRotationSpeed = maxJoyRotSpeed * Math::PI/180 # rad/s
    joystick.minRotationSpeed = -maxJoyRotSpeed * Math::PI/180 # rad/s

    mrControl.serial_port = "/dev/ttyUSB0"
    mrControl.baudrate = 115200
    
    joystick.motion_command.connect_to mrControl.transrot_vel
    
    # Configure and start Task
    mrControl.configure
    mrControl.start

   joystick.configure
   joystick.start
    
   Orocos.watch(mrControl)
end

