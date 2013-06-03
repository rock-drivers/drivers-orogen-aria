#!/usr/bin/ruby

require 'orocos'
require 'vizkit'
#include Orocos

## Initialize orocos ##
Orocos.initialize

#ENV['BASE_LOG_LEVEL'] = 'DEBUG'
#ENV['BASE_LOG_FORMAT'] = 'SHORT'

## create a widget that emulates a joystick and show it
joystickGui = Vizkit.default_loader.create_plugin('VirtualJoystick')
joystickGui.show()

maxSpeed = 0.4 # maximum speed: 1.2 m/s (SeekurJr)

Orocos.run 'aria::Task' => 'aria' do
  
    # Orocos.log_all_ports
    
    mrControl = Orocos.name_service.get 'aria'
    
    # set path to the serial device
    mrControl.serial_port = "/dev/ttyS0"
    
    # set power-ports that should be powered on automatically on boot
    #mrControl.poweron_boot = "2 6 7 9 10"
    # Turn on:
    #  6 SICK
    #  9 PTU
    # 10 PoE
    #mrControl.poweron_boot="2 6 9 10"
    mrControl.poweron_boot=""
    
    ## Create a sample writer for a port ##
    sampleWriter = mrControl.transrot_vel.writer
    sample = sampleWriter.new_sample

    ## glue the widget to the task writer
    joystickGui.connect(SIGNAL('axisChanged(double, double)')) do |x, y|
	    sample.translation = x * maxSpeed
	    sample.rotation = - y.abs() * Math::atan2(y, x.abs()) * maxSpeed
	    sampleWriter.write(sample)
    end
    
    # Configure and start Task
    mrControl.configure
    mrControl.start
    
    ## Write motion command sample ##
    Vizkit.exec
end
