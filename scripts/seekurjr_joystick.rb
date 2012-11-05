#!/usr/bin/ruby

require 'orocos'
require 'vizkit'
include Orocos

## Initialize orocos ##
Orocos.initialize

#ENV['BASE_LOG_LEVEL'] = 'DEBUG'
#ENV['BASE_LOG_FORMAT'] = 'SHORT'

## create a widget that emulates a joystick
joystickGui = Vizkit.default_loader.create_plugin('VirtualJoystick')

#show it
joystickGui.show()

maxSpeed = 0.4 # maximum for SeekurJr: 1.2 m/s

## Execute the deployments 'rock_tutorial' ##
Orocos.run 'robot_control' do
#Orocos.run 'seekur' do
  
    # Orocos.log_all_ports
    
    ## Connect port to vizkit plugin
#    con = Vizkit.connect_port_to 'mr_control', 'pose', :update_frequency => 33 do |sample, name|
	##pass every pose sample to our visualizer plugin
#        vizkit_rock.updatePose(sample)
#        sample
#    end 

    ## Get the specific task context ##
    #mrControl = TaskContext.get 'mr_control'
    mrControl = Orocos.name_service.get 'mr_control'
#    mrControl = TaskContext.get 'seekur_drv'
    
    mrControl.serial_port = "/dev/ttyS0"
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
    
    
    # Configure Task
    mrControl.configure
    
    ## Start the tasks ##
    mrControl.start
    
    ## Write motion command sample ##
    #Vizkit.exec
end
