#require 'orocos'
#require 'vizkit'
require 'orocos'
include Orocos

## Initialize orocos ##
Orocos.initialize

## create a widget for 3d display
#view3d = Vizkit.default_loader.create_widget('vizkit::Vizkit3DWidget')

#show it
#view3d.show()

## load and add the 3d plugin for the rock
#vizkit_rock = view3d.createPlugin('rock_tutorial', 'RockVisualization')

## create a widget that emulates a joystick
#joystickGui = Vizkit.default_loader.create_widget('VirtualJoystick')

#show it
#joystickGui.show()

maxSpeed = 0.2 #SeekurJr=1.5 m/s
maxJoyRotSpeed = 15 # degree/s

Orocos.run 'robot_control' do #, 'joystick' do
  
    ## Connect port to vizkit plugin
#    con = Vizkit.connect_port_to 'mr_control', 'pose', :update_frequency => 33 do |sample, name|
	##pass every pose sample to our visualizer plugin
#        vizkit_rock.updatePose(sample)
#        sample
#    end 

    ## Get the specific task context ##
    mrControl = Orocos.name_service.get 'mr_control'
   # joystick = Orocos.name_service.get 'joystick'
    
    # Properties
    mrControl.serial_port = "/dev/ttyUSB0"
    #joystick.device = "/dev/input/js0"
    #joystick.maxSpeed = maxSpeed # m/s
    #joystick.minSpeed = -maxSpeed # m/s
    #joystick.maxRotationSpeed = maxJoyRotSpeed * Math::PI/180 # rad/s
    #joystick.minRotationSpeed = -maxJoyRotSpeed * Math::PI/180 # rad/s

    ## Create a sample writer for a port ##
    #sampleWriter = mrControl.transrot_vel.writer

    #sample = sampleWriter.new_sample

    ## glue the widget to the task writer
    
    ## Connect the ports ##
#    joystick.motion_command.connect_to mrControl.transrot_vel
    
    # Configure Task
    mrControl.configure
 #   joystick.configure
    
    ## Start the tasks ##
    mrControl.start
    #joystick.start
    
    ## Write motion command sample ##
   while true do
   end
   # Vizkit.exec
end
