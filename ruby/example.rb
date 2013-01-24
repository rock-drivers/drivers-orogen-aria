require "mr_control"

p = MrControl.new "/dev/ttyS0", false

p.power_off 6

