require "aria"

p = AriaInterface.new "/dev/ttyS0", false

p.power_off 6

