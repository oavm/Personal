from dwl_lcmtypes import WholeBodyController_t

# Base positions
for b in range(3,6):
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.base[b].position)
    addSignal('ROBOT_CS', msg.utime, msg.actual.base[b].position)
  
# Base velocities
for b in range(3,6):
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.base[b].velocity)
    addSignal('ROBOT_CS', msg.utime, msg.actual.base[b].velocity)
