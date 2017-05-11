from dwl_lcmtypes import WholeBodyController_t
 
# Joint positions
for j in range(0,12):
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.joints[j].position)
    addSignal('ROBOT_CS', msg.utime, msg.actual.joints[j].position)


# Joint velocities
for j in range(0,12):
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.joints[j].velocity)
    addSignal('ROBOT_CS', msg.utime, msg.actual.joints[j].velocity)
    
# Joint efforts
for j in range(0,12):
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.joints[j].effort)
    addSignal('ROBOT_CS', msg.utime, msg.actual.joints[j].effort)
    

# Joint commands
for j in range(0,12):
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.command[j].feedforward)
    addSignal('ROBOT_CS', msg.utime, msg.command[j].feedback)