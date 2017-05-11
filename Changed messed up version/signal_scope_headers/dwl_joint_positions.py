from dwl_lcmtypes import WholeBodyController_t

# Joint positions
for j in range(0,12):
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.joints[j].position)
    addSignal('ROBOT_CS', msg.utime, msg.actual.joints[j].position)
