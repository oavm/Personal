from dwl_lcmtypes import ContactState_t

# Joint positions
for c in range(0,4):
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.contacts[c].position.x)
    addSignal('ROBOT_CS', msg.utime, msg.actual.joints[c].position.x)
    
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.contacts[c].position.y)
    addSignal('ROBOT_CS', msg.utime, msg.actual.joints[c].position.y)
    
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.contacts[c].position.z)
    addSignal('ROBOT_CS', msg.utime, msg.actual.joints[c].position.z)
    
# Joint velocities
for c in range(0,4):
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.contacts[c].velocity.x)
    addSignal('ROBOT_CS', msg.utime, msg.actual.joints[c].velocity.x)
    
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.contacts[c].velocity.y)
    addSignal('ROBOT_CS', msg.utime, msg.actual.joints[c].velocity.y)
    
    addPlot()
    addSignal('ROBOT_CS', msg.utime, msg.desired.contacts[c].velocity.z)
    addSignal('ROBOT_CS', msg.utime, msg.actual.joints[c].velocity.z)
