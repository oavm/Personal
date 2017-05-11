current_file_dir = os.path.dirname(__file__)
findLCMTypes(os.path.join(current_file_dir,'include/exlcm'))
findLCMModules(os.path.join(current_file_dir,'include/exlcm'))

findLCMTypes(os.path.expanduser('~/catkin_ws/packages/pronto-distro/build/lib/python2.7/dist-packages/*'))

# print out all the available lcm types:
#print getMessageTypeNames()

#do it singularly
#signals1 = ['footPosDesLFx','footPosDesLFy','footPosDesLFz','footPosLFx','footPosLFy','footPosLFz']
signals1 = ['des_baseXd','des_baseYd','des_baseZd','actual_CoMXd','actual_CoMYd','actual_CoMZd']
#signals2 = ['footPosDesLHx','footPosDesLHy','footPosDesLHz','footPosLHx','footPosLHy','footPosLHz']
signals2 = ['des_baseX','des_baseY','des_baseZ','actual_CoMX','actual_CoMY','actual_CoMZ']
#signals3 = ['footPosDesLHz_HF','footPosDesLFz_HF']
#signals3 = ['learningRate','pitchForce']
signals4 = ['roll','pitch','yaw','des_roll','des_pitch','des_yaw']
#signals5 = ['roll_d','pitch_d','yaw_d','omegaX','omegaY','omegaZ']
signals5 = ['state_machine_LF','state_machine_LH','state_machine_RF','state_machine_RH']
signals3 = ['Actual_duty_LF','Actual_stance_LF','Actual_swing_LF','Period_counter_LF']
#signals1 = ['feetForcesLFx','feetForcesLFy','feetForcesLFz','grForcesLFx','grForcesLFy','grForcesLFz']
#signals2 = ['feetForcesLHx','feetForcesLHy','feetForcesLHz','grForcesLHx','grForcesLHy','grForcesLHz']
#signals3 = ['feetForcesRFx','feetForcesRFy','feetForcesRFz','grForcesRFx','grForcesRFy','grForcesRFz']
#signals4 = ['feetForcesRHx','feetForcesRHy','feetForcesRHz','grForcesRHx','grForcesRHy','grForcesRHz']


p1 = addPlot()
p2 = addPlot()
p3 = addPlot()
p4 = addPlot()
p5 = addPlot()
#p6 = addPlot()


addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals1,  keyLookup=msg.name, plot=p1)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals2,  keyLookup=msg.name, plot=p2)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals3,  keyLookup=msg.name, plot=p3)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals4,  keyLookup=msg.name, plot=p4)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals5,  keyLookup=msg.name, plot=p5)
#addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals6,  keyLookup=msg.name, plot=p6)

setFormatOptions(pointSize=4,timeWindow=7.0,curveStyle="lines")

