current_file_dir = os.path.dirname(__file__)
findLCMTypes(os.path.join(current_file_dir,'include/exlcm'))
findLCMModules(os.path.join(current_file_dir,'include/exlcm'))

findLCMTypes(os.path.expanduser('~/catkin_ws/packages/pronto-distro/build/lib/python2.7/dist-packages/*'))

# print out all the available lcm types:
#print getMessageTypeNames()

#do it singularly
signals1 = ['footVelDesLFx_HF','footVelDesLFy_HF','footVelDesLFz_HF','footPosDesLFx_HF','footPosDesLFy_HF','footPosDesLFz_HF']
signals2 = ['footVelDesLHx_HF','footVelDesLHy_HF','footVelDesLHz_HF','footPosDesLHx_HF','footPosDesLHy_HF','footPosDesLHz_HF']
#signals3 = ['footVelDesRFx','footVelDesRFy','footVelDesRFz','footPosDesRFx','footPosDesRFy','footPosDesRFz']
#signals1 = ['roll','pitch','yaw','des_roll','des_pitch','des_yaw']
#signals2 = ['roll_d','pitch_d','yaw_d','omegaX','omegaY','omegaZ']
signals3 = ['state_machine_LF','state_machine_LH','state_machine_RF','state_machine_RH']
#signals5 = ['des_baseBX','des_baseBY','des_baseBZ','actual_CoMBX','actual_CoMBY','actual_CoMBZ']
#signals7 = ['roll','filtered_roll']
#signals4 = ['act_Joint_Kp_LF','act_Joint_Kp_RF','act_Joint_Kp_LH','act_Joint_Kp_RH']
#signals5 = ['KinAdjGain']
#signals3 = ['feetForcesRFx','feetForcesRFy','feetForcesRFz','grForcesRFx','grForcesRFy','grForcesRFz']
#signals4 = ['feetForcesRHx','feetForcesRHy','feetForcesRHz','grForcesRHx','grForcesRHy','grForcesRHz']


p1 = addPlot()
p2 = addPlot()
p3 = addPlot()
#p4 = addPlot()
#p5 = addPlot()
#p6 = addPlot()
#p7 = addPlot()
#p8 = addPlot()


addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals1,  keyLookup=msg.name, plot=p1)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals2,  keyLookup=msg.name, plot=p2)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals3,  keyLookup=msg.name, plot=p3)
#addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals4,  keyLookup=msg.name, plot=p4)
#addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals5,  keyLookup=msg.name, plot=p5)
#addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals6,  keyLookup=msg.name, plot=p6)
#addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals7,  keyLookup=msg.name, plot=p7)
#addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals8,  keyLookup=msg.name, plot=p8)

setFormatOptions(pointSize=4,timeWindow=7.0,curveStyle="lines")

