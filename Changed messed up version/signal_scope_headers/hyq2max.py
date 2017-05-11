if os.environ.has_key("PRONTO_ROOT"):
    print os.environ.get('PRONTO_ROOT')
else:
    print 'PRONTO_ROOT NOT SET!'
findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))
execfile(os.path.join(os.path.dirname(__file__), 'util.py'))

# print out all the available lcm types:
#print getMessageTypeNames()

#do it singularly
#signals = ['grForcesLFy','grForcesRFy']
#addSignals('debug', msg.usec, msg.array, signals,  keyLookup=msg.name)


#makePlots( ['grForcesLFy','grForcesRFy'], p1)

#signals1 = ['des_baseBX' , 'actual_CoMBX']
 #signals2 = ['des_baseBY', 'actual_CoMBY']
signals3 = ['des_height', 'actual_height']
#signals2 = ['grForcesLFz','feetForcesLFz']
signals1 = ['des_base_roll', 'roll','terrainRoll']
signals2 = ['des_base_pitch', 'pitch','terrainPitch']
#signals3 = ['grForcesLFz','grForcesRFz','grForcesLHz','grForcesRHz']

#signals4 = ['baseTwistLx','baseTwistLy', 'baseTwistLz','baseTwistAx','baseTwistAy','baseTwistAz']
signals4 = ['footPosDesLFz','footPosLFz' , 'footPosDesRFz','footPosRFz','footPosDesLHz','footPosLHz','footPosDesRHz','footPosRHz']

#signals4 = ['comWrenchLX','comWrenchLY', 'comWrenchLZ']
#signals5 = ['comWrenchAX','comWrenchAY', 'comWrenchAZ']
signals5 = ['status_LF','status_RF', 'status_LH', 'status_RH']


p1 = addPlot()
p2 = addPlot()
p3 = addPlot()
p4 = addPlot()
p5 = addPlot()



addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals1,  keyLookup=msg.name, plot=p1)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals2,  keyLookup=msg.name, plot=p2)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals3,  keyLookup=msg.name, plot=p3)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals4,  keyLookup=msg.name, plot=p4)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals5,  keyLookup=msg.name, plot=p5)


setFormatOptions(pointSize=4,timeWindow=7.0,curveStyle="lines")

#//
#
#//roll 4
#//des_base_roll 4
#//pitch 5
#//des_base_pitch 5
#//terrainPitch 5
#//yaw 6
#//des_base_yaw 6
#////
#
#slack 7
#//heading 5
#//robotSpeed 6
#
#footPosRHx 5
#footPosDesRHx 5 
#
#footPosRFx 4
#footPosDesRFx 4 
#
#//step_lengthHip 7
#//box  1
#//
#//box1 2
#//
#//box2 3
#
#//CoMWrenchLx 1 
#//CoMWrenchLy 2
#//CoMWrenchLz 3
#//CoMWrenchAx 4 
#//CoMWrenchAy 5
#//CoMWrenchAz 6
#
#//cycle_time 4
#//linearSpeedX 5
#
#
#//wrenchErrorLx 4
#//wrenchErrorLy 5
#//wrenchErrorLz 6
#//wrenchErrorAx 8
#//wrenchErrorAy 9
#//wrenchErrorAz 10
#   
#//friction_violSLF  7
#//friction_violSRF  7
#//friction_violSLH  7
#//friction_violSRH  7   
#   
#unilateral_violSLF  8
#unilateral_violSRF  8
#unilateral_violSLH  8
#unilateral_violSRH  8
#
#//friction_violLF 8
#//friction_violRF 8
#//friction_violLH 8
#//friction_violRH 8
#//
#//joint_violLF 9
#//joint_violRF 9
#//joint_violLH 9
#//joint_violRH 9
#//
#//torque_violLF 7
#//torque_violRF 7
#//torque_violLH 7
#//torque_violRH 7
#
#//
#//footPosRFz 9
#//footPosDesRFz 9
#//footPosRHz 10
#//footPosDesRHz 10
#//
#
#stance_legsLF 9
#stance_legsRF 9
#stance_legsLH 10
#stance_legsRH 10
#
#//
#
#
#
#////
#//footPosDesRFz 7
#//footPosRFz 7
#//
#//grForcesLFx 10
#//feet_forcesLFx 10
#//
#//
#//grForcesLFy 11
#//feet_forcesLFy 11
#//
#//grForcesLFz 12
#//feet_forcesLFz 12
#//
#//
#
