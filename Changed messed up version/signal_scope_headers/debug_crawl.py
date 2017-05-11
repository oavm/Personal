
if os.environ.has_key("PRONTO_ROOT"):
    print os.environ.get('PRONTO_ROOT')
else:
    print 'PRONTO_ROOT NOT SET!'
findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))


#execfile(os.path.join(os.path.dirname(__file__), 'util.py'))
#makePlots( ['grForcesLFy','grForcesRFy'], p1)

#signals1 = ['des_baseBX' , 'actual_CoMBX']
#signals2 = ['des_baseBY', 'actual_CoMBY']
signals1 = ['des_base_pitch', 'pitch','terrainPitch']
signals2 = ['des_base_roll', 'roll','terrainRoll']
#signals1 = ['actual_baseX']
#signals2 = ['actual_baseZ']
#signals2 = ['des_height', 'actual_height']
#signals3 = [ 'footPosDesLFz','footPosDesRFz','footPosDesLHz','footPosDesRHz']

#tracking
signals3 = [ 'footPosDesLFz','footPosLFz','footPosDesRFz','footPosRFz']
#signals4 = [ 'footPosDesLHz','footPosLHz','footPosDesRHz','footPosRHz']

#signals2 = ['grForcesLFz','feetForcesRFz','feetForcesLHz','feetForcesRHz','swingDown']
#signals4 = ['grForcesLFz','grForcesRFz','grForcesLHz','grForcesRHz']
#signals4 = ['feetForcesLFz','feetForcesRFz','feetForcesLHz','feetForcesRHz']
#signals4 = ['baseTwistLx','baseTwistLy', 'baseTwistLz','baseTwistAx','baseTwistAy','baseTwistAz']

#signals4 = ['comWrenchLX','comWrenchLY', 'comWrenchLZ']
#signals5 = ['comWrenchAX','comWrenchAY', 'comWrenchAZ']

#signals5 = ['LH_HFE_th','LH_HFE_th_min','LH_HFE_th_max'] 
#signals5 = ['LH_KFE_th','LH_KFE_th_min','LH_KFE_th_max']

#signals5 = ['RH_HAA_th','RH_HAA_des_th', 'RH_HFE_th','RH_HFE_des_th', 'RH_KFE_th','RH_KFE_des_th'] 

#signals5 = ['climbingStairsLF','climbingStairsRF', 'climbingStairsLH', 'climbingStairsRH']

signals4 = ['stance_LF','stance_RF', 'stance_LH', 'stance_RH']
signals5 = ['shinCollisionLF', 'shinCollisionRF', 'shinCollisionLH', 'shinCollisionRH']
signals6 = ['contactSwitchLF', 'contactSwitchRF', 'contactSwitchLH', 'contactSwitchRH']


p1 = addPlot()
p2 = addPlot()
p3 = addPlot()
p4 = addPlot()
p5 = addPlot()
p6 = addPlot()

addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals1,  keyLookup=msg.name, plot=p1)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals2,  keyLookup=msg.name, plot=p2)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals3,  keyLookup=msg.name, plot=p3)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals4,  keyLookup=msg.name, plot=p4)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals5,  keyLookup=msg.name, plot=p5)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals6,  keyLookup=msg.name, plot=p6)

setFormatOptions(pointSize=4,timeWindow=7.0,curveStyle="lines")


