if os.environ.has_key("PRONTO_ROOT"):
    print os.environ.get('PRONTO_ROOT')
else:
    print 'PRONTO_ROOT NOT SET!'
findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))
execfile(os.path.join(os.path.dirname(__file__), 'util.py'))

signals1 = ['des_baseBX' , 'actual_CoMBX']
signals2 = ['des_baseBY', 'actual_CoMBY']

#signals2 = ['des_base_pitch', 'pitch','terrainPitch']
#signals2 = ['grForcesLFz','feetForcesLFz']
#signals3 = ['grForcesLFz','grForcesRFz','grForcesLHz','grForcesRHz']

#signals4 = ['baseTwistLx','baseTwistLy', 'baseTwistLz','baseTwistAx','baseTwistAy','baseTwistAz']
#signals4 = ['footPosDesLFz','footPosLFz' ,'footPosDesRFz','footPosRFz','footPosDesLHz','footPosLHz', 'footPosDesRHz','footPosRHz']

#signals3 = ['comWrenchLX','comWrenchLY', 'comWrenchLZ']
#signals4 = ['comWrenchAX','comWrenchAY', 'comWrenchAZ']
signals3 = ['status_LF','status_RF', 'status_LH', 'status_RH']
signals4 = ['footPosDesLFz' ,'footPosDesRFz','footPosDesLHz','footPosDesRHz']

#signals5 = ['des_height', 'actual_height']
signals5 = ['zmpX','zmpY']

setFormatOptions(pointSize=4,timeWindow=7.0,curveStyle="lines")

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


formatOptions(pointSize=4,timeWindow=10.0,curveStyle="lines")


