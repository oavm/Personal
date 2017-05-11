if os.environ.has_key("PRONTO_ROOT"):
    print os.environ.get('PRONTO_ROOT')
else:
    print 'PRONTO_ROOT NOT SET!'
findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))
execfile(os.path.join(os.path.dirname(__file__), 'util.py'))



signals1 = ['comWrenchLX']
signals2 = ['comWrenchLY']
signals3 = ['comWrenchLZ']
signals4 = ['comWrenchAX']
signals5 = ['comWrenchAY']
signals6 = ['comWrenchAZ']

signals7 = ['LF_KFE_uff']

#signals5 = ['stance_LF','stance_RF', 'stance_LH', 'stance_RH']

p1 = addPlot()
p2 = addPlot()
p3 = addPlot()
p4 = addPlot()
p5 = addPlot()
p6 = addPlot()
p7 = addPlot()

addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals1,  keyLookup=msg.name, plot=p1)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals2,  keyLookup=msg.name, plot=p2)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals3,  keyLookup=msg.name, plot=p3)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals4,  keyLookup=msg.name, plot=p4)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals5,  keyLookup=msg.name, plot=p5)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals6,  keyLookup=msg.name, plot=p6)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals7,  keyLookup=msg.name, plot=p7)

setFormatOptions(pointSize=4,timeWindow=7.0,curveStyle="lines")

