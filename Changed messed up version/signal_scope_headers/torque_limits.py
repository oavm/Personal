current_file_dir = os.path.dirname(__file__)
findLCMTypes(os.path.join(current_file_dir,'exlcm'))
findLCMModules(os.path.join(current_file_dir,'exlcm'))
execfile(os.path.join(os.path.dirname(__file__), 'util.py'))

# print out all thde available lcm types:
#print getMessageTypeNames()



signals1 = ['LF_HAA_load','LF_HAAmin','LF_HAAmax']
signals2 = ['LF_HFE_load','LF_HFEmin','LF_HFEmax']
signals3 = ['LF_KFE_load','LF_KFEmin','LF_KFEmax'] 
signals4 = ['RF_HAA_load','RF_HAAmin','RF_HAAmax']
signals5 = ['RF_HFE_load','RF_HFEmin','RF_HFEmax'] 
signals6 = ['RF_KFE_load','RF_KFEmin','RF_KFEmax']
signals7 = ['LH_HAA_load','LH_HAAmin','LH_HAAmax'] 
signals8 = ['LH_HFE_load','LH_HFEmin','LH_HFEmax'] 
signals9 = ['LH_KFE_load','LH_KFEmin','LH_KFEmax']
signals10 =['RH_HAA_load','RH_HAAmin','RH_HAAmax'] 
signals11 =['RH_HFE_load','RH_HFEmin','RH_HFEmax'] 
signals12 =['RH_KFE_load','RH_KFEmin','RH_KFEmax']


p1 = addPlot()
p2 = addPlot()
p3 = addPlot()
p4 = addPlot()
p5 = addPlot()
p6 = addPlot()
p7 = addPlot()
p8 = addPlot()
p9 = addPlot()
p10 = addPlot()
p11 = addPlot()
p12 = addPlot()



addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals1 ,  keyLookup=msg.name, plot=p1 )
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals2 ,  keyLookup=msg.name, plot=p2 )
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals3 ,  keyLookup=msg.name, plot=p3 )
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals4 ,  keyLookup=msg.name, plot=p4 )
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals5 ,  keyLookup=msg.name, plot=p5 )
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals6 ,  keyLookup=msg.name, plot=p6 )
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals7 ,  keyLookup=msg.name, plot=p7 )
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals8 ,  keyLookup=msg.name, plot=p8 )
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals9 ,  keyLookup=msg.name, plot=p9 )
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals10,  keyLookup=msg.name, plot=p10)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals11,  keyLookup=msg.name, plot=p11)
addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals12,  keyLookup=msg.name, plot=p12)

formatOptions(pointSize=4,timeWindow=7.0,curveStyle="lines")
