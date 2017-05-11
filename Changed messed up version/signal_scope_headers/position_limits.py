current_file_dir = os.path.dirname(__file__)
findLCMTypes(os.path.join(current_file_dir,'exlcm'))
findLCMModules(os.path.join(current_file_dir,'exlcm'))
execfile(os.path.join(os.path.dirname(__file__), 'util.py'))

# print out all thde available lcm types:
#print getMessageTypeNames()



signals1 = ['LF_HAA_th','LF_HAA_th_min','LF_HAA_th_max']
signals2 = ['LF_HFE_th','LF_HFE_th_min','LF_HFE_th_max']
signals3 = ['LF_KFE_th','LF_KFE_th_min','LF_KFE_th_max'] 
signals4 = ['RF_HAA_th','RF_HAA_th_min','RF_HAA_th_max']
signals5 = ['RF_HFE_th','RF_HFE_th_min','RF_HFE_th_max'] 
signals6 = ['RF_KFE_th','RF_KFE_th_min','RF_KFE_th_max']
signals7 = ['LH_HAA_th','LH_HAA_th_min','LH_HAA_th_max'] 
signals8 = ['LH_HFE_th','LH_HFE_th_min','LH_HFE_th_max'] 
signals9 = ['LH_KFE_th','LH_KFE_th_min','LH_KFE_th_max']
signals10 =['RH_HAA_th','RH_HAA_th_min','RH_HAA_th_max'] 
signals11 =['RH_HFE_th','RH_HFE_th_min','RH_HFE_th_max'] 
signals12 =['RH_KFE_th','RH_KFE_th_min','RH_KFE_th_max']


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
