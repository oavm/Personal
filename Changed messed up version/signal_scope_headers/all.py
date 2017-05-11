current_file_dir = os.path.dirname(__file__)
findLCMTypes(os.path.join(current_file_dir,'exlcm'))
findLCMModules(os.path.join(current_file_dir,'exlcm'))
execfile(os.path.join(os.path.dirname(__file__), 'util.py'))

# print out all thde available lcm types:
#print getMessageTypeNames()



signals1 = [
'lf_haa_joint_thd',
'lf_hfe_joint_thd', 
'lf_kfe_joint_thd', 
'rf_haa_joint_thd',
'rf_hfe_joint_thd', 
'rf_kfe_joint_thd',
'lh_haa_joint_thd', 
'lh_hfe_joint_thd', 
'lh_kfe_joint_thd',
'rh_haa_joint_thd', 
'rh_hfe_joint_thd', 
'rh_kfe_joint_thd'] 

p1 = addPlot()
#p2 = addPlot()
#p3 = addPlot()
#p4 = addPlot()

addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals1,  keyLookup=msg.name, plot=p1)
#addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals2,  keyLookup=msg.name, plot=p2)
#addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals3,  keyLookup=msg.name, plot=p3)
#addSignals('ROBOT_DEBUG', msg.usec, msg.array, signals4,  keyLookup=msg.name, plot=p4)


formatOptions(pointSize=4,timeWindow=7.0,curveStyle="lines")
