
if os.environ.has_key("PRONTO_ROOT"):
    print os.environ.get('PRONTO_ROOT')
else:
    print 'PRONTO_ROOT NOT SET!'
findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))


#execfile(os.path.join(os.path.dirname(__file__), 'util.py'))
#makePlots( ['grForcesLFy','grForcesRFy'], p1)

#signals1 = ['stance_LF','stance_RF', 'stance_LH', 'stance_RH']
#signals1 = ['des_base_pitch', 'pitch']
signals1 = ['heightReflexActive']
#signals1 =['mobility_LF','mobility_RF','mobility_LH','mobility_RH']
#signals2 = ['des_height', 'actual_height']

#signals2 = ['stance_LF','stance_RF', 'stance_LH', 'stance_RH']

signals3 =['D_LF','D_RF']
#signals4 = ['footPosDesLFz','footPosDesRFz','footPosDesCorrLFz', 'footPosDesCorrRFz']
signals4 = [ 'footPosDesLFz','footPosDesRFz', 'footPosLFz','footPosRFz']

signals5 =['D_LH','D_RH']
#signals6 = [  'footPosDesLHz','footPosDesRHz','footPosDesCorrLHz', 'footPosDesCorrRHz']
signals6 = [ 'footPosDesLHz','footPosDesRHz', 'footPosLHz','footPosRHz']

#debug
#signals1 = ['grForcesLFz','grForcesRFz']
#signals2 = ['LF_HFE_th','LF_HFE_des_th', 'RF_HFE_th','RF_HFE_des_th','heightReflexActive']
#signals3 = ['swingDown']
#signals4 = ['jointLim_LF_HFE', 'jointLim_LF_KFE']
#signals5 = ['RH_KFE_des_th', 'RF_KFE_des_th','RH_KFE_des_th_corr','RF_KFE_des_th_corr'] #not used anymore


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


