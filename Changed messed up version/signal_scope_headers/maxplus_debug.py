findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))

addPlot()
lf_1 = ['CPG_angularFreqLF','CPG_angularFreqRF','CPG_angularFreqLH','CPG_angularFreqRH']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, lf_1, keyLookup=msg.name)

addPlot()
lf_2 = ['P_Local_BF_LFz','P_Local_BF_RFz','P_Local_BF_LHz','P_Local_BF_RHz']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, lf_2, keyLookup=msg.name)

addPlot()
lf_3 = ['stance_leg_LF','stance_leg_RF','stance_leg_LH','stance_leg_RH']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, lf_3, keyLookup=msg.name)

addPlot()
lf_4 = ['footSensorLFz','footSensorRFz','footSensorLHz','footSensorRHz']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, lf_4, keyLookup=msg.name)
