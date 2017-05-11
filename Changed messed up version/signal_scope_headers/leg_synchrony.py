findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))

addPlot()
lf_1 = ['stance_leg_LF']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, lf_1, keyLookup=msg.name)

addPlot()
lf_2 = ['stance_leg_RF']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, lf_2, keyLookup=msg.name)

addPlot()
lf_3 = ['stance_leg_LH']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, lf_3, keyLookup=msg.name)

addPlot()
lf_4 = ['stance_leg_RH']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, lf_4, keyLookup=msg.name)
