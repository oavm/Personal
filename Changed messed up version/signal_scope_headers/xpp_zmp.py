findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))


addPlot()
data_to_plot = ['cogx']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, data_to_plot, keyLookup=msg.name)

addPlot()
data_to_plot = ['cogy']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, data_to_plot, keyLookup=msg.name)

addPlot()
data_to_plot = ['trigger']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, data_to_plot, keyLookup=msg.name)