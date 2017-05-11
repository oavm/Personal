findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))

addPlot()
p1 = ['lf_haa_joint_hst_x','lf_haa_joint_hst_dx']
addSignals('MOTOR', msg.usec, msg.array, p1, keyLookup=msg.name)

addPlot()
p2 = ['lf_hfe_joint_hst_x','lf_hfe_joint_hst_dx']
addSignals('MOTOR', msg.usec, msg.array, p2, keyLookup=msg.name)

addPlot()
p3 = ['lf_kfe_joint_hst_x','lf_kfe_joint_hst_dx']
addSignals('MOTOR', msg.usec, msg.array, p3, keyLookup=msg.name)

