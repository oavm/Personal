findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))

addPlot()
lf_1 = ['lf_haa_joint_u_des','lf_haa_joint_u_hyd']
addSignals('MOTOR', msg.usec, msg.array, lf_1, keyLookup=msg.name)

addPlot()
lf_2 = ['lf_hfe_joint_u_des','lf_hfe_joint_u_hyd']
addSignals('MOTOR', msg.usec, msg.array, lf_2, keyLookup=msg.name)

addPlot()
lf_3 = ['lf_kfe_joint_u_des','lf_kfe_joint_u_hyd']
addSignals('MOTOR', msg.usec, msg.array, lf_3, keyLookup=msg.name)

