findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))

addPlot()
lf = ['lf_haa_joint_u_des','lf_haa_joint_u_hyd']
addSignals('MOTOR', msg.usec, msg.array, lf, keyLookup=msg.name)

addPlot()
rf = ['rf_haa_joint_u_des','rf_haa_joint_u_hyd']
addSignals('MOTOR', msg.usec, msg.array, rf, keyLookup=msg.name)

addPlot()
lh = ['lh_haa_joint_u_des','lh_haa_joint_u_hyd']
addSignals('MOTOR', msg.usec, msg.array, lh, keyLookup=msg.name)

addPlot()
rh = ['rh_haa_joint_u_des','rh_haa_joint_u_hyd']
addSignals('MOTOR', msg.usec, msg.array, rh, keyLookup=msg.name)
