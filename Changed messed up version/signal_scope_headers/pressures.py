findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))

#addPlot()
#lf_2 = ['lf_hfe_joint_hst_pa','lf_hfe_joint_hst_pb','lf_hfe_joint_hst_approxPa','lf_hfe_joint_hst_approxPb','lf_hfe_joint_hst_oldApproxPa','lf_hfe_joint_hst_oldApproxPb']
#addSignals('MOTOR', msg.usec, msg.array, lf_2, keyLookup=msg.name)

addPlot()
p_haa = ['lf_haa_joint_hst_pa','lf_haa_joint_hst_pb']
#['lf_kfe_joint_hst_pa','lf_kfe_joint_hst_pb','lf_kfe_joint_hst_approxPa','lf_kfe_joint_hst_approxPb','lf_kfe_joint_hst_oldApproxPa','lf_kfe_joint_hst_oldApproxPb','lf_kfe_joint_hst_newApproxPa','lf_kfe_joint_hst_newApproxPb']
addSignals('MOTOR', msg.usec, msg.array, p_haa, keyLookup=msg.name)

addPlot()
#error = ['lf_kfe_joint_hst_errorPaNew','lf_kfe_joint_hst_errorPbNew','lf_kfe_joint_hst_errorPaOld','lf_kfe_joint_hst_errorPbOld','lf_kfe_joint_hst_estErrorPaNew','lf_kfe_joint_hst_estErrorPaOld']
p_hfe = ['lf_hfe_joint_hst_pa','lf_hfe_joint_hst_pb']
addSignals('MOTOR', msg.usec, msg.array, p_hfe, keyLookup=msg.name)

#addPlot()
#actPos = ['lf_kfe_joint_hst_x','lf_hfe_joint_hst_x']
#addSignals('MOTOR', msg.usec, msg.array, actPos, keyLookup=msg.name)

addPlot()
#actVel = ['lf_kfe_joint_hst_dx','lf_hfe_joint_hst_dx']
p_kfe = ['lf_kfe_joint_hst_pa','lf_kfe_joint_hst_pb']
addSignals('MOTOR', msg.usec, msg.array, p_kfe, keyLookup=msg.name)

#addPlot()
#actAcc = ['lf_kfe_joint_hst_ddx','lf_hfe_joint_hst_ddx']
#addSignals('MOTOR', msg.usec, msg.array, actAcc, keyLookup=msg.name)

addPlot()
actVel = ['lf_kfe_joint_hst_dx','lf_hfe_joint_hst_dx']
addSignals('MOTOR', msg.usec, msg.array, actVel, keyLookup=msg.name)
