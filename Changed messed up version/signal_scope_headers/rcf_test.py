findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))

joints = ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['lf_kfe_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
lf = ['stanceLegLF','stanceLegRF']
addSignals('ROBOT_DEBUG', msg.usec, msg.array, lf, keyLookup=msg.name)

