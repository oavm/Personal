if os.environ.has_key("PRONTO_ROOT"):
	print os.environ.get('PRONTO_ROOT')
else:
	print 'PRONTO_ROOT NOT SET!'
	
findLCMTypes(os.path.expanduser(os.getenv('PRONTO_ROOT', '~/pronto-distro') + '/build/lib/python2.7/dist-packages/*'))

addPlot()
joints = ['lf_haa_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['lf_hfe_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['lf_kfe_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['rf_haa_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['rf_hfe_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['rf_kfe_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['lh_haa_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['lh_hfe_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['lh_kfe_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['rh_haa_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['rh_hfe_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['rh_kfe_joint']
addSignals('ROBOT_ACT_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('ROBOT_DES_JS', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
