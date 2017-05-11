current_file_dir = os.path.dirname(__file__)
findLCMTypes(os.path.join(current_file_dir,'include/exlcm'))
findLCMModules(os.path.join(current_file_dir,'include/exlcm'))

findLCMTypes(os.path.expanduser('~/catkin_ws/packages/pronto-distro/build/lib/python2.7/dist-packages/*'))

# print out all the available lcm types:
#print getMessageTypeNames()

#do it singularly


#signals1 = ['lf_kfe_joint_u_des','lf_kfe_joint_u_hyd','lf_kfe_joint_u']
#signals1 = ['footPosDesLFz','footPosLFz','footPosDesRFz','footPosRFz','footPosDesLHz','footPosLHz','footPosDesRHz','footPosRHz']
signals1 = ['footPosDesLFz','footPosLFz','footPosDesRFz','footPosRFz','footPosDesLHz','footPosLHz','footPosDesRHz','footPosRHz']
#signals2 = ['footPosDesRFz','footPosRFz','grForcesRFz','ForceProfile_RF']
#signals3 = ['footPosDesLHz','footPosLHz','grForcesLHz','ForceProfile_LH']
#signals4 = ['footPosDesRHz','footPosRHz','grForcesRHz','ForceProfile_RH']
signals4 = ['state_machine_LF','state_machine_LH','state_machine_RF','state_machine_RH']

#signals1 = ['feetForcesLFx','feetForcesLFy','feetForcesLFz','grForcesLFx','grForcesLFy','grForcesLFz']
#signals2 = ['feetForcesLHx','feetForcesLHy','feetForcesLHz','grForcesLHx','grForcesLHy','grForcesLHz']
#signals3 = ['feetForcesRFx','feetForcesRFy','feetForcesRFz','grForcesRFx','grForcesRFy','grForcesRFz']
#signals4 = ['feetForcesRHx','feetForcesRHy','feetForcesRHz','grForcesRHx','grForcesRHy','grForcesRHz']


signals2 = ['grForcesHF_LFz','grForcesHF_RFz','grForcesHF_LHz','grForcesHF_RHz','grfDes_Z_LF_HF' , 'grfDes_Z_RF_HF', 'grfDes_Z_LH_HF','grfDes_Z_RH_HF','ForceProfile_Z_LF']
signals3 = ['grForcesHF_LFx','grForcesHF_RFx','grForcesHF_LHx','grForcesHF_RHx','grfDes_X_LF_HF' , 'grfDes_X_RF_HF', 'grfDes_X_LH_HF','grfDes_X_RH_HF','ForceProfile_X_LF']

#signals2 = ['feetForcesRFz','rf_haa_joint_uff','rf_hfe_joint_uff','rf_kfe_joint_uff','grForcesRFz','ForceProfile_RF']
#signals3 = ['feetForcesLHz','lh_haa_joint_uff','lh_hfe_joint_uff','lh_kfe_joint_uff','grForcesLHz','ForceProfile_LH']
#signals4 = ['feetForcesRHz','rh_haa_joint_uff','rh_hfe_joint_uff','rh_kfe_joint_uff','grForcesRHz','ForceProfile_RH']
signals5 = ['Actual_duty_LF','Actual_stance_LF','Actual_swing_LF','Period_counter_LF','Period_counter_LH','Period_counter_RF','Period_counter_RH']
signals6 = ['actual_speed_HF_X','linearSpeedX','actual_speed_HF_Y','linearSpeedY']


#signals3 = ['ffwd_pitch_LF','ffwd_pitch_RF','ffwd_pitch_LH','ffwd_pitch_RH']
#signals4 = ['des_baseBX','actual_CoMBX','des_baseBY','actual_CoMBY','des_baseBZ','actual_CoMBZ','roll','pitch','yaw','des_roll','des_pitch','des_yaw']
#signals5 = ['actual_CoMBXd','actual_CoMBYd','actual_CoMBZd','roll_d','pitch_d','yaw_d']

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

setFormatOptions(pointSize=4,timeWindow=3.0,curveStyle="lines")


