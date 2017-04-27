/*
 * RCFController.cpp
 *
 *  Created on: Jul 27, 2015
 */
#include <rcf_controller/RCFController.h>
#include <dls_controller/support/ConsoleUtility.h>
#include <ros/package.h>

#include <pluginlib/class_list_macros.h>

using namespace iit;
using namespace HyQ2Max;

PLUGINLIB_EXPORT_CLASS(dls_controller::RCFController, dls_controller::Controller)



namespace dls_controller
{

RCFController::RCFController()
{

}

RCFController::~RCFController()
{

}

bool RCFController::construct(int nJoints,
							  int nFTSensors,
							  int taskRate,
							  std::string robot)
{
	Controller::setControllerName("RCFController");
	Controller::initBase(nJoints, nFTSensors, taskRate, robot);
	zeroGainTh.resize(12);
	zeroGainThd.resize(12);
	zeroGainInt.resize(12);
	tauDeltaPos.resize(12);
	deltaUff.resize(12);
	gaitPattern = 0;
	des_q_ori.resize(nJoints);
	des_qd_ori.resize(nJoints);
	des_tau_ori.resize(nJoints);

	// Robot node-handle
	ros::NodeHandle robot_node(robot);
	node_ = robot_node;

	// Reading the urdf model
	XmlRpc::XmlRpcValue robot_model;
	if (!node_.getParam("robot_model", robot_model)) {
		ROS_ERROR("Could not find the robot model");
		return false;
	}

	// Reading the yarf model
	std::string robot_config;
	if (!node_.getParam("robot_config", robot_config)) {
		ROS_ERROR("Could not find the robot config file");
		return false;
	}

	// Initializing the floating-base system properties
	fbs_.resetFromURDFModel(robot_model, robot_config);
	wbd_.modelFromURDFModel(robot_model, robot_config);

	// Computing the joint mapping for RobCoGen conversion
	joint_id_map_.resize(fbs_.getJointDoF());
	joint_id_map_[dog::LEG_LF_HAA] = fbs_.getJointId("lf_haa_joint");
	joint_id_map_[dog::LEG_LF_HFE] = fbs_.getJointId("lf_hfe_joint");
	joint_id_map_[dog::LEG_LF_KFE] = fbs_.getJointId("lf_kfe_joint");
	joint_id_map_[dog::LEG_RF_HAA] = fbs_.getJointId("rf_haa_joint");
	joint_id_map_[dog::LEG_RF_HFE] = fbs_.getJointId("rf_hfe_joint");
	joint_id_map_[dog::LEG_RF_KFE] = fbs_.getJointId("rf_kfe_joint");
	joint_id_map_[dog::LEG_LH_HAA] = fbs_.getJointId("lh_haa_joint");
	joint_id_map_[dog::LEG_LH_HFE] = fbs_.getJointId("lh_hfe_joint");
	joint_id_map_[dog::LEG_LH_KFE] = fbs_.getJointId("lh_kfe_joint");
	joint_id_map_[dog::LEG_RH_HAA] = fbs_.getJointId("rh_haa_joint");
	joint_id_map_[dog::LEG_RH_HFE] = fbs_.getJointId("rh_hfe_joint");
	joint_id_map_[dog::LEG_RH_KFE] = fbs_.getJointId("rh_kfe_joint");

	// Computing the leg mapping for RobCoGen conversion
	leg_name_map_.resize(fbs_.getNumberOfEndEffectors(dwl::model::FOOT));
	leg_name_map_[dog::LF] = "lf_foot";
	leg_name_map_[dog::RF] = "rf_foot";
	leg_name_map_[dog::LH] = "lh_foot";
	leg_name_map_[dog::RH] = "rh_foot";

	linksInertia.reset(new iit::HyQ2Max::dyn::InertiaProperties(inertiasGetter));
	//linksInertia(inertiasGetter),
	forceTransforms.reset(new iit::HyQ2Max::ForceTransforms(default_pg));
	jsim.reset(new dyn::JSIM(*linksInertia, *forceTransforms));

	hyq2maxIK.reset(new InverseKinematicsHyQ2Max(taskRate));
	hyqIK.reset(new InverseKinematicsHyQ(taskRate));

	jacobiansHyQ.reset(new iit::HyQ::Jacobians(default_pg2));
	jacobiansHyQ2Max.reset(new iit::HyQ2Max::Jacobians(default_pg));
	if(robotName == "hyq" || robotName == "centaur1arm"){
		JFootLF =  &jacobiansHyQ->fr_trunk_J_LF_foot;// just a simpler alias
		JFootRF =  &jacobiansHyQ->fr_trunk_J_RF_foot;// just a simpler alias
		JFootLH =  &jacobiansHyQ->fr_trunk_J_LH_foot;// just a simpler alias
		JFootRH =  &jacobiansHyQ->fr_trunk_J_RH_foot;// just a simpler alias
	}else{
		JFootLF =  &jacobiansHyQ2Max->fr_trunk_J_LF_foot;// just a simpler alias
		JFootRF =  &jacobiansHyQ2Max->fr_trunk_J_RF_foot;// just a simpler alias
		JFootLH =  &jacobiansHyQ2Max->fr_trunk_J_LH_foot;// just a simpler alias
		JFootRH =  &jacobiansHyQ2Max->fr_trunk_J_RH_foot;// just a simpler alias
	}

	trotProfiles[0] = {"0.00 m/s",	0.12,			0.015,	1.73,				0.45,		2.0,		70,			10	};
	trotProfiles[1] = {"0.25 m/s",	0.12,			0.25,	1.92,				0.45,		1.0,		70,			10	};
	trotProfiles[2] = {"0.50 m/s",	0.12,			0.50,	1.90,				0.45,		0.25,		70,			10	};
	trotProfiles[3] = {"0.75 m/s",	0.12,			0.75,	1.90,				0.4,		0.25,		70,			10	};
	trotProfiles[4] = {"1.00 m/s",	0.12,			1.00,	1.90,				0.4,		0.25,		70,			20	};
	trotProfiles[5] = {"1.25 m/s",	0.12,			1.25,	2.00,				0.4,		0.25,		70,			20	};
	trotProfiles[6] = {"1.50 m/s",	0.12,			1.50,	2.00,				0.4,		0.25,		70,			20	};
	trotProfiles[7] = {"1.75 m/s",	0.12,			1.75,	2.05,				0.4,		0.25,		70,			30	};
	trotProfiles[8] = {"2.00 m/s",	0.12,			2.00,	2.2,				0.35,		0.25,		70,			30	};


	std::string param_file;
	if(isRobotReal){
	    if(robotName == "hyq") {
	        param_file = ros::package::getPath(std::string("rcf_controller")) + "/config/hyq_real_options.ini";
	    }
	    else if(robotName == "hyq2max") {
	        param_file = ros::package::getPath(std::string("rcf_controller")) + "/config/hyq2max_real_options.ini";
	    }
	    else if(robotName == "centaur1arm"){
	        param_file = ros::package::getPath(std::string("rcf_controller")) + "/config/real_options.ini";
	    }
	    else {
	        param_file = ros::package::getPath(std::string("rcf_controller")) + "/config/real_options.ini";
	        std::cout << "Robot name does not match any option when reading the config file!" << std::endl;
	    }
	}else{
        if(robotName == "hyq") {
            param_file = ros::package::getPath(std::string("rcf_controller")) + "/config/hyq_sim_options.ini";
        }
        else if(robotName == "hyq2max") {
            param_file = ros::package::getPath(std::string("rcf_controller")) + "/config/hyq2max_sim_options.ini";
        }
        else if(robotName == "centaur1arm"){
            param_file = ros::package::getPath(std::string("rcf_controller")) + "/config/sim_options.ini";
        }
        else {
            param_file = ros::package::getPath(std::string("rcf_controller")) + "/config/sim_options.ini";
            std::cout << "Robot name does not match any option when reading the config file!" << std::endl;
        }
	}

	if (std::ifstream(param_file))
	{
		std::cout << "File exists" << std::endl;
	}

	std::cout << "Option file: " << param_file << std::endl;
	std::ifstream s(param_file);
	if(!s)
	{
		std::cerr<<"Error, config filed does not exist"<<std::endl;
	}else{
		boost::property_tree::ini_parser::read_ini(param_file.c_str(), config_);
	}

	KpTrunkRollDefault = config_.get<double>("TrunkController.KpTrunkRoll");
	KpTrunkPitchDefault = config_.get<double>("TrunkController.KpTrunkPitch");
	KpTrunkYawDefault = config_.get<double>("TrunkController.KpTrunkYaw");
	KpTrunkX_Default = config_.get<double>("TrunkController.KpTrunkX");
	KpTrunkY_Default = config_.get<double>("TrunkController.KpTrunkY");
	KpTrunkZ_Default = config_.get<double>("TrunkController.KpTrunkZ");

	KdTrunkRollDefault = config_.get<double>("TrunkController.KdTrunkRoll");
	KdTrunkPitchDefault = config_.get<double>("TrunkController.KdTrunkPitch");
	KdTrunkYawDefault = config_.get<double>("TrunkController.KdTrunkYaw");
	KdTrunkX_Default = config_.get<double>("TrunkController.KdTrunkX");
	KdTrunkY_Default = config_.get<double>("TrunkController.KdTrunkY");
	KdTrunkZ_Default = config_.get<double>("TrunkController.KdTrunkZ");

	bodyWeightDefault = config_.get<double>("TrunkController.bodyWeight");

	//Print outs for a user safety check
    std::cout << "KpTrunkRollDefault: "  << KpTrunkRollDefault << std::endl;
    std::cout << "KdTrunkRollDefault: "  << KdTrunkRollDefault << std::endl;
    std::cout << "KpTrunkPitchDefault: " << KpTrunkPitchDefault << std::endl;
    std::cout << "KdTrunkPitchDefault: " << KdTrunkPitchDefault << std::endl;
    std::cout << "KpTrunkYawDefault:  "  << KpTrunkYawDefault << std::endl;
    std::cout << "KdTrunkYawDefault: "   << KdTrunkYawDefault << std::endl;
    std::cout << "KdTrunkX_Default: "    << KdTrunkX_Default << std::endl;
    std::cout << "KpTrunkX_Default:  "   << KpTrunkX_Default << std::endl;
    std::cout << "KpTrunkY_Default: "    << KpTrunkY_Default << std::endl;
    std::cout << "KdTrunkY_Default: "    << KdTrunkY_Default << std::endl;
    std::cout << "KpTrunkZ_Default: "    << KpTrunkZ_Default << std::endl;
    std::cout << "KdTrunkZ_Default: "    << KdTrunkZ_Default << std::endl;
    std::cout << "bodyWeightDefault: "   << bodyWeightDefault << std::endl << std::endl;


	P0_Local_HF_Default[dog::LF] <<
			config_.get<double>("DesiredOrigins.LFX"),
			config_.get<double>("DesiredOrigins.LFY"),
			config_.get<double>("DesiredOrigins.LFZ");
	P0_Local_HF_Default[dog::RF] <<
			config_.get<double>("DesiredOrigins.RFX"),
			config_.get<double>("DesiredOrigins.RFY"),
			config_.get<double>("DesiredOrigins.RFZ");
	P0_Local_HF_Default[dog::LH] <<
			config_.get<double>("DesiredOrigins.LHX"),
			config_.get<double>("DesiredOrigins.LHY"),
			config_.get<double>("DesiredOrigins.LHZ");
	P0_Local_HF_Default[dog::RH] <<
			config_.get<double>("DesiredOrigins.RHX"),
			config_.get<double>("DesiredOrigins.RHY"),
			config_.get<double>("DesiredOrigins.RHZ");

	/*
	  ___           _   _     _           _
	 |_ _|  _ __   (_) | |_  (_)   __ _  | |
	 | |   | '_ \  | | | __| | |  / _` | | |
	 | |   | | | | | | | |_  | | | (_| | | |
	 |___| |_| |_| |_|  \__| |_|  \__,_| |_|

	  ____                       _   _   _     _
	 / ___|   ___    _ __     __| | (_) | |_  (_)   ___    _ __    ___
	 | |     / _ \  | '_ \   / _` | | | | __| | |  / _ \  | '_ \  / __|
	 | |___ | (_) | | | | | | (_| | | | | |_  | | | (_) | | | | | \__ \
     \____|  \___/  |_| |_|  \__,_| |_|  \__| |_|  \___/  |_| |_| |___/
	 */

     std::cout << "Init con" << std::endl;

	for (int leg = dog::LF; leg <= dog::RH; leg++){
			deltaPR_Pos[leg].setZero();
			deltaPR_Vel[leg].setZero();
			jointDesPos[leg].setZero();
			jointDesVel[leg].setZero();
			jointDesAcc[leg].setZero();
			WCPG[leg].setZero();
			WCPGb[leg].setZero();
			dWCPGb[leg].setZero();
			dWCPG[leg].setZero();
			P_Local_BF[leg].setZero();
			Pd_Local_BF[leg].setZero();
			Pdd_Local_BF[leg].setZero();
			P_DesLocal_BF[leg].setZero();
			Pd_DesLocal_BF[leg].setZero();
			Pdd_DesLocal_BF[leg].setZero();
			last_Pd_Local_BF[leg].setZero();
			P_Local_HF[leg].setZero();
			Pd_Local_HF[leg].setZero();
			Pdd_Local_HF[leg].setZero();
			P_DesLocal_HF[leg].setZero();
			Pd_DesLocal_HF[leg].setZero();
			Pdd_DesLocal_HF[leg].setZero();
			P0_Local_HF[leg].setZero();
			P0_Local_HF_New[leg].setZero();
			last_P_Local_HF[leg].setZero();
			last_Pd_Local_HF[leg].setZero();
			last_Pd_DesLocal_HF[leg].setZero();
    }

	eulerAngles.setZero();
	eulerAnglesNew.setZero();

	uffTorques.setZero();

	//Reset CPG initial conditions
	resetCPGFlag = true;

	//Set joints PID gains for swing phase
	double KpHAA = config_.get<double>("JointGainsSwing.KpHAA");
	double KpHFE = config_.get<double>("JointGainsSwing.KpHFE");
	double KpKFE = config_.get<double>("JointGainsSwing.KpKFE");
	double KdHAA = config_.get<double>("JointGainsSwing.KdHAA");
	double KdHFE = config_.get<double>("JointGainsSwing.KdHFE");
	double KdKFE = config_.get<double>("JointGainsSwing.KdKFE");

	zeroGainTh[dog::LF_HFE] = KpHFE;
	zeroGainThd[dog::LF_HFE] = KdHFE;
	zeroGainInt[dog::LF_HFE] = 0;
	zeroGainTh[dog::LF_HAA] = KpHAA;
	zeroGainThd[dog::LF_HAA] = KdHAA;
	zeroGainInt[dog::LF_HAA] = 0;
	zeroGainTh[dog::LF_KFE] = KpKFE;
	zeroGainThd[dog::LF_KFE] = KdKFE;
	zeroGainInt[dog::LF_KFE] = 0;

	zeroGainTh[dog::RF_HFE] = KpHFE;
	zeroGainThd[dog::RF_HFE] = KdHFE;
	zeroGainInt[dog::RF_HFE] = 0;
	zeroGainTh[dog::RF_HAA] = KpHAA;
	zeroGainThd[dog::RF_HAA] = KdHAA;
	zeroGainInt[dog::RF_HAA] = 0;
	zeroGainTh[dog::RF_KFE] = KpKFE;
        zeroGainThd[dog::RF_KFE] = KdKFE;
	zeroGainInt[dog::RF_KFE] = 0;

	zeroGainTh[dog::LH_HFE] = KpHFE;
	zeroGainThd[dog::LH_HFE] = KdHFE;
	zeroGainInt[dog::LH_HFE] = 0;
	zeroGainTh[dog::LH_HAA] = KpHAA;
	zeroGainThd[dog::LH_HAA] = KdHAA;
	zeroGainInt[dog::LH_HAA] = 0;
	zeroGainTh[dog::LH_KFE] = KpKFE;
	zeroGainThd[dog::LH_KFE] = KdKFE;
	zeroGainInt[dog::LH_KFE] = 0;

	zeroGainTh[dog::RH_HFE] = KpHFE;
	zeroGainThd[dog::RH_HFE] = KdHFE;
	zeroGainInt[dog::RH_HFE] = 0;
	zeroGainTh[dog::RH_HAA] = KpHAA;
	zeroGainThd[dog::RH_HAA] = KdHAA;
	zeroGainInt[dog::RH_HAA] = 0;
	zeroGainTh[dog::RH_KFE] = KpKFE;
	zeroGainThd[dog::RH_KFE] = KdKFE;
	zeroGainInt[dog::RH_KFE] = 0;

	setPID_Flag = true;

	//Update PIDManager with default joint gains
	PIDManager.setDefaultJointPIDGains(PIDManager.LF_HAA, KpHAA, 0.0, KdHAA);
	PIDManager.setDefaultJointPIDGains(PIDManager.LF_HFE, KpHFE, 0.0, KdHFE);
	PIDManager.setDefaultJointPIDGains(PIDManager.LF_KFE, KpKFE, 0.0, KdKFE);
	PIDManager.setDefaultJointPIDGains(PIDManager.RF_HAA, KpHAA, 0.0, KdHAA);
	PIDManager.setDefaultJointPIDGains(PIDManager.RF_HFE, KpHFE, 0.0, KdHFE);
	PIDManager.setDefaultJointPIDGains(PIDManager.RF_KFE, KpKFE, 0.0, KdKFE);
	PIDManager.setDefaultJointPIDGains(PIDManager.LH_HAA, KpHAA, 0.0, KdHAA);
	PIDManager.setDefaultJointPIDGains(PIDManager.LH_HFE, KpHFE, 0.0, KdHFE);
	PIDManager.setDefaultJointPIDGains(PIDManager.LH_KFE, KpKFE, 0.0, KdKFE);
	PIDManager.setDefaultJointPIDGains(PIDManager.RH_HAA, KpHAA, 0.0, KdHAA);
	PIDManager.setDefaultJointPIDGains(PIDManager.RH_HFE, KpHFE, 0.0, KdHFE);
	PIDManager.setDefaultJointPIDGains(PIDManager.RH_KFE, KpKFE, 0.0, KdKFE);

	//Set joints PID gains for stance phase
    KpHAA = config_.get<double>("JointGainsStance.KpHAA");
    KpHFE = config_.get<double>("JointGainsStance.KpHFE");
    KpKFE = config_.get<double>("JointGainsStance.KpKFE");
    KdHAA = config_.get<double>("JointGainsStance.KdHAA");
    KdHFE = config_.get<double>("JointGainsStance.KdHFE");
    KdKFE = config_.get<double>("JointGainsStance.KdKFE");

	PIDManager.setStanceJointPIDGains(PIDManager.LF_HAA, KpHAA, 0.0, KdHAA);
	PIDManager.setStanceJointPIDGains(PIDManager.LF_HFE, KpHFE, 0.0, KdHFE);
	PIDManager.setStanceJointPIDGains(PIDManager.LF_KFE, KpKFE, 0.0, KdKFE);
	PIDManager.setStanceJointPIDGains(PIDManager.RF_HAA, KpHAA, 0.0, KdHAA);
	PIDManager.setStanceJointPIDGains(PIDManager.RF_HFE, KpHFE, 0.0, KdHFE);
	PIDManager.setStanceJointPIDGains(PIDManager.RF_KFE, KpKFE, 0.0, KdKFE);
	PIDManager.setStanceJointPIDGains(PIDManager.LH_HAA, KpHAA, 0.0, KdHAA);
	PIDManager.setStanceJointPIDGains(PIDManager.LH_HFE, KpHFE, 0.0, KdHFE);
	PIDManager.setStanceJointPIDGains(PIDManager.LH_KFE, KpKFE, 0.0, KdKFE);
	PIDManager.setStanceJointPIDGains(PIDManager.RH_HAA, KpHAA, 0.0, KdHAA);
	PIDManager.setStanceJointPIDGains(PIDManager.RH_HFE, KpHFE, 0.0, KdHFE);
	PIDManager.setStanceJointPIDGains(PIDManager.RH_KFE, KpKFE, 0.0, KdKFE);

	PIDManager.usePIDManager = true;

	if(robotName == "hyq" || robotName == "centaur1arm"){
	    des_q_[0] = -0.2;
	    des_q_[1] = 0.7;
	    des_q_[2] = -1.4;
	    des_q_[3] = -0.2;
	    des_q_[4] = 0.7;
	    des_q_[5] = -1.4;
	    des_q_[6] = -0.2;
	    des_q_[7] = -0.7;
	    des_q_[8] = 1.4;
	    des_q_[9] = -0.2;
	    des_q_[10] = -0.7;
	    des_q_[11] = 1.4;
	}else if(robotName == "hyq2max"){
	    des_q_[0] =-0.075;
	    des_q_[1] =0.63;
	    des_q_[2] =-1.32;
	    des_q_[3] =-0.075;
	    des_q_[4] =0.63;
	    des_q_[5] =-1.32;
	    des_q_[6] =-0.075;
	    des_q_[7] =-0.63;
	    des_q_[8] =1.32;
	    des_q_[9] =-0.075;
	    des_q_[10] = -0.63;
	    des_q_[11] = 1.32;
	}
	else{
	    des_q_[0] = -0.2;
	    des_q_[1] = 0.7;
	    des_q_[2] = -1.4;
	    des_q_[3] = -0.2;
	    des_q_[4] = 0.7;
	    des_q_[5] = -1.4;
	    des_q_[6] = -0.2;
	    des_q_[7] = -0.7;
	    des_q_[8] = 1.4;
	    des_q_[9] = -0.2;
	    des_q_[10] = -0.7;
	    des_q_[11] = 1.4;
	}

	//Set desired robot position
	resetRobotDesiredPosition();
	robotPositionErrorHF.setZero();

	//For Debugging
	std::cout << "des_q_ init LF " << des_q_[0] << " " << des_q_[1] << " " << des_q_[2] << std::endl;
	std::cout << "des_q_ init RF " << des_q_[3] << " " << des_q_[4] << " " << des_q_[5] << std::endl;
	std::cout << "des_q_ init LH " << des_q_[6] << " " << des_q_[7] << " " << des_q_[8] << std::endl;
	std::cout << "des_q_ init RH " << des_q_[9] << " " << des_q_[10] << " " << des_q_[11] << std::endl;

	// State updating
	//iit::commons::SLtoRobogen<Traits>::pos(joint_des_state, des_q_);//NEED TO BE CONVERTED
	//iit::HyQ2Max::ParamsGetter_lengths pm; //What is going on here?

	//Forward Kinematics: calculates the actual end-effector position in the base frame
	if(robotName == "hyq" || robotName == "centaur1arm"){
	    iit::HyQ::HomogeneousTransforms ht(default_pg2);
	    P_Local_BF[dog::LF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LF_foot(des_q_));
	    P_Local_BF[dog::RF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RF_foot(des_q_));
	    P_Local_BF[dog::LH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LH_foot(des_q_));
	    P_Local_BF[dog::RH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RH_foot(des_q_));
	} else {
	    iit::HyQ2Max::HomogeneousTransforms ht(default_pg);
	    P_Local_BF[dog::LF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LF_foot(des_q_));
	    P_Local_BF[dog::RF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RF_foot(des_q_));
	    P_Local_BF[dog::LH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LH_foot(des_q_));
	    P_Local_BF[dog::RH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RH_foot(des_q_));
	}


	if(robotName == "hyq" || robotName == "centaur1arm"){
		jacobiansHyQ->fr_trunk_J_LF_foot(des_q_);
		jacobiansHyQ->fr_trunk_J_RF_foot(des_q_);
		jacobiansHyQ->fr_trunk_J_LH_foot(des_q_);
		jacobiansHyQ->fr_trunk_J_RH_foot(des_q_);
    } else{
		jacobiansHyQ2Max->fr_trunk_J_LF_foot(des_q_);
		jacobiansHyQ2Max->fr_trunk_J_RF_foot(des_q_);
		jacobiansHyQ2Max->fr_trunk_J_LH_foot(des_q_);
		jacobiansHyQ2Max->fr_trunk_J_RH_foot(des_q_);
	}

    //Forward Kinematics: calculates the actual end-effector velocity in the base frame
    Pd_Local_BF[dog::LF] = (*JFootLF).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(0, 0);
    Pd_Local_BF[dog::RF] = (*JFootRF).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(3, 0);
    Pd_Local_BF[dog::LH] = (*JFootLH).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(6, 0);
    Pd_Local_BF[dog::RH] = (*JFootRH).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(9, 0);

    //Compute initial conditions
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        //Base Frame
        last_Pd_Local_BF[leg].setZero();
        P_DesLocal_BF[leg] = P_Local_BF[leg];
        Pd_DesLocal_BF[leg].setZero();

        //Horizontal Frame
        P0_Local_HF[leg] = baseToHF(P_DesLocal_BF[leg]);
        //P0_Local_HF[leg] = P_DesLocal_BF[leg];
        P0_Local_HF_New[leg] = P0_Local_HF[leg];
       	P_Local_HF[leg] = baseToHF(P_Local_BF[leg]);
        //P_Local_HF[leg] = P_Local_BF[leg];
        last_P_Local_HF[leg] = P0_Local_HF[leg];
        last_Pd_Local_HF[leg].setZero();
        P_DesLocal_HF[leg] = P_Local_BF[leg];
        Pd_DesLocal_HF[leg].setZero();
        last_Pd_DesLocal_HF[leg].setZero();
    }

    //Setting filter response
	originChangingRate = filterResponse::mediumC;

	//Virtual leg initial position
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        virtualLegPosition =  P0_Local_HF[leg] / 4.0;
	}

	desVirtualLegPosition = virtualLegPosition;

	//Print out the initial feet position
	printf("P0_Local_LF:  %f  %f %f\n", P0_Local_HF[dog::LF](rbd::X),
			P0_Local_HF[dog::LF](rbd::Y), P0_Local_HF[dog::LF](rbd::Z));

	printf("P0_Local_RF:  %f %f %f\n", P0_Local_HF[dog::RF](rbd::X),
			P0_Local_HF[dog::RF](rbd::Y), P0_Local_HF[dog::RF](rbd::Z));

	printf("P0_Local_LH: %f  %f %f\n", P0_Local_HF[dog::LH](rbd::X),
			P0_Local_HF[dog::LH](rbd::Y), P0_Local_HF[dog::LH](rbd::Z));

	printf("P0_Local_RH: %f %f %f\n", P0_Local_HF[dog::RH](rbd::X),
			P0_Local_HF[dog::RH](rbd::Y), P0_Local_HF[dog::RH](rbd::Z));


	myTime = 0.0; //This really needs a better name

	//Trunk COG position offset from the base frame.
	deltaCOG.setZero();
	deltaCOG << config_.get<double>("TrunkController.deltaCOGX"),
				config_.get<double>("TrunkController.deltaCOGY"),
				config_.get<double>("TrunkController.deltaCOGZ");

	//Push Recovery Initialization
	PushRecovery.setDeltaLimits(0.2, 0.25);
	PushRecovery.setTrunkParameters(precTrunkWeight, 6.7, 0.747 / 2, 0.414 / 2);
	PushRecovery.setFeetHeight(sqrt(P0_Local_HF[dog::LF](rbd::Z) * P0_Local_HF[dog::LF](rbd::Z)));
	PushRecovery.setRecoveryGains(precX, precY, precYaw);
	PushRecovery.setTaskFrequency((double)taskServoRate);
	PushRecovery.setOutputFilters(minCutOffFreq);

	//Zero push recovery outputs
    deltaX.setZero();
	deltaY.setZero();

	//Terrain Estimation Initialization
	TerrainEstimator.setForceThreshold(50.0);
    TerrainEstimator.setLimits(20.0 * M_PI / 180.0, 20.0 * M_PI / 180.0);
    TerrainEstimator.setFilter(1.0 / (double)taskServoRate, 0.2);

    //Terrain based adjustments initialization
    terrainBasedAdjustment.setAdjustmentFactor(0.75);
    terrainBasedAdjustment.setTaskFrequency((double)taskServoRate);
    terrainBasedAdjustment.filterTimeConstant = 0.2;


	//Kinematic Adjustment Initialization
	KinAdjustment.setTaskFrequency((double)taskServoRate);
	KinAdjustment.setKinAdjustment(kadjRoll, kadjPitch);
    KinAdjustment.setRollLimit(5 * M_PI / 180);
    KinAdjustment.setPitchLimit(5 * M_PI / 180);
	KinAdjustment.setSaturationRate(100, 100);

	//State Estimation Initialization
	forceCov << 0, 0, 20;
	kinematicsCov << 0.0025, 0.0025, 0.25;
	IMU_Cov << 0.1, 0.1, 0.1;
	kinematicsOffset.setZero();
	IMU_OffsetHF.setZero();
	stateEstimation.setInSimulation(true);
	stateEstimation.setConsiderGRF(true);
	stateEstimation.setMinStanceLegs(1);
	stateEstimation.setImuNoiseCov(IMU_Cov);
	stateEstimation.setImuAccelOffset(IMU_OffsetHF);
	stateEstimation.setKinematicsNoiseCov(kinematicsCov);
	stateEstimation.setKinematicsVelOffset(kinematicsOffset);
	stateEstimation.setForceNoiseCov(forceCov);
	stateEstimation.setBodyWeight(bodyWeightKF);
	stateEstimation.setSamplingRate((double)taskServoRate);

	lastXbEstHFKinIMU.setZero();

	//CPG Initialization
	for (int leg = dog::LF; leg <= dog::RH; leg++) {
		CPG[leg].setTaskFrequency((double)taskServoRate);
		CPG[leg].subSteps = subStep;
		CPG[leg].enableHeightModulation(CPG_StepHeightModulationFlag);
		CPG[leg].stanceHs = 0.001;
	}


	CPG[dog::LF].Xp << 0.0001, 0.0, -0.0001;
	CPG[dog::RH].Xp = CPG[dog::LF].Xp;
	CPG[dog::RF].Xp << -0.0001, 0.0, 0.0001;
	CPG[dog::LH].Xp = CPG[dog::RF].Xp;

	for (int leg = dog::LF; leg <= dog::RH; leg++)
		CPG[leg].Xf = CPG[leg].Xp;



	//CPG pattern
	//trot
  //   couplingMatrix((int)Pattern::Trot) << 0.0, -1.0, -1.0,  1.0,
  //                                        -1.0,  0.0,  1.0, -1.0,
  //                                        -1.0,  1.0,  0.0, -1.0,
  //                                         1.0, -1.0, -1.0,  0.0;
	// //bound
  //   couplingMatrix((int)Pattern::Bound) << 0.0,  1.0, -1.0, -1.0,
  //                                          1.0,  0.0, -1.0, -1.0,
  //                                         -1.0, -1.0,  0.0,  1.0,
  //                                         -1.0, -1.0, -1.0,  0.0;
	// //walk
  //   couplingMatrix((int)Pattern::Walk) <<  0.0, -1.0,  1.0, -1.0,
  //                                         -1.0,  0.0, -1.0,  1.0,
  //                                         -1.0,  1.0,  0.0, -1.0,
  //                                          1.0, -1.0, -1.0,  0.0;
	//pace (to be included)

	//Define initial locomotion pattern
	// pattern = Pattern::Trot; /*OCTAVIO*/

    //Print out the initial feet position
    cout << "Initial Feet position:" << endl;
    cout << "LF: " << (CPG[dog::LF].Xf).transpose() << endl;
    cout << "RF: " << (CPG[dog::RF].Xf).transpose() << endl;
    cout << "LH: " << (CPG[dog::LH].Xf).transpose() << endl;
    cout << "RH: " << (CPG[dog::RH].Xf).transpose() << endl << endl;

	//Initialize PIDManager
    PIDManager.setMode(1); //Based only on leg stance condition
	PIDManager.setCPGPosReferences(CPG[dog::LF].Xp, CPG[dog::RF].Xp,
	                               CPG[dog::LH].Xp, CPG[dog::RH].Xp);

    PIDManager.setCPGVelReferences(CPG[dog::LF].dXp, CPG[dog::RF].dXp,
                                   CPG[dog::LH].dXp, CPG[dog::RH].dXp);

    PIDManager.inertialCompJointGain(PIDManager.LF_HAA) = 0.25;
    PIDManager.inertialCompJointGain(PIDManager.RF_HAA) = 0.25;
    PIDManager.inertialCompJointGain(PIDManager.LH_HAA) = 0.25;
    PIDManager.inertialCompJointGain(PIDManager.RH_HAA) = 0.25;

    //Collision reflex variables
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        P_Reflex_HF[leg].setZero();
        Pd_Reflex_HF[leg].setZero();
    }

    //Joystick variables
    axes_memory.setZero();
    buttons_memory.setZero();


    //Initialize variables to compute continous yaw angle
    lastYaw = bs->getYaw_W();


	char_sub_ = node_.subscribe("char", 100, &RCFController::charICTPCallback,this);
}


void RCFController::kill()
{

}

bool RCFController::init(double time)
{
	for (int leg = dog::LF; leg <= dog::RH; leg++){
			deltaPR_Pos[leg].setZero();
			deltaPR_Vel[leg].setZero();
			jointDesPos[leg].setZero();
			jointDesVel[leg].setZero();
			jointDesAcc[leg].setZero();
			WCPG[leg].setZero();
			WCPGb[leg].setZero();
			dWCPGb[leg].setZero();
			dWCPG[leg].setZero();
			P_Local_BF[leg].setZero();
			Pd_Local_BF[leg].setZero();
			Pdd_Local_BF[leg].setZero();
			P_DesLocal_BF[leg].setZero();
			Pd_DesLocal_BF[leg].setZero();
			Pdd_DesLocal_BF[leg].setZero();
			last_Pd_Local_BF[leg].setZero();
			P_Local_HF[leg].setZero();
			Pd_Local_HF[leg].setZero();
			Pdd_Local_HF[leg].setZero();
			P_DesLocal_HF[leg].setZero();
			Pd_DesLocal_HF[leg].setZero();
			Pdd_DesLocal_HF[leg].setZero();
			P0_Local_HF[leg].setZero();
			P0_Local_HF_New[leg].setZero();
			last_P_Local_HF[leg].setZero();
			last_Pd_Local_HF[leg].setZero();
			last_Pd_DesLocal_HF[leg].setZero();
			PdDesLocalOld[leg].setZero();
			P_DesFoot[leg].setZero();
			PdDesFoot[leg].setZero();
			PdDesFootOld[leg].setZero();
    }

	eulerAngles.setZero();
	eulerAnglesNew.setZero();

	uffTorques.setZero();


	//Save initial state
	des_q_ori = des_q;
	des_qd_ori = des_qd;
	des_tau_ori = des_tau;
	orig_des_spline = 0;

	//Set reference to initial state:
	input_reference_q = des_q_ori;
	input_reference_qd = des_qd_ori;


    if(robotName == "hyq" || robotName == "centaur1arm"){
        des_q_[0] = -0.2;
        des_q_[1] = 0.7;
        des_q_[2] = -1.4;
        des_q_[3] = -0.2;
        des_q_[4] = 0.7;
        des_q_[5] = -1.4;
        des_q_[6] = -0.2;
        des_q_[7] = -0.7;
        des_q_[8] = 1.4;
        des_q_[9] = -0.2;
        des_q_[10] = -0.7;
        des_q_[11] = 1.4;
    }else if(robotName == "hyq2max"){
        des_q_[0] =-0.075;
        des_q_[1] =0.63;
        des_q_[2] =-1.32;
        des_q_[3] =-0.075;
        des_q_[4] =0.63;
        des_q_[5] =-1.32;
        des_q_[6] =-0.075;
        des_q_[7] =-0.63;
        des_q_[8] =1.32;
        des_q_[9] =-0.075;
        des_q_[10] = -0.63;
        des_q_[11] = 1.32;
    }
    else{
        des_q_[0] = -0.2;
        des_q_[1] = 0.7;
        des_q_[2] = -1.4;
        des_q_[3] = -0.2;
        des_q_[4] = 0.7;
        des_q_[5] = -1.4;
        des_q_[6] = -0.2;
        des_q_[7] = -0.7;
        des_q_[8] = 1.4;
        des_q_[9] = -0.2;
        des_q_[10] = -0.7;
        des_q_[11] = 1.4;
    }

	//Copy state back from Rocogen variables;
	for(int i = 0; i != des_qd_.size(); i++) {
		des_qd_[i] = 0.0;
	}

	//For Debugging
	// std::cout << "des_q_ init LF " << des_q_[0] << " " << des_q_[1] << " " << des_q_[2] << std::endl;
	// std::cout << "des_q_ init RF " << des_q_[3] << " " << des_q_[4] << " " << des_q_[5] << std::endl;
	// std::cout << "des_q_ init LH " << des_q_[6] << " " << des_q_[7] << " " << des_q_[8] << std::endl;
	// std::cout << "des_q_ init RH " << des_q_[9] << " " << des_q_[10] << " " << des_q_[11] << std::endl;

	// State updating
	//iit::commons::SLtoRobogen<Traits>::pos(joint_des_state, des_q_);//NEED TO BE CONVERTED
	//iit::HyQ2Max::ParamsGetter_lengths pm; //What is going on here?

	if(robotName == "hyq" || robotName == "centaur1arm"){
	    iit::HyQ::HomogeneousTransforms ht(default_pg2);
	    //Forward Kinematics: calculates the actual end-effector position in the base frame
	    P_Local_BF[dog::LF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LF_foot(des_q_));
	    P_Local_BF[dog::RF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RF_foot(des_q_));
	    P_Local_BF[dog::LH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LH_foot(des_q_));
	    P_Local_BF[dog::RH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RH_foot(des_q_));
	} else{
	    iit::HyQ2Max::HomogeneousTransforms ht(default_pg);
	    //Forward Kinematics: calculates the actual end-effector position in the base frame
	    P_Local_BF[dog::LF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LF_foot(des_q_));
	    P_Local_BF[dog::RF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RF_foot(des_q_));
	    P_Local_BF[dog::LH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LH_foot(des_q_));
	    P_Local_BF[dog::RH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RH_foot(des_q_));
	}


	if(robotName == "hyq" || robotName == "centaur1arm"){
		jacobiansHyQ->fr_trunk_J_LF_foot(des_q_);
		jacobiansHyQ->fr_trunk_J_RF_foot(des_q_);
		jacobiansHyQ->fr_trunk_J_LH_foot(des_q_);
		jacobiansHyQ->fr_trunk_J_RH_foot(des_q_);
    } else{
		jacobiansHyQ2Max->fr_trunk_J_LF_foot(des_q_);
		jacobiansHyQ2Max->fr_trunk_J_RF_foot(des_q_);
		jacobiansHyQ2Max->fr_trunk_J_LH_foot(des_q_);
		jacobiansHyQ2Max->fr_trunk_J_RH_foot(des_q_);
	}

    //Forward Kinematics: calculates the actual end-effector velocity in the base frame
    Pd_Local_BF[dog::LF] = (*JFootLF).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(0, 0);
    Pd_Local_BF[dog::RF] = (*JFootRF).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(3, 0);
    Pd_Local_BF[dog::LH] = (*JFootLH).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(6, 0);
    Pd_Local_BF[dog::RH] = (*JFootRH).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(9, 0);

    //Compute initial conditions
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        //Base Frame
        last_Pd_Local_BF[leg].setZero();
        P_DesLocal_BF[leg] = P_Local_BF[leg];
        Pd_DesLocal_BF[leg].setZero();

        //Horizontal Frame
        //P0_Local_HF[leg] = baseToHF(P_DesLocal_BF[leg]);
        P0_Local_HF[leg] = P_DesLocal_BF[leg];
        P0_Local_HF_New[leg] = P0_Local_HF[leg];
       	//P_Local_HF[leg] = baseToHF(P_Local_BF[leg]);
        P_Local_HF[leg] = P_Local_BF[leg];
        last_P_Local_HF[leg] = P0_Local_HF[leg];
        last_Pd_Local_HF[leg].setZero();
        P_DesLocal_HF[leg] = P_Local_BF[leg];
        Pd_DesLocal_HF[leg].setZero();
        last_Pd_DesLocal_HF[leg].setZero();
    }

    //Setting filter response
	originChangingRate = filterResponse::mediumC;

	//Virtual leg initial position
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        virtualLegPosition =  P0_Local_HF[leg] / 4.0;
	}

	desVirtualLegPosition = virtualLegPosition;

	// ===== OCTAVIO MAXPLUS ===== //
	int numberOfLegs; // Set number of legs of the platform
	double dutyFactorMax,stepFrequencyMax; //
	Eigen::ArrayXXd xInitial(8,1);

	// Eigen::ArrayXXd gaitPatternMax(4,1);
	// Eigen::ArrayXXd timeDifference(1,4);
	// gaitPatternMax(0,0) = 1;
	// gaitPatternMax(1,0) = 2;
	// gaitPatternMax(2,0) = 3;
	// gaitPatternMax(3,0) = 4;
	// timeDifference << 0.15,0.15,0.15,0.15;
	// dutyFactorMax = 0.8;
	// stepFrequencyMax = 1/(double)3;

	Eigen::ArrayXXd gaitPatternMax(2,2);
	Eigen::ArrayXXd timeDifference(1,2);
	gaitPatternMax << 1,4,
	               		2,3;
	timeDifference << 0.5,0.5;
	dutyFactorMax = 0.8;
	stepFrequencyMax = 1/(double)3;
	xInitial << 0,0,0,0,0,0,0,0; /* Initial max-plus algebra state */


	numberOfLegs = 4;
	schedule.set_gaitParametersMax(numberOfLegs,dutyFactorMax,stepFrequencyMax,myTime,
	                            timeDifference,gaitPatternMax);
	eventsLog = schedule.initiallist(xInitial);

    /*
	  ____           _   _                 _     _
	 / ___|   ___   | | | |   ___    ___  | |_  (_)  _ __     __ _
	| |      / _ \  | | | |  / _ \  / __| | __| | | | '_ \   / _` |
	| |___  | (_) | | | | | |  __/ | (__  | |_  | | | | | | | (_| |
	 \____|  \___/  |_| |_|  \___|  \___|  \__| |_| |_| |_|  \__, |
	                                                         |___/
 	  ____            _
	 |  _ \    __ _  | |_    __ _
	 | | | |  / _` | | __|  / _` |
	 | |_| | | (_| | | |_  | (_| |
	 |____/   \__,_|  \__|  \__,_|
	 */


	//Add the set of different functions to the menu:
	addPromptCommands();

	Eigen::Quaterniond temp = bs->getOrientation_W();
	Eigen::Vector3d tempPos = bs->getPosition_W();
	initialEst = Eigen::Affine3d::Identity();
	initialEst.translate(tempPos);
	initialEst.rotate(temp);
	std::cout << "Initial estimate = " << initialEst.matrix() << std::endl;

	return true;
}

void RCFController::run(double time,
						double period)
{
	bool debug_print = false;

	//Smooth parameter changes
	computeFilteredChanges();


	// Converting WholeBodyState to RobCoGen
	for (unsigned int i = 0; i < fbs_.getJointDoF(); i++) {
		unsigned int joint_id = joint_id_map_[i];

		// Converting the actual whole-body states
		q_(i) = actual_ws->joint_pos(joint_id);
		qd_(i) = actual_ws->joint_vel(joint_id);
		tau_(i) = actual_ws->joint_eff(joint_id);
	}

    if(debug_print) std::cout << "Run " << std::endl;
	//Temporary solution to run simulations on Gazebo
	//because there is an issue to obtain joint torques on Ubuntu 12.04
    // for (unsigned int i=0; i < 12; i++) {
    //     tau_[i] += (des_q_[i] - q_[i]) * kp[i];
    //     tau_[i] += (des_qd_[i] - qd_[i]) * kd[i];
    //     tau_[i] += des_tau_[i];
    // }

	//!!!BYPASS footSensor from robot with locally calculated version!!!!
	computeGRFfromLegTorquesBase(q_,qd_,tau_,footSensor_);
	//Testing:
	//std::cout << "footSensor_ LF " << footSensor_[dog::LF](rbd::X) << " " <<
	//footSensor_[dog::LF](rbd::Y) << " " << footSensor_[dog::LF](rbd::Z) << std::endl;
	for(int i = 0; i != footSensor->force.size(); i++) {
		footSensor->force[i] << footSensor_[i][0],footSensor_[i][1],footSensor_[i][2];
	}

	if (setPID_Flag){
		//changePIDGains(zeroGainTh, zeroGainThd, zeroGainInt); // From SL_controller.c: //NEED TO BE CONVERTED
	    for(int joint = dog::LF_HAA; joint <= dog::RH_KFE; joint++){
	        kp[joint] = zeroGainTh[joint];
	        kd[joint] = zeroGainThd[joint];
	        ki[joint] = zeroGainInt[joint];
	    }
		setPID_Flag = false;
        cout << endl << "Zero Gain Th = ";
		cout << zeroGainTh[0] << " " << zeroGainTh[1] << " " << zeroGainTh[2] << endl;
        cout << "kp = ";
		cout << kp[0] << " " << kp[1] << " " << kp[2] << endl;
		cout << "PID Gains changed!!!" << endl << endl;
	}

	//readDataFromRobot();
	if(robotName == "hyq" || robotName == "centaur1arm"){
		jacobiansHyQ->fr_trunk_J_LF_foot(q_);
		jacobiansHyQ->fr_trunk_J_RF_foot(q_);
		jacobiansHyQ->fr_trunk_J_LH_foot(q_);
		jacobiansHyQ->fr_trunk_J_RH_foot(q_);
	}else{
		jacobiansHyQ2Max->fr_trunk_J_LF_foot(q_);
		jacobiansHyQ2Max->fr_trunk_J_RF_foot(q_);
		jacobiansHyQ2Max->fr_trunk_J_LH_foot(q_);
		jacobiansHyQ2Max->fr_trunk_J_RH_foot(q_);
	}


    static bool oooHolder = false;
    static double oooHolderTime = 0.0;


    //Simple assumption with 50 N threshold to get legs state (stance/swing)
	for (int leg = dog::LF; leg <= dog::RH; leg++) {
		if(footSensor->force[leg][rbd::Z] > 50 && WCPGb[leg](rbd::Z) < 0.5*stepHeight) { //to avoid false stance triggering
		//if(footSensor->force[leg][rbd::Z] > 50) {
            stanceLegs[leg]=true;
            forcedStanceLegs[leg]=true;
		} else {
			stanceLegs[leg]=false;
			forcedStanceLegs[leg]=false;
		}

		if(WCPGb[leg](rbd::Z) < -0.01){
		    forcedStanceLegs[leg]=true;
		}

		//Forced swing phase state for the stanceLegs status vector
		if(!oooHolder && (WCPGb[leg](rbd::Z) > 0.2*stepHeight)){
		    forcedStanceLegs[leg]=false;
		}
	}



	//Force and keep all legs in stance phase when it is noticed that the
	// "ooo" command was called.
	if(oooFlag && oooHolder){
		for (int leg = dog::LF; leg <= dog::RH; leg++) {
            stanceLegs[leg] = true;

            //To avoid uff switching between legs when the robot must be
            //standing still.
            forcedStanceLegs[leg]=stanceLegs[leg];
        }
	}
	else {
	    oooHolder = false;
	}

	//To avoid uff switching between legs when the robot must be
	//standing still.
    if(oooFlag){
        for (int leg = dog::LF; leg <= dog::RH; leg++) {
            forcedStanceLegs[leg]=stanceLegs[leg];
        }
    }

	if(!oooHolder && oooFlag &&
	   stanceLegs[dog::LF] == true && stanceLegs[dog::RF] == true &&
	   stanceLegs[dog::LH] == true && stanceLegs[dog::RH] == true){

	    oooHolderTime += 1 / (double)taskServoRate;

        if(oooHolderTime >= 1.0) {
            oooHolder = true;
            cout << endl << "oooHolder set to true!!!" << endl;
            oooHolderTime = 0.0;
        }
	}

	if(useStanceHolderFlag){
	    if(dutyF >= 0.5){
	        //Force stance phase of RH if LF is in stance according to foot sensor and CPG
	        if(stanceLegs[dog::LF] && WCPGb[dog::LF](rbd::Z) < CPG[dog::LF].z_td && !stanceLegs[dog::RH])
	            stanceLegs[dog::RH] = true;

	        //Force stance phase of LF if RH is in stance according to foot sensor and CPG
	        if(stanceLegs[dog::RH] && WCPGb[dog::RH](rbd::Z) < CPG[dog::RH].z_td && !stanceLegs[dog::LF])
	            stanceLegs[dog::LF] = true;

	        //Force stance phase of LH if RF is in stance according to foot sensor and CPG
	        if(stanceLegs[dog::RF] && WCPGb[dog::RF](rbd::Z) < CPG[dog::RF].z_td &&
	                !stanceLegs[dog::LH])
	            stanceLegs[dog::LH] = true;

	        //Force stance phase of RF if LH is in stance according to foot sensor and CPG
	        if(stanceLegs[dog::LH] && WCPGb[dog::LH](rbd::Z) < CPG[dog::LH].z_td &&
	                !stanceLegs[dog::RF])
	            stanceLegs[dog::RF] = true;
	    }
	}



	//Stance legs counter
	stanceLegsNumber = 0;
	for (int leg = dog::LF; leg <= dog::RH; leg++){
	    if(stanceLegs[leg])
	        stanceLegsNumber++;
	}



	//Forward Kinematics Calculations
/*	P_Local[dog::LF] = fwdKin -> getLFFootPos(q);
	P_Local[dog::RF] = fwdKin -> getRFFootPos(q);
	P_Local[dog::LH] = fwdKin -> getLHFootPos(q);
	P_Local[dog::RH] = fwdKin -> getRHFootPos(q); //NEED TO BE CONVERTED
*/
	//iit::HyQ2Max::ParamsGetter_lengths pm; //What is going on here?
	if(robotName == "hyq" || robotName == "centaur1arm"){
	    iit::HyQ::HomogeneousTransforms ht(default_pg2);
	    P_Local_BF[dog::LF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LF_foot(q_));
	    //cout << "iit::rbd::Utils::positionVector(ht.fr_trunk_X_LF_foot(q_): " << (iit::rbd::Utils::positionVector(ht.fr_trunk_X_LF_foot(q_))).transpose() << endl;
	    P_Local_BF[dog::RF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RF_foot(q_));
	    //cout << "iit::rbd::Utils::positionVector(ht.fr_trunk_X_RF_foot(q_)): " << (iit::rbd::Utils::positionVector(ht.fr_trunk_X_RF_foot(q_))).transpose() << endl;
	    P_Local_BF[dog::LH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LH_foot(q_));
	    //cout << "iit::rbd::Utils::positionVector(ht.fr_trunk_X_LH_foot(q_)): " << (iit::rbd::Utils::positionVector(ht.fr_trunk_X_LH_foot(q_))).transpose() << endl;
	    P_Local_BF[dog::RH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RH_foot(q_));
	    //cout << "iit::rbd::Utils::positionVector(ht.fr_trunk_X_RH_foot(q_)): " << (iit::rbd::Utils::positionVector(ht.fr_trunk_X_RH_foot(q_))).transpose() << endl << endl;
	} else{
	    iit::HyQ2Max::HomogeneousTransforms ht(default_pg);
	    P_Local_BF[dog::LF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LF_foot(q_));
	    P_Local_BF[dog::RF] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RF_foot(q_));
	    P_Local_BF[dog::LH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_LH_foot(q_));
	    P_Local_BF[dog::RH] =  iit::rbd::Utils::positionVector(ht.fr_trunk_X_RH_foot(q_));
	}


    //Forward Kinematics: calculates the actual end-effector velocity in the base frame
    Pd_Local_BF[dog::LF] = (*JFootLF).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(0, 0);
    Pd_Local_BF[dog::RF] = (*JFootRF).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(3, 0);
    Pd_Local_BF[dog::LH] = (*JFootLH).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(6, 0);
    Pd_Local_BF[dog::RH] = (*JFootRH).block<3,3>(rbd::LX,0) * qd_.block<3, 1>(9, 0);

    //Forward Kinematics: calculates the actual end-effector acceleration in the base frame
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        Pdd_Local_BF[leg] = (double)taskServoRate * (Pd_Local_BF[leg] - last_Pd_Local_BF[leg]);
        last_Pd_Local_BF[leg] = Pd_Local_BF[leg];
    }

	//Compute actual feet position in the horizontal frame
	for (int leg = dog::LF; leg <= dog::RH; leg++) {
	    P_Local_HF[leg] = baseToHF(P_Local_BF[leg]);
	}

	//Compute actual feet velocity in the horizontal frame
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        Pd_Local_HF[leg] = (double)taskServoRate * (P_Local_HF[leg] - last_P_Local_HF[leg]);
        last_P_Local_HF[leg] = P_Local_HF[leg];
    }

    //Compute actual feet acceleration in the horizontal frame
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        Pdd_Local_HF[leg] = (double)taskServoRate * (Pd_Local_HF[leg] - last_Pd_Local_HF[leg]);
        last_Pd_Local_HF[leg] = Pd_Local_HF[leg];
    }



    /*
     ____                                        _     _
    |  _ \    ___   _ __    ___    ___   _ __   | |_  (_)   ___    _ __
    | |_) |  / _ \ | '__|  / __|  / _ \ | '_ \  | __| | |  / _ \  | '_ \
    |  __/  |  __/ | |    | (__  |  __/ | |_) | | |_  | | | (_) | | | | |
    |_|      \___| |_|     \___|  \___| | .__/   \__| |_|  \___/  |_| |_|
                                        |_|

     _
    | |       __ _   _   _    ___   _ __
    | |      / _` | | | | |  / _ \ | '__|
    | |___  | (_| | | |_| | |  __/ | |
    |_____|  \__,_|  \__, |  \___| |_|
                      |___/
     */

    if(terrainEstimationFlag){
        //Update feet and body information
        TerrainEstimator.setBaseAngles(bs->getRoll_W(), bs->getPitch_W());

        //Temporary conversion
        Eigen::Matrix<double, 3,4> footPositions;
        footPositions << P_Local_BF[dog::LF](rbd::X), P_Local_BF[dog::RF](rbd::X), P_Local_BF[dog::LH](rbd::X), P_Local_BF[dog::RH](rbd::X),
                         P_Local_BF[dog::LF](rbd::Y), P_Local_BF[dog::RF](rbd::Y), P_Local_BF[dog::LH](rbd::Y), P_Local_BF[dog::RH](rbd::Y),
                         P_Local_BF[dog::LF](rbd::Z), P_Local_BF[dog::RF](rbd::Z), P_Local_BF[dog::LH](rbd::Z), P_Local_BF[dog::RH](rbd::Z);

        TerrainEstimator.setFootPositionsBF(footPositions);
        TerrainEstimator.setFootForcesBF(footSensor_[dog::LF](rbd::Z),
                                         footSensor_[dog::RF](rbd::Z),
                                         footSensor_[dog::LH](rbd::Z),
                                         footSensor_[dog::RH](rbd::Z));
    }

    //Compute terrain estimation
    TerrainEstimator.ComputeTerrainEstimation(estTerrainRoll, estTerrainPitch);


	/*
          ____    _             _
         / ___|  | |_    __ _  | |_    ___
         \___ \  | __|  / _` | | __|  / _ \
          ___) | | |_  | (_| | | |_  |  __/
         |____/   \__|  \__,_|  \__|  \___|

         _____         _     _                       _     _
        | ____|  ___  | |_  (_)  _ __ ___     __ _  | |_  (_)   ___    _ __
        |  _|   / __| | __| | | | '_ ` _ \   / _` | | __| | |  / _ \  | '_ \
        | |___  \__ \ | |_  | | | | | | | | | (_| | | |_  | | | (_) | | | | |
        |_____| |___/  \__| |_| |_| |_| |_|  \__,_|  \__| |_|  \___/  |_| |_|
	 */
      if(debug_print)  std::cout << "State Est " << std::endl;

	//Trunk State Estimation
	estBodyVelHF.setZero();
	stateEstimation.setLegsJacobian(*JFootLF, *JFootRF, *JFootLH, *JFootRH);  //NEED TO BE CONVERTED

	Eigen::Matrix<double, 3,4> feetPosition;
	feetPosition << P_Local_BF[dog::LF](rbd::X),P_Local_BF[dog::RF](rbd::X),P_Local_BF[dog::LH](rbd::X),P_Local_BF[dog::RH](rbd::X),
					P_Local_BF[dog::LF](rbd::Y),P_Local_BF[dog::RF](rbd::Y),P_Local_BF[dog::LH](rbd::Y),P_Local_BF[dog::RH](rbd::Y),
					P_Local_BF[dog::LF](rbd::Z),P_Local_BF[dog::RF](rbd::Z),P_Local_BF[dog::LH](rbd::Z),P_Local_BF[dog::RH](rbd::Z);
	stateEstimation.setFeetPosition(feetPosition);

	Eigen::Matrix<bool, 4,1> stanceLegsMatrix;
	stanceLegsMatrix << stanceLegs[dog::LF],stanceLegs[dog::RF],stanceLegs[dog::LH],stanceLegs[dog::RH];
	stateEstimation.setStanceFeet(stanceLegsMatrix);

    stateEstimation.setBodyAnglesPos(bs->getRoll_W(), bs->getPitch_W(), bs->getYaw_W());
    stateEstimation.setBodyAnglesVel(bs->getRotationRate_B()[Roll],
                                     bs->getRotationRate_B()[Pitch],
                                     bs->getRotationRate_B()[Yaw]);
    stateEstimation.setBodyAnglesAccel(bs->getRotAcceleration_B()[Roll],
                                       bs->getRotAcceleration_B()[Pitch],
                                       bs->getRotAcceleration_B()[Yaw]);
    stateEstimation.setBodyAccelFromIMU(bs->getAcceleration_B()[0],
            bs->getAcceleration_B()[1], bs->getAcceleration_B()[2]);

	stateEstimation.setJointTorques(tau_);
	stateEstimation.setJointVelocities(qd_);
	estBodyVelHF = stateEstimation.trunkLinearVelocitiesInTheHF();

	estBodyVelHF_Check.setZero();
	bodyVelocitiesEstimation(stanceLegs, estBodyVelHF_Check);

	defaultEstBodyVelHF = bs->getVelocity_H();

	if(useRCF_StateEstimator) {
	    bs->setVelocity_H(estBodyVelHF);
	}

    //Compute continuous robot yaw angle
    double yawInc = bs->getYaw_W()-lastYaw;
    if(sqrt(yawInc*yawInc) < 5*3.1415/180){robotYaw += bs->getYaw_W()-lastYaw;}

    lastYaw = bs->getYaw_W();

    // // OVERRIDE VICTORS VALUES!!!
    // estBodyVelHF = bs->getVelocity_H();
    // END OF VICTORS OVERRIDE!!!

	rbd::Vector3d auxAngles;
    auxAngles << bs->getRoll_W(), bs->getPitch_W(), bs->getYaw_W();

	//Compute body height
    computeBodyHeight(auxAngles, P_Local_BF, estBodyVelHF_Check, stanceLegs, trunkHeight);

	//Step frequency, stance period and CPG measurements
	stepCycleMeasurement(footSensor->force[dog::LF][2],
							double(taskServoRate),
								myTime,
								WCPG[dog::LF](rbd::Z),
								stanceLegs,
								flightPhase,
								flightPhaseTime,
								CPG_DutyFactor,
								actualDutyFactor,
								actualStepFrequency,
								actualStancePeriod);

	//Compute desired and actual robot position in the WF and in the HF
	computeDesiredAndActualRobotPosition();


	/*   ____                  _
        |  _ \   _   _   ___  | |__
        | |_) | | | | | / __| | '_ \
        |  __/  | |_| | \__ \ | | | |
        |_|      \__,_| |___/ |_| |_|

         ____
        |  _ \    ___    ___    ___   __   __   ___   _ __   _   _
        | |_) |  / _ \  / __|  / _ \  \ \ / /  / _ \ | '__| | | | |
        |  _ <  |  __/ | (__  | (_) |  \ V /  |  __/ | |    | |_| |
        |_| \_\  \___|  \___|  \___/    \_/    \___| |_|     \__, |
                                                             |___/
	 */

      if(debug_print)  std::cout << "Push Recovery " << std::endl;
	// Compute push recovery outputs
	PushRecovery.setStancePeriod(stepLength/sqrt(forwVel*forwVel));

	if (desiredTs_pRec > 0)
		PushRecovery.setStancePeriod(desiredTs_pRec);

	PushRecovery.setFeetHeight(sqrt(P0_Local_HF[dog::LF](rbd::Z)*P0_Local_HF[dog::LF](rbd::Z)));


	//cutoff_frequency in function of step position
	Eigen::Vector4d dx;
	Eigen::Vector4d cutOffFrequency;

	for (int leg = dog::LF; leg <= dog::RH; leg++) {
		dx(leg) = (stepLength/2 + WCPGb[leg](rbd::X))/stepLength;
		if (dx(leg) < 0) {
		    dx(leg) = 0.0;
		}
	}

	cutOffFrequency = minCutOffFreq * Eigen::Vector4d::Ones() + (maxCutOffFreq-minCutOffFreq) * dx;

	for (int leg = dog::LF; leg <= dog::RH; leg++) {
		PushRecovery.setLegOutputFilter(leg,cutOffFrequency(leg));
	}

    estBodyVelHF(rbd::X) = bs->getVelocity_H()(rbd::X);
    estBodyVelHF(rbd::Y) = bs->getVelocity_H()(rbd::Y);
    PushRecovery.ComputeDeltas(VfX, VfY, Psit, estBodyVelHF(rbd::X), estBodyVelHF(rbd::Y), bs->getRotationRate_B()[Yaw]);


    for (int leg = dog::LF; leg <= dog::RH; leg++){
        if(!stanceLegs[leg]){
            deltaPR_Pos[leg] = PushRecovery.deltaPos[leg];
            deltaPR_Vel[leg] = PushRecovery.deltaVel[leg];
        }
        else {
            deltaPR_Vel[leg].setZero();
        }
    }

    //PushRecovery.ComputeDeltas(VfX, VfY, Psit, misc_sensor[B_Xd], misc_sensor[B_Yd], bs->getRotationRate_B(,[Yaw] deltaX, deltaY);

	/*
         _____                    _                 _
        |_   _|  _ __    __ _    (_)   ___    ___  | |_    ___    _ __   _   _
          | |   | '__|  / _` |   | |  / _ \  / __| | __|  / _ \  | '__| | | | |
          | |   | |    | (_| |   | | |  __/ | (__  | |_  | (_) | | |    | |_| |
          |_|   |_|     \__,_|  _/ |  \___|  \___|  \__|  \___/  |_|     \__, |
                               |__/                                      |___/

          ____                                        _     _
         / ___|   ___   _ __     ___   _ __    __ _  | |_  (_)   ___    _ __
        | |  _   / _ \ | '_ \   / _ \ | '__|  / _` | | __| | |  / _ \  | '_ \
        | |_| | |  __/ | | | | |  __/ | |    | (_| | | |_  | | | (_) | | | | |
         \____|  \___| |_| |_|  \___| |_|     \__,_|  \__| |_|  \___/  |_| |_|
	 */

    if(debug_print)    std::cout << "Traj Gen " << std::endl;

	//CPG outputs
	//compute_WCPG_integration();
	if(!oooHolder) {
		CPG_Modulation(WCPGb, dWCPGb, WCPG, dWCPG);
	}



	terrainBasedAdjustment.setTerrainAngles(estTerrainRoll, estTerrainPitch);
	terrainBasedAdjustment.setPrimitivesPos(WCPG);
	terrainBasedAdjustment.setPrimitivesVel(dWCPG);
	terrainBasedAdjustment.setTrajectoryOriginsInTheHF(P0_Local_HF);
	rbd::Vector3d auxTrunkVel;
	auxTrunkVel << VfX, VfY, 0.0;
	terrainBasedAdjustment.setDesiredTrunkVelocityHF(auxTrunkVel);
	terrainBasedAdjustment.computeAdjustment();

	rollAdj = terrainBasedAdjustment.adjPostureAngle(rbd::X);
	pitchAdj = terrainBasedAdjustment.adjPostureAngle(rbd::Y);

	desRollAngle = eulerAngles(0) + rollAdj;
	desPitchAngle = eulerAngles(1) + pitchAdj;


	//Assign desired periodic foot trajectories to each end-effector in the horizontal frame
	for (int leg = dog::LF; leg <= dog::RH; leg++){
	    P_DesLocal_HF[leg] = terrainBasedAdjustment.adjustedPrimitivesPos[leg];
	    Pd_DesLocal_HF[leg] = terrainBasedAdjustment.adjustedPrimitivesVel[leg];
	}

	//Add desired end-effector zero positions in horizontal frame
	for (int leg = dog::LF; leg <= dog::RH; leg++){
	    P_DesLocal_HF[leg] += P0_Local_HF[leg] + terrainBasedAdjustment.deltaOrigins[leg];
	}

	//Add relative displacement from the push recovery block in the horizontal frame
	for (int leg = dog::LF; leg <= dog::RH; leg++){
	    P_DesLocal_HF[leg] += deltaPR_Pos[leg];
	    Pd_DesLocal_HF[leg] += deltaPR_Vel[leg];
	}

	//Compensate for feet positioning error during touch-down
	//It depends on the value of the gain "td_error_comp".
	for (int leg = dog::LF; leg <= dog::RH; leg++){
	    P_DesLocal_HF[leg](rbd::Z) -= tdErrorComp * CPG[leg].z_error_td;
	}

	// Add foot position displacement in case of push-ups
	double delta_push_up = sinusoidalMotion(sinOnOffFlag, sinAmplitude, sinFrequency, myTime);
	for (int leg = dog::LF; leg <= dog::RH; leg++) {
	    P_DesLocal_HF[leg](rbd::Z) += delta_push_up;

	}

	// Compute foot acceleration in the horizontal frame
	for (int leg = dog::LF; leg <= dog::RH; leg++){
	    Pdd_DesLocal_HF[leg] = (double)taskServoRate * (Pd_DesLocal_HF[leg] - last_Pd_DesLocal_HF[leg]);
	    last_Pd_DesLocal_HF[leg] = Pd_DesLocal_HF[leg];
	}

	//Add Reflex componet
	for (int leg = dog::LF; leg <= dog::RH; leg++) {
	    P_DesLocal_HF[leg] += P_Reflex_HF[leg];
        Pd_DesLocal_HF[leg] += Pd_Reflex_HF[leg];
	}

	//Workspace check for push recovery reactions
	if (P_DesLocal_HF[dog::LF](rbd::Y) <  0.03) {P_DesLocal_HF[dog::LF](rbd::Y) =  0.03;}
	if (P_DesLocal_HF[dog::RF](rbd::Y) > -0.03) {P_DesLocal_HF[dog::RF](rbd::Y) = -0.03;}
	if (P_DesLocal_HF[dog::LH](rbd::Y) <  0.03) {P_DesLocal_HF[dog::LH](rbd::Y) =  0.03;}
	if (P_DesLocal_HF[dog::RH](rbd::Y) > -0.03) {P_DesLocal_HF[dog::RH](rbd::Y) = -0.03;}

	//Workspace check for push recovery reactions
	if (P_DesLocal_HF[dog::LF](rbd::Z) > -0.10) {P_DesLocal_HF[dog::LF](rbd::Z) = -0.10;}
	if (P_DesLocal_HF[dog::RF](rbd::Z) > -0.10) {P_DesLocal_HF[dog::RF](rbd::Z) = -0.10;}
	if (P_DesLocal_HF[dog::LH](rbd::Z) > -0.10) {P_DesLocal_HF[dog::LH](rbd::Z) = -0.10;}
	if (P_DesLocal_HF[dog::RH](rbd::Z) > -0.10) {P_DesLocal_HF[dog::RH](rbd::Z) = -0.10;}






	/*   _  __  _                                      _     _
        | |/ / (_)  _ __     ___   _ __ ___     __ _  | |_  (_)   ___
        | ' /  | | | '_ \   / _ \ | '_ ` _ \   / _` | | __| | |  / __|
        | . \  | | | | | | |  __/ | | | | | | | (_| | | |_  | | | (__
        |_|\_\ |_| |_| |_|  \___| |_| |_| |_|  \__,_|  \__| |_|  \___|

            _          _     _                 _
           / \      __| |   (_)  _   _   ___  | |_   _ __ ___     ___   _ __   | |_
          / _ \    / _` |   | | | | | | / __| | __| | '_ ` _ \   / _ \ | '_ \  | __|
         / ___ \  | (_| |   | | | |_| | \__ \ | |_  | | | | | | |  __/ | | | | | |_
        /_/   \_\  \__,_|  _/ |  \__,_| |___/  \__| |_| |_| |_|  \___| |_| |_|  \__|
                          |__/
	 */

      if(debug_print)  std::cout << "Kin Adj " << std::endl;
    KinAdjustment.setRollLimit(rollMaxKadj);
    KinAdjustment.setPitchLimit(pitchMaxKadj);
    KinAdjustment.setKinAdjustment(kadjRoll, kadjPitch);
    KinAdjustment.setBaseAngles(bs->getRoll_W(),
                                bs->getPitch_W(),
                                bs->getRotationRate_B()[Roll],
                                bs->getRotationRate_B()[Pitch]);

	// if(KinAdjustment.KinematicAdjustmentFlag)
	// {
 //    	std::cout << "IMU VEL! " << imu->getAngRate()[Roll] << " " << imu->getAngRate()[Pitch] <<std::endl;
 //    	std::cout << "IMU POS! " << imu->getRoll() << " " << imu->getPitch() <<std::endl;
 //    	std::cout << "P_DesLocal_HF " << P_DesLocal_HF <<std::endl;
 //    	std::cout << "Pd_DesLocal_HF " << Pd_DesLocal_HF <<std::endl;
 //    	std::cout << "Pdd_DesLocal_HF " << Pdd_DesLocal_HF <<std::endl;
	// }

    KinAdjustment.setDesiredFootPosInHF(P_DesLocal_HF);
    KinAdjustment.setDesiredFootVelInHF(Pd_DesLocal_HF);
    KinAdjustment.setDesiredFootAccInHF(Pdd_DesLocal_HF);
    KinAdjustment.computeKinAdjustment();


    //Safety Check (Kinematic adjustment should not work without trunk controller ON).
    if (attitudeFlag) {

        P_DesLocal_BF = KinAdjustment.getKinAdjustmentPos();
        Pd_DesLocal_BF = KinAdjustment.getKinAdjustmentVel();
        Pdd_DesLocal_BF = KinAdjustment.getKinAdjustmentAcc();

    } else {
        for (int leg = dog::LF; leg <= dog::RH; leg++) {
            P_DesLocal_BF[leg] = P_DesLocal_HF[leg];
            //std::cout << "Pd_DesLocal_HF " << Pd_DesLocal_HF << std::endl;
            Pd_DesLocal_BF[leg] = Pd_DesLocal_HF[leg];
            Pdd_DesLocal_BF[leg] = Pdd_DesLocal_HF[leg];
        }

    }


	// if(KinAdjustment.KinematicAdjustmentFlag)
	// {

	// if(std::isinf(Pd_DesLocal_BF[0](0)) || std::isinf(Pd_DesLocal_BF[0](1)) || std::isinf(Pd_DesLocal_BF[0](2)) ||
	// std::isinf(Pd_DesLocal_BF[1](0)) || std::isinf(Pd_DesLocal_BF[1](1)) || std::isinf(Pd_DesLocal_BF[1](2)) ||
	// std::isinf(Pd_DesLocal_BF[2](0)) || std::isinf(Pd_DesLocal_BF[2](1)) || std::isinf(Pd_DesLocal_BF[2](2)) ||
	// std::isinf(Pd_DesLocal_BF[3](0)) || std::isinf(Pd_DesLocal_BF[3](1)) || std::isinf(Pd_DesLocal_BF[3](2)))
	// {
	// 	std::cout << ">>> P_DesLocal_BF " << P_DesLocal_BF <<std::endl;
	// 	std::cout << ">>> Pd_DesLocal_BF " << Pd_DesLocal_BF <<std::endl;
	// 	std::cout << ">>> Pdd_DesLocal_BF " << Pdd_DesLocal_BF <<std::endl;

	// 	std::cout << "ABORT! " << std::endl;
	// 	abort();
	// }
	// }



	/*
         ____             __          _
        / ___|    __ _   / _|   ___  | |_   _   _
        \___ \   / _` | | |_   / _ \ | __| | | | |
         ___) | | (_| | |  _| |  __/ | |_  | |_| |
        |____/   \__,_| |_|    \___|  \__|  \__, |

          ____   _                     _
         / ___| | |__     ___    ___  | | __
        | |     | '_ \   / _ \  / __| | |/ /
        | |___  | | | | |  __/ | (__  |   <
         \____| |_| |_|  \___|  \___| |_|\_\
	 */
       if(debug_print) std::cout << "Safety che " << std::endl;

    //***************************************************************************************
	//Self-righting check
	//More than 60 degrees for roll or pitch put the robot in fall detection mode
    if((bs->getRoll_W() > 1.05 || bs->getRoll_W() < -1.05 || bs->getPitch_W() > 1.05 || bs->getPitch_W() < -1.05) && !fallDetectionFlag) {
		std::cout << "Fall detection flag set" << std::endl;
        std::cout << "angPos = " << bs->getRoll_W() << " " << bs->getPitch_W() << " " << bs->getYaw_W() << std::endl;

		fallDetectionFlag = true;
		walkStopping();
		if (attitudeFlag){ toggleTrunkControl(); }
		if (kinAdjustmentFlag){ toggleKinematicAdjustmentFlag(); }
		if (pushRecoveryFlag){ togglePushRecoveryFlag(); }
		if (walkAdaptationFlag){ toggleWalkAdaptationFlag(); }
	}

	if(executeSelfRightingFlag) {
		fallDetectionFlag = true;
		selfRighting();
	}
	if(executeLayDownFlag) {
		layDown();
	}

	//End of self-righting check
	//***************************************************************************************


    //***************************************************************************************
    //Temporary workspace check functions

    static double maxLegLength = 0.75;
    double shortenFactor = 1;
    rbd::Vector3d hip_position;
    hip_position << rl.get_haa_x(), rl.get_haa_y(), rl.get_haa_z();

    double legLength = sqrt((P_DesLocal_BF[dog::LF](rbd::X)-hip_position(rbd::X))*(P_DesLocal_BF[dog::LF](rbd::X)-hip_position(rbd::X)) +
                      (P_DesLocal_BF[dog::LF](rbd::Y)-hip_position(rbd::Y))*(P_DesLocal_BF[dog::LF](rbd::Y)-hip_position(rbd::Y)) +
                      (P_DesLocal_BF[dog::LF](rbd::Z)-hip_position(rbd::Z))*(P_DesLocal_BF[dog::LF](rbd::Z)-hip_position(rbd::Z)));

    if(legLength > maxLegLength) {
        shortenFactor = legLength / maxLegLength;
        P_DesLocal_BF[dog::LF] = 1 / shortenFactor * P_DesLocal_BF[dog::LF];
    }


    hip_position << rl.get_haa_x(), -rl.get_haa_y(), rl.get_haa_z();

    legLength = sqrt((P_DesLocal_BF[dog::RF](rbd::X)-hip_position(rbd::X))*(P_DesLocal_BF[dog::RF](rbd::X)-hip_position(rbd::X)) +
                      (P_DesLocal_BF[dog::RF](rbd::Y)-hip_position(rbd::Y))*(P_DesLocal_BF[dog::RF](rbd::Y)-hip_position(rbd::Y)) +
                      (P_DesLocal_BF[dog::RF](rbd::Z)-hip_position(rbd::Z))*(P_DesLocal_BF[dog::RF](rbd::Z)-hip_position(rbd::Z)));

    if(legLength > maxLegLength) {
        shortenFactor = legLength / maxLegLength;
        P_DesLocal_BF[dog::RF] = 1 / shortenFactor * P_DesLocal_BF[dog::RF];
    }


    hip_position << -rl.get_haa_x(), rl.get_haa_y(), rl.get_haa_z();

    legLength = sqrt((P_DesLocal_BF[dog::LH](rbd::X)-hip_position(rbd::X))*(P_DesLocal_BF[dog::LH](rbd::X)-hip_position(rbd::X)) +
                      (P_DesLocal_BF[dog::LH](rbd::Y)-hip_position(rbd::Y))*(P_DesLocal_BF[dog::LH](rbd::Y)-hip_position(rbd::Y)) +
                      (P_DesLocal_BF[dog::LH](rbd::Z)-hip_position(rbd::Z))*(P_DesLocal_BF[dog::LH](rbd::Z)-hip_position(rbd::Z)));

    if(legLength > maxLegLength) {
        shortenFactor = legLength / maxLegLength;
        P_DesLocal_BF[dog::LH] = 1 / shortenFactor * P_DesLocal_BF[dog::LH];
    }


    hip_position << -rl.get_haa_x(), -rl.get_haa_y(), rl.get_haa_z();

    legLength = sqrt((P_DesLocal_BF[dog::RH](rbd::X)-hip_position(rbd::X))*(P_DesLocal_BF[dog::RH](rbd::X)-hip_position(rbd::X)) +
                      (P_DesLocal_BF[dog::RH](rbd::Y)-hip_position(rbd::Y))*(P_DesLocal_BF[dog::RH](rbd::Y)-hip_position(rbd::Y)) +
                      (P_DesLocal_BF[dog::RH](rbd::Z)-hip_position(rbd::Z))*(P_DesLocal_BF[dog::RH](rbd::Z)-hip_position(rbd::Z)));

    if(legLength > maxLegLength) {
        shortenFactor = legLength / maxLegLength;
        P_DesLocal_BF[dog::RH] = 1 / shortenFactor * P_DesLocal_BF[dog::RH];
    }

    //End of workspace check
    //***************************************************************************************




	/*
         ___
        |_ _|  _ __   __   __   ___   _ __   ___    ___
         | |  | '_ \  \ \ / /  / _ \ | '__| / __|  / _ \
         | |  | | | |  \ V /  |  __/ | |    \__ \ |  __/
        |___| |_| |_|   \_/    \___| |_|    |___/  \___|

         _  __  _                                      _     _
        | |/ / (_)  _ __     ___   _ __ ___     __ _  | |_  (_)   ___   ___
        | ' /  | | | '_ \   / _ \ | '_ ` _ \   / _` | | __| | |  / __| / __|
        | . \  | | | | | | |  __/ | | | | | | | (_| | | |_  | | | (__  \__ \
        |_|\_\ |_| |_| |_|  \___| |_| |_| |_|  \__,_|  \__| |_|  \___| |___/
	 */
        if(debug_print)       std::cout << "IK " << std::endl;

	if(!fallDetectionFlag){

		LegDataMap<bool> iK_Check;


		if(robotName == "hyq" || robotName == "centaur1arm"){
			iK_Check = hyqIK->calculate(P_DesLocal_BF, Pd_DesLocal_BF, Pdd_DesLocal_BF,
					jointDesPos, jointDesVel, jointDesAcc,q_);
		}else{
			iK_Check = hyq2maxIK->calculate(P_DesLocal_BF, Pd_DesLocal_BF, Pdd_DesLocal_BF,
					jointDesPos, jointDesVel, jointDesAcc,q_);
		}

        if(iK_Check[dog::LF]) {

            des_q_(dog::LF_HFE) = jointDesPos[dog::LF](legJointsID::HFE);
            des_q_(dog::LF_HAA) = jointDesPos[dog::LF](legJointsID::HAA);
            des_q_(dog::LF_KFE) = jointDesPos[dog::LF](legJointsID::KFE);
            des_qd_(dog::LF_HFE) = jointDesVel[dog::LF](legJointsID::HFE);
            des_qd_(dog::LF_HAA) = jointDesVel[dog::LF](legJointsID::HAA);
            des_qd_(dog::LF_KFE) = jointDesVel[dog::LF](legJointsID::KFE);
            des_qdd_(dog::LF_HFE) = jointDesAcc[dog::LF](legJointsID::HFE);
            des_qdd_(dog::LF_HAA) = jointDesAcc[dog::LF](legJointsID::HAA);
            des_qdd_(dog::LF_KFE) = jointDesAcc[dog::LF](legJointsID::KFE);
        }
        else {
            des_q_(dog::LF_HFE) = q_(dog::LF_HFE);
            des_q_(dog::LF_HAA) = q_(dog::LF_HAA);
            des_q_(dog::LF_KFE) = q_(dog::LF_KFE);
            des_qd_(dog::LF_HFE) = qd_(dog::LF_HFE);
            des_qd_(dog::LF_HAA) = qd_(dog::LF_HAA);
            des_qd_(dog::LF_KFE) = qd_(dog::LF_KFE);
            des_qdd_(dog::LF_HFE) = 0.0;
            des_qdd_(dog::LF_HAA) = 0.0;
            des_qdd_(dog::LF_KFE) = 0.0;

            referencesBackTracingPrintOuts(dog::LF);
        }

        if(iK_Check[dog::RF]){

            des_q_(dog::RF_HFE) = jointDesPos[dog::RF](legJointsID::HFE);
            des_q_(dog::RF_HAA) = jointDesPos[dog::RF](legJointsID::HAA);
            des_q_(dog::RF_KFE) = jointDesPos[dog::RF](legJointsID::KFE);
            des_qd_(dog::RF_HFE) = jointDesVel[dog::RF](legJointsID::HFE);
            des_qd_(dog::RF_HAA) = jointDesVel[dog::RF](legJointsID::HAA);
            des_qd_(dog::RF_KFE) = jointDesVel[dog::RF](legJointsID::KFE);
            des_qdd_(dog::RF_HFE) = jointDesAcc[dog::RF](legJointsID::HFE);
            des_qdd_(dog::RF_HAA) = jointDesAcc[dog::RF](legJointsID::HAA);
            des_qdd_(dog::RF_KFE) = jointDesAcc[dog::RF](legJointsID::KFE);
        }
        else {
            des_q_(dog::RF_HFE) =  q_(dog::RF_HFE);
            des_q_(dog::RF_HAA) =  q_(dog::RF_HAA);
            des_q_(dog::RF_KFE) =  q_(dog::RF_KFE);
            des_qd_(dog::RF_HFE) = qd_(dog::RF_HFE);
            des_qd_(dog::RF_HAA) = qd_(dog::RF_HAA);
            des_qd_(dog::RF_KFE) = qd_(dog::RF_KFE);
            des_qdd_(dog::RF_HFE) = 0.0;
            des_qdd_(dog::RF_HAA) = 0.0;
            des_qdd_(dog::RF_KFE) = 0.0;

            referencesBackTracingPrintOuts(dog::RF);
        }


        if(iK_Check[dog::LH]){

            des_q_(dog::LH_HFE) = jointDesPos[dog::LH](legJointsID::HFE);
            des_q_(dog::LH_HAA) = jointDesPos[dog::LH](legJointsID::HAA);
            des_q_(dog::LH_KFE) = jointDesPos[dog::LH](legJointsID::KFE);
            des_qd_(dog::LH_HFE) = jointDesVel[dog::LH](legJointsID::HFE);
            des_qd_(dog::LH_HAA) = jointDesVel[dog::LH](legJointsID::HAA);
            des_qd_(dog::LH_KFE) = jointDesVel[dog::LH](legJointsID::KFE);
            des_qdd_(dog::LH_HFE) = jointDesAcc[dog::LH](legJointsID::HFE);
            des_qdd_(dog::LH_HAA) = jointDesAcc[dog::LH](legJointsID::HAA);
            des_qdd_(dog::LH_KFE) = jointDesAcc[dog::LH](legJointsID::KFE);
        }
        else {
            des_q_(dog::LH_HFE) = q_(dog::LH_HFE);
            des_q_(dog::LH_HAA) = q_(dog::LH_HAA);
            des_q_(dog::LH_KFE) = q_(dog::LH_KFE);
            des_qd_(dog::LH_HFE) = qd_(dog::LH_HFE);
            des_qd_(dog::LH_HAA) = qd_(dog::LH_HAA);
            des_qd_(dog::LH_KFE) = qd_(dog::LH_KFE);
            des_qdd_(dog::LH_HFE) = 0.0;
            des_qdd_(dog::LH_HAA) = 0.0;
            des_qdd_(dog::LH_KFE) = 0.0;

            referencesBackTracingPrintOuts(dog::LH);
        }


        if(iK_Check[dog::RH]){

            des_q_(dog::RH_HFE) = jointDesPos[dog::RH](legJointsID::HFE);
            des_q_(dog::RH_HAA) = jointDesPos[dog::RH](legJointsID::HAA);
            des_q_(dog::RH_KFE) = jointDesPos[dog::RH](legJointsID::KFE);
            des_qd_(dog::RH_HFE) = jointDesVel[dog::RH](legJointsID::HFE);
            des_qd_(dog::RH_HAA) = jointDesVel[dog::RH](legJointsID::HAA);
            des_qd_(dog::RH_KFE) = jointDesVel[dog::RH](legJointsID::KFE);
            des_qdd_(dog::RH_HFE) = jointDesAcc[dog::RH](legJointsID::HFE);
            des_qdd_(dog::RH_HAA) = jointDesAcc[dog::RH](legJointsID::HAA);
            des_qdd_(dog::RH_KFE) = jointDesAcc[dog::RH](legJointsID::KFE);
        }
        else {
            des_q_(dog::RH_HFE) = q_(dog::RH_HFE);
            des_q_(dog::RH_HAA) = q_(dog::RH_HAA);
            des_q_(dog::RH_KFE) = q_(dog::RH_KFE);
            des_qd_(dog::RH_HFE) = qd_(dog::RH_HFE);
            des_qd_(dog::RH_HAA) = qd_(dog::RH_HAA);
            des_qd_(dog::RH_KFE) = qd_(dog::RH_KFE);
            des_qdd_(dog::RH_HFE) = 0.0;
            des_qdd_(dog::RH_HAA) = 0.0;
            des_qdd_(dog::RH_KFE) = 0.0;

            referencesBackTracingPrintOuts(dog::RH);
        }


		//Robogen to SL convertion
/*		commons::RobogenToSL<HyQ2Max::Traits>::pos(des_q, joint_des_state);
		commons::RobogenToSL<HyQ2Max::Traits>::vel(des_qd, joint_des_state);
		commons::RobogenToSL<HyQ2Max::Traits>::acc(des_qdd, joint_des_state);*/ //NEED TO BE CONVERTED

	}

	/*
         _____                          _
        |_   _|  _ __   _   _   _ __   | | __
          | |   | '__| | | | | | '_ \  | |/ /
          | |   | |    | |_| | | | | | |   <
          |_|   |_|     \__,_| |_| |_| |_|\_\

          ____                   _                    _
         / ___|   ___    _ __   | |_   _ __    ___   | |
        | |      / _ \  | '_ \  | __| | '__|  / _ \  | |
        | |___  | (_) | | | | | | |_  | |    | (_) | | |
         \____|  \___/  |_| |_|  \__| |_|     \___/  |_|
	 */
       if(debug_print) std::cout << "tunk cont" << std::endl;
	//Reset feedforward action
/*	for (int joint = 1; joint <= nDOF; joint++) {
		joint_des_state[joint].uff=0.0;
	}*/ //NEED TO BE CONVERTED

	//Compute Jump Force Profile
	if(jumpFlag)
		jump();

/*	if(computeDeltaJumpHeight) {
        if(bs->pos[2] < heightMemory) {
            cout << "Max Height: " << (bs->pos[2] + 1) << endl;
            cout << "Delta Height: " << (bs->pos[2] + 1) - liftoffHeight << endl;
			computeDeltaJumpHeight = false;
		}
        heightMemory = bs->pos[2];
	}else{
        heightMemory = bs->pos[2];
	}*/


	//Compute Trunk Control
	if (attitudeFlag)
	{
	    uffTorques.setZero();

	    currentEst = Eigen::Affine3d::Identity();
	    currentEst.translate(bs->getPosition_W());
	    currentEst.rotate(bs->getOrientation_W());
	    currentEst = initialEst.inverse()*currentEst;

	    bs->setPosition_W(currentEst.translation());
	    bs->setOrientation_W(Eigen::Quaterniond(currentEst.rotation()));

	    computeDesTrunkWrench();


	    for(int leg = dog::LF; leg <= dog::RH; leg++) {
	        P_Local_BF[leg] -= deltaCOG;
	    }

	    trunkController.setTrunkControllerOption(ctcOption);
	    if(forcedStanceFlag){
	        trunkController.setStanceLegs(forcedStanceLegs);
	    } else {
	        trunkController.setStanceLegs(stanceLegs);
	    }
	    trunkController.setActualFeetPos(P_Local_BF);
	    trunkController.setFeetJacobians(*JFootLF, *JFootRF, *JFootLH, *JFootRH);
	    trunkController.setMinStanceLegs(2);
	    trunkController.setTrunkAttitude(bs->getRoll_W(), bs->getPitch_W());
	    trunkController.setTrunkWrench(desTrunkWrench);
	    trunkController.computeOutputTorques();

	    uffTorques = trunkController.jointTorques;

	    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	    //%%%%%%%%%%%%%%%%   Constrained Inverse Dynamics   %%%%%%%%%%%%%%%%%%%
	    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	    if (constrainedInvDynFlag) {

	        //Reset general uff torques vector
	        //uffTorques.setZero();

	        //Reset method output torques
	        Eigen::VectorXd invDynTorques = Eigen::VectorXd::Zero(12);

	        // Converting RobCoGen to WholeBodyState
	        desired_ws->base_vel.setZero();
	        desired_ws->base_vel(dwl::rbd::LX) = VfX;
	        desired_ws->base_vel(dwl::rbd::LY) = VfY;
	        desired_ws->base_vel(dwl::rbd::AZ) = Psit;

	        //Set base position
	        dwl::rbd::Vector6d base_pos_hf = actual_ws->base_pos;
	        base_pos_hf(dwl::rbd::AX) = bs->getRoll_W();
	        base_pos_hf(dwl::rbd::AY) = bs->getPitch_W();
	        base_pos_hf(dwl::rbd::AZ) = 0.0;


	        if(invDynOnlyForGravCompFlag){

	            desired_ws->base_acc.setZero();
	        }
	        else {

	            uffTorques.setZero();

	            Eigen::MatrixXd inertial_mat;
	            wbd_.computeJointSpaceInertialMatrix(inertial_mat, base_pos_hf, actual_ws->joint_pos);

	            //cout << inertial_mat.block<6,6>(0,0) << endl << endl;
	            //From commom.xacro
	            //<property name="ixx_trunk" value="1.61053"/>
	            //<property name="iyy_trunk" value="8.53505"/>
	            //<property name="izz_trunk" value="9.19102"/>


	            dwl::rbd::Vector6d desBaseAcc;
	            dwl::rbd::Vector6d auxDesTrunkWrench;

	            auxDesTrunkWrench << desTrunkWrench(3), desTrunkWrench(4), desTrunkWrench(5),
	                    desTrunkWrench(0), desTrunkWrench(1), desTrunkWrench(2);

	            /*          desBaseAcc = (inertial_mat.block<6,6>(0,0)).inverse() * auxDesTrunkWrench;
	                        desired_ws->base_acc = desBaseAcc;

	                        cout << inertial_mat(rbd::AX,rbd::AX) << ", " << inertial_mat(rbd::AY,rbd::AY) << ", "
	                                << inertial_mat(rbd::AZ,rbd::AZ) << ", " << inertial_mat(rbd::LX,rbd::LX) << ", "
	                                << inertial_mat(rbd::LY,rbd::LY) << ", " << inertial_mat(rbd::LZ,rbd::LZ) << endl << endl;
	             */

	            //Considering values from the main matrix diagonal
	            desired_ws->base_acc(dwl::rbd::AX) = auxDesTrunkWrench(rbd::AX)/inertial_mat(rbd::AX,rbd::AX);
	            desired_ws->base_acc(dwl::rbd::AY) = auxDesTrunkWrench(rbd::AY)/inertial_mat(rbd::AY,rbd::AY);
	            desired_ws->base_acc(dwl::rbd::AZ) = auxDesTrunkWrench(rbd::AZ)/inertial_mat(rbd::AZ,rbd::AZ);
	            desired_ws->base_acc(dwl::rbd::LX) = auxDesTrunkWrench(rbd::LX)/inertial_mat(rbd::LX,rbd::LX);
	            desired_ws->base_acc(dwl::rbd::LY) = auxDesTrunkWrench(rbd::LY)/inertial_mat(rbd::LY,rbd::LY);
	            desired_ws->base_acc(dwl::rbd::LZ) = auxDesTrunkWrench(rbd::LZ)/inertial_mat(rbd::LZ,rbd::LZ);


	            /*
	                        //Considering fixed values from the main matrix diagonal
	                        desired_ws->base_acc(dwl::rbd::AX) = desTrunkWrench(rbd::LX)/4.0;
	                        desired_ws->base_acc(dwl::rbd::AY) = desTrunkWrench(rbd::LY)/11.0;
	                        desired_ws->base_acc(dwl::rbd::AZ) = desTrunkWrench(rbd::LZ)/12.0;
	                        desired_ws->base_acc(dwl::rbd::LX) = desTrunkWrench(rbd::AX)/97.55;
	                        desired_ws->base_acc(dwl::rbd::LY) = desTrunkWrench(rbd::AY)/97.55;
	                        desired_ws->base_acc(dwl::rbd::LZ) = desTrunkWrench(rbd::AZ)/97.55;

	                        desired_ws->base_acc(dwl::rbd::AX) = desTrunkWrench(rbd::LX)/inertial_mat(rbd::AX,rbd::AX);
	                        desired_ws->base_acc(dwl::rbd::AY) = desTrunkWrench(rbd::LY)/inertial_mat(rbd::AY,rbd::AY);
	                        desired_ws->base_acc(dwl::rbd::AZ) = desTrunkWrench(rbd::LZ)/inertial_mat(rbd::AZ,rbd::AZ);
	                        desired_ws->base_acc(dwl::rbd::LX) = desTrunkWrench(rbd::AX)/inertial_mat(rbd::LX,rbd::LX);
	                        desired_ws->base_acc(dwl::rbd::LY) = desTrunkWrench(rbd::AY)/inertial_mat(rbd::LY,rbd::LY);
	                        desired_ws->base_acc(dwl::rbd::LZ) = desTrunkWrench(rbd::AZ)/inertial_mat(rbd::LZ,rbd::LZ);
	             */

	            //if(desired_ws->base_acc(dwl::rbd::LZ) < -9.8) {
	            //    std::cout << desired_ws->base_acc(dwl::rbd::LZ) << std::endl;
	            //    desired_ws->base_acc(dwl::rbd::LZ) = -9.8;
	            //}

	        }


	        for (unsigned int i = 0; i < fbs_.getJointDoF(); i++) {
	            unsigned int joint_id = joint_id_map_[i];
	            // Converting the desired states to desired whole-body ones
	            desired_ws->joint_pos(joint_id) = des_q_(i);
	            desired_ws->joint_vel(joint_id) = des_qd_(i);
	            desired_ws->joint_acc(joint_id) = 0.0*des_qdd_(i);
	        }

	        //Get vector of leg stance status
	        dwl::rbd::BodySelector stance;

	        //Selecting leg stance status between actual and forced status
	        if(forcedStanceFlag){
	            for (unsigned int i = 0; i < leg_name_map_.size(); i++) {
	                if (forcedStanceLegs[i] == true) {
	                    stance.push_back((std::string) leg_name_map_[i]);
	                }
	            }
	        } else {
	            for (unsigned int i = 0; i < leg_name_map_.size(); i++) {
	                if (stanceLegs[i] == true) {
	                    stance.push_back((std::string) leg_name_map_[i]);
	                }
	            }
	        }



	        //Criteria of at least 2 legs in stance phase
	        if(stance.size() >= 2) {
	            wbd_.computeConstrainedFloatingBaseInverseDynamics(invDynTorques,
	                    base_pos_hf, actual_ws->joint_pos,
	                    actual_ws->base_vel, actual_ws->joint_vel,
	                    desired_ws->base_acc, desired_ws->joint_acc,
	                    stance);
	        }


	        // Converting ID-based torques from URDF to RobCoGen
	        // This sum is not needed if the desired_base_acc is
	        // computed according to the stabilizing trunk wrench
	        for (unsigned int i = 0; i < fbs_.getJointDoF(); i++) {
	            unsigned int joint_id = joint_id_map_[i];

	            uffTorques(i) += invDynTorques(joint_id);
	        }



	    }


	}else{

	    uffTorques.setZero();
	}




	/*   __  __   _                       _   _
        |  \/  | (_)  ___    ___    ___  | | | |   __ _   _ __     ___    ___    _   _   ___
        | |\/| | | | / __|  / __|  / _ \ | | | |  / _` | | '_ \   / _ \  / _ \  | | | | / __|
        | |  | | | | \__ \ | (__  |  __/ | | | | | (_| | | | | | |  __/ | (_) | | |_| | \__ \
        |_|  |_| |_| |___/  \___|  \___| |_| |_|  \__,_| |_| |_|  \___|  \___/   \__,_| |___/
	 */
      if(debug_print)  std::cout << "misc " << std::endl;
	//Compute Perturbation
	if (perturbationDuration > 0.0){
		//applyPerturbation(); //NEED TO BE CONVERTED BUT MOST LIKELY THIS CANT EXIST
		perturbationDuration -= 1/(double)taskServoRate;
	}
	else{
		lateralForce=0.0;
		yawTorque=0.0;
	}

	iit::HyQ::ForceTransforms forceTransforms2(default_pg2);
	iit::HyQ::dyn::InertiaProperties linksInertia2(default_pg2);
	iit::HyQ::dyn::JSIM hyq_jsim(linksInertia2, forceTransforms2);

	//Leg impedance changes
	PIDManager.setCPGPosReferences(WCPGb); //Updating PID manager with CPG info
	PIDManager.setCPGVelReferences(dWCPGb); //Updating PID manager with CPG info
	PIDManager.setJointStates(q_, qd_);
	PIDManager.setJointDesiredStates(des_q_, des_qd_, des_qdd_);
	PIDManager.setLegStatus(stanceLegs);
	PIDManager.setInertiaCompensationGain(inertialCompGain);

	if(robotName == "hyq" || robotName == "centaur1arm")
	{
		hyq_jsim.update(q_);
		PIDManager.setInertiaMatrix( hyq_jsim.getFixedBaseBlock() ); //HyQ
	}else{
		jsim->update(q_);
		PIDManager.setInertiaMatrix( jsim->getFixedBaseBlock() ); //HyQ2Max

	}
	PIDManager.computeCompensatingTorques();


    //Create lateral internal forces to evaluate the robot mechanical flexibility
    double internalForce = 0.0;

    internalForce = sinusoidalForce(sinOnOffForceFlag, sinForceAmplitude, sinForceFrequency, myTime);

    if(chirpOnOffFlag){

        internalForce = chirpForce( (myTime-chirpTime0) );

        if((myTime-chirpTime0) > chirpDuration) { chirpOnOffFlag = false;}
    }

    if(squareWaveOnOffFlag){

        internalForce = squareForce( (myTime-squareTime0) );
    }

    if(triangularWaveOnOffFlag){

        internalForce = triangularForce( (myTime-triangularTime0) );
    }

    if(sineWaveOnOffFlag){

        internalForce = sineForce( (myTime-sineTime0) );
    }



    //Foot frontal collision detection
    if(useCollisionDetectionFlag)
        evaluateCollisionDetection();

    //Foot frontal collision detection
    if(useStepReactionFlag){
        computeReflex(P_Reflex_HF, Pd_Reflex_HF);
    } else {
        for (int leg = dog::LF; leg <= dog::RH; leg++) {
            P_Reflex_HF[leg].setZero();
            Pd_Reflex_HF[leg].setZero();
        }
    }

    //Joystick commands
    if(useJoystickFlag)
    	checkJoystickCommands();



    iit::dog::LegDataMap<rbd::Vector3d> intForces;
    intForces[dog::LF].setZero();
    intForces[dog::RF].setZero();
    intForces[dog::LH].setZero();
    intForces[dog::RH].setZero();

    intForces[dog::LF](rbd::Y) = -internalForce * internalForceGain[dog::LF];
    intForces[dog::RF](rbd::Y) = internalForce * internalForceGain[dog::RF];
    intForces[dog::LH](rbd::Y) = -internalForce * internalForceGain[dog::LH];
    intForces[dog::RH](rbd::Y) = internalForce * internalForceGain[dog::RH];

    iit::dog::JointState extraJointTorques = computeTorquesForInternalForces(intForces);





     if(debug_print)   std::cout << "output " << std::endl;
	/*  ___        _               _     _____
	   / _ \ _   _| |_ _ __  _   _| |_  |_   _|__  _ __ __ _ _   _  ___  ___
	  | | | | | | | __| '_ \| | | | __|   | |/ _ \| '__/ _` | | | |/ _ \/ __|
	  | |_| | |_| | |_| |_) | |_| | |_    | | (_) | | | (_| | |_| |  __/\__ \
	   \___/ \__,_|\__| .__/ \__,_|\__|   |_|\___/|_|  \__, |\__,_|\___||___/
	                  |_|                                 |_|
	*/

	tauInertiaComp.setZero();


	if(!fallDetectionFlag){

		des_tau_[iit::dog::LF_HFE]  = uffTorques(dog::LF_HFE) + PIDManager.outputTorques(PIDManager.LF_HFE) +
				0.0*tauInertiaComp(dog::LF_HFE);
		des_tau_[iit::dog::LF_HAA]  = uffTorques(dog::LF_HAA) + PIDManager.outputTorques(PIDManager.LF_HAA) +
		        0.0*tauInertiaComp(dog::LF_HAA);
		des_tau_[iit::dog::LF_KFE]  = uffTorques(dog::LF_KFE) + PIDManager.outputTorques(PIDManager.LF_KFE) +
		        0.0*tauInertiaComp(dog::LF_KFE);

		des_tau_[iit::dog::RF_HFE]  = uffTorques(dog::RF_HFE) + PIDManager.outputTorques(PIDManager.RF_HFE) +
		        0.0*tauInertiaComp(dog::RF_HFE);
		des_tau_[iit::dog::RF_HAA]  = uffTorques(dog::RF_HAA) + PIDManager.outputTorques(PIDManager.RF_HAA) +
		        0.0*tauInertiaComp(dog::RF_HAA);
		des_tau_[iit::dog::RF_KFE]  = uffTorques(dog::RF_KFE) + PIDManager.outputTorques(PIDManager.RF_KFE) +
		        0.0*tauInertiaComp(dog::RF_KFE);

		des_tau_[iit::dog::LH_HFE]  = uffTorques(dog::LH_HFE) + PIDManager.outputTorques(PIDManager.LH_HFE) +
		        0.0*tauInertiaComp(dog::LH_HFE);
		des_tau_[iit::dog::LH_HAA]  = uffTorques(dog::LH_HAA) + PIDManager.outputTorques(PIDManager.LH_HAA) +
		        0.0*tauInertiaComp(dog::LH_HAA);
		des_tau_[iit::dog::LH_KFE]  = uffTorques(dog::LH_KFE) + PIDManager.outputTorques(PIDManager.LH_KFE) +
		        0.0*tauInertiaComp(dog::LH_KFE);

		des_tau_[iit::dog::RH_HFE]  = uffTorques(dog::RH_HFE) + PIDManager.outputTorques(PIDManager.RH_HFE) +
		        0.0*tauInertiaComp(dog::RH_HFE);
		des_tau_[iit::dog::RH_HAA]  = uffTorques(dog::RH_HAA) + PIDManager.outputTorques(PIDManager.RH_HAA) +
		        0.0*tauInertiaComp(dog::RH_HAA);
		des_tau_[iit::dog::RH_KFE]  = uffTorques(dog::RH_KFE) + PIDManager.outputTorques(PIDManager.RH_KFE) +
		        0.0*tauInertiaComp(dog::RH_KFE);

		des_tau_ += extraJointTorques;
	}
	else
	{
		for(unsigned int i = 0; i < des_tau_.size();i++)
		{
			des_tau_[i] = 0.0;
		}
	}


	//Copy state back from Rocogen variables;
	for(int i = 0; i != des_tau_.size(); i++) {
		des_tau[i] = des_tau_[i];
		des_q[i] = des_q_[i];
		des_qd[i] = des_qd_[i];
	}

	//Spline the initial desired state of the robot with THIS controllers desired state(avoid spikes)
	if(orig_des_spline < 1000){
		orig_des_spline++;
		if(orig_des_spline == 1000) std::cout << "Initialized " << orig_des_spline << std::endl;
	}
	//Copy state back from Rocogen variables;
	for(int i = 0; i != des_tau_.size(); i++) {
		des_tau[i] = orig_des_spline*0.001*des_tau_[i] + des_tau_ori[i]*(1 - orig_des_spline*0.001);
		des_q[i] = orig_des_spline*0.001*des_q_[i] + des_q_ori[i]*(1 - orig_des_spline*0.001);
		des_qd[i] = orig_des_spline*0.001*des_qd_[i] + des_qd_ori[i]*(1 - orig_des_spline*0.001);
	}

	//If centaur, bypass arm commands with external input
	if(robotName == "centaur1arm")
	{
		for(int i = 12; i < des_tau.size(); i++ )
		{
			des_tau[i] = 0.0;
			des_q[i] = input_reference_q[i];
			des_qd[i] = input_reference_qd[i];
		}
	}


	myTime += 1/(double)taskServoRate;
	// myTime += 1;


	//Write interesting variables to logger:
	debugClear();

	//Trunk states
	debugPush("estimatedTrunkVelHFx",(double)estBodyVelHF(rbd::X));
	debugPush("estimatedTrunkVelHFy",(double)estBodyVelHF(rbd::Y));
	debugPush("estimatedTrunkVelCheckHFx",(double)estBodyVelHF_Check(rbd::X));
	debugPush("estimatedTrunkVelCheckHFy",(double)estBodyVelHF_Check(rbd::Y));
	debugPush("defaultEstimatedTrunkVelHFx",(double)defaultEstBodyVelHF(rbd::X));
	debugPush("defaultEstimatedTrunkVelHFy",(double)defaultEstBodyVelHF(rbd::Y));


	//Actual leg stance status
	debugPush("stanceStatusLF",(double)stanceLegs[dog::LF]);
	debugPush("stanceStatusRF",(double)stanceLegs[dog::RF]);
	debugPush("stanceStatusLH",(double)stanceLegs[dog::LH]);
	debugPush("stanceStatusRH",(double)stanceLegs[dog::RH]);
	debugPush("stanceLegsNumber",(int)stanceLegsNumber);

	//Forced leg stance status
	debugPush("forcedStanceStatusLF",(double)forcedStanceLegs[dog::LF]);
	debugPush("forcedStanceStatusRF",(double)forcedStanceLegs[dog::RF]);
	debugPush("forcedStanceStatusLH",(double)forcedStanceLegs[dog::LH]);
	debugPush("forcedStanceStatusRH",(double)forcedStanceLegs[dog::RH]);

	debugPush("footSensorLFz",footSensor->force[dog::LF][2]);
	debugPush("footSensorRFz",footSensor->force[dog::RF][2]);
	debugPush("footSensorLHz",footSensor->force[dog::LH][2]);
	debugPush("footSensorRHz",footSensor->force[dog::RH][2]);

	//Desired inputs
	debugPush("VfX",(double)VfX);
	debugPush("VfY",(double)VfY);
	debugPush("forwVel",(double)forwVel);
	debugPush("stepHeight",(double)stepHeight);
	debugPush("stepLength",(double)stepLength);
	debugPush("stepFrequency",(double)stepFrequency);
	debugPush("dutyFactor",(double)dutyF);

	debugPush("desRollAngle",(double)desRollAngle);
	debugPush("desPitchAngle",(double)desPitchAngle);
	debugPush("desYawAngle",(double)desYawAngle);

	//Control flags
	debugPush("attitudeFlag",(int)attitudeFlag);
	debugPush("walkAdaptationFlag",(int)walkAdaptationFlag);
	debugPush("pushRecoveryFlag",(int)pushRecoveryFlag);
	debugPush("kinAdjustmentFlag",(int)kinAdjustmentFlag);
	debugPush("terrainEstimationFlag",(int)terrainEstimationFlag);
	debugPush("walkingTrotFlag",(int)walkingTrotFlag);
	debugPush("runningTrotFlag",(int)runningTrotFlag);
	debugPush("jumpFlag",(int)jumpFlag);
	debugPush("resetCPGFlag",(int)resetCPGFlag);
	debugPush("integralAction",(int)integralAction);
	debugPush("inProfileTransitionFlag",(int)inProfileTransitionFlag);
	debugPush("desStateFlag",(int)desStateFlag);
	//debugPush("freezeRatioFlag",(int)freezeRatioFlag);
	debugPush("oooFlag",(int)oooFlag);
	debugPush("sinOnOffFlag",(int)sinOnOffFlag);
	debugPush("fallDetectionFlag",(int)fallDetectionFlag);
	debugPush("executeSelfRightingFlag",(int)executeSelfRightingFlag);
	debugPush("executeLayDownFlag",(int)executeLayDownFlag);
	debugPush("inertialCompFlag",(int)inertialCompFlag);
	//debugPush("keyPressedFlag",(int)keyPressedFlag);
	//debugPush("exitFlag",(int)exitFlag);
	debugPush("constrainedInvDynFlag",(int)constrainedInvDynFlag);
	debugPush("invDynOnlyForGravCompFlag",(int)invDynOnlyForGravCompFlag);
	debugPush("CPG_StepHeightModulationFlag",(int)CPG_StepHeightModulationFlag);
	debugPush("ctcOption",(int)ctcOption);
	debugPush("forcedStanceFlag",(int)forcedStanceFlag);
	debugPush("useRCF_StateEstimator",(int)useRCF_StateEstimator);
	debugPush("frontalCollisionFlag",(int)frontalCollisionFlag);
	debugPush("useCollisionDetectionFlag",(int)useCollisionDetectionFlag);
    debugPush("useStepReactionFlag",(int)useCollisionDetectionFlag);

	//CPG variables for main oscillators
	debugPush("CPGb_LFx",(double)WCPGb[dog::LF](rbd::X));
	debugPush("CPGb_LFy",(double)WCPGb[dog::LF](rbd::Y));
	debugPush("CPGb_LFz",(double)WCPGb[dog::LF](rbd::Z));
	debugPush("CPGb_RFx",(double)WCPGb[dog::RF](rbd::X));
	debugPush("CPGb_RFy",(double)WCPGb[dog::RF](rbd::Y));
	debugPush("CPGb_RFz",(double)WCPGb[dog::RF](rbd::Z));
	debugPush("CPGb_LHx",(double)WCPGb[dog::LH](rbd::X));
	debugPush("CPGb_LHy",(double)WCPGb[dog::LH](rbd::Y));
	debugPush("CPGb_LHz",(double)WCPGb[dog::LH](rbd::Z));
	debugPush("CPGb_RHx",(double)WCPGb[dog::RH](rbd::X));
	debugPush("CPGb_RHy",(double)WCPGb[dog::RH](rbd::Y));
	debugPush("CPGb_RHz",(double)WCPGb[dog::RH](rbd::Z));
	debugPush("dCPGb_LFx",(double)dWCPGb[dog::LF](rbd::X));
	debugPush("dCPGb_LFy",(double)dWCPGb[dog::LF](rbd::Y));
	debugPush("dCPGb_LFz",(double)dWCPGb[dog::LF](rbd::Z));
	debugPush("dCPGb_RFx",(double)dWCPGb[dog::RF](rbd::X));
	debugPush("dCPGb_RFy",(double)dWCPGb[dog::RF](rbd::Y));
	debugPush("dCPGb_RFz",(double)dWCPGb[dog::RF](rbd::Z));
	debugPush("dCPGb_LHx",(double)dWCPGb[dog::LH](rbd::X));
	debugPush("dCPGb_LHy",(double)dWCPGb[dog::LH](rbd::Y));
	debugPush("dCPGb_LHz",(double)dWCPGb[dog::LH](rbd::Z));
	debugPush("dCPGb_RHx",(double)dWCPGb[dog::RH](rbd::X));
	debugPush("dCPGb_RHy",(double)dWCPGb[dog::RH](rbd::Y));
	debugPush("dCPGb_RHz",(double)dWCPGb[dog::RH](rbd::Z));

	//CPG variables for output filter
    debugPush("CPG_LFx",(double)WCPG[dog::LF](rbd::X));
    debugPush("CPG_LFy",(double)WCPG[dog::LF](rbd::Y));
    debugPush("CPG_LFz",(double)WCPG[dog::LF](rbd::Z));
    debugPush("CPG_RFx",(double)WCPG[dog::RF](rbd::X));
    debugPush("CPG_RFy",(double)WCPG[dog::RF](rbd::Y));
    debugPush("CPG_RFz",(double)WCPG[dog::RF](rbd::Z));
    debugPush("CPG_LHx",(double)WCPG[dog::LH](rbd::X));
    debugPush("CPG_LHy",(double)WCPG[dog::LH](rbd::Y));
    debugPush("CPG_LHz",(double)WCPG[dog::LH](rbd::Z));
    debugPush("CPG_RHx",(double)WCPG[dog::RH](rbd::X));
    debugPush("CPG_RHy",(double)WCPG[dog::RH](rbd::Y));
    debugPush("CPG_RHz",(double)WCPG[dog::RH](rbd::Z));
    debugPush("dCPG_LFx",(double)dWCPG[dog::LF](rbd::X));
    debugPush("dCPG_LFy",(double)dWCPG[dog::LF](rbd::Y));
    debugPush("dCPG_LFz",(double)dWCPG[dog::LF](rbd::Z));
    debugPush("dCPG_RFx",(double)dWCPG[dog::RF](rbd::X));
    debugPush("dCPG_RFy",(double)dWCPG[dog::RF](rbd::Y));
    debugPush("dCPG_RFz",(double)dWCPG[dog::RF](rbd::Z));
    debugPush("dCPG_LHx",(double)dWCPG[dog::LH](rbd::X));
    debugPush("dCPG_LHy",(double)dWCPG[dog::LH](rbd::Y));
    debugPush("dCPG_LHz",(double)dWCPG[dog::LH](rbd::Z));
    debugPush("dCPG_RHx",(double)dWCPG[dog::RH](rbd::X));
    debugPush("dCPG_RHy",(double)dWCPG[dog::RH](rbd::Y));
    debugPush("dCPG_RHz",(double)dWCPG[dog::RH](rbd::Z));

    //CPG variables
    debugPush("CPG_angularFreqLF",(double)CPG[dog::LF].getAngularFrequency());
    debugPush("CPG_angularFreqRF",(double)CPG[dog::RF].getAngularFrequency());
    debugPush("CPG_angularFreqLH",(double)CPG[dog::LH].getAngularFrequency());
    debugPush("CPG_angularFreqRH",(double)CPG[dog::RH].getAngularFrequency());
    debugPush("CPG_couplingTermLF",(double)CPG[dog::LF].couplingTerm);
    debugPush("CPG_couplingTermRF",(double)CPG[dog::RF].couplingTerm);
    debugPush("CPG_couplingTermLH",(double)CPG[dog::LH].couplingTerm);
    debugPush("CPG_couplingTermRH",(double)CPG[dog::RH].couplingTerm);



	//Foot variables
	debugPush("P_Local_BF_LFx",(double)P_Local_BF[dog::LF](rbd::X));
	debugPush("P_Local_BF_LFy",(double)P_Local_BF[dog::LF](rbd::Y));
	debugPush("P_Local_BF_LFz",(double)P_Local_BF[dog::LF](rbd::Z));
	debugPush("P_Local_BF_RFx",(double)P_Local_BF[dog::RF](rbd::X));
	debugPush("P_Local_BF_RFy",(double)P_Local_BF[dog::RF](rbd::Y));
	debugPush("P_Local_BF_RFz",(double)P_Local_BF[dog::RF](rbd::Z));
	debugPush("P_Local_BF_LHx",(double)P_Local_BF[dog::LH](rbd::X));
	debugPush("P_Local_BF_LHy",(double)P_Local_BF[dog::LH](rbd::Y));
	debugPush("P_Local_BF_LHz",(double)P_Local_BF[dog::LH](rbd::Z));
	debugPush("P_Local_BF_RHx",(double)P_Local_BF[dog::RH](rbd::X));
	debugPush("P_Local_BF_RHy",(double)P_Local_BF[dog::RH](rbd::Y));
	debugPush("P_Local_BF_RHz",(double)P_Local_BF[dog::RH](rbd::Z));

    debugPush("Pd_Local_BF_LFx",(double)Pd_Local_BF[dog::LF](rbd::X));
    debugPush("Pd_Local_BF_LFy",(double)Pd_Local_BF[dog::LF](rbd::Y));
    debugPush("Pd_Local_BF_LFz",(double)Pd_Local_BF[dog::LF](rbd::Z));
    debugPush("Pd_Local_BF_RFx",(double)Pd_Local_BF[dog::RF](rbd::X));
    debugPush("Pd_Local_BF_RFy",(double)Pd_Local_BF[dog::RF](rbd::Y));
    debugPush("Pd_Local_BF_RFz",(double)Pd_Local_BF[dog::RF](rbd::Z));
    debugPush("Pd_Local_BF_LHx",(double)Pd_Local_BF[dog::LH](rbd::X));
    debugPush("Pd_Local_BF_LHy",(double)Pd_Local_BF[dog::LH](rbd::Y));
    debugPush("Pd_Local_BF_LHz",(double)Pd_Local_BF[dog::LH](rbd::Z));
    debugPush("Pd_Local_BF_RHx",(double)Pd_Local_BF[dog::RH](rbd::X));
    debugPush("Pd_Local_BF_RHy",(double)Pd_Local_BF[dog::RH](rbd::Y));
    debugPush("Pd_Local_BF_RHz",(double)Pd_Local_BF[dog::RH](rbd::Z));

    debugPush("Pdd_Local_BF_LFx",(double)Pdd_Local_BF[dog::LF](rbd::X));
    debugPush("Pdd_Local_BF_LFy",(double)Pdd_Local_BF[dog::LF](rbd::Y));
    debugPush("Pdd_Local_BF_LFz",(double)Pdd_Local_BF[dog::LF](rbd::Z));
    debugPush("Pdd_Local_BF_RFx",(double)Pdd_Local_BF[dog::RF](rbd::X));
    debugPush("Pdd_Local_BF_RFy",(double)Pdd_Local_BF[dog::RF](rbd::Y));
    debugPush("Pdd_Local_BF_RFz",(double)Pdd_Local_BF[dog::RF](rbd::Z));
    debugPush("Pdd_Local_BF_LHx",(double)Pdd_Local_BF[dog::LH](rbd::X));
    debugPush("Pdd_Local_BF_LHy",(double)Pdd_Local_BF[dog::LH](rbd::Y));
    debugPush("Pdd_Local_BF_LHz",(double)Pdd_Local_BF[dog::LH](rbd::Z));
    debugPush("Pdd_Local_BF_RHx",(double)Pdd_Local_BF[dog::RH](rbd::X));
    debugPush("Pdd_Local_BF_RHy",(double)Pdd_Local_BF[dog::RH](rbd::Y));
    debugPush("Pdd_Local_BF_RHz",(double)Pdd_Local_BF[dog::RH](rbd::Z));

    debugPush("P_DesLocal_BF_LFx",(double)P_DesLocal_BF[dog::LF](rbd::X));
    debugPush("P_DesLocal_BF_LFy",(double)P_DesLocal_BF[dog::LF](rbd::Y));
    debugPush("P_DesLocal_BF_LFz",(double)P_DesLocal_BF[dog::LF](rbd::Z));
    debugPush("P_DesLocal_BF_RFx",(double)P_DesLocal_BF[dog::RF](rbd::X));
    debugPush("P_DesLocal_BF_RFy",(double)P_DesLocal_BF[dog::RF](rbd::Y));
    debugPush("P_DesLocal_BF_RFz",(double)P_DesLocal_BF[dog::RF](rbd::Z));
    debugPush("P_DesLocal_BF_LHx",(double)P_DesLocal_BF[dog::LH](rbd::X));
    debugPush("P_DesLocal_BF_LHy",(double)P_DesLocal_BF[dog::LH](rbd::Y));
    debugPush("P_DesLocal_BF_LHz",(double)P_DesLocal_BF[dog::LH](rbd::Z));
    debugPush("P_DesLocal_BF_RHx",(double)P_DesLocal_BF[dog::RH](rbd::X));
    debugPush("P_DesLocal_BF_RHy",(double)P_DesLocal_BF[dog::RH](rbd::Y));
    debugPush("P_DesLocal_BF_RHz",(double)P_DesLocal_BF[dog::RH](rbd::Z));

    debugPush("Pd_DesLocal_BF_LFx",(double)Pd_DesLocal_BF[dog::LF](rbd::X));
    debugPush("Pd_DesLocal_BF_LFy",(double)Pd_DesLocal_BF[dog::LF](rbd::Y));
    debugPush("Pd_DesLocal_BF_LFz",(double)Pd_DesLocal_BF[dog::LF](rbd::Z));
    debugPush("Pd_DesLocal_BF_RFx",(double)Pd_DesLocal_BF[dog::RF](rbd::X));
    debugPush("Pd_DesLocal_BF_RFy",(double)Pd_DesLocal_BF[dog::RF](rbd::Y));
    debugPush("Pd_DesLocal_BF_RFz",(double)Pd_DesLocal_BF[dog::RF](rbd::Z));
    debugPush("Pd_DesLocal_BF_LHx",(double)Pd_DesLocal_BF[dog::LH](rbd::X));
    debugPush("Pd_DesLocal_BF_LHy",(double)Pd_DesLocal_BF[dog::LH](rbd::Y));
    debugPush("Pd_DesLocal_BF_LHz",(double)Pd_DesLocal_BF[dog::LH](rbd::Z));
    debugPush("Pd_DesLocal_BF_RHx",(double)Pd_DesLocal_BF[dog::RH](rbd::X));
    debugPush("Pd_DesLocal_BF_RHy",(double)Pd_DesLocal_BF[dog::RH](rbd::Y));
    debugPush("Pd_DesLocal_BF_RHz",(double)Pd_DesLocal_BF[dog::RH](rbd::Z));

    debugPush("Pdd_DesLocal_BF_LFx",(double)Pdd_DesLocal_BF[dog::LF](rbd::X));
    debugPush("Pdd_DesLocal_BF_LFy",(double)Pdd_DesLocal_BF[dog::LF](rbd::Y));
    debugPush("Pdd_DesLocal_BF_LFz",(double)Pdd_DesLocal_BF[dog::LF](rbd::Z));
    debugPush("Pdd_DesLocal_BF_RFx",(double)Pdd_DesLocal_BF[dog::RF](rbd::X));
    debugPush("Pdd_DesLocal_BF_RFy",(double)Pdd_DesLocal_BF[dog::RF](rbd::Y));
    debugPush("Pdd_DesLocal_BF_RFz",(double)Pdd_DesLocal_BF[dog::RF](rbd::Z));
    debugPush("Pdd_DesLocal_BF_LHx",(double)Pdd_DesLocal_BF[dog::LH](rbd::X));
    debugPush("Pdd_DesLocal_BF_LHy",(double)Pdd_DesLocal_BF[dog::LH](rbd::Y));
    debugPush("Pdd_DesLocal_BF_LHz",(double)Pdd_DesLocal_BF[dog::LH](rbd::Z));
    debugPush("Pdd_DesLocal_BF_RHx",(double)Pdd_DesLocal_BF[dog::RH](rbd::X));
    debugPush("Pdd_DesLocal_BF_RHy",(double)Pdd_DesLocal_BF[dog::RH](rbd::Y));
    debugPush("Pdd_DesLocal_BF_RHz",(double)Pdd_DesLocal_BF[dog::RH](rbd::Z));

    debugPush("P_Local_HF_LFx",(double)P_Local_HF[dog::LF](rbd::X));
    debugPush("P_Local_HF_LFy",(double)P_Local_HF[dog::LF](rbd::Y));
    debugPush("P_Local_HF_LFz",(double)P_Local_HF[dog::LF](rbd::Z));
    debugPush("P_Local_HF_RFx",(double)P_Local_HF[dog::RF](rbd::X));
    debugPush("P_Local_HF_RFy",(double)P_Local_HF[dog::RF](rbd::Y));
    debugPush("P_Local_HF_RFz",(double)P_Local_HF[dog::RF](rbd::Z));
    debugPush("P_Local_HF_LHx",(double)P_Local_HF[dog::LH](rbd::X));
    debugPush("P_Local_HF_LHy",(double)P_Local_HF[dog::LH](rbd::Y));
    debugPush("P_Local_HF_LHz",(double)P_Local_HF[dog::LH](rbd::Z));
    debugPush("P_Local_HF_RHx",(double)P_Local_HF[dog::RH](rbd::X));
    debugPush("P_Local_HF_RHy",(double)P_Local_HF[dog::RH](rbd::Y));
    debugPush("P_Local_HF_RHz",(double)P_Local_HF[dog::RH](rbd::Z));

    debugPush("Pd_Local_HF_LFx",(double)Pd_Local_HF[dog::LF](rbd::X));
    debugPush("Pd_Local_HF_LFy",(double)Pd_Local_HF[dog::LF](rbd::Y));
    debugPush("Pd_Local_HF_LFz",(double)Pd_Local_HF[dog::LF](rbd::Z));
    debugPush("Pd_Local_HF_RFx",(double)Pd_Local_HF[dog::RF](rbd::X));
    debugPush("Pd_Local_HF_RFy",(double)Pd_Local_HF[dog::RF](rbd::Y));
    debugPush("Pd_Local_HF_RFz",(double)Pd_Local_HF[dog::RF](rbd::Z));
    debugPush("Pd_Local_HF_LHx",(double)Pd_Local_HF[dog::LH](rbd::X));
    debugPush("Pd_Local_HF_LHy",(double)Pd_Local_HF[dog::LH](rbd::Y));
    debugPush("Pd_Local_HF_LHz",(double)Pd_Local_HF[dog::LH](rbd::Z));
    debugPush("Pd_Local_HF_RHx",(double)Pd_Local_HF[dog::RH](rbd::X));
    debugPush("Pd_Local_HF_RHy",(double)Pd_Local_HF[dog::RH](rbd::Y));
    debugPush("Pd_Local_HF_RHz",(double)Pd_Local_HF[dog::RH](rbd::Z));

    debugPush("Pdd_Local_HF_LFx",(double)Pdd_Local_HF[dog::LF](rbd::X));
    debugPush("Pdd_Local_HF_LFy",(double)Pdd_Local_HF[dog::LF](rbd::Y));
    debugPush("Pdd_Local_HF_LFz",(double)Pdd_Local_HF[dog::LF](rbd::Z));
    debugPush("Pdd_Local_HF_RFx",(double)Pdd_Local_HF[dog::RF](rbd::X));
    debugPush("Pdd_Local_HF_RFy",(double)Pdd_Local_HF[dog::RF](rbd::Y));
    debugPush("Pdd_Local_HF_RFz",(double)Pdd_Local_HF[dog::RF](rbd::Z));
    debugPush("Pdd_Local_HF_LHx",(double)Pdd_Local_HF[dog::LH](rbd::X));
    debugPush("Pdd_Local_HF_LHy",(double)Pdd_Local_HF[dog::LH](rbd::Y));
    debugPush("Pdd_Local_HF_LHz",(double)Pdd_Local_HF[dog::LH](rbd::Z));
    debugPush("Pdd_Local_HF_RHx",(double)Pdd_Local_HF[dog::RH](rbd::X));
    debugPush("Pdd_Local_HF_RHy",(double)Pdd_Local_HF[dog::RH](rbd::Y));
    debugPush("Pdd_Local_HF_RHz",(double)Pdd_Local_HF[dog::RH](rbd::Z));

    debugPush("P_DesLocal_HF_LFx",(double)P_DesLocal_HF[dog::LF](rbd::X));
    debugPush("P_DesLocal_HF_LFy",(double)P_DesLocal_HF[dog::LF](rbd::Y));
    debugPush("P_DesLocal_HF_LFz",(double)P_DesLocal_HF[dog::LF](rbd::Z));
    debugPush("P_DesLocal_HF_RFx",(double)P_DesLocal_HF[dog::RF](rbd::X));
    debugPush("P_DesLocal_HF_RFy",(double)P_DesLocal_HF[dog::RF](rbd::Y));
    debugPush("P_DesLocal_HF_RFz",(double)P_DesLocal_HF[dog::RF](rbd::Z));
    debugPush("P_DesLocal_HF_LHx",(double)P_DesLocal_HF[dog::LH](rbd::X));
    debugPush("P_DesLocal_HF_LHy",(double)P_DesLocal_HF[dog::LH](rbd::Y));
    debugPush("P_DesLocal_HF_LHz",(double)P_DesLocal_HF[dog::LH](rbd::Z));
    debugPush("P_DesLocal_HF_RHx",(double)P_DesLocal_HF[dog::RH](rbd::X));
    debugPush("P_DesLocal_HF_RHy",(double)P_DesLocal_HF[dog::RH](rbd::Y));
    debugPush("P_DesLocal_HF_RHz",(double)P_DesLocal_HF[dog::RH](rbd::Z));

    debugPush("Pd_DesLocal_HF_LFx",(double)Pd_DesLocal_HF[dog::LF](rbd::X));
    debugPush("Pd_DesLocal_HF_LFy",(double)Pd_DesLocal_HF[dog::LF](rbd::Y));
    debugPush("Pd_DesLocal_HF_LFz",(double)Pd_DesLocal_HF[dog::LF](rbd::Z));
    debugPush("Pd_DesLocal_HF_RFx",(double)Pd_DesLocal_HF[dog::RF](rbd::X));
    debugPush("Pd_DesLocal_HF_RFy",(double)Pd_DesLocal_HF[dog::RF](rbd::Y));
    debugPush("Pd_DesLocal_HF_RFz",(double)Pd_DesLocal_HF[dog::RF](rbd::Z));
    debugPush("Pd_DesLocal_HF_LHx",(double)Pd_DesLocal_HF[dog::LH](rbd::X));
    debugPush("Pd_DesLocal_HF_LHy",(double)Pd_DesLocal_HF[dog::LH](rbd::Y));
    debugPush("Pd_DesLocal_HF_LHz",(double)Pd_DesLocal_HF[dog::LH](rbd::Z));
    debugPush("Pd_DesLocal_HF_RHx",(double)Pd_DesLocal_HF[dog::RH](rbd::X));
    debugPush("Pd_DesLocal_HF_RHy",(double)Pd_DesLocal_HF[dog::RH](rbd::Y));
    debugPush("Pd_DesLocal_HF_RHz",(double)Pd_DesLocal_HF[dog::RH](rbd::Z));

    debugPush("Pdd_DesLocal_HF_LFx",(double)Pdd_DesLocal_HF[dog::LF](rbd::X));
    debugPush("Pdd_DesLocal_HF_LFy",(double)Pdd_DesLocal_HF[dog::LF](rbd::Y));
    debugPush("Pdd_DesLocal_HF_LFz",(double)Pdd_DesLocal_HF[dog::LF](rbd::Z));
    debugPush("Pdd_DesLocal_HF_RFx",(double)Pdd_DesLocal_HF[dog::RF](rbd::X));
    debugPush("Pdd_DesLocal_HF_RFy",(double)Pdd_DesLocal_HF[dog::RF](rbd::Y));
    debugPush("Pdd_DesLocal_HF_RFz",(double)Pdd_DesLocal_HF[dog::RF](rbd::Z));
    debugPush("Pdd_DesLocal_HF_LHx",(double)Pdd_DesLocal_HF[dog::LH](rbd::X));
    debugPush("Pdd_DesLocal_HF_LHy",(double)Pdd_DesLocal_HF[dog::LH](rbd::Y));
    debugPush("Pdd_DesLocal_HF_LHz",(double)Pdd_DesLocal_HF[dog::LH](rbd::Z));
    debugPush("Pdd_DesLocal_HF_RHx",(double)Pdd_DesLocal_HF[dog::RH](rbd::X));
    debugPush("Pdd_DesLocal_HF_RHy",(double)Pdd_DesLocal_HF[dog::RH](rbd::Y));
    debugPush("Pdd_DesLocal_HF_RHz",(double)Pdd_DesLocal_HF[dog::RH](rbd::Z));

    debugPush("P0_Local_HF_LFx",(double)P0_Local_HF[dog::LF](rbd::X));
    debugPush("P0_Local_HF_LFy",(double)P0_Local_HF[dog::LF](rbd::Y));
    debugPush("P0_Local_HF_LFz",(double)P0_Local_HF[dog::LF](rbd::Z));
    debugPush("P0_Local_HF_RFx",(double)P0_Local_HF[dog::RF](rbd::X));
    debugPush("P0_Local_HF_RFy",(double)P0_Local_HF[dog::RF](rbd::Y));
    debugPush("P0_Local_HF_RFz",(double)P0_Local_HF[dog::RF](rbd::Z));
    debugPush("P0_Local_HF_LHx",(double)P0_Local_HF[dog::LH](rbd::X));
    debugPush("P0_Local_HF_LHy",(double)P0_Local_HF[dog::LH](rbd::Y));
    debugPush("P0_Local_HF_LHz",(double)P0_Local_HF[dog::LH](rbd::Z));
    debugPush("P0_Local_HF_RHx",(double)P0_Local_HF[dog::RH](rbd::X));
    debugPush("P0_Local_HF_RHy",(double)P0_Local_HF[dog::RH](rbd::Y));
    debugPush("P0_Local_HF_RHz",(double)P0_Local_HF[dog::RH](rbd::Z));

    //Desired foot position component from the push recovery
    debugPush("deltaPushRecPosLFx",(double)deltaPR_Pos[dog::LF](rbd::X));
    debugPush("deltaPushRecPosRFx",(double)deltaPR_Pos[dog::RF](rbd::X));
    debugPush("deltaPushRecPosLHx",(double)deltaPR_Pos[dog::LH](rbd::X));
    debugPush("deltaPushRecPosRHx",(double)deltaPR_Pos[dog::RH](rbd::X));
    debugPush("deltaPushRecPosLFy",(double)deltaPR_Pos[dog::LF](rbd::Y));
    debugPush("deltaPushRecPosRFy",(double)deltaPR_Pos[dog::RF](rbd::Y));
    debugPush("deltaPushRecPosLHy",(double)deltaPR_Pos[dog::LH](rbd::Y));
    debugPush("deltaPushRecPosRHy",(double)deltaPR_Pos[dog::RH](rbd::Y));
    debugPush("deltaPushRecVelLFx",(double)deltaPR_Vel[dog::LF](rbd::X));
    debugPush("deltaPushRecVelRFx",(double)deltaPR_Vel[dog::RF](rbd::X));
    debugPush("deltaPushRecVelLHx",(double)deltaPR_Vel[dog::LH](rbd::X));
    debugPush("deltaPushRecVelRHx",(double)deltaPR_Vel[dog::RH](rbd::X));
    debugPush("deltaPushRecVelLFy",(double)deltaPR_Vel[dog::LF](rbd::Y));
    debugPush("deltaPushRecVelRFy",(double)deltaPR_Vel[dog::RF](rbd::Y));
    debugPush("deltaPushRecVelLHy",(double)deltaPR_Vel[dog::LH](rbd::Y));
    debugPush("deltaPushRecVelRHy",(double)deltaPR_Vel[dog::RH](rbd::Y));

    //Push recovery variables
    debugPush("deltaPushRecPosLFx_raw",(double)PushRecovery.deltaPos[dog::LF](rbd::X));
    debugPush("deltaPushRecPosRFx_raw",(double)PushRecovery.deltaPos[dog::RF](rbd::X));
    debugPush("deltaPushRecPosLHx_raw",(double)PushRecovery.deltaPos[dog::LH](rbd::X));
    debugPush("deltaPushRecPosRHx_raw",(double)PushRecovery.deltaPos[dog::RH](rbd::X));
    debugPush("deltaPushRecPosLFy_raw",(double)PushRecovery.deltaPos[dog::LF](rbd::Y));
    debugPush("deltaPushRecPosRFy_raw",(double)PushRecovery.deltaPos[dog::RF](rbd::Y));
    debugPush("deltaPushRecPosLHy_raw",(double)PushRecovery.deltaPos[dog::LH](rbd::Y));
    debugPush("deltaPushRecPosRHy_raw",(double)PushRecovery.deltaPos[dog::RH](rbd::Y));
    debugPush("deltaPushRecVelLFx_raw",(double)PushRecovery.deltaPos[dog::LF](rbd::X));
    debugPush("deltaPushRecVelRFx_raw",(double)PushRecovery.deltaPos[dog::RF](rbd::X));
    debugPush("deltaPushRecVelLHx_raw",(double)PushRecovery.deltaPos[dog::LH](rbd::X));
    debugPush("deltaPushRecVelRHx_raw",(double)PushRecovery.deltaPos[dog::RH](rbd::X));
    debugPush("deltaPushRecVelLFy_raw",(double)PushRecovery.deltaPos[dog::LF](rbd::Y));
    debugPush("deltaPushRecVelRFy_raw",(double)PushRecovery.deltaPos[dog::RF](rbd::Y));
    debugPush("deltaPushRecVelLHy_raw",(double)PushRecovery.deltaPos[dog::LH](rbd::Y));
    debugPush("deltaPushRecVelRHy_raw",(double)PushRecovery.deltaPos[dog::RH](rbd::Y));

    //Reflex component
    debugPush("reflexPosLF_HFx",(double)P_Reflex_HF[dog::LF](rbd::X));
    //debugPush("reflexPosLF_HFy",(double)P_Reflex_HF[dog::LF](rbd::Y));
    //debugPush("reflexPosLF_HFz",(double)P_Reflex_HF[dog::LF](rbd::Z));
    debugPush("reflexPosRF_HFx",(double)P_Reflex_HF[dog::RF](rbd::X));
    //debugPush("reflexPosRF_HFy",(double)P_Reflex_HF[dog::RF](rbd::Y));
    //debugPush("reflexPosRF_HFz",(double)P_Reflex_HF[dog::RF](rbd::Z));
    debugPush("reflexPosLH_HFx",(double)P_Reflex_HF[dog::LH](rbd::X));
    //debugPush("reflexPosLH_HFy",(double)P_Reflex_HF[dog::LH](rbd::Y));
    //debugPush("reflexPosLH_HFz",(double)P_Reflex_HF[dog::LH](rbd::Z));
    debugPush("reflexPosRH_HFx",(double)P_Reflex_HF[dog::RH](rbd::X));
    //debugPush("reflexPosRH_HFy",(double)P_Reflex_HF[dog::RH](rbd::Y));
    //debugPush("reflexPosRH_HFz",(double)P_Reflex_HF[dog::RH](rbd::Z));

    debugPush("reflexVelLF_HFx",(double)Pd_Reflex_HF[dog::LF](rbd::X));
    //debugPush("reflexVelLF_HFy",(double)Pd_Reflex_HF[dog::LF](rbd::Y));
    //debugPush("reflexVelLF_HFz",(double)Pd_Reflex_HF[dog::LF](rbd::Z));
    debugPush("reflexVelRF_HFx",(double)Pd_Reflex_HF[dog::RF](rbd::X));
    //debugPush("reflexVelRF_HFy",(double)Pd_Reflex_HF[dog::RF](rbd::Y));
    //debugPush("reflexVelRF_HFz",(double)Pd_Reflex_HF[dog::RF](rbd::Z));
    debugPush("reflexVelLH_HFx",(double)Pd_Reflex_HF[dog::LH](rbd::X));
    //debugPush("reflexVelLH_HFy",(double)Pd_Reflex_HF[dog::LH](rbd::Y));
    //debugPush("reflexVelLH_HFz",(double)Pd_Reflex_HF[dog::LH](rbd::Z));
    debugPush("reflexVelRH_HFx",(double)Pd_Reflex_HF[dog::RH](rbd::X));
    //debugPush("reflexVelRH_HFy",(double)Pd_Reflex_HF[dog::RH](rbd::Y));
    //debugPush("reflexVelRH_HFz",(double)Pd_Reflex_HF[dog::RH](rbd::Z));

    //Robot odometry
    debugPush("desRobotPosWFx",(double)desRobotPositionWF(rbd::X));
    debugPush("desRobotPosWFy",(double)desRobotPositionWF(rbd::Y));
    debugPush("desRobotPosHFx",(double)desRobotPositionHF(rbd::X));
    debugPush("desRobotPosHFy",(double)desRobotPositionHF(rbd::Y));
    debugPush("actualRobotPosHFx",(double)actualRobotPositionHF(rbd::X));
    debugPush("actualRobotPosHFy",(double)actualRobotPositionHF(rbd::Y));
    debugPush("actualRobotPosWFx",(double)actualRobotPositionWF(rbd::X));
    debugPush("actualRobotPosWFy",(double)actualRobotPositionWF(rbd::Y));
    debugPush("desRobotYaw",(double)desRobotYaw);

    //Trunk wrench
    //Desired trunk wrench
    debugPush("desTrunkWrenchHF_Fx",(double)desTrunkWrench(0));
    debugPush("desTrunkWrenchHF_Fy",(double)desTrunkWrench(1));
    debugPush("desTrunkWrenchHF_Fz",(double)desTrunkWrench(2));
    debugPush("desTrunkWrenchHF_Mx",(double)desTrunkWrench(3));
    debugPush("desTrunkWrenchHF_My",(double)desTrunkWrench(4));
    debugPush("desTrunkWrenchHF_Mz",(double)desTrunkWrench(5));

    //Actual trunk wrench

}

///*********************************************************************************************///

long long int counter = 0;
rbd::Vector3d RCFController::baseToHF(const rbd::Vector3d& pos){

    rbd::Matrix33d R;

    R(0, 0) = cos(bs->getRoll_W());
    R(0, 1) = sin(bs->getPitch_W()) * sin(bs->getRoll_W());
    R(0, 2) = cos(bs->getRoll_W()) * sin(bs->getPitch_W());
    R(1, 0) = 0.0;
    R(1, 1) = cos(bs->getRoll_W());
    R(1, 2) = -sin(bs->getRoll_W());
    R(2, 0) = -sin(bs->getPitch_W());
    R(2, 1) = cos(bs->getPitch_W()) * sin(bs->getRoll_W());
    R(2, 2) = cos(bs->getPitch_W()) * cos(bs->getRoll_W());
    return R*pos;
}


///*********************************************************************************************///

//Here the nonlinear changes of each locomotion parameter are filtered by a second order filter.
void RCFController::computeFilteredChanges() {

	std::map<std::string,double> alphaGain;
	double fastSplineDuration = 2;

	alphaGain["filt_chang1"]= 1 / (double) taskServoRate
			/ (0.125 / 2 + 1 / (double) taskServoRate);
	alphaGain["filt_chang2"]= 1 / (double) taskServoRate
			/ (fastSplineDuration / 8 + 1 / (double) taskServoRate);
	alphaGain["002sec"]= 1 / (double) taskServoRate
			/ (0.02 + 1 / (double) taskServoRate);
	alphaGain["005sec"]= 1 / (double) taskServoRate
            / (0.05 + 1 / (double) taskServoRate);
	alphaGain["010sec"]= 1 / (double) taskServoRate
			/ (0.10 + 1 / (double) taskServoRate);
	alphaGain["075sec"]= 1 / (double) taskServoRate
			/ (0.75 + 1 / (double) taskServoRate);
	alphaGain["300sec"]= 1 / (double) taskServoRate
			/ (300 + 1 / (double) taskServoRate);

	//Default feet positions (or CPG origins)
	double originAlphaFilter = 0.5;

	switch(originChangingRate) {
	case filterResponse::slowC:
	    originAlphaFilter = alphaGain["075sec"];
	    break;
	case filterResponse::mediumC:
	    originAlphaFilter = alphaGain["010sec"];
	    break;
	case filterResponse::fastC:
	    originAlphaFilter = alphaGain["002sec"];
	    break;

	}


	static double Psit2 = PsitNew;
	static double terrainPitch2 = terrainPitchNew;
	static double forwVel2 = forwVelNew;
	static double VfX2 = VfX_New;
	static double VfY2 = VfY_New;
	static double stepLength2 = stepLengthNew;
	static double stepHeight2 = stepHeightNew;
	static double stepFrequency2 = stepFrequencyNew;
	static double alphaGamma2 = alphaGammaNew;
	static double Kc2 = KcNew;
	static double Kcoup2 = KcoupNew;
	static double dutyF2 = dutyF_New;
	static double KpTrunkRoll2 = KpTrunkRollNew;
	static double KdTrunkRoll2 = KdTrunkRollNew;
	static double KpTrunkPitch2 = KpTrunkPitchNew;
	static double KdTrunkPitch2 = KdTrunkPitchNew;
	static double KpTrunkYaw2 = KpTrunkYawNew;
	static double KdTrunkYaw2 = KdTrunkYawNew;
	static double KpTrunkX2 = KpTrunkX_New;
	static double KdTrunkX2 = KdTrunkX_New;
	static double KpTrunkY2 = KpTrunkY_New;
	static double KdTrunkY2 = KdTrunkY_New;
	static double KpTrunkZ2 = KpTrunkZ_New;
	static double KdTrunkZ2 = KdTrunkZ_New;
	static double bodyWeight2 = bodyWeightNew;
	static double Kbp2 = KbpNew;
	static double Kbf2 = KbfNew;
	static double Kbv2 = KbvNew;
	static double Kvf2 = KvfNew;
	static double kadjRoll2 = kadjRollNew;
	static double kadjPitch2 = kadjPitchNew;
	static double rollMaxKadj2 = rollMaxKadjNew;
	static double pitchMaxKadj2 = pitchMaxKadjNew;
	static double xForceOffset2 = xForceOffsetNew;
	static double yForceOffset2 = yForceOffsetNew;
	static double zForceOffset2 = zForceOffsetNew;
	static double xMomentOffset2 = xMomentOffsetNew;
	static double yMomentOffset2 = yMomentOffsetNew;
	static double zMomentOffset2 = zMomentOffsetNew;


	Psit = secondOrderFilter(Psit, Psit2, PsitNew, alphaGain["filt_chang1"]);

	terrainPitch = secondOrderFilter(terrainPitch, terrainPitch2, terrainPitchNew, alphaGain["filt_chang1"]);

	forwVel = secondOrderFilter(forwVel, forwVel2, forwVelNew,alphaGain["filt_chang2"]);

	VfX = secondOrderFilter(VfX, VfX2, VfX_New,alphaGain["filt_chang2"]);
	VfY = secondOrderFilter(VfY, VfY2, VfY_New,alphaGain["filt_chang2"]);

	stepLength = secondOrderFilter(stepLength, stepLength2, stepLengthNew,alphaGain["filt_chang2"]);
	stepHeight = secondOrderFilter(stepHeight, stepHeight2, stepHeightNew,alphaGain["filt_chang2"]);
	stepFrequency = secondOrderFilter(stepFrequency, stepFrequency2, stepFrequencyNew,alphaGain["filt_chang2"]);

	alphaGamma = secondOrderFilter(alphaGamma, alphaGamma2, alphaGammaNew,alphaGain["filt_chang2"]);

	Kc = secondOrderFilter(Kc, Kc2, KcNew,alphaGain["filt_chang2"]);

	Kcoup = secondOrderFilter(Kcoup, Kcoup2, KcoupNew,alphaGain["filt_chang2"]);

	dutyF = secondOrderFilter(dutyF, dutyF2, dutyF_New,alphaGain["filt_chang2"]);

	KpTrunkRoll = secondOrderFilter(KpTrunkRoll, KpTrunkRoll2, KpTrunkRollNew,alphaGain["075sec"]);
	KdTrunkRoll = secondOrderFilter(KdTrunkRoll, KdTrunkRoll2, KdTrunkRollNew,alphaGain["075sec"]);
	KpTrunkPitch = secondOrderFilter(KpTrunkPitch, KpTrunkPitch2, KpTrunkPitchNew,alphaGain["075sec"]);
	KdTrunkPitch = secondOrderFilter(KdTrunkPitch, KdTrunkPitch2, KdTrunkPitchNew,alphaGain["075sec"]);
	KpTrunkYaw = secondOrderFilter(KpTrunkYaw, KpTrunkYaw2, KpTrunkYawNew,alphaGain["075sec"]);
	KdTrunkYaw = secondOrderFilter(KdTrunkYaw, KdTrunkYaw2, KdTrunkYawNew,alphaGain["075sec"]);
	KpTrunkX = secondOrderFilter(KpTrunkX, KpTrunkX2, KpTrunkX_New,alphaGain["075sec"]);
	KdTrunkX = secondOrderFilter(KdTrunkX, KdTrunkX2, KdTrunkX_New,alphaGain["075sec"]);
	KpTrunkY = secondOrderFilter(KpTrunkY, KpTrunkY2, KpTrunkY_New,alphaGain["075sec"]);
	KdTrunkY = secondOrderFilter(KdTrunkY, KdTrunkY2, KdTrunkY_New,alphaGain["075sec"]);
	KpTrunkZ = secondOrderFilter(KpTrunkZ, KpTrunkZ2, KpTrunkZ_New,alphaGain["075sec"]);
	KdTrunkZ = secondOrderFilter(KdTrunkZ, KdTrunkZ2, KdTrunkZ_New,alphaGain["075sec"]);
	bodyWeight = secondOrderFilter(bodyWeight, bodyWeight2, bodyWeightNew,alphaGain["075sec"]);

	//CPG parameters
	Kbp = secondOrderFilter(Kbp, Kbp2, KbpNew,alphaGain["010sec"]);
	Kbf = secondOrderFilter(Kbf, Kbf2, KbfNew,alphaGain["010sec"]);
	Kbv = secondOrderFilter(Kbv, Kbv2, KbvNew,alphaGain["010sec"]);
	Kvf = secondOrderFilter(Kvf, Kvf2, KvfNew,alphaGain["010sec"]);

	//Kinematic adjustment parameters
	kadjRoll = secondOrderFilter(kadjRoll, kadjRoll2, kadjRollNew,alphaGain["010sec"]);
	kadjPitch = secondOrderFilter(kadjPitch, kadjPitch2, kadjPitchNew,alphaGain["010sec"]);
	rollMaxKadj = secondOrderFilter(rollMaxKadj, rollMaxKadj2, rollMaxKadjNew,alphaGain["010sec"]);
	pitchMaxKadj = secondOrderFilter(pitchMaxKadj, pitchMaxKadj2, pitchMaxKadjNew,alphaGain["010sec"]);

	//Trunk offset forces and moments
	xForceOffset = secondOrderFilter(xForceOffset, xForceOffset2, xForceOffsetNew,alphaGain["010sec"]);
	yForceOffset = secondOrderFilter(yForceOffset, yForceOffset2, yForceOffsetNew,alphaGain["010sec"]);
	zForceOffset = secondOrderFilter(zForceOffset, zForceOffset2, zForceOffsetNew,alphaGain["010sec"]);
	xMomentOffset = secondOrderFilter(xMomentOffset, xMomentOffset2, xMomentOffsetNew,alphaGain["010sec"]);
	yMomentOffset = secondOrderFilter(yMomentOffset, yMomentOffset2, yMomentOffsetNew,alphaGain["010sec"]);
	zMomentOffset = secondOrderFilter(zMomentOffset, zMomentOffset2, zMomentOffsetNew,alphaGain["010sec"]);

	//Origin changes
	static iit::dog::LegDataMap<rbd::Vector3d> P0_Local_HF_Filter = P0_Local_HF_New;

	if(instantaneousOriginChangesFlag){
	    P0_Local_HF_Filter = P0_Local_HF_New;
	    P0_Local_HF = P0_Local_HF_New;
	    instantaneousOriginChangesFlag = false;
	}

	for(int leg = dog::LF; leg <= dog::RH; leg++) {
        P0_Local_HF_Filter[leg] = (1 - originAlphaFilter) * P0_Local_HF_Filter[leg]
                                  + originAlphaFilter * P0_Local_HF_New[leg];
        P0_Local_HF[leg] = (1 - originAlphaFilter) * P0_Local_HF[leg]
                   + originAlphaFilter * P0_Local_HF_Filter[leg];
    }

	//Desired trunk angles
	static Eigen::Matrix<double, 3, 1> eulerAngles2 = eulerAnglesNew;
	eulerAngles2 = (1 - alphaGain["002sec"]) * eulerAngles2
			+ alphaGain["002sec"] * eulerAnglesNew;
	eulerAngles = (1 - alphaGain["002sec"]) * eulerAngles
			+ alphaGain["002sec"] * eulerAngles2;

	//Timer to manage running trot profiles
	profileTransitionTime = (1 - alphaGain["075sec"]) * profileTransitionTime
			+ alphaGain["075sec"] * 1.0;
	if (profileTransitionTime < 0.95) {
		inProfileTransitionFlag = true;
	} else {
		inProfileTransitionFlag = false;
	}

}


///*********************************************************************************************///

void RCFController::bodyVelocitiesEstimation(
		const iit::dog::LegDataMap<bool>& support_legs,
		Eigen::Matrix<double, 3, 1>& Xb_est_HF_aux) {

	Eigen::Matrix<double, 3, 1> Xb1_est_HF;
	Eigen::Matrix<double, 3, 1> Xb2_est_HF;
	Eigen::Matrix<double, 3, 1> Xb3_est_HF;
	Eigen::Matrix<double, 3, 1> Xb4_est_HF;
	Eigen::Matrix<double, 3, 1> Trunk_angles_dot;
	static Eigen::Matrix<double, 3, 1> Xb_est_HF_KinIMU;
	Eigen::Matrix<double, 3, 1> last_Xb_est_HF_KinIMU;

	Eigen::Matrix<double, 3, 3> dRdPhi;
	Eigen::Matrix<double, 3, 3> dRdTheta;
	Eigen::Matrix<double, 3, 3> dRdPsi;
	Eigen::Matrix<double, 3, 3> M1;
	Eigen::Matrix<double, 3, 3> M2;
	Eigen::Matrix<double, 3, 3> M3;
	Eigen::Matrix<double, 3, 3> M4;
	Eigen::Matrix<double, 3, 3> Xf11, Xf12, Xf13;
	Eigen::Matrix<double, 3, 3> Xf21, Xf22, Xf23;
	Eigen::Matrix<double, 3, 3> Xf31, Xf32, Xf33;
	Eigen::Matrix<double, 3, 3> Xf41, Xf42, Xf43;
	Eigen::Matrix<double, 3, 3> R;

	Xb1_est_HF.setZero();
	Xb2_est_HF.setZero();
	Xb3_est_HF.setZero();
	Xb4_est_HF.setZero();

    Trunk_angles_dot(0) = bs->getRotAcceleration_B()[Roll];
    Trunk_angles_dot(1) = bs->getRotAcceleration_B()[Pitch];
    Trunk_angles_dot(2) = bs->getRotAcceleration_B()[Yaw];

	double n_stance_legs = 0;
	if (support_legs[dog::LF]) {
		n_stance_legs++;
	}
	if (support_legs[dog::RF]) {
		n_stance_legs++;
	}
	if (support_legs[dog::LH]) {
		n_stance_legs++;
	}
	if (support_legs[dog::RH]) {
		n_stance_legs++;
	}

	if (n_stance_legs >= 2) {

		double average_coef = 1 / n_stance_legs;

		//Interface SL to Eigen math
		Xf11 << P_Local_BF[dog::LF](rbd::X), 0, 0, P_Local_BF[dog::LF](rbd::Y), 0, 0, P_Local_BF[dog::LF](
				rbd::Z), 0, 0;
		Xf12 << 0, P_Local_BF[dog::LF](rbd::X), 0, 0, P_Local_BF[dog::LF](rbd::Y), 0, 0, P_Local_BF[dog::LF](
				rbd::Z), 0;
		Xf13 << 0, 0, P_Local_BF[dog::LF](rbd::X), 0, 0, P_Local_BF[dog::LF](rbd::Y), 0, 0, P_Local_BF[dog::LF](
				rbd::Z);

		Xf21 << P_Local_BF[dog::RF](rbd::X), 0, 0, P_Local_BF[dog::RF](rbd::Y), 0, 0, P_Local_BF[dog::RF](
				rbd::Z), 0, 0;
		Xf22 << 0, P_Local_BF[dog::RF](rbd::X), 0, 0, P_Local_BF[dog::RF](rbd::Y), 0, 0, P_Local_BF[dog::RF](
				rbd::Z), 0;
		Xf23 << 0, 0, P_Local_BF[dog::RF](rbd::X), 0, 0, P_Local_BF[dog::RF](rbd::Y), 0, 0, P_Local_BF[dog::RF](
				rbd::Z);

		Xf31 << P_Local_BF[dog::LH](rbd::X), 0, 0, P_Local_BF[dog::LH](rbd::Y), 0, 0, P_Local_BF[dog::LH](
				rbd::Z), 0, 0;
		Xf32 << 0, P_Local_BF[dog::LH](rbd::X), 0, 0, P_Local_BF[dog::LH](rbd::Y), 0, 0, P_Local_BF[dog::LH](
				rbd::Z), 0;
		Xf33 << 0, 0, P_Local_BF[dog::LH](rbd::X), 0, 0, P_Local_BF[dog::LH](rbd::Y), 0, 0, P_Local_BF[dog::LH](
				rbd::Z);

		Xf41 << P_Local_BF[dog::RH](rbd::X), 0, 0, P_Local_BF[dog::RH](rbd::Y), 0, 0, P_Local_BF[dog::RH](
				rbd::Z), 0, 0;
		Xf42 << 0, P_Local_BF[dog::RH](rbd::X), 0, 0, P_Local_BF[dog::RH](rbd::Y), 0, 0, P_Local_BF[dog::RH](
				rbd::Z), 0;
		Xf43 << 0, 0, P_Local_BF[dog::RH](rbd::X), 0, 0, P_Local_BF[dog::RH](rbd::Y), 0, 0, P_Local_BF[dog::RH](
				rbd::Z);

		//Rotation matrix
        R(0, 0) = cos(bs->getPitch_W());
        R(0, 1) = sin(bs->getPitch_W()) * sin(bs->getRoll_W());
        R(0, 2) = cos(bs->getRoll_W()) * sin(bs->getPitch_W());
		R(1, 0) = 0.0;
        R(1, 1) = cos(bs->getRoll_W());
        R(1, 2) = -sin(bs->getRoll_W());
        R(2, 0) = -sin(bs->getPitch_W());
        R(2, 1) = cos(bs->getPitch_W()) * sin(bs->getRoll_W());
        R(2, 2) = cos(bs->getPitch_W()) * cos(bs->getRoll_W());

		// Rt11=cos(theta)*cos(psi)
		double dR11dPhi = 0.0;
        double dR11dTheta = -sin(bs->getPitch_W());
		double dR11dPsi = 0.0;

		// R12=-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi)
        double dR12dPhi = cos(bs->getRoll_W()) * sin(bs->getPitch_W());
        double dR12dTheta = sin(bs->getRoll_W()) * cos(bs->getPitch_W());
        double dR12dPsi = -cos(bs->getRoll_W());

		// R13=sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos*(psi)
        double dR13dPhi = -sin(bs->getRoll_W()) * sin(bs->getPitch_W());
        double dR13dTheta = cos(bs->getRoll_W()) * cos(bs->getPitch_W());
        double dR13dPsi = sin(bs->getRoll_W());

		// R21=cos(theta)*sin(psi)
		double dR21dPhi = 0.0;
		double dR21dTheta = 0.0;
        double dR21dPsi = cos(bs->getPitch_W());

		// R22=cos(phi)*cos(psi)+sin(psi)*sin(theta)*sin(psi)
        double dR22dPhi = -sin(bs->getRoll_W());
		double dR22dTheta = 0.0;
        double dR22dPsi = sin(bs->getRoll_W()) * sin(bs->getPitch_W());

		// R23=-sin(phi)*cos(psi)+cos(psi)*sin(theta)*sin(psi)
        double dR23dPhi = -cos(bs->getRoll_W());
		double dR23dTheta = 0.0;
        double dR23dPsi = cos(bs->getRoll_W()) * sin(bs->getPitch_W());

		// R31=-sin(theta)
		double dR31dPhi = 0.0;
        double dR31dTheta = -cos(bs->getPitch_W());
		double dR31dPsi = 0.0;

		// R32=sin(psi)*cos(theta)
        double dR32dPhi = cos(bs->getRoll_W()) * cos(bs->getPitch_W());
        double dR32dTheta = -sin(bs->getRoll_W()) * sin(bs->getPitch_W());
		double dR32dPsi = 0;

		// R33=cos(psi)*cos(theta)
        double dR33dPhi = -sin(bs->getRoll_W()) * cos(bs->getPitch_W());
        double dR33dTheta = -cos(bs->getRoll_W()) * sin(bs->getPitch_W());
		double dR33dPsi = 0;

		dRdPhi << dR11dPhi, dR12dPhi, dR13dPhi, dR21dPhi, dR22dPhi, dR23dPhi, dR31dPhi, dR32dPhi, dR33dPhi;

		dRdTheta << dR11dTheta, dR12dTheta, dR13dTheta, dR21dTheta, dR22dTheta, dR23dTheta, dR31dTheta, dR32dTheta, dR33dTheta;

		dRdPsi << dR11dPsi, dR12dPsi, dR13dPsi, dR21dPsi, dR22dPsi, dR23dPsi, dR31dPsi, dR32dPsi, dR33dPsi;

		M1 = dRdPhi * Xf11 + dRdTheta * Xf12 + dRdPsi * Xf13;
		M2 = dRdPhi * Xf21 + dRdTheta * Xf22 + dRdPsi * Xf23;
		M3 = dRdPhi * Xf31 + dRdTheta * Xf32 + dRdPsi * Xf33;
		M4 = dRdPhi * Xf41 + dRdTheta * Xf42 + dRdPsi * Xf43;

		Eigen::Matrix<double, 4, 1> count_to_variance;
		count_to_variance.setZero();

		if (support_legs[dog::LF]) {
			Xb1_est_HF = -(R * (*JFootLF).block<3,3>(rbd::LX,0)
					* qd_.block<3, 1>(0, 0) + M1 * Trunk_angles_dot);
			count_to_variance(0) = 1.0;
			estimTesZd(0) = Xb1_est_HF(2) * count_to_variance(0);
		}
		if (support_legs[dog::RF]) {
			Xb2_est_HF = -(R * (*JFootRF).block<3,3>(rbd::LX,0)
					* qd_.block<3, 1>(3, 0) + M2 * Trunk_angles_dot);
			count_to_variance(1) = 1.0;
			estimTesZd(1) = Xb2_est_HF(2) * count_to_variance(1);
		}

		if (support_legs[dog::LH]) {
			Xb3_est_HF = -(R * (*JFootLH).block<3,3>(rbd::LX,0)
					* qd_.block<3, 1>(6, 0) + M3 * Trunk_angles_dot);
			count_to_variance(2) = 1.0;
			estimTesZd(2) = Xb3_est_HF(2) * count_to_variance(2);
		}
		if (support_legs[dog::RH]) {
			Xb4_est_HF = -(R * (*JFootRH).block<3,3>(rbd::LX,0)
					* qd_.block<3, 1>(9, 0) + M4 * Trunk_angles_dot);
			count_to_variance(3) = 1.0;
			estimTesZd(3) = Xb4_est_HF(2) * count_to_variance(3);
		}

		if ((count_to_variance(0) + count_to_variance(1) + count_to_variance(2)
				+ count_to_variance(3)) >= 2) {
			average_coef = 1
					/ (count_to_variance(0) + count_to_variance(1)
							+ count_to_variance(2) + count_to_variance(3));

			Eigen::Matrix<double, 3, 1> Xb_est_HF_KinIMU_to_filter;
			Xb_est_HF_KinIMU_to_filter = average_coef
					* (count_to_variance(0) * Xb1_est_HF
							+ count_to_variance(1) * Xb2_est_HF
							+ count_to_variance(2) * Xb3_est_HF
							+ count_to_variance(3) * Xb4_est_HF);

			varianceEst = average_coef
					* (count_to_variance(0)
							* (Xb_est_HF_KinIMU_to_filter(2) - Xb1_est_HF(2))
							* (Xb_est_HF_KinIMU_to_filter(2) - Xb1_est_HF(2))
							+ count_to_variance(1)
									* (Xb_est_HF_KinIMU_to_filter(2)
											- Xb2_est_HF(2))
									* (Xb_est_HF_KinIMU_to_filter(2)
											- Xb2_est_HF(2))
							+ count_to_variance(2)
									* (Xb_est_HF_KinIMU_to_filter(2)
											- Xb3_est_HF(2))
									* (Xb_est_HF_KinIMU_to_filter(2)
											- Xb3_est_HF(2))
							+ count_to_variance(3)
									* (Xb_est_HF_KinIMU_to_filter(2)
											- Xb4_est_HF(2))
									* (Xb_est_HF_KinIMU_to_filter(2)
											- Xb4_est_HF(2)));

			//Filtering according t variance
			double alpha_est = 1 / (double) taskServoRate
					/ (0.001 + 1 / (double) taskServoRate);
			Xb_est_HF_KinIMU = (1 - alpha_est) * Xb_est_HF_KinIMU
					+ alpha_est * Xb_est_HF_KinIMU_to_filter;

		} else {
			Xb_est_HF_KinIMU(2) -= 9.81 * (1 / (double) taskServoRate);
		}

		Xb_est_HF_aux = Xb_est_HF_KinIMU;

	} else {
		Xb_est_HF_KinIMU(2) -= 9.81 * (1 / (double) taskServoRate);
		Xb_est_HF_aux = Xb_est_HF_KinIMU;
	}

}


///*********************************************************************************************///

void RCFController::computeBodyHeight(const rbd::Vector3d angles,
		const iit::dog::LegDataMap<rbd::Vector3d>& foot_pos,
		const rbd::Vector3d& trunk_velocities,
		const iit::dog::LegDataMap<bool>& support_legs, double& trunk_height) {

	int roll_index = 0;
	int pitch_index = 1;
	rbd::Vector3d trunk_pos_LF = rbd::Vector3d().setZero();
	rbd::Vector3d trunk_pos_RF = rbd::Vector3d().setZero();
	rbd::Vector3d trunk_pos_LH = rbd::Vector3d().setZero();
	rbd::Vector3d trunk_pos_RH = rbd::Vector3d().setZero();
	static rbd::Vector3d trunk_pos_LF_memory = rbd::Vector3d().setZero();
	static rbd::Vector3d trunk_pos_RF_memory = rbd::Vector3d().setZero();
	static rbd::Vector3d trunk_pos_LH_memory = rbd::Vector3d().setZero();
	static rbd::Vector3d trunk_pos_RH_memory = rbd::Vector3d().setZero();
	rbd::Vector3d trunk_pos = Eigen::Matrix<double, 3, 1>::Zero();
	static double lift_off_trunk_velocity_z = 0;
	static double lift_off_pos_z = 0;
	static double lift_off_time = 0;

	double n_stance_legs = 0;
	if (support_legs[dog::LF]) {
		n_stance_legs++;
	}
	if (support_legs[dog::RF]) {
		n_stance_legs++;
	}
	if (support_legs[dog::LH]) {
		n_stance_legs++;
	}
	if (support_legs[dog::RH]) {
		n_stance_legs++;
	}

	if (n_stance_legs > 1) {

		rbd::Matrix33d R;
		R(0, 0) = cos(angles(roll_index));
		R(0, 1) = sin(angles(pitch_index)) * sin(angles(roll_index));
		R(0, 2) = cos(angles(roll_index)) * sin(angles(pitch_index));
		R(1, 0) = 0.0;
		R(1, 1) = cos(angles(roll_index));
		R(1, 2) = -sin(angles(roll_index));
		R(2, 0) = -sin(angles(pitch_index));
		R(2, 1) = cos(angles(pitch_index)) * sin(angles(roll_index));
		R(2, 2) = cos(angles(pitch_index)) * cos(angles(roll_index));

		if (support_legs[dog::LF]) {
			trunk_pos_LF = -R * foot_pos[dog::LF];
			trunk_pos_LF_memory = trunk_pos_LF;
		} else {
			trunk_pos_LF.setZero();
		}

		if (support_legs[dog::RF]) {
			trunk_pos_RF = -R * foot_pos[dog::RF];
			trunk_pos_RF_memory = trunk_pos_RF;
		} else {
			trunk_pos_RF.setZero();
		}

		if (support_legs[dog::LH]) {
			trunk_pos_LH = -R * foot_pos[dog::LH];
			trunk_pos_LH_memory = trunk_pos_LH;
		} else {
			trunk_pos_LH.setZero();
		}

		if (support_legs[dog::RH]) {
			trunk_pos_RH = -R * foot_pos[dog::RH];
			trunk_pos_RH_memory = trunk_pos_RH;
		} else {
			trunk_pos_RH.setZero();
		}

		trunk_pos = (trunk_pos_LF + trunk_pos_RF + trunk_pos_LH + trunk_pos_RH)
				/ n_stance_legs;


		lift_off_trunk_velocity_z = trunk_velocities(2);
		lift_off_pos_z = trunk_pos(2);
		lift_off_time = 0;

		//trunk_height = trunk_pos(2);
		trunk_height = (trunk_pos_LF_memory(rbd::Z) + trunk_pos_RF_memory(rbd::Z) +
				        trunk_pos_LH_memory(rbd::Z) + trunk_pos_RH_memory(rbd::Z))/4;

	} else {

		lift_off_time += 1 / (double) taskServoRate;
		trunk_height = lift_off_pos_z
				+ lift_off_trunk_velocity_z * lift_off_time
                + bs->getAcceleration_B()[2] * lift_off_time * lift_off_time / 2;

	}
}

///*********************************************************************************************///

void RCFController::stepCycleMeasurement(const double& foot_force,
                                         const double& task_freq,
                                         const double& task_time,
                                         const double& CPG_primite_LF_z,
                                         const iit::dog::LegDataMap<bool> support_leg,
                                         int& flight_phase,
                                         double& flight_phase_time,
                                         double& CPG_duty_factor,
                                         double& actual_duty_factor,
                                         double& actual_step_freq,
                                         double& actual_stance_per) {



	step_period += 1 / task_freq;

	if (foot_force > force_threshold && !leg_touch_down) {
		if (step_period > 0.0)
			actual_step_freq = 1 / step_period;

		step_period = 0.0;
		leg_touch_down = true;
	}

	if (foot_force < force_threshold && leg_touch_down) {
		leg_touch_down = false;
		actual_stance_per = step_period;
	}

	//Flight phase check
	flight_phase = 2;

	if (support_leg[dog::LF] || support_leg[dog::RF] || support_leg[dog::LH]
			|| support_leg[dog::RH]) {
		flight_phase = 0;
	}

	if (dutyF < 0.5) {

        if ((flight_phase != flight_phase_last) && flight_phase == 2) {
            strideTime = task_time - flight_phase_starting_time;

			if (strideTime != 0) {
				actual_duty_factor = 100 * (strideTime - flight_phase_time) / 2
						/ strideTime;
			}

			flight_phase_starting_time = task_time;

			flight_phase_last = flight_phase; //reseting memory flag
		}

		if ((flight_phase != flight_phase_last) && flight_phase == 0) {
			flight_phase_ending_time = task_time;

            // flight_phase_time = task_time - 0
			flight_phase_time = flight_phase_ending_time
					- flight_phase_starting_time;

            // flight_phase = flight_phase... but we don't care because
            // you reset flight_phase_last every time you call this function!!
			flight_phase_last = flight_phase;
		}

		//CPG check
		CPG_swing_phase = true;

		if (CPG_primite_LF_z < 0)
			CPG_swing_phase = false;

		if ((CPG_swing_phase != CPG_swing_phase_last) && CPG_swing_phase) {

			CPG_cicle_time = task_time - CPG_swing_phase_starting_time;

			if (CPG_cicle_time != 0) {
				CPG_duty_factor = 100 * (CPG_cicle_time - CPG_swing_phase_time)
						/ CPG_cicle_time;
			}

			CPG_swing_phase_starting_time = task_time;

			CPG_swing_phase_last = CPG_swing_phase; //reseting memory flag
		}

		if ((CPG_swing_phase != CPG_swing_phase_last) && !CPG_swing_phase) {

			CPG_swing_phase_time = task_time - CPG_swing_phase_starting_time;

			CPG_swing_phase_last = CPG_swing_phase;
		}

	}

}

///*********************************************************************************************///

//void RCFController::CPG_Modulation(LegDataMap<rbd::Vector3d>& primitives,
//		LegDataMap<Eigen::Vector3d>& primitives_d,
//		LegDataMap<Eigen::Vector3d>& output_filter,
//		LegDataMap<Eigen::Vector3d>& output_filter_d) {
//
//	static LegDataMap<bool> flag_td = false; //used for adaptive walk
//	LegDataMap<double> couplingTerm;
//	LegDataMap<Eigen::Vector3d> V_translation;
//	LegDataMap<Eigen::Vector3d> V_rotation;
//	LegDataMap<Eigen::Vector3d> P_local_HF;
//	LegDataMap<double> roll_p(0.0);
//	LegDataMap<double> pitch_p(0.0);
//	LegDataMap<double> yaw_p(0.0);
//
//	for (int leg = dog::LF; leg <= dog::RH; leg++) {
//
//		//Assign oscillators parameters
//		CPG[leg].set_bp(Kbp);
//		CPG[leg].set_bf(Kbf);
//		CPG[leg].set_bv(Kbv);
//		CPG[leg].set_kvf(Kvf);
//		CPG[leg].set_alpha(alphaGamma);
//		CPG[leg].set_gamma(alphaGamma);
//
//		//Compute angles for the primitive rotation matrix R_p (this matrix rotates the CPG primitive)
//		yaw_p[leg] = atan2(VfY, VfX);
//
//		//Rotation matrix to transform from the base frame to the horizontal frame
//		P_Local_HF[leg] = commons::rpyToRot(
//				Eigen::Vector3d(bs->getRoll_W(), bs->getPitch_W(), 0.0)).transpose()
//				* P_Local_BF[leg];
//
//		//Creating relative velocity vector for each foot considering rotational and translational motion.
//		V_translation[leg] << VfX, VfY, 0;
//		V_rotation[leg] << P_Local_HF[leg](rbd::Y) * Psit, -P_Local_HF[leg](
//				rbd::X) * Psit, 0;
//
//		//Compute the coupling term to create the synchronism
//		couplingTerm[leg] = 0.0;
//		for (int legCoup = dog::LF; legCoup <= dog::RH; legCoup++)
//			couplingTerm[leg] += (CPG[leg].Hs / CPG[legCoup].Hs)
//					* couplingMatrix((int)pattern)(leg, legCoup)
//					* CPG[legCoup].Xp(rbd::Z);
//
//		//Assign locomotion parameters
//		CPG[leg].setStepHeight(stepHeight);
//		CPG[leg].setStepLength(stepLength);
//		CPG[leg].setDutyFactor(dutyF);
//		//CPG[leg].setAbsoluteVelocity( sqrt(Vf_x * Vf_x + Vf_y * Vf_y) );
//		CPG[leg].setAbsoluteVelocity(forwVel);
//		CPG[leg].setVelocity(V_translation[leg] - V_rotation[leg]);
//		CPG[leg].setRotationMatrix(roll_p[leg], pitch_p[leg], yaw_p[leg]);
//		CPG[leg].deltaOrigin << deltaX(leg), deltaY(leg), 0; //relative displacements from the push recovery block
//		CPG[leg].couplingTerm = couplingTerm[leg];
//
//	}
//
//	//Assigning step frequency for monitoring with "wst" command.
//	if (forwVel != 0.0) {
//		stepFrequency = forwVel / stepLength * dutyF;
//	} else {
//		stepFrequency = 999999;
//	}
//
//	//Adaptive walk
//	if (walkAdaptationFlag) {
//
//		for (int leg = dog::LF; leg <= dog::RH; leg++) {
//			std::cout << "Got here, leg " << leg << std::endl;
//			if (footSensor->force[leg][2] > 50 && CPG[leg].Xp(rbd::Z) <= 0.07
//					&& CPG[leg].Xp(rbd::X) > 0.0 && !flag_td[leg]) {
//
//				CPG[leg].z_td = CPG[leg].Xp(rbd::Z);
//				CPG[leg].z_error_td = P_DesLocal_HF[leg](rbd::Z)
//						- P_Local_HF[leg](rbd::Z);
//
//				if (CPG[leg].Xp(rbd::Z) < -0.05)
//					CPG[leg].z_td = CPG[leg].Xp(rbd::Z) - 0.005;
//				flag_td[leg] = true;
//
//				if (!adapWalkFlag[leg]) { //This command is to return from the adaptive mode without abrupt motion.
//					CPG[leg].z_td = 0.0;
//					adapWalkFlag[leg] = true;
//				}
//
//			}
//
//			if (CPG[leg].Xp(rbd::Z) <= -0.04 && CPG[leg].Xp(rbd::X) > 0.0
//					&& !flag_td[leg]) {
//
//				CPG[leg].z_td = CPG[leg].Xp(rbd::Z) - 0.005;
//				CPG[leg].z_error_td = P_DesLocal_HF[leg](rbd::Z)
//						- P_Local_HF[leg](rbd::Z);
//				flag_td[leg] = true;
//
//				if (!adapWalkFlag[leg]) {
//					CPG[leg].z_td = 0.0;
//					CPG[leg].z_error_td = 0.0;
//					adapWalkFlag[leg] = true;
//				}
//			}
//
//			if (CPG[leg].Xp(rbd::Z) > CPG[leg].z_td / 10
//					&& CPG[leg].Xp(rbd::X) < 0.0 && flag_td[leg]) {
//				CPG[leg].z_td = -2 * CPG[leg].Hs;
//				flag_td[leg] = false;
//			}
//
//		}
//
//	} else {
//
//		for (int leg = dog::LF; leg <= dog::RH; leg++) {
//			if (CPG[leg].Xp(rbd::Z) > 0.0) {
//				CPG[leg].z_td = 0.0;
//				CPG[leg].z_error_td = 0.0;
//				flag_td[leg] = false;
//			}
//		}
//
//	}
//
//	//CPG Integration
//	for (int leg = dog::LF; leg <= dog::RH; leg++) {
//		CPG[leg].computeCPG();
//		output_filter[leg] = CPG[leg].Xf;
//		output_filter_d[leg] = CPG[leg].dXf;
//		primitives[leg] = CPG[leg].Xp;
//		primitives_d[leg] = CPG[leg].dXp;
//	}
//
//}


void RCFController:: CPG_Modulation(dog::LegDataMap<rbd::Vector3d>& primitives,
        dog::LegDataMap<rbd::Vector3d>& primitives_d,
        dog::LegDataMap<rbd::Vector3d>& output_filter,
        dog::LegDataMap<rbd::Vector3d>& output_filter_d) {

    static dog::LegDataMap<bool> flag_td = false; //used for adaptive walk
    dog::LegDataMap<double> couplingTerm;
    dog::LegDataMap<Eigen::Vector3d> V_translation;
    dog::LegDataMap<Eigen::Vector3d> V_rotation;
    dog::LegDataMap<Eigen::Vector3d> P_local_HF;
    dog::LegDataMap<double> roll_p(0.0);
    dog::LegDataMap<double> pitch_p(0.0);
    dog::LegDataMap<double> yaw_p(0.0);

		// if (RCFController::walkStarting::walkingFlag == true){
		// 	myTime = 0;
		// }
		/*ADDED BY OCTAVIO */
		eventsLog = schedule.updatefutureevents(eventsLog,myTime);

		for (int i = 0; i < 4; i++)
    {
      if (myTime >= eventsLog(0,i) && myTime < eventsLog(1,i + 4))
      {
        omegaVector[i] = M_PI/(double)(eventsLog(1,i + 4) - eventsLog(0,i)); // Stance phase
      }
      else if (myTime >= eventsLog(1,i + 4) && myTime <= eventsLog(1,i))
      {
        omegaVector[i] = M_PI/(double)(eventsLog(1,i) - eventsLog(1,i + 4)); // Swing phase
      }
      else
      {
        omegaVector[i] = M_PI/(double)(eventsLog(2,i + 4) - eventsLog(1,i));
      }
    }

    for (int leg = dog::LF; leg <= dog::RH; leg++) {

        //Assign oscillators parameters
        CPG[leg].set_bp(Kbp);
        CPG[leg].set_bf(Kbf);
        CPG[leg].set_bv(Kbv);
        CPG[leg].set_kvf(Kvf);
        CPG[leg].set_alpha(alphaGamma);
        CPG[leg].set_gamma(alphaGamma);

        //Compute angles for the primitive rotation matrix R_p (this matrix rotates the CPG primitive)
        yaw_p[leg] = atan2(VfY, VfX);

        //Rotation matrix to transform from the base frame to the horizontal frame
        P_local_HF[leg] = commons::rpyToRot(
                Eigen::Vector3d(bs->getRoll_W(), bs->getPitch_W(), 0.0)).transpose()
                * P_Local_BF[leg];

        //Creating relative velocity vector for each foot considering rotational and translational motion.
        V_translation[leg] << VfX, VfY, 0;
        V_rotation[leg] << P_local_HF[leg](rbd::Y) * Psit, -P_local_HF[leg](rbd::X) * Psit, 0;

        //Compute the coupling term to create the synchronism
        // couplingTerm[leg] = 0.0;
        // for (int legCoup = dog::LF; legCoup <= dog::RH; legCoup++)
        //     couplingTerm[leg] += (CPG[leg].Hs / CPG[legCoup].Hs)
        //             * couplingMatrix((int)pattern)(leg, legCoup)
        //             * CPG[legCoup].Xp(rbd::Z);

        //Assign locomotion parameters
        CPG[leg].setStepHeight(stepHeight);
        CPG[leg].setStepLength(stepLength);
        // CPG[leg].setDutyFactor(dutyF); /*OCTAVIO*/
        //CPG[leg].setAbsoluteVelocity( sqrt(Vf_x * Vf_x + Vf_y * Vf_y) );
        CPG[leg].setAbsoluteVelocity(forwVel);
        CPG[leg].setVelocity(V_translation[leg] - V_rotation[leg]);
        CPG[leg].setRotationMatrix(roll_p[leg], pitch_p[leg], yaw_p[leg]);
        //CPG[leg].deltaOrigin << DeltaX(leg), DeltaY(leg), 0; //relative displacements from the push recovery block
        CPG[leg].deltaOrigin << 0.0, 0.0, 0.0; //Done outside this function
        // CPG[leg].couplingTerm = Kcoup * couplingTerm[leg]; /* OCTAVIO */

				// THIS IS DONE BY OCTAVIO, ADDING MAXPLUS
				CPG[leg].set_omega(omegaVector[leg]);


    }

    //Assigning step frequency for monitoring with "wst" command.
    if (forwVel != 0.0) {
        stepFrequency = forwVel / stepLength * dutyF;
    } else {
        stepFrequency = 999999;
    }

    //Adaptive walk
    if (walkAdaptationFlag) {

        for (int leg = dog::LF; leg <= dog::RH; leg++) {
            if (footSensor->force[leg](rbd::Z) > 50 && CPG[leg].Xp(rbd::Z) <= 0.7*CPG[leg].Hs
                    && CPG[leg].Xp(rbd::X) > 0.0 && !flag_td[leg]) {

                CPG[leg].z_td = CPG[leg].Xp(rbd::Z);
                CPG[leg].z_error_td = P_DesLocal_HF[leg](rbd::Z)
                        - P_local_HF[leg](rbd::Z);

                if (CPG[leg].Xp(rbd::Z) < -0.05)
                    CPG[leg].z_td = CPG[leg].Xp(rbd::Z) - 0.005;
                flag_td[leg] = true;

                if (!adapWalkFlag[leg]) { //This command is to return from the adaptive mode without abrupt motion.
                    CPG[leg].z_td = 0.0;
                    adapWalkFlag[leg] = true;
                }

            }

            if (CPG[leg].Xp(rbd::Z) <= -0.4*CPG[leg].Hs && CPG[leg].Xp(rbd::X) > 0.0
                    && !flag_td[leg]) {

                CPG[leg].z_td = CPG[leg].Xp(rbd::Z) - 0.005;
                CPG[leg].z_error_td = P_DesLocal_HF[leg](rbd::Z)
                        - P_local_HF[leg](rbd::Z);
                flag_td[leg] = true;

                if (!adapWalkFlag[leg]) {
                    CPG[leg].z_td = 0.0;
                    CPG[leg].z_error_td = 0.0;
                    adapWalkFlag[leg] = true;
                }
            }

            if (CPG[leg].Xp(rbd::Z) > CPG[leg].z_td / 10
                    && CPG[leg].Xp(rbd::X) < 0.0 && flag_td[leg]) {
                CPG[leg].z_td = -2 * CPG[leg].Hs;
                flag_td[leg] = false;
            }

        }

    } else {

        for (int leg = dog::LF; leg <= dog::RH; leg++) {
            if (CPG[leg].Xp(rbd::Z) > 0.0) {
                CPG[leg].z_td = 0.0;
                CPG[leg].z_error_td = 0.0;
                flag_td[leg] = false;
            }
        }

    }

    //CPG Integration
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        CPG[leg].computeCPG();
        output_filter[leg] = CPG[leg].Xf;
        output_filter_d[leg] = CPG[leg].dXf;
        primitives[leg] = CPG[leg].Xp;
        primitives_d[leg] = CPG[leg].dXp;
    }

}




///*********************************************************************************************///

void RCFController::terrainAdjustment(double terrainRollAdj,
		double terrainPitchAdj, double& deltaRoll, double& deltaPitch) {

	deltaRoll = 0.0;
	deltaPitch = 0.0;
    double maxPitch = 5 * M_PI / 180;
    double maxRoll = 5 * M_PI / 180;

	if (terrainRollAdj > maxRoll) {
		deltaRoll = terrainRollAdj - maxRoll;
		terrainRollAdj = maxRoll;
	}

	if (terrainRollAdj < -maxRoll) {
		deltaRoll = terrainRollAdj - (-maxRoll);
		terrainRollAdj = -maxRoll;
	}

	if (terrainPitchAdj > maxPitch) {
		deltaPitch = terrainPitchAdj - maxPitch;
		terrainPitchAdj = maxPitch;
	}

	if (terrainPitchAdj < -maxPitch) {
		deltaPitch = terrainPitchAdj - (-maxPitch);
		terrainPitchAdj = -maxPitch;
	}

	rbd::Matrix33d terrainRotMatrix;

	terrainRotMatrix(0, 0) = cos(terrainPitchAdj - deltaPitch);
	terrainRotMatrix(0, 1) = sin(terrainPitchAdj - deltaPitch)
			* sin(terrainRollAdj - deltaRoll);
	terrainRotMatrix(0, 2) = cos(terrainRollAdj - deltaRoll)
			* sin(terrainPitchAdj - deltaPitch);

	terrainRotMatrix(1, 0) = 0.0;
	terrainRotMatrix(1, 1) = cos(terrainRollAdj - deltaRoll);
	terrainRotMatrix(1, 2) = -sin(terrainRollAdj - deltaRoll);

	terrainRotMatrix(2, 0) = -sin(terrainPitchAdj - deltaPitch);
	terrainRotMatrix(2, 1) = cos(terrainPitchAdj - deltaPitch)
			* sin(terrainRollAdj - deltaRoll);
	terrainRotMatrix(2, 2) = cos(terrainPitchAdj - deltaPitch)
			* cos(terrainRollAdj - deltaRoll);

	//Rotating ellipses to fit the terrain
	P_DesFoot[dog::LF] = terrainRotMatrix * WCPG[dog::LF];
	PdDesFoot[dog::LF] = terrainRotMatrix * dWCPG[dog::LF];

	P_DesFoot[dog::RF] = terrainRotMatrix * WCPG[dog::RF];
	PdDesFoot[dog::RF] = terrainRotMatrix * dWCPG[dog::RF];

	P_DesFoot[dog::LH] = terrainRotMatrix * WCPG[dog::LH];
	PdDesFoot[dog::LH] = terrainRotMatrix * dWCPG[dog::LH];

	P_DesFoot[dog::RH] = terrainRotMatrix * WCPG[dog::RH];
	PdDesFoot[dog::RH] = terrainRotMatrix * dWCPG[dog::RH];

	//Changing the origins to fit the terrain
	Eigen::Matrix<double, 3, 3> R;
	Eigen::Matrix<double, 3, 1> P01;
	Eigen::Matrix<double, 3, 1> P02;
	Eigen::Matrix<double, 3, 1> P03;
	Eigen::Matrix<double, 3, 1> P04;
	Eigen::Matrix<double, 3, 1> P0Z;
	double Pz01, Pz02, Pz03, Pz04;

	//First adjustment with kept posture.

	P01 << P0_Local_HF[dog::LF](rbd::X), P0_Local_HF[dog::LF](rbd::Y), 0;
	P02 << P0_Local_HF[dog::RF](rbd::X), P0_Local_HF[dog::RF](rbd::Y), 0;
	P03 << P0_Local_HF[dog::LH](rbd::X), P0_Local_HF[dog::LH](rbd::Y), 0;
	P04 << P0_Local_HF[dog::RH](rbd::X), P0_Local_HF[dog::RH](rbd::Y), 0;
	P0Z.setZero();

	R(0, 0) = cos(terrainPitchAdj);
	R(0, 1) = sin(terrainPitchAdj) * sin(terrainRollAdj);
	R(0, 2) = cos(terrainRollAdj) * sin(terrainPitchAdj);

	R(1, 0) = 0.0;
	R(1, 1) = cos(terrainRollAdj);
	R(1, 2) = -sin(terrainRollAdj);

	R(2, 0) = -sin(terrainPitchAdj);
	R(2, 1) = cos(terrainPitchAdj) * sin(terrainRollAdj);
	R(2, 2) = cos(terrainPitchAdj) * cos(terrainRollAdj);

	P01 = R * P01;
	P02 = R * P02;
	P03 = R * P03;
	P04 = R * P04;
	Pz01 = P01(rbd::Z);
	Pz02 = P02(rbd::Z);
	Pz03 = P03(rbd::Z);
	Pz04 = P04(rbd::Z);

	//Second adjustment.
	R(0, 0) = cos(-deltaPitch);
	R(0, 1) = sin(-deltaPitch) * sin(-deltaRoll);
	R(0, 2) = cos(-deltaRoll) * sin(-deltaPitch);

	R(1, 0) = 0.0;
	R(1, 1) = cos(-deltaRoll);
	R(1, 2) = -sin(-deltaRoll);

	R(2, 0) = -sin(-deltaPitch);
	R(2, 1) = cos(-deltaPitch) * sin(-deltaRoll);
	R(2, 2) = cos(-deltaPitch) * cos(-deltaRoll);

	P01 << P0_Local_HF[dog::LF](rbd::X), P0_Local_HF[dog::LF](rbd::Y), Pz01;
	P02 << P0_Local_HF[dog::RF](rbd::X), P0_Local_HF[dog::RF](rbd::Y), Pz02;
	P03 << P0_Local_HF[dog::LH](rbd::X), P0_Local_HF[dog::LH](rbd::Y), Pz03;
	P04 << P0_Local_HF[dog::RH](rbd::X), P0_Local_HF[dog::RH](rbd::Y), Pz04;
	P0Z << 0, 0, P0_Local_HF[dog::RH](rbd::Z);

	P01 = R * P01;
	P02 = R * P02;
	P03 = R * P03;
	P04 = R * P04;
	P0Z = R * P0Z;

	P_DesFoot[dog::LF](rbd::X) = P_DesFoot[dog::LF](rbd::X) + P0Z(rbd::X);
	P_DesFoot[dog::RF](rbd::X) = P_DesFoot[dog::RF](rbd::X) + P0Z(rbd::X);
	P_DesFoot[dog::LH](rbd::X) = P_DesFoot[dog::LH](rbd::X) + P0Z(rbd::X);
	P_DesFoot[dog::RH](rbd::X) = P_DesFoot[dog::RH](rbd::X) + P0Z(rbd::X);

	P_DesFoot[dog::LF](rbd::Y) = P_DesFoot[dog::LF](rbd::Y) + P0Z(rbd::Y);
	P_DesFoot[dog::RF](rbd::Y) = P_DesFoot[dog::RF](rbd::Y) + P0Z(rbd::Y);
	P_DesFoot[dog::LH](rbd::Y) = P_DesFoot[dog::LH](rbd::Y) + P0Z(rbd::Y);
	P_DesFoot[dog::RH](rbd::Y) = P_DesFoot[dog::RH](rbd::Y) + P0Z(rbd::Y);

	P_DesFoot[dog::LF](rbd::Z) = P_DesFoot[dog::LF](rbd::Z) + Pz01;
	P_DesFoot[dog::RF](rbd::Z) = P_DesFoot[dog::RF](rbd::Z) + Pz02;
	P_DesFoot[dog::LH](rbd::Z) = P_DesFoot[dog::LH](rbd::Z) + Pz03;
	P_DesFoot[dog::RH](rbd::Z) = P_DesFoot[dog::RH](rbd::Z) + Pz04;

}

///*********************************************************************************************///

double RCFController::sinusoidalMotion(bool on_off, const double desAmplitude,
	const double desFrequency, const double pushUpTime) {

	double alphaFilter = 1 / (double) taskServoRate
			/ (0.5 + 1 / (double) taskServoRate);
	static double amplitude = 0.0;
	static double amplitude2 = 0.0;
	static double amplitudeNew = 0.0;
	static double pushUpFrequencyNew = 0.0;

	if (on_off) {
		amplitudeNew = desAmplitude;
		pushUpFrequencyNew = desFrequency;
	}else{
		amplitudeNew = 0.0;
	}

	amplitude = secondOrderFilter(amplitude, amplitude2, amplitudeNew, alphaFilter);

	return amplitude*cos(2 * M_PI * pushUpFrequencyNew * pushUpTime);
}


///*********************************************************************************************///

double RCFController::sinusoidalForce(bool on_off, const double desForceAmplitude,
    const double desForceFrequency, const double forceTime) {

    double alphaFilter = 1 / (double) taskServoRate
            / (0.5 + 1 / (double) taskServoRate);
    static double amplitude = 0.0;
    static double amplitude2 = 0.0;
    static double amplitudeNew = 0.0;
    static double forceFrequencyNew = 0.0;

    if (on_off) {
        amplitudeNew = desForceAmplitude;
        forceFrequencyNew = desForceFrequency;
    }else{
        amplitudeNew = 0.0;
    }

    amplitude = secondOrderFilter(amplitude, amplitude2, amplitudeNew, alphaFilter);

    return amplitude*cos(2 * M_PI * forceFrequencyNew * forceTime);
}

///*********************************************************************************************///

double RCFController::chirpForce(const double& chirpTime) {

    return chirpAmplitude * sin(2*3.1415*((initialChirpFrequency + chirpRate/2*chirpTime)*chirpTime));
}


///*********************************************************************************************///

double RCFController::squareForce(const double& squareTime){

    static double alpha = 0.004/(0.004 + 0.05);

    if(squareTime > squareWavePeriod/2){
        if(squareWaveOutput==0){
            squareWaveOutput = squareWaveAmplitude;
        }
        else{

            squareWaveOutput = -squareWaveOutput;
        }

        squareTime0 = myTime;
    }

    squareWaveOutputFiltered = (1.0 - alpha) * squareWaveOutputFiltered + alpha * squareWaveOutput;

    return squareWaveOutputFiltered;
}


///*********************************************************************************************///

double RCFController::triangularForce(const double& triangularTime){

    double x = 2.0*triangularTime/triangularWavePeriod + 1.0/2.0;

    int fl = floor(x);

    double minusOnePower = 1;

    for(int i = 0; i <= fl; i++){
        minusOnePower = -1.0 * minusOnePower;
    }

    //triangularWaveOutput = 1 / triangularWavePeriod * (triangularTime - triangularWavePeriod*fl/2)*(-1.0)^fl;
    triangularWaveOutput = 4 * 1 / triangularWavePeriod * (triangularTime - triangularWavePeriod*fl/2.0)*minusOnePower;

    cout <<  triangularWaveOutput << endl;
    triangularWaveOutput = triangularWaveOutput * triangularWaveAmplitude;


    return triangularWaveOutput;
}

///*********************************************************************************************///

double RCFController::sineForce(const double& sineTime){


    sineWaveOutput = sineWaveAmplitude * sin(2 * 3.1415 * sineTime / sineWavePeriod);


    return sineWaveOutput;
}


///*********************************************************************************************///

void RCFController::computeDesTrunkWrench() {

	//Resetting variables
	for (int i = 1; i <= nDOF; i++)
		tauDeltaPos[i] = 0.0;

	for (int i = 1; i <= 3; i++) {
		forceDeltaPos[i] = 0.0;
		tauDeltaOrient[i] = 0.0;
	}

    PsitError = Psit - bs->getRotationRate_B()[Yaw];

	//To inject energy to keep the trunk height during flight phase
	double Delta_Force = 0.0;
	if ((WCPGb[dog::LF](rbd::Z) < -0.1 * stepHeight)
			&& (WCPGb[dog::LF](rbd::X) < 0)
			&& (WCPGb[dog::RH](rbd::Z) < -0.1 * stepHeight)
			&& (WCPGb[dog::RH](rbd::X) < 0) && (estBodyVelHF(rbd::Z) > 0))
		Delta_Force = D_Force;
	if ((WCPGb[dog::RF](rbd::Z) < -0.1 * stepHeight)
			&& (WCPGb[dog::RF](rbd::X) < 0)
			&& (WCPGb[dog::LH](rbd::Z) < -0.1 * stepHeight)
			&& (WCPGb[dog::LH](rbd::X) < 0) && (estBodyVelHF(rbd::Z) > 0))
		Delta_Force = D_Force;

	double roll_integrated_error = 0.0;
	double pitch_integrated_error = 0.0;
	double KiTrunkRoll = 100.0;
	double KiTrunkPitch = 100.0;
	//static double minTrunk_Fz = 15.0; //It was 150 N

	desTrunkHeight = -(P0_Local_HF[dog::LF](rbd::Z) + P0_Local_HF[dog::RF](rbd::Z)
			+ P0_Local_HF[dog::LH](rbd::Z) + P0_Local_HF[dog::RH](rbd::Z)) / 4;

    virtualLegPosition = computeVirtualLegPosition(P_Local_HF);
    desVirtualLegPosition = computeDesVirtualLegPosition(P_DesLocal_HF);

    desVirtualLegPosition(rbd::Z) = -desTrunkHeight;

	if (integralAction) {
        roll_integrated_error += eulerAngles(0) + rollAdj - bs->getRoll_W();
        pitch_integrated_error += eulerAngles(1) + pitchAdj - bs->getPitch_W();
		integralActionRoll = KiTrunkRoll * roll_integrated_error;
		integralActionPitch = KiTrunkPitch * pitch_integrated_error;
	}

    //Compute robot positioning error in the horizontal frame
    robotPositionErrorHF = desRobotPositionHF - actualRobotPositionHF;

    //Computing heading error
	double yawError = desRobotYaw - robotYaw;
	if(yawError > 45*3.14/180){yawError = 45*3.14/180;}
	if(yawError < -45*3.14/180){yawError = -45*3.14/180;}

	//Computing forces to correct robot position error in X
	Eigen::Matrix<double, 3, 1> positionControlForce;
	positionControlForce(rbd::X) = KpTrunkX * robotPositionErrorHF[rbd::X];
	if (positionControlForce(rbd::X) > 200) positionControlForce(rbd::X) = 200.0; //Saturation
	if (positionControlForce(rbd::X) < -200) positionControlForce(rbd::X) = -200.0; //Saturation

	//Computing forces to correct robot position error in Y
	positionControlForce(rbd::Y) = KpTrunkY * robotPositionErrorHF[rbd::Y];
	if (positionControlForce(rbd::Y) > 200) positionControlForce(rbd::Y) = 200.0; //Saturation
	if (positionControlForce(rbd::Y) < -200) positionControlForce(rbd::Y) = -200.0; //Saturation

	//Computing forces to correct robot position error in Z
	positionControlForce(rbd::Z) = KpTrunkZ * (-desVirtualLegPosition(rbd::Z) + virtualLegPosition(rbd::Z));

    //Compute the desired forces and moments to be applied on the trunk.
	tauDeltaOrient[X] = + KpTrunkRoll * (eulerAngles(0) + rollAdj - bs->getRoll_W())
	                    - KdTrunkRoll * bs->getRotationRate_B()[Roll]
	                    + integralAction * integralActionRoll
	                    + xMomentOffset;

	tauDeltaOrient[Y] = + KpTrunkPitch * (eulerAngles(1) + pitchAdj - bs->getPitch_W())
	                    - KdTrunkPitch * bs->getRotationRate_B()[Pitch]
	                    + integralAction * integralActionPitch
	                    + yMomentOffset;

	tauDeltaOrient[Z] = + KpTrunkYaw * yawError
			            + KdTrunkYaw * (Psit - bs->getRotationRate_B()[Yaw])
			            + zMomentOffset;

	forceDeltaPos[X] =  + positionControlForce(rbd::X)//(0.0 - virtualLegPosition(rbd::X))
	                    + KdTrunkX * (VfX - estBodyVelHF(rbd::X))
	                    + xForceOffset;


	forceDeltaPos[Y] =  + positionControlForce(rbd::Y)//(0.0 - virtualLegPosition(rbd::Y))
	                    + KdTrunkY * (VfY - estBodyVelHF(rbd::Y))
	                    + yForceOffset;

	//If gravity compensation is not based on the model, consider manual gravity compensation
	//(i.e., set by the user)
	if(constrainedInvDynFlag) {
	    forceDeltaPos[Z] = + positionControlForce(rbd::Z)
	                       + KdTrunkZ * (0.0 - estBodyVelHF(rbd::Z))
	                       + Delta_Force
	                       + jumpForce
	                       + zForceOffset;

	}
	else {
	    forceDeltaPos[Z] = + positionControlForce(rbd::Z)
	                       + KdTrunkZ * (0.0 - estBodyVelHF(rbd::Z))
	                       + 9.81 * bodyWeight //manual gravity compensation
	                       + Delta_Force
	                       + jumpForce
	                       + zForceOffset;
	}

	// std::cout << (bs->getPosition_W()[0]) << " " << (bs->getPosition_W()[1]) <<
	// 	" " << tauDeltaOrient[Z] << " " << forceDeltaPos[X] << " " << forceDeltaPos[Y] << std::endl;
	// std::cout << "Rotation rate: " << bs->getRotationRate_B()[Roll]<< " "<< bs->getRotationRate_B()[Pitch]
	// 			<< " "<< bs->getRotationRate_B()[Yaw] << std::endl;
	//To mostly disable them
    //tauDeltaOrient[Z] = -1 * bs->getRotationRate_B()[Yaw];
    //tauDeltaOrient[X] = -1 * bs->getRotationRate_B()[Roll];
    //tauDeltaOrient[Y] = -1 * bs->getRotationRate_B()[Pitch];
	//tauDeltaOrient[Z] = 0;
	// forceDeltaPos[X] = 0;
	// forceDeltaPos[Y] = 0;
	// forceDeltaPos[Z] = 0;
	////to avoid weak feet contacts
	//if (forceDeltaPos[Z] < minTrunk_Fz)
	//	forceDeltaPos[Z] = minTrunk_Fz;

	//Interface between SL and Eigen definitions.
	Eigen::Matrix<double, 6, 1> aux_trunk_forces;
	Eigen::Matrix<double, 12, 1> aux_joint_torques;
	desTrunkWrench(0) = forceDeltaPos[X];
	desTrunkWrench(1) = forceDeltaPos[Y];
	desTrunkWrench(2) = forceDeltaPos[Z];
	desTrunkWrench(3) = tauDeltaOrient[X];
	desTrunkWrench(4) = tauDeltaOrient[Y];
	desTrunkWrench(5) = tauDeltaOrient[Z];

}

///*********************************************************************************************///

void RCFController::switchRunningTrotProfile(const Throttle speed) {

	unsigned int currProfile = 0;
    int desiredProfile = 0;

	if(forwVelNew < 0.25){
		currProfile = 0;
	}else if(forwVelNew >= 0.25 || forwVelNew < 0.50){
		currProfile = 1;
	}else if(forwVelNew >= 0.50 || forwVelNew < 0.75){
		currProfile = 2;
	}else if(forwVelNew >= 0.75 || forwVelNew < 1.00){
		currProfile = 3;
	}else if(forwVelNew >= 1.00 || forwVelNew < 1.25){
		currProfile = 4;
	}else if(forwVelNew >= 1.25 || forwVelNew < 1.50){
		currProfile = 5;
	}else if(forwVelNew >= 1.50 || forwVelNew < 1.75){
		currProfile = 6;
	}else if(forwVelNew >= 1.75 || forwVelNew < 2.0){
		currProfile = 7;
	}else if(forwVelNew >= 2.00){
		currProfile = 8;
	}



	if (inProfileTransitionFlag == false) { // if a transition is active don't change
		if(speed == Throttle::Faster){
			desiredProfile = currProfile + 1;
		}else if(speed == Throttle::Slower){
			desiredProfile = currProfile -1;
		}else{
			//Do nothing
		}

		if((desiredProfile < 0) || (desiredProfile > trotProfiles.size()))
		{
			desiredProfile = currProfile; //if outside bounds, do nothing with the current profile.
		}
		runningTrot(desiredProfile);
	}
}

///*********************************************************************************************///
/*--------------------This where maxplus could start--------------------------*/

void RCFController::runningTrot(const unsigned int profile) {
	if (gaitPattern == 1) {
		changeGaitPattern();
		gaitPattern = 0;
	}

	if (freezeRatioFlag) {
		printf("Ls/Vf Ratio Released \n");
		freezeRatioFlag = false;
	}

	stepHeightNew = trotProfiles[0].stepHeightNew;
	VfX_New = trotProfiles[0].VfX_New;
	forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
	dutyF_New = trotProfiles[0].dutyF_New;
	stepFrequencyNew = trotProfiles[0].stepFrequencyNew;
	stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
	KcoupNew = trotProfiles[0].KcoupNew;

	//trunk controller tunning
	bodyWeight = trotProfiles[0].bodyWeight;
	D_Force = trotProfiles[0].D_Force;
	oooFlag = false;
	runningTrotFlag = true;
	walkingTrotFlag = false;
	std::cout << "TROT " << trotProfiles[0].name << " started" << std::endl;
}


///*********************************************************************************************///

void RCFController::changeGaitPattern() {
	if (gaitPattern == 0) {
		printf("Gait pattern changed to WALK! \n");
		gaitPattern = 1;
	} else {
		printf("Gait pattern changed to TROT! \n");
		gaitPattern = 0;
	}
}

///*********************************************************************************************///

void RCFController::toggleTrunkControl() {
	if (attitudeFlag) {
		attitudeFlag = false;
		printf("Trunk Controller OFF\n");

		resetImpedance();
		printf("Resetting joint impedances!!!\n");

		if (kinAdjustmentFlag) {
		    toggleKinematicAdjustmentFlag();
		}
		KpTrunkRollNew = 0;
		KpTrunkPitchNew = 0;

		KdTrunkRollNew = 0;
		KdTrunkPitchNew = 0;

		KdTrunkYawNew = 0;

		KpTrunkYawNew = 0.0;
		KpTrunkX_New = 0.0;
		KpTrunkY_New = 0.0;
		KpTrunkZ_New = 0.0;

		KdTrunkX_New = 0;
		KdTrunkY_New = 0;
		KdTrunkZ_New = 0;

		bodyWeightNew = 0.0;

	} else {

	    //Reset desired robot position in the world frame
	    resetRobotDesiredPosition();

		KpTrunkRollNew = KpTrunkRollDefault;
		KpTrunkPitchNew = KpTrunkPitchDefault;
		KpTrunkZ_New = KpTrunkZ_Default;


		KdTrunkRollNew = KdTrunkRollDefault;
		KdTrunkPitchNew = KdTrunkPitchDefault;
		KdTrunkYawNew = KdTrunkYawDefault;
		KdTrunkX_New = KdTrunkX_Default;
		KdTrunkY_New = KdTrunkY_Default;
		KdTrunkZ_New = KdTrunkZ_Default;

		//Gains related to robot position control
		if(positionControlFlag){
		    KpTrunkYawNew = KpTrunkYawDefault;
		    KpTrunkX_New = KpTrunkX_Default;
		    KpTrunkY_New = KpTrunkY_Default;
		}

		bodyWeightNew = bodyWeightDefault;
		printf("Trunk Controller ON\n");
		attitudeFlag = true;


	}
}

///*********************************************************************************************///

void RCFController::toggleTerrainEstimationFlag() {
	if (terrainEstimationFlag) {
		TerrainEstimator.Enable((terrainEstimationFlag = false));
	} else {
	    TerrainEstimator.resetEstimator();
		TerrainEstimator.Enable((terrainEstimationFlag = true));
	}
}

///*********************************************************************************************///

void RCFController::toggleKinematicAdjustmentFlag() {
	if (kinAdjustmentFlag) {
		KinAdjustment.enable((kinAdjustmentFlag = false));
		kadjRollMemory = kadjRollNew;
		kadjPitchMemory =  kadjPitchNew;
        kadjRollNew = 0.0;
        kadjPitchNew = 0.0;
	} else {
		if (attitudeFlag && KpTrunkRoll > 0.0 && KpTrunkPitch > 0.0) {
			KinAdjustment.enable((kinAdjustmentFlag = true));
			kadjRollNew = kadjRollMemory;
			kadjPitchNew = kadjPitchMemory;
		} else {
			std::cout << "Current: " << KpTrunkRoll << " Desired " << KpTrunkRollNew << std::endl;
			std::cout << "Current: " << KpTrunkPitch << " Desired " << KpTrunkPitchNew << std::endl;
			printf(
					"Impossible to toggle! Attitude control is off or its gains are not positive \n");
		}
	}
}

///*********************************************************************************************///

void RCFController::walkStopping() {
	stepHeightNew = 0.0001;
	VfX_New = 0.0001;
	VfY_New = 0.0;
	forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
	stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
	PsitNew = 0.0;

	/*ADDED BY OCTAVIO*/
	// myTime = 0.0;
	// eventsLog = schedule.initiallist(xInitial);

	oooFlag = true;
	printf("Walk Stopped\n");
}

///*********************************************************************************************///

void RCFController::togglePushRecoveryFlag() {
	if (pushRecoveryFlag) {
		pushRecoveryFlag = false;
	} else {
		pushRecoveryFlag = true;
	}
	PushRecovery.Enable(pushRecoveryFlag);
}

///*********************************************************************************************///

void RCFController::toggleWalkAdaptationFlag() {
	if (walkAdaptationFlag) {
		walkAdaptationFlag = false;
		adapWalkFlag = false;
		printf("Walk adaptation turned OFF! \n");
	} else {
		walkAdaptationFlag = true;
		adapWalkFlag = false;
		printf("Walk adaptation turned ON! \n");
		if(CPG_StepHeightModulationFlag) {
		    CPG_StepHeightModulationFlag = false;
		    for (int leg = dog::LF; leg <= dog::RH; leg++) {
		    CPG[leg].enableHeightModulation(CPG_StepHeightModulationFlag);
		    }
		    printf("CPG height modulation OFF!!! Not allowed with adaptive walk! \n");
		}
	}
}

///*********************************************************************************************///

bool RCFController::isInfOrNan(const double& x) {
    return (std::isinf(x) || std::isnan(x));
}

///*********************************************************************************************///

rbd::Vector3d RCFController::computeVirtualLegPosition(const dog::LegDataMap<rbd::Vector3d>& posHF) {

    rbd::Vector3d virtualPos;
    virtualPos.setZero();

    if(stanceLegs[dog::LF] || !stanceLegs[dog::RF] ||
       !stanceLegs[dog::LH] || stanceLegs[dog::RH]) {
        virtualPos(rbd::X) = (posHF[dog::LF](rbd::X) + posHF[dog::RH](rbd::X)) / 2;
        virtualPos(rbd::Y) = (posHF[dog::LF](rbd::Y) + posHF[dog::RH](rbd::Y)) / 2;
        virtualPos(rbd::Z) = (posHF[dog::LF](rbd::Z) + posHF[dog::RH](rbd::Z)) / 2;
    }

    if(!stanceLegs[dog::LF] || stanceLegs[dog::RF] ||
        stanceLegs[dog::LH] || !stanceLegs[dog::RH]) {
        virtualPos(rbd::X) = (posHF[dog::RF](rbd::X) + posHF[dog::LH](rbd::X)) / 2;
        virtualPos(rbd::Y) = (posHF[dog::RF](rbd::Y) + posHF[dog::LH](rbd::Y)) / 2;
        virtualPos(rbd::Z) = (posHF[dog::RF](rbd::Z) + posHF[dog::LH](rbd::Z)) / 2;
    }


    return virtualPos;

}


///*********************************************************************************************///

rbd::Vector3d RCFController::computeDesVirtualLegPosition(const dog::LegDataMap<rbd::Vector3d>& posHF) {

    rbd::Vector3d virtualPos;
    virtualPos.setZero();

    if(stanceLegs[dog::LF] || !stanceLegs[dog::RF] ||
       !stanceLegs[dog::LH] || stanceLegs[dog::RH]) {
        virtualPos(rbd::X) = (posHF[dog::LF](rbd::X) + posHF[dog::RH](rbd::X)) / 2;
        virtualPos(rbd::Y) = (posHF[dog::LF](rbd::Y) + posHF[dog::RH](rbd::Y)) / 2;
        virtualPos(rbd::Z) = (posHF[dog::LF](rbd::Z) + posHF[dog::RH](rbd::Z)) / 2;
    }

    if(!stanceLegs[dog::LF] || stanceLegs[dog::RF] ||
        stanceLegs[dog::LH] || !stanceLegs[dog::RH]) {
        virtualPos(rbd::X) = (posHF[dog::RF](rbd::X) + posHF[dog::LH](rbd::X)) / 2;
        virtualPos(rbd::Y) = (posHF[dog::RF](rbd::Y) + posHF[dog::LH](rbd::Y)) / 2;
        virtualPos(rbd::Z) = (posHF[dog::RF](rbd::Z) + posHF[dog::LH](rbd::Z)) / 2;
    }


    return virtualPos;

}


///*********************************************************************************************///

void RCFController::referencesBackTracingPrintOuts(const iit::dog::LegID leg) {

	switch(leg) {
	case dog::LF :
		cout << "LF foot reference out of the workspace!!!" << endl;
		break;

	case dog::RF :
		cout << "RF foot reference out of the workspace!!!" << endl;
		break;

	case dog::LH :
		cout << "LH foot reference out of the workspace!!!" << endl;
		break;

	case dog::RH :
		cout << "RH foot reference out of the workspace!!!" << endl;
		break;
	}

    cout << "WCPGb = " << WCPGb[leg].transpose() << endl;
    cout << "dWCPGb = " << dWCPGb[leg].transpose() << endl;
    cout << "WCPG = " << WCPG[leg].transpose() << endl;
    cout << "dWCPG = " << dWCPG[leg].transpose() << endl;
    cout << endl;
    cout << "P_DesLocal_HF = "  << P_DesLocal_HF[leg].transpose() << endl;
    cout << "Pd_Deslocal_HF = "  << Pd_DesLocal_HF[leg].transpose() << endl;
    cout << "P_DesLocal_BF = " << P_DesLocal_BF[leg].transpose() << endl;
    cout << "Pd_DesLocal_BF = " << Pd_DesLocal_BF[leg].transpose() << endl;
    cout << "Pdd_DesLocal_BF = " << Pdd_DesLocal_BF[leg].transpose() << endl << endl;

}

///*********************************************************************************************///

void RCFController::walkStarting() {
	if (gaitPattern == 0) {
		stepHeightNew = 0.10;
		VfX_New = 0.0075;
		VfY_New = 0.0;
		dutyF_New = 0.5;
		forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
		stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
		KcoupNew = 0.5;


		/* ADDED BY OCTAVIO */
		// myTime = 0.0;
		// eventsLog = schedule.initiallist(xInitial);
		// bool walkingFlag = true;

		//Kp_trunk_roll_new = 2000;
		//Kd_trunk_roll_new = 170;
		//Kp_trunk_pitch_new = 2000;
		//Kd_trunk_pitch_new = 320;
		//Kd_trunk_x_new = 200.0;
		//Kd_trunk_z_new = 200.0;

		//Settings for vision experiments
		//Kp_trunkZnew = 10000;
		//Kd_trunkZnew = 1300;
		//body_weight = 75;

		PsitNew = 0.0;

		//Step velocity tunning
		KvfNew = 1.0;
		oooFlag = false;
		runningTrotFlag = false;
		walkingTrotFlag = true;
		fallDetectionFlag = false;
		printf("WALKING TROT Started!!!\n");
	} else {
		stepHeightNew = 0.075;
		VfX_New = 0.01;
		VfY_New = 0.0;
		dutyF_New = 0.5;
		forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
		stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
		KcoupNew = 0.5;
		dutyF_New = 0.83; //Equivalent to Duty=5;
		PsitNew = 0.0;

		oooFlag = false;
		runningTrotFlag = false;
		walkingTrotFlag = true;
		fallDetectionFlag = false;
		printf("WALK Started!!!\n");
	}

}

///*********************************************************************************************///


void RCFController::computeGRFfromLegTorquesBase(const iit::dog::JointState & q_curr,
	const iit::dog::JointState & qd_curr, const iit::dog::JointState & tau,
	iit::dog::LegDataMap<rbd::Vector3d> & feet_forces){

	rbd::Vector3d temp, legTorques;

	if(robotName == "hyq" || robotName == "centaur1arm"){
		jacobiansHyQ->fr_trunk_J_LF_foot(q_curr);
		jacobiansHyQ->fr_trunk_J_RF_foot(q_curr);
		jacobiansHyQ->fr_trunk_J_LH_foot(q_curr);
		jacobiansHyQ->fr_trunk_J_RH_foot(q_curr);
	}else{
		jacobiansHyQ2Max->fr_trunk_J_LF_foot(q_curr);
		jacobiansHyQ2Max->fr_trunk_J_RF_foot(q_curr);
		jacobiansHyQ2Max->fr_trunk_J_LH_foot(q_curr);
		jacobiansHyQ2Max->fr_trunk_J_RH_foot(q_curr);
	}

	//compute the coriolis/ gravity terms
	// iit::rbd::VelocityVector gW, gB;
	// gW <<0.0, 0.0, 0.0, 0.0, 0.0, -iit::rbd::g; gB.setZero();
	// gB.segment(rbd::LX,3) = rbd::Utils::rotationMx(SL::world_X_base).transpose()*gW.segment(rbd::LX,3);
	rbd::ForceVector baseWrench;
	iit::dog::JointState  h_joints;
	//the base twist is set to zero because il will affect mainly the baseWrench (otherwise we should estimate the base twist)
	//SL::invDynEngine->id_fully_actuated(baseWrench, h_joints, gB, rbd::VelocityVector::Zero(), rbd::VelocityVector::Zero(), q_curr, qd_curr, JointState::Zero());


	legTorques = tau.segment(dog::LF_HAA,3);
    // Rotate the force from base to foot coordinates.
	Eigen::Matrix3d jac_transpose = (*JFootLF).block<3,3>(rbd::LX, 0).transpose();
    temp = jac_transpose.inverse()*(legTorques);// + h_joints.segment(dog::LF_HAA, 3));
	feet_forces[dog::LF] = -temp;

	legTorques = tau.segment(dog::RF_HAA,3);
    // Rotate the force from base to foot coordinates.
	jac_transpose = (*JFootRF).block<3,3>(rbd::LX, 0).transpose();
    temp = jac_transpose.inverse()*(legTorques);// + h_joints.segment(dog::RF_HAA, 3));
    feet_forces[dog::RF]  = -temp;


	legTorques = tau.segment(dog::LH_HAA,3);
    // Rotate the force from base to foot coordinates.
	jac_transpose = (*JFootLH).block<3,3>(rbd::LX, 0).transpose();
    temp = jac_transpose.inverse()*(legTorques);// + h_joints.segment(dog::LH_HAA, 3));
    feet_forces[dog::LH]  = -temp;


	legTorques = tau.segment(dog::RH_HAA,3);

	// Rotate the force from base to foot coordinates.
	jac_transpose = (*JFootRH).block<3,3>(rbd::LX, 0).transpose();
	temp = jac_transpose.inverse()*(legTorques);// + h_joints.segment(dog::RH_HAA, 3));
	feet_forces[dog::RH]  = -temp;


}


//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::jump() {
    double alphaFiltJumpForce = 1 / (double) taskServoRate
            / (0.01 + 1 / (double) taskServoRate);
    static double jumpForce2 = 0.0;

    jumpForce2 = (1 - alphaFiltJumpForce) * jumpForce2 + alphaFiltJumpForce * jumpForceNew;
    jumpForce = (1 - alphaFiltJumpForce) * jumpForce + alphaFiltJumpForce * jumpForce2;

    cout << jumpForce << endl;

    if (trunkHeight >= 0.6) {
        jumpOff = true;
    } else {
        for (int leg = dog::LF; leg <= dog::RH; leg++) {
            P0_Local_HF[leg] = P_Local_HF[leg];
            P0_Local_HF_New[leg] = P_Local_HF[leg];
            originChangingRate = filterResponse::fastC;
        }

    }

    //if((InitJumpFlag && Jump_Force>0.98*Jump_Force_new) || (InitJumpFlag && JumpOff))
    if (initJumpFlag && jumpOff) {
        initJumpFlag = false;
        jumpForceNew = 0.0;
        endJumpFlag = true;
        setPID(300, 150, 150, 20, 10, 10);
        PIDManager.usePIDManager = false;
        toggleTrunkControl();
        //liftOffHeight = misc_sensor[B_Z] + 1;
        liftOffHeight = 0.0;
        std::cout << "Lift-off Pos: " << liftOffHeight << std::endl;
        std::cout << "trunkHeight: " << trunkHeight << std::endl;

        computeDeltaJumpHeight = true;
    }
    if (endJumpFlag && jumpForce < 0.01) {
        initJumpFlag = false;
        jumpForce = 0.0;
        jumpForce2 = 0.0;
        jumpFlag = false;
        jumpOff = false;
        landingOrigins();
    }

}


//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::landingOrigins() {

    P0_Local_HF_New[dog::LF] <<  0.420508,  0.284500, -0.526790;
    P0_Local_HF_New[dog::RF] <<  0.420508, -0.284500, -0.526790;
    P0_Local_HF_New[dog::LH] << -0.420508,  0.284500, -0.526790;
    P0_Local_HF_New[dog::RH] << -0.420508, -0.284500, -0.526790;
    originChangingRate = filterResponse::fastC;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::setPID(const double &HAA_gain, const double &HFE_gain,
                                const double &KFE_gain, const double &HAA_gain_d,
                                const double &HFE_gain_d, const double &KFE_gain_d) {

    zeroGainTh[dog::LF_HAA]  = HAA_gain;
    zeroGainTh[dog::LF_HFE]  = HFE_gain;
    zeroGainTh[dog::LF_KFE]  = KFE_gain;
    zeroGainThd[dog::LF_HAA]  = HAA_gain_d;
    zeroGainThd[dog::LF_HFE]  = HFE_gain_d;
    zeroGainThd[dog::LF_KFE]  = KFE_gain_d;

    zeroGainTh[dog::RF_HAA]  = HAA_gain;
    zeroGainTh[dog::RF_HFE]  = HFE_gain;
    zeroGainTh[dog::RF_KFE]  = KFE_gain;
    zeroGainThd[dog::RF_HAA]  = HAA_gain_d;
    zeroGainThd[dog::RF_HFE]  = HFE_gain_d;
    zeroGainThd[dog::RF_KFE]  = KFE_gain_d;

    zeroGainTh[dog::LH_HAA]  = HAA_gain;
    zeroGainTh[dog::LH_HFE]  = HFE_gain;
    zeroGainTh[dog::LH_KFE]  = KFE_gain;
    zeroGainThd[dog::LH_HAA]  = HAA_gain_d;
    zeroGainThd[dog::LH_HFE]  = HFE_gain_d;
    zeroGainThd[dog::LH_KFE]  = KFE_gain_d;

    zeroGainTh[dog::RH_HAA]  = HAA_gain;
    zeroGainTh[dog::RH_HFE]  = HFE_gain;
    zeroGainTh[dog::RH_KFE]  = KFE_gain;
    zeroGainThd[dog::RH_HAA]  = HAA_gain_d;
    zeroGainThd[dog::RH_HFE]  = HFE_gain_d;
    zeroGainThd[dog::RH_KFE]  = KFE_gain_d;

    setPID_Flag = true;
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LF_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RF_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.LH_KFE, KFE_gain, 0.0, KFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_HAA, HAA_gain, 0.0, HAA_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_HFE, HFE_gain, 0.0, HFE_gain_d);
    PIDManager.setDefaultJointPIDGains(PIDManager.RH_KFE, KFE_gain, 0.0, KFE_gain_d);

}



//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::selfRighting(void) {

    static double selfRightingTime = 0.0;
    static bool stage1 = false;
    static bool stage2 = false;
    static bool stage3 = false;
    static bool stage4 = false;
    static bool stage5 = false;
    static bool stage6 = false;
    static bool stage7 = false;
    static bool stage8 = false;
    static double Ts = 1/(double)taskServoRate;

    static Eigen::Matrix<double, 12, 1> jointTarget;
    static Eigen::Matrix<double, 12, 1> jointTargetf;
    static double stageDuration;
    static double alphaStage;
    static double stageTime = 10.0;
    static bool isBellyUp = false;
    static bool bellyCheck = false;

    //Check if the belly is up or down
    //First condition: belly status must be checked and roll is in the interval [-90,90]
    if(!bellyCheck && ((bs->getRoll_W() > 90*3.14/180) || (bs->getRoll_W() < -90*3.14/180))){
        isBellyUp = true;
        bellyCheck = true; //confirm that belly status was checked

        for (int joint = dog::LF_HAA; joint <= dog::RH_KFE; joint++) {
            des_q_[joint] = q_[joint];
            des_qd_[joint] = 0.0;
            jointTargetf(joint) = q_[joint];
        }

        std::cout << "Belly check..." << std::endl;
        if(isBellyUp){
            std::cout << "Robot belly is up! Starting self-righting..." << std::endl;
        }

    }
    //Second condition: belly status must be checked and roll is out of the interval [-90,90]
    else if(!bellyCheck) {
        isBellyUp = false;
        bellyCheck  = true;  //confirm that belly status was checked

//        for (int joint = dog::LF_HAA; joint <= dog::RH_KFE; joint++) {
//            des_q_[joint] = q_[joint];
//            des_qd_[joint] = 0.0;
//            jointTargetf(joint) = q_[joint];
//        }

        stage1 = true;
        stage2 = true;
        stage3 = true;
        stage4 = true;
        stage5 = true;
        stage6 = true;
        stage7 = true;
        stage8 = true;
        stageTime = 1 + 2 + 2 + 1 + 0.5 + 0.75 + 1.0 + 4.0;
        selfRightingTime = stageTime;

        executeSelfRightingFlag = false;

        std::cout << "Belly check..." << std::endl;
        std::cout << "Robot belly is down! Self-righting routine aborted!!!" << std::endl;

    }


    if(bellyCheck){
    //Safe position to avoid breaking legs
    if(!stage1 && fallDetectionFlag) {
        cout << "stage 1: safe position to avoid damaging the legs \n";
        jointTarget << 0.0, 1.65, -2.85,   0.0, 1.65, -2.85,    0.0, -1.65, 2.85,    0.0, -1.65, 2.85;
        cout << jointTarget.transpose() << endl << endl;
        //jointTarget << 0.0, 1.35, -2.7,   0.0, 1.35, -2.7, 0.0, -1.35, 2.7,    0.0, -1.35, 2.7;

        stage1 = true;
        stageDuration = 1;
        stageTime = stageDuration;

    }

    //Right hips positioning
    if(!stage2 && stage1 && selfRightingTime >= stageTime) {
        cout << "stage 2: right hips positioning \n";
        //jointTarget << 0.0, 1.65, -3.15,   -0.4, -1.65, -3.15,    0.0, -1.65, 3.15,    -0.4, 1.65, 3.15;
        jointTarget << 0.0, 1.35, -2.7,   -0.4, -1.3, -2.75,    0.0, -1.35, 2.7,    -0.4, 1.3, 2.75;
        cout << jointTarget.transpose() << endl << endl;

        stage2 = true;
        stageDuration = 2.0;
        stageTime += stageDuration;
    }

    //Push
    if(!stage3 && stage2 && selfRightingTime >= stageTime) {
        cout << "stage 3: push to turn \n";
        //jointTarget << -0.3, 2.3, -3.15,   -0.8, -2.3, -2.05,    -0.3, -2.3, 3.15,    -0.8, 2.3, 2.05;
        jointTarget << -0.3, 2.0, -2.7,   -0.7, -2.3, -1.0, -0.3, -2.0, 2.7,    -0.7, 2.3, 1.0;
        cout << jointTarget.transpose() << endl << endl;

        stage3 = true;
        stageDuration = 2.0;
        stageTime += stageDuration;
    }

    //Starting landing posture
    if(!stage4 && stage3 && selfRightingTime >= stageTime) {
        cout << "stage 4: start landing posture \n";
        //jointTarget << -0.3, 2.3, -3.15,    0.0, 1.65, -3.15,    -0.3, -2.3, 3.15,    0.0, -1.65, 3.15;
        jointTarget << 0.6, 2.15, -2.6,    -0.6,  1.9025, -2.3,    0.6, -2.15, 2.6,    -0.6, -1.9025, 2.3;
        cout << jointTarget.transpose() << endl << endl;

        stage4 = true;
        stageDuration = 1.0;
        stageTime += stageDuration;
    }

    //Retract legs to fall
    if(!stage5 && stage4 && selfRightingTime >= stageTime) {
        cout << "stage 5: retract legs to fall \n";
        //jointTarget <<  0.0, 1.65, -3.15,    0.0, 1.65, -2.15,    0.0, -1.65, 3.15,    0.0, -1.65, 2.15;
        //jointTarget << 0.0, 1.7, -2.6,    0.0,  1.9, -2.4,   0.0, -1.7, 2.6,    0.0, -1.9, 2.4;
        cout << jointTarget.transpose() << endl << endl;

        stage5 = true;
        stageDuration = 0.5;
        stageTime += stageDuration;
    }

    //Prepare KFEs and HAAs to lift up
    if(!stage6 && stage5 && selfRightingTime >= stageTime) {
        cout << "stage 6: prepare KFEs and HAAs to lift up \n";
        //jointTarget << -0.1, 1.65, -3.15,   -0.1, 1.65, -3.15,    -0.1, -1.65, 3.15,    -0.1, -1.65, 3.15;
        jointTarget << -0.4, 2.3, -2.85,    -0.4,  2.3, -2.85, -0.4, -2.3, 2.85,    -0.4, -2.3, 2.85;
        cout << jointTarget.transpose() << endl << endl;

        stage6 = true;
        stageDuration = 0.75;
        stageTime += stageDuration;
    }

    //Prepare HFEs and to lift up
    if(!stage7 && stage6 && selfRightingTime >= stageTime) {
        cout << "stage 7: prepare HFEs to lift up \n";
        jointTarget << -0.05, 1.57, -2.85,   -0.05, 1.57, -2.85,    -0.05, -1.57, 2.85,    -0.05, -1.57, 2.85;
        cout << endl << "jointTarget = " << jointTarget.transpose() << endl << endl;

        stage7 = true;
        stageDuration = 1.0;
        stageTime += stageDuration;
    }


    //Lift up
    if(!stage8 && stage7 && selfRightingTime >= stageTime) {
        cout<<"stage 8: lift up \n";
        //jointTarget << 0.0, 0.75, -1.5,   0.0, 0.75, -1.5,    0.0, -0.75, 1.5,    0.0, -0.75, 1.5;
        jointTarget << -0.05, 0.65, -1.18,   -0.05, 0.65, -1.18, -0.05, -0.65, 1.18,    -0.05, -0.65, 1.18;
        cout << jointTarget.transpose() << endl << endl;

        stage8 = true;
        stageDuration = 4.0;
        stageTime += stageDuration;
    }

    //Execute self-righting trajectories
    if(executeSelfRightingFlag){
        alphaStage = Ts/(stageDuration/5+Ts);

        for(int joint = dog::LF_HAA; joint <= dog::RH_KFE; joint++) {
            jointTargetf(joint) = (1-alphaStage) * jointTargetf(joint) + alphaStage * jointTarget(joint);
            des_q_[joint] = (1-alphaStage) * des_q_[joint] + alphaStage * jointTargetf(joint);
        }
    }


    if(stage8 && selfRightingTime >= stageTime) {
        stage1 = false;
        stage2 = false;
        stage3 = false;
        stage4 = false;
        stage5 = false;
        stage6 = false;
        stage7 = false;
        stage8 = false;
        stageTime = 10;
        selfRightingTime = 0.0;

        //if((roll_angle < 0.1) && (roll_angle > -0.1) && (pitch_angle < 0.1) && (pitch_angle > -0.1)) {
        fallDetectionFlag = false;
        //}

        executeSelfRightingFlag = false;
        bellyCheck = false;

        //Keep posture after finishing the self-righting
        setFootOriginsInstantaneouslyFromActualDesiredJointPositions();

        std::cout << "Self-righting finished!!!" << std::endl;
    }
    else {
        selfRightingTime += 1.0/(double)taskServoRate;
    }
    }

}


void RCFController::layDown(void) {

    static double layDownTime = 0.0;
    static double Ts = 1.0/(double)taskServoRate;
    static Eigen::Matrix<double, 12, 1> jointTargetLayDown;
    static Eigen::Matrix<double, 12, 1> jointTargetLayDownf;
    static double stageDuration;
    static double stageTime;
    static double alphaStage;

    static bool stage1 = false;
    static bool stage2 = false;
    static bool stage3 = false;
    static bool stage4 = false;
    static bool stage5 = false;
    static bool stage6 = false;
    static bool stage7 = false;


    //INIT it enters here only once
    if (!stage1) {
        cout << "stage 1\n";

        //set desired squat position
        jointTargetLayDown << 0.0, 1.57, -2.920, 0.0, 1.57, -2.920, 0.0, -1.57, 2.920, 0.0, -1.57, 2.920;
        cout << endl << "jointTargetLayDown = " << jointTargetLayDown.transpose() << endl << endl;

        //save actual position and init the filter with it
        for (int joint = dog::LF_HAA; joint <= dog::RH_KFE; joint++) {
            des_q_[joint] = q_[joint];
            des_qd_[joint] = 0.0;
            jointTargetLayDownf(joint) = q_[joint];
        }

        stage1 = true;
        stageDuration = 2.5;
        stageTime = stageDuration;
    }

    //...
    if(!stage2 && stage1 && layDownTime >= stageTime) {
        cout<<"stage 2\n";

        //jointTargetLayDown << 0.0, 1.57, -2.920,   -0.4, 1.1, -2.2,    0.6, -1.57, 2.920,    -0.4, -1.1, 2.2;
        jointTargetLayDown << 0.4, 1.57, -2.920,   0.5, 0.92, -1.84,    0.4, -1.57, 2.920,    0.5, -0.92, 1.84;
        cout << endl << "jointTargetLayDown = " << jointTargetLayDown.transpose() << endl << endl;

        stage2 = true;
        stageDuration = 2.0;
        stageTime += stageDuration;
    }

    //...
    if(!stage3 && stage2 && layDownTime >= stageTime) {
        cout<<"stage 3\n";

        //jointTargetLayDown << 0.6, 1.1, -2.2,   -0.4, 0.92, -1.84,    0.6, -1.1, 2.2,    -0.4, -0.92, 1.84;
        jointTargetLayDown << 0.65, 1.2, -2.2,   -0.6, 0.5, -1.0,    0.65, -1.2, 2.2,    -0.6, -0.5, 1.0;
        cout << endl << "jointTargetLayDown = " << jointTargetLayDown.transpose() << endl << endl;

        stage3 = true;
        stageDuration = 2.0;
        stageTime += stageDuration;
    }

    if(!stage4 && stage3 && layDownTime >= stageTime) {
            cout<<"stage 4\n";

            //jointTargetLayDown << 0.0, 1.57, -2.920, 0.0, 1.57, -2.920, 0.0, -1.57, 2.920,0.0, -1.57, 2.920;
            jointTargetLayDown << 0.65, 1.2, -2.2,   -0.4, -1.6, -2.55,    0.65, -1.2, 2.2,    -0.4, 1.6, 2.55;
            cout << endl << "jointTargetLayDown = " << jointTargetLayDown.transpose() << endl << endl;

            stage4 = true;
            stageDuration = 1.0;
            stageTime += stageDuration;
        }

    if(!stage5 && stage4 && layDownTime >= stageTime) {
        cout<<"stage 5\n";

        jointTargetLayDown << -0.2, 1.2, -2.2,   -0.4, -1.6, -2.55,    -0.2, -1.2, 2.2,    -0.4, 1.6, 2.55;
        cout << endl << "jointTargetLayDown = " << jointTargetLayDown.transpose() << endl << endl;

        stage5 = true;
        stageDuration = 0.5;
        stageTime += stageDuration;
    }

    if(!stage6 && stage5 && layDownTime >= stageTime) {
           cout<<"stage 6\n";

           jointTargetLayDown << -0.2, 1.2, -2.2,   -0.4, -1.6, -2.35,    -0.2, -1.2, 2.2,    -0.4, 1.6, 2.35;
           cout << endl << "jointTargetLayDown = " << jointTargetLayDown.transpose() << endl << endl;

           stage6 = true;
           stageDuration = 1.5;
           stageTime += stageDuration;
       }


    if(!stage7 && stage6 && layDownTime >= stageTime) {
        cout<<"stage 7\n";

        jointTargetLayDown << 0.0, 1.57, -2.85,  0.0, 1.57, -2.85,  0.0, -1.57, 2.85,  0.0, -1.57, 2.85;
        cout << endl << "jointTargetLayDown = " << jointTargetLayDown.transpose() << endl << endl;

        stage7 = true;
        stageDuration = 2.0;
        stageTime += stageDuration;
    }



    //compute filter
    alphaStage = Ts/(stageDuration/5+Ts);
    for(int joint = dog::LF_HAA; joint <= dog::RH_KFE; joint++) {
        jointTargetLayDownf(joint) = (1-alphaStage) * jointTargetLayDownf(joint) + alphaStage * jointTargetLayDown(joint);
        des_q_[joint] = (1-alphaStage) * des_q_[joint] + alphaStage * jointTargetLayDownf(joint);
    }


    //when stageTime is elapsed
    if(stage7 && layDownTime >= stageTime) {
        cout << "Im down\n";
        stage1 = false;
        stage2 = false;
        stage3 = false;
        stage4 = false;
        stage5 = false;
        stage6 = false;
        stage7 = false;
        layDownTime = 0.0;
        executeLayDownFlag = false;
    }
    else {
        layDownTime += 1.0 /(double)taskServoRate;
    }

}


//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
iit::dog::JointState RCFController::computeTorquesForInternalForces(const iit::dog::LegDataMap<rbd::Vector3d>& footForces){

iit::dog::JointState outputTorques;

outputTorques.block<3,1>(iit::dog::LF_HAA,0) =  ((*JFootLF).block<3,3>(rbd::LX,0).transpose()) * footForces[dog::LF];
outputTorques.block<3,1>(iit::dog::RF_HAA,0) =  ((*JFootRF).block<3,3>(rbd::LX,0).transpose()) * footForces[dog::RF];
outputTorques.block<3,1>(iit::dog::LH_HAA,0) =  ((*JFootLH).block<3,3>(rbd::LX,0).transpose()) * footForces[dog::LH];
outputTorques.block<3,1>(iit::dog::RH_HAA,0) =  ((*JFootRH).block<3,3>(rbd::LX,0).transpose()) * footForces[dog::RH];

return outputTorques;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::setFootOriginsInstantaneouslyFromActualDesiredJointPositions(void){

    iit::HyQ2Max::HomogeneousTransforms ht(default_pg);

    P0_Local_HF_New[dog::LF] = iit::rbd::Utils::positionVector(ht.fr_trunk_X_LF_foot(des_q_));
    P0_Local_HF_New[dog::RF] = iit::rbd::Utils::positionVector(ht.fr_trunk_X_RF_foot(des_q_));
    P0_Local_HF_New[dog::LH] = iit::rbd::Utils::positionVector(ht.fr_trunk_X_LH_foot(des_q_));
    P0_Local_HF_New[dog::RH] = iit::rbd::Utils::positionVector(ht.fr_trunk_X_RH_foot(des_q_));
    instantaneousOriginChangesFlag = true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::computeDesiredAndActualRobotPosition() {

//Update desired robot position according to desired trunk velocity
    double auxVfX, auxVfY;

    auxVfX = 0.0;
    auxVfY = 0.0;

    if(VfX > 0.01 || VfX < -0.01){
        auxVfX += + cos(robotYaw)*VfX - sin(robotYaw)*0.0;
        auxVfY += + sin(robotYaw)*VfX + cos(robotYaw)*0.0;
    }

    if(VfY > 0.01 || VfY < -0.01){
        auxVfX += + cos(robotYaw)*0.0 - sin(robotYaw)*VfY;
        auxVfY += + sin(robotYaw)*0.0 + cos(robotYaw)*VfY;
    }

    desRobotPositionWF[rbd::X] += auxVfX / (double)taskServoRate;
    desRobotPositionWF[rbd::Y] += auxVfY / (double)taskServoRate;


    if(Psit > 0.005 || Psit < -0.005){
        desRobotYaw = desRobotYaw + Psit / (double)taskServoRate;
    }

    //Get actual robot position described in the horizontal frame
    actualRobotPositionWF << bs->getPosition_W()[rbd::X], bs->getPosition_W()[rbd::Y], 0.0;

    actualRobotPositionHF[rbd::X] = + cos(-robotYaw)*actualRobotPositionWF(rbd::X)
                                    - sin(-robotYaw)*actualRobotPositionWF(rbd::Y);
    actualRobotPositionHF[rbd::Y] = + sin(-robotYaw)*actualRobotPositionWF(rbd::X)
                                    + cos(-robotYaw)*actualRobotPositionWF(rbd::Y);

    //Get desired robot position described in the horizontal frame
    desRobotPositionHF[rbd::X] = + cos(-robotYaw)*desRobotPositionWF(rbd::X)
                                 - sin(-robotYaw)*desRobotPositionWF(rbd::Y);
    desRobotPositionHF[rbd::Y] = + sin(-robotYaw)*desRobotPositionWF(rbd::X)
                                 + cos(-robotYaw)*desRobotPositionWF(rbd::Y);

}


//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
void RCFController::evaluateCollisionDetection() {

    static double frontalCollisionLinearVelocityXMemory = -0.01;
    static double frontalCollisionLinearVelocityYMemory = 0.0;
    static double frontalCollisionAngularVelocityMemory = 0.0;
    static double frontalCollisionBreakingTime = 0.0;
    static double frontalCollisionReactionTime = 2.0;

    if(!frontalCollisionFlag && VfX > 0.0 &&
            footSensor->force[dog::LF][rbd::X] < -75 && WCPGb[dog::LF](rbd::Z) > 0.33*stepHeight &&
            (180/3.1415 * atan2(footSensor->force[dog::LF][rbd::Z], footSensor->force[dog::LF][rbd::X]) > 135)){
        frontalCollisionFlag = true;
        frontalCollisionLinearVelocityXMemory = VfX_New;
        frontalCollisionLinearVelocityYMemory = VfY_New;
        frontalCollisionAngularVelocityMemory = PsitNew;
        VfX_New = -0.01;
        VfY_New = 0.0;
        PsitNew = 0.0;
        forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
        stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        std::cout << "LF foot force: " << footSensor->force[dog::LF][rbd::X] << ", " << footSensor->force[dog::LF][rbd::Z] << std::endl;
        std::cout << "Angle: " << 180/3.1415 * atan2(footSensor->force[dog::LF][rbd::Z], footSensor->force[dog::LF][rbd::X]) << std::endl;
    }

    if(!frontalCollisionFlag && VfX > 0.0 &&
            footSensor->force[dog::RF][rbd::X] < -75 && WCPGb[dog::RF](rbd::Z) > 0.33*stepHeight &&
            (180/3.1415 * atan2(footSensor->force[dog::RF][rbd::Z], footSensor->force[dog::RF][rbd::X]) > 135)){
        frontalCollisionFlag = true;
        frontalCollisionLinearVelocityXMemory = VfX_New;
        frontalCollisionLinearVelocityYMemory = VfY_New;
        frontalCollisionAngularVelocityMemory = PsitNew;
        VfX_New = -0.01;
        VfY_New = 0.0;
        PsitNew = 0.0;
        forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
        stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        std::cout << "RF foot force: " << footSensor->force[dog::RF][rbd::X] << ", " << footSensor->force[dog::RF][rbd::Z] << std::endl;
        std::cout << "Angle: " << 180/3.1415 * atan2(footSensor->force[dog::RF][rbd::Z], footSensor->force[dog::RF][rbd::X]) << std::endl;
    }

    if(!frontalCollisionFlag && VfX < 0.0 &&
            footSensor->force[dog::LH][rbd::X] > 75 && WCPGb[dog::LH](rbd::Z) > 0.33*stepHeight &&
            (180/3.1415 * atan2(footSensor->force[dog::LH][rbd::Z], footSensor->force[dog::LH][rbd::X]) < 45)){
        frontalCollisionFlag = true;
        frontalCollisionLinearVelocityXMemory = VfX_New;
        frontalCollisionLinearVelocityYMemory = VfY_New;
        frontalCollisionAngularVelocityMemory = PsitNew;
        VfX_New = +0.01;
        VfY_New = 0.0;
        PsitNew = 0.0;
        forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
        stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        std::cout << "LH foot force: " << footSensor->force[dog::LH][rbd::X] << ", " << footSensor->force[dog::LH][rbd::Z] << std::endl;
        std::cout << "Angle: " << 180/3.1415 * atan2(footSensor->force[dog::LH][rbd::Z], footSensor->force[dog::LH][rbd::X]) << std::endl;
    }

    if(!frontalCollisionFlag && VfX < 0.0 &&
            footSensor->force[dog::RH][rbd::X] > 75 && WCPGb[dog::RH](rbd::Z) > 0.33*stepHeight &&
            (180/3.1415 * atan2(footSensor->force[dog::RH][rbd::Z], footSensor->force[dog::RH][rbd::X]) > 135)){
        frontalCollisionFlag = true;
        frontalCollisionLinearVelocityXMemory = VfX_New;
        frontalCollisionLinearVelocityYMemory = VfY_New;
        frontalCollisionAngularVelocityMemory = PsitNew;
        VfX_New = +0.01;
        VfY_New = 0.0;
        PsitNew = 0.0;
        forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
        stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
        std::cout << "RH foot force: " << footSensor->force[dog::RH][rbd::X] << ", " << footSensor->force[dog::RH][rbd::Z] << std::endl;
        std::cout << "Angle: " << 180/3.1415 * atan2(footSensor->force[dog::RH][rbd::Z], footSensor->force[dog::RH][rbd::X]) << std::endl;
    }



    if(frontalCollisionFlag){

        frontalCollisionBreakingTime += 1.0/(double)taskServoRate;

        if(frontalCollisionBreakingTime > frontalCollisionReactionTime){
            frontalCollisionBreakingTime = 0.0;
            frontalCollisionFlag = false;
            VfX_New = frontalCollisionLinearVelocityXMemory;
            VfY_New = frontalCollisionLinearVelocityYMemory;
            PsitNew = frontalCollisionAngularVelocityMemory;
            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
            std::cout << "End of collision reaction!!!" << std::endl;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************///
//*********************************************************************************************///
 void RCFController::computeReflex(iit::dog::LegDataMap<rbd::Vector3d>& reflexPos, iit::dog::LegDataMap<rbd::Vector3d>& reflexVel){

    double swingTime = 1/stepFrequency * (1-dutyF);
    double alphaReflex = (1/(double)taskServoRate) / (1/(double)taskServoRate + swingTime/16);
    double reflexTime = swingTime/2;

    static iit::dog::LegDataMap<bool> turnOffReflex(false);
    static iit::dog::LegDataMap<bool> reflexRetractionOn(false);
    static iit::dog::LegDataMap<double> deltaReflex(0);
    static iit::dog::LegDataMap<double> pos(0);
    static iit::dog::LegDataMap<double> lastPos(0);
    static iit::dog::LegDataMap<double> pos2(0);
    static iit::dog::LegDataMap<double> vel(0);
    static iit::dog::LegDataMap<double> retractionTime(0);
    static bool reflexOn = false;
    static iit::dog::LegDataMap<bool> reflexTriggered(false);

    double deltaStepFrontLegs = (CPG[dog::LF].z_td - CPG[dog::RF].z_td)*(CPG[dog::LF].z_td - CPG[dog::RF].z_td);
    double deltaStepHindLegs = (CPG[dog::LF].z_td - CPG[dog::RF].z_td)*(CPG[dog::LF].z_td - CPG[dog::RF].z_td);

    //Check if reflex must be triggered
    if(!reflexOn && frontalCollisionFlag){
        reflexOn = true;
        std::cout << "Reflex up to be activated" << std::endl;
    }
    else{
        if(!reflexOn && deltaStepFrontLegs > 0.5*stepHeight && ((CPG[dog::LF].z_td > 0.5*stepHeight) || (CPG[dog::RF].z_td > 0.5*stepHeight)) ){
            reflexOn = true;
        }
    }

    //Trigger reflex
    for(int leg = dog::LF; leg <= dog::RH; leg++) {
        if(reflexOn && !reflexTriggered[leg] && CPG[leg].Xp(rbd::Z) > 0 && !stanceLegs[leg]){
            reflexTriggered[leg] = true;
            deltaReflex[dog::LF] = -stepLength/2;
            deltaReflex[dog::RF] = -stepLength/2;
            retractionTime[leg] = 0.0;
            pos[leg] = 0.0;
            pos2[leg] = 0.0;
            lastPos[leg] = 0.0;
            std::cout << "Reflex triggered!!!" << std::endl;
        }
    }

    for(int leg = dog::LF; leg <= dog::RH; leg++) {
        if(reflexTriggered[leg]) {
            pos2[leg] = (1 - alphaReflex) * pos2[leg] + alphaReflex * deltaReflex[leg];
            pos[leg] = (1 - alphaReflex) * pos[leg] + alphaReflex * pos2[leg];

            if(retractionTime[leg] > reflexTime/2){
                deltaReflex[leg] = 0;
            }

            if(retractionTime[leg] > reflexTime && stanceLegs[leg]){
                turnOffReflex[leg] = true;
                std::cout << "End of reflex for leg: " << leg << std::endl << std::endl;
            }

            retractionTime[leg] += 1/(double)taskServoRate;
        }

        reflexPos[leg] << pos[leg], 0.0, 0.0;
        reflexVel[leg] << (pos[leg]-lastPos[leg])*(double)taskServoRate, 0.0, 0.0;
        lastPos[leg] = pos[leg];
    }

    if(turnOffReflex[dog::LF] && turnOffReflex[dog::RF] && turnOffReflex[dog::LH] && turnOffReflex[dog::RH]){
        reflexOn = false;
        turnOffReflex[dog::LF] = false;
        turnOffReflex[dog::RF] = false;
        turnOffReflex[dog::LH] = false;
        turnOffReflex[dog::RH] = false;
        reflexTriggered[dog::LF] = false;
        reflexTriggered[dog::RF] = false;
        reflexTriggered[dog::LH] = false;
        reflexTriggered[dog::RH] = false;

    }
}


void RCFController::checkJoystickCommands(void) {

/*    **********  Using wireless CSL gamepad for PS3 **********

    Axes channels     /     Commands
         0                Left Analog Left/Right
         1                Left Analog Up/Down
         2                Right Analog Left/Right
         3                Right Analog Up/Down
         4                Left  left and right cross buttons
         5                Left  up and down cross buttons
         6                Not used

    Button channels    /     Commands
          0                  Button 1
          1                  Button 2
          2                  Button 3
          3                  Button 4
          4                  Button L1
          5                  Button R1
          6                  Button L2
          7                  Button R2
          8                  Button SELECT
          9                  Button START
          10                 Button of left analog joystick
          11                 Button of right analog joystick
*/


	    //Longitudinal velocity and turning rate
	    if(roughTerrainMode){
	        //Longitudinal velocity
	        VfX_buffer = 0.5 * joystick_axes(1);
	        //Angular velocity (robot turning)
	        Psit_buffer =  joystick_axes(2)* 45 * M_PI / 180 ;
	    }
	    else{
	        VfX_buffer = 0.75 * joystick_axes(1);
	        Psit_buffer =  joystick_axes(2)* 60 * M_PI / 180 ;
	    }

	    //Dead zone for VfX
	    if(VfX_buffer > -0.025 && VfX_buffer < 0.025)
	        VfX_buffer = 0.005;

	    //Lateral velocity
	    VfY_buffer = 0.5 * joystick_axes(0);

	    //Dead zone for VfY
	    if(VfY_buffer > -0.025 && VfY_buffer < 0.025)
	        VfY_buffer = 0.0;





	    //Update velocities when there is swing phase to avoid foot frontal
	    //collision and the CPG get out of phase when dutyF is greater than 0.5
	    if(!frontalCollisionFlag){
	        if(forwVel > 0.025 || WCPGb[dog::LF](rbd::Z) > 0.33*stepHeight || WCPGb[dog::RF](rbd::Z) > 0.33*stepHeight){
	            VfX_New = VfX_buffer;
	            VfY_New = VfY_buffer;
	            PsitNew = Psit_buffer;

	            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
	            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
	        }
	    } else{
            VfX_New = -0.01;
            VfY_New = 0.0;
            PsitNew = 0.0;

            forwVelNew = sqrt(VfX_New * VfX_New + VfY_New * VfY_New);
            stepLengthNew = forwVelNew * dutyF_New / stepFrequencyNew;
	    }

	    //Trunk controller
	    if(joystick_buttons(0) && (joystick_buttons(0) != buttons_memory(0))){
	        toggleTrunkControl();
	    }

	    //Kinematic adjustment
	    if(joystick_buttons(1) && (joystick_buttons(1) != buttons_memory(1))){
	        toggleKinematicAdjustmentFlag();
	    }

        //Exit jctp
        if(joystick_buttons(2))
        {
            toggleJoystick();
        }

	    //Push recovery
	    if(joystick_buttons(3) && (joystick_buttons(3) != buttons_memory(3))){
	        togglePushRecoveryFlag();
	    }

	    //Terrain estimator
	    if(joystick_buttons(4) && (joystick_buttons(4) != buttons_memory(4))){
	        toggleTerrainEstimationFlag();
	    }

        //Decrease trunk height
        if(joystick_buttons(5) == 1  && (joystick_buttons(5) != buttons_memory(5))) {
            P0_Local_HF_New[dog::LF](rbd::Z) = -0.52;
            P0_Local_HF_New[dog::RF](rbd::Z) = -0.52;
            P0_Local_HF_New[dog::LH](rbd::Z) = -0.52;
            P0_Local_HF_New[dog::RH](rbd::Z) = -0.52;

            originChangingRate = filterResponse::slowC;
        }

        //Adaptive walking trot
        if(joystick_buttons(6) && (joystick_buttons(6) != buttons_memory(6))){
            toggleWalkAdaptationFlag();
        }

        //Reset trunk height
        if(joystick_buttons(7) == 1  && (joystick_buttons(7) != buttons_memory(7))) {
            P0_Local_HF_New[dog::LF](rbd::Z) = -0.6;
            P0_Local_HF_New[dog::RF](rbd::Z) = -0.6;
            P0_Local_HF_New[dog::LH](rbd::Z) = -0.6;
            P0_Local_HF_New[dog::RH](rbd::Z) = -0.6;

            originChangingRate = filterResponse::mediumC;
        }

        //Change between flat/rought terrain gait params
        if(joystick_buttons(8) == 1  && (joystick_buttons(8) != buttons_memory(8))) {
            if(roughTerrainMode){
                stepFrequencyNew = 1.7;
                dutyF_New = 0.5;
                roughTerrainMode = false;
                std::cout << "Selected gait params for flat terrain!!!" << std::endl;
            }
            else {
                stepFrequencyNew = 1.2;
                stepHeightNew = 0.13;
                dutyF_New = 0.65;
                setImpedanceRoughTerrain();
                roughTerrainMode = true;
                std::cout << "Selected gait params for rough terrain!!!" << std::endl;
            }
        }


        //Increase step height
        if(joystick_axes(5) == 1.0  && (joystick_axes(5) != axes_memory(5))) {
            stepHeightNew += 0.01;
            if(stepHeightNew > 0.2)
                stepHeightNew = 0.2;
        }

        //Decrease step height
        if(joystick_axes(5) == -1.0  && (joystick_axes(5) != axes_memory(5))) {
            stepHeightNew = 0.10;
        }

        //Start walking/trotting
        if(joystick_buttons(9) && (joystick_buttons(9) != buttons_memory(9))) {
            walkStarting();
        }

	    //OOO = stop walking/trotting
	    if(joystick_buttons(10) && (joystick_buttons(10) != buttons_memory(10))){
	        walkStopping();
	    }


	    //Update last input commands
	    axes_memory = joystick_axes;
	    buttons_memory = joystick_buttons;

}



} //@namespace dls_controller
