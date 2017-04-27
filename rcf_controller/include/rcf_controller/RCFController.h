 /*
 * RCFController.h
 *
 *  Created on: Aug 13, 2015
 *
 *  RCF Coordinator Implementation
 */

#ifndef RCFCONTROLLER_H_
#define RCFCONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Char.h>


//External Libraries
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <array>

//Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

//RobCoGen
#include <iit/robots/hyq2max/kinematics_parameters.h>
#include <iit/robots/hyq2max/dynamics_parameters.h>
#include <iit/robots/hyq2max/inertia_properties.h>
#include <iit/robots/hyq2max/forward_dynamics.h>
#include <iit/robots/hyq2max/inverse_dynamics.h>
#include <iit/commons/dog/leg_data_map.h>
#include <iit/commons/dog/joint_id_declarations.h>


#include <iit/robots/hyq2max/traits.h>
#include <iit/robots/hyq2max/default_parameters_getter.h>
#include <iit/robots/hyq2max/jacobians.h>
#include <iit/robots/hyq2max/transforms.h>
#include <iit/rbd/rbd.h>

#include <iit/robots/hyq/transforms.h>
#include <iit/robots/hyq/default_parameters_getter.h>
#include <iit/robots/hyq/inertia_properties.h>
#include <iit/robots/hyq/jsim.h>


#include <dls_controller/Controller.h>
#include <math_utils/second_order_filter.h>

#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/model/WholeBodyDynamics.h>

#include <hyq2max_ik/ik.h>
#include <hyq2max_ik/RobotLengths.h>
#include <hyq_ik/ik.h>

//RCF HEADERS
#include "HyQ2MaxWorkspaceCheck.h"
#include "CPIDManager.h"
#include <central_pattern_generator/CCPG.h>
#include <kinematic_adjustment/CKinematicAdjustment.h>
#include <push_recovery/CPushRecovery2DOF.h>
#include <terrain_estimator/CTerrainEstimator.h>
#include <kalman_state_estimator/KalmanStateEstimator.h>
#include "CTerrainBasedAdjustment.h"
#include "CTrunkController.h"

//Max-plus algebra headers
#include "MPA.h"
#include "MPScheduler.h"

#include <math_utils/utils.h>



namespace dls_controller
{


class RCFController: public dls_controller::Controller
{
public:
	RCFController();
	virtual ~RCFController();

	bool construct(int nJoints,
				   int nFTSensors,
				   int taskRate,
				   std::string robot);

	bool init(double time);
	void run(double time,
			 double period);
	void kill();

	bool consoleCallFunction(std::string func_name);

	typedef void (RCFController::*function_pointer)();
	function_pointer funcPtr;

	struct console_command {
		std::string command;
		std::string comment;
		function_pointer func_ptr;
	};

	std::vector<console_command> menuFunctions;

	bool addFunction(const std::string command,const std::string comment,const function_pointer funcPtr);
	bool addPromptCommands();
	void manPrint(void);

	bool writeDebugVariablesOut(std::vector<double>& variables);


	enum Axis{ X = 0, Y = 1, Z = 2 };
	enum RPY{ Roll = 0 , Pitch = 1 , Yaw = 2 };
	enum legJointsID { HAA = 0 , HFE = 1 , KFE = 2};

	enum class filterResponse{slowC = 0, mediumC, fastC};

	struct runningTrotProfile {
	std::string name;
	double stepHeightNew;
	double VfX_New;
	double stepFrequencyNew;
	double dutyF_New;
	double KcoupNew;
	double bodyWeight;
	double D_Force;
	};

	enum class Pattern : int {
		Trot = 0,
		Walk = 1,
		Bound = 2
	};

	enum class Throttle : int {
		Slower = -1,
		Neutral = 0,
		Faster = 1
	};


private:

	rbd::Vector3d baseToHF(const rbd::Vector3d& pos);

	void computeFilteredChanges();

	void bodyVelocitiesEstimation(const iit::dog::LegDataMap<bool>& support_legs,
		Eigen::Matrix<double, 3, 1>& Xb_est_HF_aux);

	void computeBodyHeight(const rbd::Vector3d angles,
		const iit::dog::LegDataMap<rbd::Vector3d>& foot_pos,
		const rbd::Vector3d& trunk_velocities,
		const iit::dog::LegDataMap<bool>& support_legs, double& trunk_height);

	void stepCycleMeasurement(const double& foot_force,
		const double& task_freq, const double& task_time,
		const double& CPG_primite_LF_z, const iit::dog::LegDataMap<bool> support_leg,
		int& flight_phase, double& flight_phase_time, double& CPG_duty_factor,
		double& actual_duty_factor, double& actual_step_freq,
		double& actual_stance_per);

	void CPG_Modulation(iit::dog::LegDataMap<Eigen::Vector3d>& primitives,
		iit::dog::LegDataMap<Eigen::Vector3d>& primitives_d,
		iit::dog::LegDataMap<Eigen::Vector3d>& output_filter,
		iit::dog::LegDataMap<Eigen::Vector3d>& output_filter_d);

	void terrainAdjustment(double terrainRollAdj,
		double terrainPitchAdj, double& deltaRoll, double& deltaPitch);

	double sinusoidalMotion(bool on_off, const double desAmplitude,
		const double desFrequency, const double pushUpTime);

	double sinusoidalForce(bool on_off, const double desForceAmplitude,
	    const double desForceFrequency, const double forceTime);

	double chirpForce(const double& chirpTime);

	double squareForce(const double& squareTime);

	double triangularForce(const double& triangularTime);

	double sineForce(const double& sineTime);

	void computeTrunkControl();

	void switchRunningTrotProfile(const Throttle speed);

	void runningTrot(const unsigned int profile);

	void changeGaitPattern();

	void toggleTrunkControl();

	void toggleTerrainEstimationFlag();

	void toggleKinematicAdjustmentFlag();

	void walkStopping();

	void togglePushRecoveryFlag();

	void toggleWalkAdaptationFlag();

	bool isInfOrNan(const double& x);

	rbd::Vector3d computeVirtualLegPosition(const iit::dog::LegDataMap<rbd::Vector3d>& posHF);

	void computeDesTrunkWrench();

	rbd::Vector3d computeDesVirtualLegPosition(const iit::dog::LegDataMap<rbd::Vector3d>& posHF);

	void referencesBackTracingPrintOuts(const iit::dog::LegID leg);

	void walkStarting();

	void computeGRFfromLegTorquesBase(const iit::dog::JointState & q_curr,
		const iit::dog::JointState & qd_curr, const iit::dog::JointState & tau,
		iit::dog::LegDataMap<rbd::Vector3d> & feet_forces);

	void jump();

	void landingOrigins();

	void setPID(const double &HAA_gain, const double &HFE_gain, const double &KFE_gain,
            const double &HAA_gain_d, const double &HFE_gain_d, const double &KFE_gain_d);

	void selfRighting();

	void layDown();

	void setFootOriginsInstantaneouslyFromActualDesiredJointPositions();

	void computeDesiredAndActualRobotPosition();

	// ===================   CONSOLE FUNCTIONS   ==========================


	//void resetRCF_Odometry();

	void changeTrunkControllerGains();

	void changeKinematicDependency();

	void changeCPG_Params();

	void changePrecParameters();

	void changeTrunkOffset();

	void changeOrigins();

	void controlStatus();

	void menuOptions();

	void whereOrigins();

	void narrowStance();

	void wideStance();

	void changeSinParameters();

	void changeSinParametersForce();

	void changeChirpParameters();

	void changeSquareWaveParameters();

	void changeTriangularWaveParameters();

	void changeSineWaveParameters();

	void togglePushUpTask();

	void toggleInternalForcesTask();

	void toggleChirpTask();

	void toggleSquareWaveTask();

	void toggleTriangularWaveTask();

	void toggleSineWaveTask();

	void startRunningTrot();

	void changeStepFrequency(void);

	void changeForwardVelocity(void);

	void changeDutyCycle(void);

	void changeLegImpedanceDuringStancePhase(void);

	void changePID_Manager(void);

	void changeRCF_Parameters(void);

	void changeCOG(void);

	void interactiveChangeTrotTask(void);
	void interactiveChangeTaskForDemos(void);
	void joyChangeTrotTask(void);
	void joyChangeTrotTaskForVisitors(void);
	bool ICTPLogic(const std::string& str);
	bool ICTPLogicForDemos(const std::string& str);

	void imuPrint(void);
	void footSensorPrint(void);
	void desJointStatePrint(void);
	void jointStatePrint(void);
	void jointErrorsPrint(void);
	void toggleJump(void);
	void goSquat();
	void goBelly();
	void startSelfRighting();
	void startLayDown();
	void toggleForcedStance();

	void resetOriginsForTrotting();

	void setPID();

	void RCF_Setup();

	void resetRobotDesiredPosition();

	void togglePositionControl();

	void showTrunkGains();

	void changeTerrainAdjustments();

	void setImpedanceRoughTerrain();

	void resetImpedance();

	void evaluateCollisionDetection();

	void computeReflex(iit::dog::LegDataMap<rbd::Vector3d>& reflexPos, iit::dog::LegDataMap<rbd::Vector3d>& reflexVel);

	iit::dog::JointState computeTorquesForInternalForces(const iit::dog::LegDataMap<rbd::Vector3d>& footForces);

	void checkJoystickCommands();

	void toggleJoystick();

	/** @brief Ros node handle */
	ros::NodeHandle node_;

	/** @brief Ros subscriber */
	ros::Subscriber char_sub_;
	void charICTPCallback(const std_msgs::CharConstPtr& msg);

	// ===================   CLASSES   ==========================
	//RCF CLASSES
	CPushRecovery2DOF PushRecovery;
	CTerrainEstimator TerrainEstimator;
	CKinematicAdjustment KinAdjustment;
	KalmanStateEstimator stateEstimation;
	HyQ2MaxWorkspaceCheck workspaceCheck;
	CPIDManager PIDManager;
	//CRCFOdometry RCFOdometry;
	CTerrainBasedAdjustment terrainBasedAdjustment;
	CTrunkController trunkController;

	iit::dog::LegDataMap<CCPG> CPG;
	RobotLengths rl;

	// ===================   MaxPlusAlgebra OCTAVIO   ==========================

	MaxPlusSchedule schedule;
	Eigen::ArrayXXd eventsLog;
	Eigen::ArrayXXd xInitial;
	Eigen::Matrix<double, 1, 4> omegaVector;
// ===================   end of MaxPlus   ===================================

	/** @brief Floating-base system information */
	dwl::model::FloatingBaseSystem fbs_;

	/** @brief Whole-body dynamics system information */
	dwl::model::WholeBodyDynamics wbd_;

	/** @brief Joint id mapping between urdf to robcogen */
	std::vector<unsigned int> joint_id_map_;

	/** @brief Leg id mapping between urdf to robcogen */
	std::vector<std::string> leg_name_map_;

        std::shared_ptr<InverseKinematicsHyQ2Max> hyq2maxIK;
        std::shared_ptr<InverseKinematicsHyQ> 	  hyqIK;


	// Update Jacobians

	// iit::HyQ2Max::Jacobians jacobians;

	// //jacobians.fr_trunk_J_LF_foot;
	// HyQ2Max::Jacobians::Type_fr_trunk_J_LF_foot& JFootLF = jacobians.fr_trunk_J_LF_foot;// just a simpler alias
	// HyQ2Max::Jacobians::Type_fr_trunk_J_RF_foot& JFootRF = jacobians.fr_trunk_J_RF_foot;// just a simpler alias
	// HyQ2Max::Jacobians::Type_fr_trunk_J_LH_foot& JFootLH = jacobians.fr_trunk_J_LH_foot;// just a simpler alias
	// HyQ2Max::Jacobians::Type_fr_trunk_J_RH_foot& JFootRH = jacobians.fr_trunk_J_RH_foot;// just a simpler alias


	std::shared_ptr<iit::HyQ2Max::Jacobians> jacobiansHyQ2Max;
	std::shared_ptr<iit::HyQ::Jacobians> jacobiansHyQ;


	Eigen::Matrix<double, 6,3 >* JFootLF;
	Eigen::Matrix<double, 6,3 >* JFootRF;
	Eigen::Matrix<double, 6,3 >* JFootLH;
	Eigen::Matrix<double, 6,3 >* JFootRH;


	iit::HyQ2Max::dyn::RuntimeParamsGetter inertiasGetter;
	iit::HyQ2Max::ParamsGetter_lengths lengthsGetter;

	iit::HyQ2Max::DefaultParamsGetter default_pg;
	iit::HyQ::DefaultParamsGetter default_pg2;

	std::shared_ptr<iit::HyQ2Max::ForceTransforms> forceTransforms;
        std::shared_ptr<iit::HyQ2Max::dyn::InertiaProperties> linksInertia;
        std::shared_ptr<iit::HyQ2Max::dyn::JSIM> jsim;
        //(*linksInertia, *forceTransforms);
	//Initialize jsim class
        /*    dyn::JSIM local_jsim(*linksInertia, *forceTransforms);
              jsim = & local_jsim;*/

        iit::HyQ2Max::HomogeneousTransforms::MatrixType world_X_base;

	iit::dog::JointState q_;
	iit::dog::JointState qd_;
	iit::dog::JointState des_q_;
	iit::dog::JointState des_qd_;
	iit::dog::JointState des_qdd_;
	iit::dog::JointState tau_;
	iit::dog::JointState des_tau_;
	iit::dog::LegDataMap<iit::rbd::Vector3d> footSensor_;
	iit::dog::JointState uffTorques;

        std::array<Eigen::Matrix3d, 4> desJointState = {{Eigen::Matrix3d::Zero(),
                                                         Eigen::Matrix3d::Zero(),
                                                         Eigen::Matrix3d::Zero(),
                                                         Eigen::Matrix3d::Zero()}};


	std::vector<double> des_q_ori;
	std::vector<double> des_qd_ori;
	std::vector<double> des_tau_ori;
	int orig_des_spline = 0;

	Eigen::Affine3d initialEst;
	Eigen::Affine3d currentEst;


	//config files
	boost::property_tree::ptree config_;

	int ictpChar;
	double jumpForce = 0.0;
	double jumpForceNew = 0.0;
	bool initJumpFlag = false;
	bool endJumpFlag = true;
	bool jumpOff = false;
	double userJumpForce = 400;
	double deltaJumpHeight = 0.0;
	double heightMemory = 0.0;
	double liftOffHeight = 0.0;
	bool computeDeltaJumpHeight = false;
	double squatHeight = 0.35;

	Eigen::Matrix4d CouplMatrix;

	iit::dog::LegDataMap<bool> stanceLegs = false;
	iit::dog::LegDataMap<bool> forcedStanceLegs = false;
	int stanceLegsNumber = 0;
	double estTerrainRoll = 0;
	double estTerrainPitch = 0;
	double trunkHeight = 0.0;
	double desTrunkHeight = 0.6;
	double myTime = 0.0;
	double perturbationDuration = 0.0;
	double perturbationForce = 0.0;
	double perturbationTorque = 0.0;
	double lateralForce = 0.0;
	double yawTorque = 0.0;
	double maxCutOffFreq = 50;
	double minCutOffFreq = 10;
	bool useVariableCutOffFreq = false;
	bool precEnableX = true;
	bool precEnableY = true;
	bool precEnablePsi = true;
	double varianceEst = 0;
	double originAlphaFilter = 0.5;
	double integralActionRoll = 0.0;
	double integralActionPitch = 0.0;
	double safetyCheckError = 0.0;
	double profileTransitionTime = 1.0;
	double D_Force = 0.0;
	double ztdAv = 0.0;
	double ztdAvF = 0.0;
	double tdErrorComp = 0.0; //Compensate for touch-down errors
	double terrainRoll = 0.0;
	double terrainPitch = 0.0;
	double terrainPitchNew = 0.0;
	double Psit = 0.0;
	double PsitNew = 0.0;
	double flightPhaseTime = 0;
	int flightPhase = 0;
	double strideTime = 0;
	double actualDutyFactor = 0;
	double CPG_DutyFactor = 0;
	double actualStepFrequency = 0.0;
	double actualStancePeriod = 0.0;
	double forwVelInc = 0.05;
        double PsitInc = 5 * M_PI / 180;
	double PsitError = 0.0;
	double sinAmplitude = 0.05;
	double sinFrequency = 0.5;
	double sinForceAmplitude = 0.05;
	double sinForceFrequency = 0.5;
	double chirpAmplitude = 25.0;
	double chirpDuration = 10.0;
	double initialChirpFrequency = 0.0;
	double finalChirpFrequency = 25.0;
	double chirpRate = 2.5;
	double chirpTime0 = 0.0;
	double squareWaveOutput = 0.0;
	double squareWaveOutputFiltered = 0.0;
	double squareTime0 = 0.0;
	double squareWavePeriod = 1.0;
	double squareWaveAmplitude = 25;

	bool triangularWaveOnOffFlag = false;
	double triangularWaveOutput = 0.0;
	double triangularTime0 = 0.0;
	double triangularWavePeriod = 1.0;
	double triangularWaveAmplitude = 25;

    bool sineWaveOnOffFlag = false;
    double sineWaveOutput = 0.0;
    double sineTime0 = 0.0;
    double sineWavePeriod = 1.0;
    double sineWaveAmplitude = 25;

	bool considerGRF_Flag = true;
	bool simulatedRunFlag = true;
	double K_MeasurementKF = 1;
	double bodyWeightKF = 69.0;
	double alphaGammaNew = 40.0;
	double alphaGamma = 0.0;
	double KcNew = 60.0;
	double Kc = 0.0;
	double subStep = 20.0;
	double Kcoup = 0.0;
	double KcoupNew = 0.5;
	double dutyF = 0.5;
	double dutyF_New = 0.5;
	double precX = 1.0;
	double precY = 1.0;
	double precYaw = 1.0;
	double desiredTs_pRec = 0.0;
	double precTrunkWeight = 50.0;
	double inertialCompGain = 0.0;
	double rollAdj = 0;
	double pitchAdj = 0;
	double kadjRoll = 0.0;
	double kadjRollNew = 0.0;
	double kadjRollMemory = 0.75;
	double kadjPitch = 0.0;
	double kadjPitchNew = 0.0;
	double kadjPitchMemory = 0.75;
	double rollMaxKadj = 20 * M_PI / 180;
	double rollMaxKadjNew = 20 * M_PI / 180;
	double pitchMaxKadj = 10 * M_PI / 180;
	double pitchMaxKadjNew = 10 * M_PI / 180;
	double KpTrunkRoll = 0;
	double KpTrunkRollNew = 0;
	double KpTrunkPitch = 0;
	double KpTrunkPitchNew = 0;
	double KpTrunkYaw = 0;
	double KpTrunkYawNew = 0;
	double KpTrunkX = 0;
	double KpTrunkX_New = 0;
	double KpTrunkY = 0;
	double KpTrunkY_New = 0;
	double KpTrunkZ = 0;
	double KpTrunkZ_New = 0;
	double KdTrunkRoll = 0;
	double KdTrunkRollNew = 0;
	double KdTrunkPitch = 0;
	double KdTrunkPitchNew = 0;
	double KdTrunkYaw = 0;
	double KdTrunkYawNew = 0;
	double KdTrunkX = 0;
	double KdTrunkX_New = 0;
	double KdTrunkY = 0;
	double KdTrunkY_New = 0;
	double KdTrunkZ = 0;
	double KdTrunkZ_New = 0;

	double KpTrunkRollDefault = 0.0;
	double KpTrunkPitchDefault = 0.0;
	double KpTrunkYawDefault = 0.0;
	double KpTrunkX_Default = 0.0;
	double KpTrunkY_Default = 0.0;
	double KpTrunkZ_Default = 0.0;

	double KdTrunkRollDefault = 0.0;
	double KdTrunkPitchDefault = 0.0;
	double KdTrunkYawDefault = 0.0;
	double KdTrunkX_Default = 0.0;
	double KdTrunkY_Default = 0.0;
	double KdTrunkZ_Default = 0.0;

	double bodyWeight = 0.0;
	double bodyWeightNew = 0.0;
	double bodyWeightDefault = 0.0;

	double xForceOffset = 0.0;
	double xForceOffsetNew = 0.0;
	double yForceOffset = 0.0;
	double yForceOffsetNew = 0.0;
	double zForceOffset = 0.0;
	double zForceOffsetNew = 0.0;
	double xMomentOffset = 0.0;
	double xMomentOffsetNew = 0.0;
	double yMomentOffset = 0.0;
	double yMomentOffsetNew = 0.0;
	double zMomentOffset = 0.0;//-3.0;
	double zMomentOffsetNew = 0.0;//-3.0;
	double Kbp = 50.0;
	double KbpNew = 50.0;
	double Kbf = 50.0;
	double KbfNew = 50.0;
	double Kbv = 50.0;
	double KbvNew = 50.0;
	double Kvf = 1.0;
	double KvfNew = 1.0;
	double couplingLF = 0.0;
	double couplingRF = 0.0;
	double couplingLH = 0.0;
	double couplingRH = 0.0;
	double stepHeightNew = 0.001;
	double stepHeight = 0.001;
	double stepLengthNew = 0.001;
	double stepLength = 0.001;
	double forwVelNew = 0.001;
	double forwVel = 0.001;
	double VfX_New = 0.001;
	double VfX = 0.001;
	double VfY_New = 0.0;
	double VfY = 0.0;
	double stepFrequencyNew = 1.7;
	double stepFrequency = 1.7;
	double desRollAngle = 0.0;
	double desPitchAngle = 0.0;
	double desYawAngle = 0.0;
	double robotYaw = 0.0;
	double lastYaw = 0.0;
	iit::dog::LegDataMap<double> zTd = -0.01;
	iit::dog::LegDataMap<bool>adapWalkFlag = false;
	bool attitudeFlag = false;
	bool walkAdaptationFlag = false;
	bool pushRecoveryFlag = false;
	bool kinAdjustmentFlag = false;
	bool terrainEstimationFlag = false;
	bool walkingTrotFlag = false;
	bool runningTrotFlag = false;
	bool jumpFlag = false;
	bool resetCPGFlag = false;
	bool integralAction = false;
	bool inProfileTransitionFlag = true;
	bool desStateFlag = false;
	bool freezeRatioFlag = false;
	bool oooFlag = true;
	bool sinOnOffFlag = false;
	bool sinOnOffForceFlag = false;
	bool chirpOnOffFlag = false;
	iit::dog::LegDataMap<double> internalForceGain = 1.0;
	bool squareWaveOnOffFlag = false;
	bool fallDetectionFlag = false;
	bool executeSelfRightingFlag = false;
	bool executeLayDownFlag = false;
	bool inertialCompFlag = true;
	bool keyPressedFlag = false;
	bool exitFlag = false;
	bool constrainedInvDynFlag = false;
	bool invDynOnlyForGravCompFlag = false;
	bool CPG_StepHeightModulationFlag = false;
	int ctcOption = 1;
	filterResponse originChangingRate = filterResponse::mediumC;
	bool instantaneousOriginChangesFlag = false;
	bool positionControlFlag = false;

	bool forcedStanceFlag = false;
	bool useRCF_StateEstimator = true;
	bool useStanceHolderFlag = false;
	bool frontalCollisionFlag = false;
	bool useCollisionDetectionFlag = false;
	bool useStepReactionFlag = false;
	bool useJoystickFlag = false;

        //For flight phase analysis
        double flight_phase_last = 2;
        double flight_phase_starting_time = 0;
        double flight_phase_ending_time = 0;
        //Step frequency measurement
        bool leg_touch_down = false;
        double step_period = 1.0;
        double force_threshold = 20;


        //For CPG analysis
        bool CPG_swing_phase = false;
        bool CPG_swing_phase_last = false;
        double CPG_cicle_time = 0;
        double CPG_swing_phase_time = 0;
        double CPG_swing_phase_starting_time = 0;

	Eigen::Matrix<Eigen::Matrix<double, 4, 4>, 4, 1> couplingMatrix;
	Eigen::Matrix<double, 3, 1> lastXbEstHFKinIMU;

	rbd::Vector3d forceCov;
	rbd::Vector3d kinematicsCov;
	rbd::Vector3d IMU_Cov;
	rbd::Vector3d IMU_OffsetHF;
	rbd::Vector3d kinematicsOffset;

	Eigen::Matrix<double, 4, 1> deltaX;
	Eigen::Matrix<double, 4, 1> deltaY;
	LegDataMap<rbd::Vector3d> deltaPR_Pos, deltaPR_Vel;


	Pattern pattern;

	iit::dog::LegDataMap<rbd::Vector3d> P_Local_BF;
	iit::dog::LegDataMap<rbd::Vector3d> Pd_Local_BF;
	iit::dog::LegDataMap<rbd::Vector3d> Pdd_Local_BF;
	iit::dog::LegDataMap<rbd::Vector3d> P_DesLocal_BF;
	iit::dog::LegDataMap<rbd::Vector3d> Pd_DesLocal_BF;
	iit::dog::LegDataMap<rbd::Vector3d> Pdd_DesLocal_BF;
	iit::dog::LegDataMap<rbd::Vector3d> last_Pd_Local_BF;

	iit::dog::LegDataMap<rbd::Vector3d> P_Local_HF;
	iit::dog::LegDataMap<rbd::Vector3d> Pd_Local_HF;
	iit::dog::LegDataMap<rbd::Vector3d> Pdd_Local_HF;
	iit::dog::LegDataMap<rbd::Vector3d> P_DesLocal_HF;
	iit::dog::LegDataMap<rbd::Vector3d> Pd_DesLocal_HF;
	iit::dog::LegDataMap<rbd::Vector3d> Pdd_DesLocal_HF;
	iit::dog::LegDataMap<rbd::Vector3d> P0_Local_HF;
	iit::dog::LegDataMap<rbd::Vector3d> P0_Local_HF_New;
	iit::dog::LegDataMap<rbd::Vector3d> P0_Local_HF_Default;
	iit::dog::LegDataMap<rbd::Vector3d> last_P_Local_HF;
	iit::dog::LegDataMap<rbd::Vector3d> last_Pd_Local_HF;
	iit::dog::LegDataMap<rbd::Vector3d> last_Pd_DesLocal_HF;

	iit::dog::LegDataMap<rbd::Vector3d> PdDesLocalOld;

	iit::dog::LegDataMap<rbd::Vector3d> P_DesFoot;
	iit::dog::LegDataMap<rbd::Vector3d> PdDesFoot;
	iit::dog::LegDataMap<rbd::Vector3d> PdDesFootOld;

	iit::dog::LegDataMap<rbd::Vector3d> P_Reflex_HF;
	iit::dog::LegDataMap<rbd::Vector3d> Pd_Reflex_HF;

	rbd::Vector3d virtualLegPosition;
	rbd::Vector3d desVirtualLegPosition;
	rbd::Vector3d desRobotPositionWF;
	rbd::Vector3d actualRobotPositionWF;
	rbd::Vector3d desRobotPositionHF;
	rbd::Vector3d actualRobotPositionHF;
	rbd::Vector3d robotPositionErrorHF;
	double desRobotYaw;

	Eigen::Matrix<double, 3, 1> deltaCOG;


	std::vector<double> zeroGainTh;
	std::vector<double> zeroGainThd;
	std::vector<double> zeroGainInt;

	std::vector<double> tauDeltaPos;
	std::vector<double> deltaUff;

	std::array<double,3> forceDeltaPos;
	std::array<double,3> tauDeltaOrient;
	Eigen::Matrix<double, 6, 1> desTrunkWrench;


	std::array<runningTrotProfile, 9> trotProfiles;

	rbd::Vector3d eulerAngles;
	rbd::Vector3d eulerAnglesNew;

	Eigen::Matrix<double, 4, 1> estimTesZd;
	Eigen::Matrix<double, 3, 1> estBodyVelHF;
	Eigen::Matrix<double, 3, 1> estBodyVelHF_Check;
	Eigen::Matrix<double, 3, 1> defaultEstBodyVelHF;

	iit::dog::LegDataMap<rbd::Vector3d> WCPG;
	iit::dog::LegDataMap<rbd::Vector3d> WCPGb;
	iit::dog::LegDataMap<rbd::Vector3d> dWCPGb;
	iit::dog::LegDataMap<rbd::Vector3d> dWCPG;

	iit::dog::LegDataMap<rbd::Vector3d> jointDesPos;
	iit::dog::LegDataMap<rbd::Vector3d> jointDesVel;
	iit::dog::LegDataMap<rbd::Vector3d> jointDesAcc;

	iit::dog::JointState tauInertiaComp;

	bool setPID_Flag = false;

	iit::dog::LegDataMap<rbd::Vector3d> PDeltaKadj;
	iit::dog::LegDataMap<rbd::Vector3d> PdDeltaKadj;

	Eigen::Matrix<double, 6, 1> axes_memory;
	Eigen::Matrix<int, 12, 1> buttons_memory;
	bool roughTerrainMode = false;
	double VfX_buffer = 0.01;
	double VfY_buffer = 0.0;
	double Psit_buffer = 0.0;

	unsigned int gaitPattern;


};


}//@namespace dls_controller




 #endif /* RCFCONTROLLER_H_ */
