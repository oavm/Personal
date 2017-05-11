/*
 * WholeBodyOptimizationStatic.h
 *
 *  Created on: Jul 28, 2014
 *      Author: mfocchi
 */

#ifndef TrunkControllerGrfOptimization_H_
#define TrunkControllerGrfOptimization_H_

#include <Eigen/Dense>
#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>
#include "eiquadprog.hpp"
#include <iit/commons/dog/leg_data_map.h>
#include <iit/commons/dog/joint_id_declarations.h>

using namespace std;
using namespace iit;

typedef Eigen::Matrix<double, 6, 3> legJac;

class TrunkControllerGrfOptimization {

public:

	struct ConstrViolationStatic {
      dog::LegDataMap<double> friction;
      dog::LegDataMap<double> unilateral;
	  //ConstrViolation() : friction(0), unilateral(0), torque(0), joint(0) {}
	  ConstrViolationStatic(): friction(0), unilateral(0){};
	};

	enum MinMethod{WRENCHDIRECTION =0, NORMALS, TORQUES}; //TODO Torque

	int _LEGS_COUNT = 4;
	int contactConstrCount = 3;
	int jointsLegCount = 3;
	/**
     * Default constructor
     */
	TrunkControllerGrfOptimization();
	~TrunkControllerGrfOptimization();

	//the convention for the optimization variables is that lambda are defined in the world frame
	//desired wrenches are in world frame
	void computeOptimization(const dog::LegDataMap<Eigen::Vector3d> & surf_normal,//TODO use dogstate to reduce state dimension
							const dog::LegDataMap<double> & muEstimate,
							const dog::LegDataMap<double> & force_max,
							const dog::LegDataMap<double> & force_min,
							const Eigen::Vector3d & com);

	//traits for getters//
	void getFeetForces(Eigen::Matrix<double, 12,1> & feet_forces);
	rbd::Vector6D getWrenchError();
	void getConstraintViolations(TrunkControllerGrfOptimization::ConstrViolationStatic & constraints_violation);
	double getSlack();
	void getSlacks(Eigen::VectorXd  & slacks_out);
	void getJointTorques(Eigen::Matrix<double, 12,1> & jointTorques);

	//traits for setters//
	void setMinimizationMethod(MinMethod min_goal);
	void setWrenchWeight(rbd::Vector6D & W_wrench);
	void setContactForceWeight(Eigen::Vector3d & W_forces);
	void setTorqueWeight(Eigen::Vector3d & W_torques);
	void setFrictionConstraint(bool frictionConstrFlag);
	void setBaseControl(bool baseControlFlag);
	void printCosts();

	void setActualFeetPos(const Eigen::Matrix<double, 3, 1>& actualFootPosLF,
	  	  	  	   	   	  const Eigen::Matrix<double, 3, 1>& actualFootPosRF,
	  	  	  	   	   	  const Eigen::Matrix<double, 3, 1>& actualFootPosLH,
	  	  	  	   	   	  const Eigen::Matrix<double, 3, 1>& actualFootPosRH);
	void setActualFeetPos(const dog::LegDataMap<rbd::Vector3d>& actualFootPosition);
	void setFeetJacobians(const legJac& JFootLF, const legJac& JFootRF,
			                 const legJac& JFootLH, const legJac& JFootRH);
	void setFeetJacobians(const dog::LegDataMap<legJac>& allJ_Leg);
	void setTrunkAttitude(const double& tRoll, const double& tPitch, const double& tYaw);
	void setStanceLegs(const dog::LegDataMap<bool>& stance);
	void setTrunkWrench(const Eigen::Matrix<double, 6,1>& trunkW);
	void setInvDynTorques(const Eigen::Matrix<double, 12,1>& h_joints);

	//utils methods
	Eigen::Matrix3d rpyToRot(const Eigen::Vector3d & rpy);
	Eigen::Matrix3d inline  skew_sim(const Eigen::Vector3d& w);
	Eigen::Vector3d getLegJointsState(dog::LegID leg, const Eigen::Matrix<double,12,1>& jstate);
protected:
	int compute_stance_legs( const dog::LegDataMap<bool> & stance_legs);
	//optimization methods
	void setCostFunction(Eigen::MatrixXd & GQ, Eigen::MatrixXd& W, Eigen::VectorXd& g0);
	void setEqualities(Eigen::MatrixXd& CE, Eigen::VectorXd& ce0);
	void setInequalities(Eigen::MatrixXd& CI, Eigen::VectorXd& ci0);
	void computeConstraintViolations();
    void computeCosts(const Eigen::VectorXd & solution,  double & taskCost, double & quadCost);
    void computeSlackCost(const Eigen::VectorXd & slacks, double & slackCost);

	//constructor variables

	//input variable for optimization
	Eigen::Matrix3d R;
	rbd::Vector6D desWrench;
    dog::LegDataMap<Eigen::Vector3d>  surf_normal;
    dog::LegDataMap<Eigen::Vector3d>  actualFootPos, actualFootVel;
    iit::rbd::VelocityVector baseTwist;
	dog::LegDataMap<double> force_max, force_min;
	dog::LegDataMap<double> muEstimate;
	int number_of_slacks;
	bool use_slacks, use_multiple_slacks;

    //optimization variables (we just optimize for leg variables)
    Eigen::Matrix<double, 12,1 >  jointTorques;
    Eigen::Matrix<double, 12,1 >  feet_forces;
	Eigen::Vector3d com;

    //user defined variables
	MinMethod min_goal;
	Eigen::Vector3d W_forces, W_torques;
	rbd::Vector6D W_wrench;
	bool frictionConstrFlag;
	bool baseControlFlag;

	//constraint violations
	ConstrViolationStatic constr_violation;

	//internal variables
	int cleg_count, contact_forces;	//number of stance legs
    int friction_constr,num_ineq;

    static const int num_cc = 6; //number of friction constraints per leg

	//internal variables for Whole body Optimization
	Eigen::MatrixXd GQ, W, CI, CE, A; //A is used for computing wrencherror
	Eigen::VectorXd g0, ce0, ci0, x, slacks, solution;

    Eigen::Matrix<double, 6, 6> S;
    Eigen::Matrix<double,6, 1> b;

    //for debug
    double quadCost;
    double taskCost;
    double slackCost;

	double trunkPitch, trunkRoll, trunkYaw;
	dog::LegDataMap<rbd::Matrix33d> J_Leg;
	dog::LegDataMap<bool> stance_legs;
	enum legID {LF=0, RF, LH, RH};
	Eigen::Matrix<double, 12,1>  h_joints;
};

#endif /* WholeBodyOptimizationStatic_H_ */
