/*
 * TrunkControllerGrfOptimization.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: mfocchi
 */

#include <trunk_controller/TrunkControllerGrfOptimization.h>
#include <stdio.h>
#include <iostream>
//default getter
#include <iit/robots/hyq/default_parameters_getter.h>
#include <iit/robots/hyq/declarations.h>
#include <iit/robots/hyq/transforms.h>

#include <iit/robots/hyq/jacobians.h>
#include <iit/robots/hyq/inertia_properties.h>
#include <iit/commons/dog/leg_data_map.h>
#include <iit/robots/hyq/inverse_dynamics.h>

#include <dls_controller/support/ConsoleUtility.h> //newline


using namespace Eigen;
using namespace std;
using namespace iit;
using namespace dog;

TrunkControllerGrfOptimization trunkController;
TrunkControllerGrfOptimization::ConstrViolationStatic constraints_violation;
HyQ::DefaultParamsGetter default_pg;
HyQ::dyn::InertiaProperties inertia_properties(default_pg);
HyQ::HomogeneousTransforms hom_transforms(default_pg);
HyQ::MotionTransforms motion_transforms(default_pg);
HyQ::dyn::InverseDynamics idObj(inertia_properties, motion_transforms);
HyQ::Jacobians jacs(default_pg);
dog::LegDataMap< Eigen::Matrix<double, 6,3 > > JFoot;
dog::LegDataMap<Vector3d> footPos;
Matrix3d R, terr_R_w;
HyQ::JointState q,qd;
double roll, pitch, yaw, terrRoll = 0.0, terrPitch = 0.0;
double friction_coeff =0.8;
double force_lim_normal = 100000;
Eigen::Matrix<double, 12,1> feet_forces;
Matrix<double,12,1>torques;
rbd::ForceVector desWrench;
dog::LegDataMap<Vector3d> vec_incl(Eigen::Vector3d(0.0, 0.0, 1.0));

int main()
{
	//set joint position/velocity
	q << 0.0 ,  0.75 ,  -1.5,   -0.0   , 0.75  , -1.5  , 0.0,   -0.75  ,  1.5 ,  -0.0,   -0.75,   1.5;
	roll = 0.0;
	pitch = 0.0;
	yaw = 0.0;

	newline::getDouble("friction_coeff:", friction_coeff,friction_coeff);
	newline::getDouble("terrRoll:", terrRoll,terrRoll);
	newline::getDouble("terrPitch:", terrPitch,terrPitch);
	newline::getDouble("force_lim_normal (for LF):", force_lim_normal,force_lim_normal);


	//base orientation
	R = trunkController.rpyToRot(Eigen::Vector3d(roll, pitch, yaw));
	terr_R_w = trunkController.rpyToRot(Eigen::Vector3d(terrRoll, terrPitch, yaw)); //this is the mapping between world and terrain frame

	//update jacs
	jacs.fr_trunk_J_LF_foot(q);
	jacs.fr_trunk_J_RF_foot(q);
	jacs.fr_trunk_J_LH_foot(q);
	jacs.fr_trunk_J_RH_foot(q);
	JFoot[dog::LF] =  jacs.fr_trunk_J_LF_foot;// just a simpler alias
	JFoot[dog::RF] =  jacs.fr_trunk_J_RF_foot;// just a simpler alias
	JFoot[dog::LH] =  jacs.fr_trunk_J_LH_foot;// just a simpler alias
	JFoot[dog::RH] =  jacs.fr_trunk_J_RH_foot;// just a simpler alias

	//get trasform at the actual state            //compute foot position in base space via fwd kinematics
	hom_transforms.fr_trunk_X_LF_foot(q);         footPos[dog::LF]=rbd::Utils::positionVector(hom_transforms.fr_trunk_X_LF_foot);
	hom_transforms.fr_trunk_X_RF_foot(q);         footPos[dog::RF]=rbd::Utils::positionVector(hom_transforms.fr_trunk_X_RF_foot);
	hom_transforms.fr_trunk_X_LH_foot(q);         footPos[dog::LH]=rbd::Utils::positionVector(hom_transforms.fr_trunk_X_LH_foot);
	hom_transforms.fr_trunk_X_RH_foot(q);         footPos[dog::RH]=rbd::Utils::positionVector(hom_transforms.fr_trunk_X_RH_foot);


	//1) set min method (NORMALS, WRENCH, TORQUES)
	trunkController.setMinimizationMethod(TrunkControllerGrfOptimization::NORMALS);
	//2) set stance legs (stance = true)
	dog::LegDataMap<bool>  stance_legs = true;
	//3) set contact surface inclination //change the normal according to terrain
	vec_incl = terr_R_w.transpose()*Eigen::Vector3d(0,0,1);

	//if you want to set specular normals for hind and front legs uncomment
	//	terr_R_w = trunkController.rpyToRot(Eigen::Vector3d(terrRoll, -terrPitch, yaw)); //this is the mapping between world and terrain frame
	//	vec_incl[LH] =  terr_R_w.transpose()*Eigen::Vector3d(0,0,1);
	//	vec_incl[RH] =  terr_R_w.transpose()*Eigen::Vector3d(0,0,1);

	//4) set friction cone limits on cone normals
	dog::LegDataMap<double> high_force_limit(10000.0);
	dog::LegDataMap<double> low_force_limit(5.0);
	//set a specific force limit on lF
	high_force_limit[LF] = force_lim_normal;
	//5) set friction cone (0 means cone degenerates to a line)
	dog::LegDataMap<double> muEstimate(friction_coeff);

	//6) set your desired wrench
	desWrench.setZero();
	desWrench(rbd::LZ) = 9.8*75; //gravity

	//compute inv dyn torques
	iit::rbd::VelocityVector gW, gB;
	gW <<0.0, 0.0, 0.0, 0.0, 0.0, -iit::rbd::g; gB.setZero();
	gB.segment(rbd::LX,3) = R*gW.segment(rbd::LX,3);
	rbd::ForceVector baseWrench; HyQ::JointState  h_joints;
	idObj.id_fully_actuated(baseWrench, h_joints, gB, rbd::VelocityVector::Zero(), rbd::VelocityVector::Zero(), q, HyQ::JointState::Zero(), HyQ::JointState::Zero());

	trunkController.setActualFeetPos(footPos);
	trunkController.setFeetJacobians(JFoot[LF], JFoot[RF], JFoot[LH], JFoot[RH]);
	trunkController.setTrunkAttitude(roll, pitch, yaw);
	trunkController.setStanceLegs(stance_legs);
	trunkController.setInvDynTorques(h_joints);
	trunkController.setTrunkWrench(desWrench);
	trunkController.computeOptimization(vec_incl,muEstimate, high_force_limit, low_force_limit, Vector3d(0.0,0.0,0.0));
	trunkController.getJointTorques(torques);
	std::cout<<std::endl;
	std::cout<<"torques:      "<<torques.transpose()<<std::endl;

	//debug
	trunkController.getConstraintViolations(constraints_violation);
	trunkController.getFeetForces(feet_forces);
	std::cout<<"feet forces:  "<<feet_forces.transpose()<<std::endl;
	//these are the normals
	std::cout<<"normals:  "<<vec_incl[LF].transpose()<<std::endl;
	std::cout<<"normals:  "<<vec_incl[RF].transpose()<<std::endl;
	std::cout<<"normals:  "<<vec_incl[LH].transpose()<<std::endl;
	std::cout<<"normals:  "<<vec_incl[RH].transpose()<<std::endl;

	std::cout<<"If the wrench is realizable the taskCost should be zero"<<std::endl;
	trunkController.printCosts();
	//	std::cout<<constraints_violation.friction<<std::endl;
	//	std::cout<<constraints_violation.unilateral<<std::endl;
	//	Eigen::VectorXd slacks;
	//	trunkController.getSlacks(slacks);
}
