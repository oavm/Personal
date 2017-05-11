/*
 * CRCFOdometry.cpp
 *
 *  Created on: Feb 23, 2015
 *      Author: Victor Barasuol
 */

#include "CRCFOdometry.h"

using namespace std;

CRCFOdometry::CRCFOdometry() {

	computeFilteredOutputsFlag_ = false;

	XPosWF0 = 0.0;
	YPosWF0 = 0.0;
	ZPosWF0 = 0.0;

	XPosWF0_f = 0.0;
	YPosWF0_f = 0.0;
	ZPosWF0_f = 0.0;

	dt_ = 0.01;
	alphaF_ = 0.0;
	trunkVelHF_.setZero();
	trunkVelHF_f_.setZero();
	trunkAngles_.setZero();
	trunkPositionWF.setZero();
	trunkPositionHF.setZero();
	trunkPositionYaw0.setZero();
	trunkPositionWF_f.setZero();
	trunkPositionHF_f.setZero();
	trunkPositionYaw0_f.setZero();

}

CRCFOdometry::~CRCFOdometry() {
	// TODO Auto-generated destructor stub
}


void CRCFOdometry::setTrunkVelocitiesHF(const rbd::Vector3d& trunk_velocities) {
	trunkVelHF_ = trunk_velocities;
}


void CRCFOdometry::setTrunkRollPitchYaw(const rbd::Vector3d& trunk_angles){
	trunkAngles_ = trunk_angles;
}


void CRCFOdometry::setTaskFrequency(const double& task_frequency){

	if(task_frequency > 0.0) {
		dt_ = 1.0 / task_frequency;
	}
	else {
		cout << "Task frequency must be greater than 0!!!" << endl;
	}

}

void CRCFOdometry::resetOdometry(void){
	trunkPositionWF.setZero();
	trunkPositionHF.setZero();
	yaw0_ = trunkAngles_(rbd::Z);
}


rbd::Vector3d CRCFOdometry::getTrunkPositionWF(void){
	return trunkPositionWF;
}


rbd::Vector3d CRCFOdometry::getTrunkPositionHF(void){
	return trunkPositionHF;
}


rbd::Vector3d CRCFOdometry::getTrunkPositionYaw0(void){
	return trunkPositionYaw0;
}


double CRCFOdometry::getXPosWF(void){
	return trunkPositionWF(rbd::X);
}


double CRCFOdometry::getYPosWF(void){
	return trunkPositionWF(rbd::Y);
}


double CRCFOdometry::getZPosWF(void){
	return trunkPositionWF(rbd::Z);
}


double CRCFOdometry::getXPosHF(void){
	return trunkPositionHF(rbd::X);
}


double CRCFOdometry::getYPosHF(void){
	return trunkPositionHF(rbd::Y);
}


double CRCFOdometry::getZPosHF(void){
	return trunkPositionHF(rbd::Z);
}


double CRCFOdometry::getXPosYaw0(void){
	return trunkPositionYaw0(rbd::X);
}


double CRCFOdometry::getYPosYaw0(void){
	return trunkPositionYaw0(rbd::Y);
}


double CRCFOdometry::getZPosYaw0(void){
	return trunkPositionYaw0(rbd::Z);
}

void CRCFOdometry::setYaw0(void){
	yaw0_ = trunkAngles_(rbd::Z);
}

void CRCFOdometry::setFilterTimeConstant(const double& tf){

	if (tf > 0) {
		alphaF_ = dt_ / (tf + dt_);
	}

}

void CRCFOdometry::useFilteredOutputs(const bool& compute_filtered_outputs){
	computeFilteredOutputsFlag_ = compute_filtered_outputs;
}

rbd::Matrix33d CRCFOdometry::getYawRotation(const double& angle){

	rbd::Matrix33d yawRot;

	yawRot << cos(angle), -sin(angle), 0.0,
			  sin(angle),  cos(angle), 0.0,
			         0.0,         0.0, 1.0;

	return yawRot;
}

void CRCFOdometry::computeOdometry(void){

	trunkPositionHF = trunkPositionHF + dt_ * trunkVelHF_;

	trunkPositionWF = trunkPositionWF + dt_ * getYawRotation(trunkAngles_(rbd::Z)) * trunkVelHF_;

	trunkPositionYaw0 = getYawRotation(yaw0_).transpose() * trunkPositionWF;

	XPosWF0 = trunkPositionYaw0(rbd::X);
	YPosWF0 = trunkPositionYaw0(rbd::Y);
	ZPosWF0 = trunkPositionYaw0(rbd::Z);

	if(computeFilteredOutputsFlag_) {

		trunkVelHF_f_ = (1.0 - alphaF_) * trunkVelHF_f_ + alphaF_ * trunkVelHF_;

		trunkPositionHF_f = trunkPositionHF_f + dt_ * trunkVelHF_f_;

		trunkPositionWF_f = trunkPositionWF_f + dt_ * getYawRotation(trunkAngles_(rbd::Z)) * trunkVelHF_f_;

		trunkPositionYaw0_f = getYawRotation(yaw0_).transpose() * trunkPositionWF_f;

		XPosWF0_f = trunkPositionYaw0_f(rbd::X);
		YPosWF0_f = trunkPositionYaw0_f(rbd::Y);
		ZPosWF0_f = trunkPositionYaw0_f(rbd::Z);
	}

}
