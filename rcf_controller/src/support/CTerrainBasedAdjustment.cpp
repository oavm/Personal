/*
 * CTerrainBasedAdjustment.cpp
 *
 *  Created on: Aug 22, 2015
 *      Author: victor
 */

#include "CTerrainBasedAdjustment.h"

CTerrainBasedAdjustment::CTerrainBasedAdjustment() {

	terrainRoll = 0.0;
	terrainPitch = 0.0;
	maxRoll = 0.8;
	maxPitch = 0.8;
	adjustmentFactor = 0.75;
	adjPostureAngle.setZero();
	//footOrigins(0);
	//primitivesPos(0);
	//primitivesVel(0);
	//adjustedPrimitivesPos(0);
	//adjustedPrimitivesVel(0);
	//deltaOrigins(0);
	desTrunkVelocity.setZero();
	adjustedDesTrunkVel.setZero();
	taskPeriod = 1.0/250.0;
	filterTimeConstant = 0.001;


}

CTerrainBasedAdjustment::~CTerrainBasedAdjustment() {
	// TODO Auto-generated destructor stub
}


void CTerrainBasedAdjustment::setMaximumAdjustmentAngles(const double& mRoll,
																			const double& mPitch){

	maxRoll = mRoll;
	maxPitch = mPitch;

}


void CTerrainBasedAdjustment::setAdjustmentFactor(const double adjRate) {

	adjustmentFactor = adjRate;
}

void CTerrainBasedAdjustment::setTaskFrequency(const double& taskFreq) {

    taskPeriod = 1.0 / taskFreq;
}


void CTerrainBasedAdjustment::computeAdjustment(void){

	rbd::Matrix33d mRot;

	mRot(0, 0) = cos(terrainPitch);
	mRot(0, 1) = sin(terrainPitch) * sin(terrainRoll);
	mRot(0, 2) = cos(terrainRoll) * sin(terrainPitch);
	mRot(1, 0) = 0.0;
	mRot(1, 1) = cos(terrainRoll);
	mRot(1, 2) = -sin(terrainRoll);
	mRot(2, 0) = -sin(terrainPitch);
	mRot(2, 1) = cos(terrainPitch) * sin(terrainRoll);
	mRot(2, 2) = cos(terrainPitch) * cos(terrainRoll);

	for(int leg = dog::LF; leg <= dog::RH; leg++){

		adjustedPrimitivesPos[leg] = mRot * primitivesPos[leg];
		adjustedPrimitivesVel[leg] = mRot * primitivesVel[leg];
	}


	//Changing the origins to fit the terrain
	rbd::Vector3d auxVector;


	for(int leg = dog::LF; leg <= dog::RH; leg++){

		auxVector << footOrigins[leg](rbd::X), footOrigins[leg](rbd::Y), 0;
		auxVector = mRot * auxVector;
		deltaOrigins[leg] << auxVector(rbd::X) - footOrigins[leg](rbd::X),
							 auxVector(rbd::Y) - footOrigins[leg](rbd::Y),
							 auxVector(rbd::Z);
	}

	//Rotated desired trunk velocity.
	adjustedDesTrunkVel = mRot * desTrunkVelocity;

	//Compute trunk posture adjustment angles
	double alphaFilter = taskPeriod / (filterTimeConstant + taskPeriod);

	adjPostureAngle(rbd::X) = (1.0 - alphaFilter) * adjPostureAngle(rbd::X) + alphaFilter * adjustmentFactor * terrainRoll;
	if(adjPostureAngle(rbd::X) > maxRoll) {
		adjPostureAngle(rbd::X) = maxRoll;
	}
	if(adjPostureAngle(rbd::X) < -maxRoll) {
		adjPostureAngle(rbd::X) = -maxRoll;
	}

	adjPostureAngle(rbd::Y) = (1.0 - alphaFilter) * adjPostureAngle(rbd::Y) + alphaFilter * adjustmentFactor * terrainPitch;
	if(adjPostureAngle(rbd::Y) > maxPitch) {
		adjPostureAngle(rbd::Y) = maxPitch;
	}
	if(adjPostureAngle(rbd::Y) < -maxPitch) {
		adjPostureAngle(rbd::Y) = -maxPitch;
	}

}
