/*
 * CTerrainBasedAdjustment.h
 *
 *  Created on: Aug 22, 2015
 *      Author: victor
 */

#ifndef CTERRAINBASEDADJUSTMENT_H_
#define CTERRAINBASEDADJUSTMENT_H_

#include <iostream>
#include <Eigen/Dense>
#include <iit/commons/dog/leg_data_map.h>
#include <iit/rbd/rbd.h>

using namespace std;
using namespace iit;

class CTerrainBasedAdjustment {
public:
	CTerrainBasedAdjustment();
	virtual ~CTerrainBasedAdjustment();


	void setTerrainAngles(const double&, const double&);
	void setTrajectoryOriginsInTheHF(const dog::LegDataMap<rbd::Vector3d>&);
	void setPrimitivesPos(const dog::LegDataMap<rbd::Vector3d>&);
	void setPrimitivesVel(const dog::LegDataMap<rbd::Vector3d>&);
	void setMaximumAdjustmentAngles(const double&, const double&);
	void setAdjustmentFactor(const double adjRate);
	void setDesiredTrunkVelocityHF(const rbd::Vector3d&);
	void setTaskFrequency(const double&);
	void computeAdjustment();
	dog::LegDataMap<rbd::Vector3d> getAdjustedPrimivesPos();
	dog::LegDataMap<rbd::Vector3d> getAdjustedPrimivesVel();
	rbd::Vector3d getAdjustedDesTrunkVelHF(const rbd::Vector3d&);
	rbd::Vector3d getTrunkPostureAdjustmentAngles();
	dog::LegDataMap<rbd::Vector3d> getDeltaFootOriginsHF();

public:
	double terrainRoll, terrainPitch, maxRoll, maxPitch, adjustmentFactor;
	double taskPeriod, filterTimeConstant;
	rbd::Vector3d adjPostureAngle;
	dog::LegDataMap<rbd::Vector3d> footOrigins, primitivesPos, primitivesVel;
	dog::LegDataMap<rbd::Vector3d> adjustedPrimitivesPos, adjustedPrimitivesVel;
	dog::LegDataMap<rbd::Vector3d> deltaOrigins;
	rbd::Vector3d adjustedDesTrunkVel, desTrunkVelocity;


};

inline void CTerrainBasedAdjustment::setTerrainAngles(const double& tRoll, const double& tPitch) {

	terrainRoll = tRoll;
	terrainPitch = tPitch;

}

inline void CTerrainBasedAdjustment::setTrajectoryOriginsInTheHF(const dog::LegDataMap<rbd::Vector3d>& origins){

	footOrigins = origins;
}

inline void CTerrainBasedAdjustment::setPrimitivesPos(const dog::LegDataMap<rbd::Vector3d>& pPos){

	primitivesPos = pPos;

}

inline void CTerrainBasedAdjustment::setPrimitivesVel(const dog::LegDataMap<rbd::Vector3d>& pVel) {

	primitivesVel = pVel;

}

inline void CTerrainBasedAdjustment::setDesiredTrunkVelocityHF(const rbd::Vector3d& dVel) {

	desTrunkVelocity = dVel;
}

inline dog::LegDataMap<rbd::Vector3d> CTerrainBasedAdjustment::getAdjustedPrimivesPos(){

	return adjustedPrimitivesPos;
}

inline dog::LegDataMap<rbd::Vector3d> CTerrainBasedAdjustment::getAdjustedPrimivesVel(){

	return adjustedPrimitivesVel;
}

inline rbd::Vector3d CTerrainBasedAdjustment::getAdjustedDesTrunkVelHF(const rbd::Vector3d& desTrunkVel){

	return adjustedDesTrunkVel;
}

inline rbd::Vector3d CTerrainBasedAdjustment::getTrunkPostureAdjustmentAngles(){

	return adjPostureAngle;
}

inline dog::LegDataMap<rbd::Vector3d> CTerrainBasedAdjustment::getDeltaFootOriginsHF(){

	return deltaOrigins;
}


#endif /* CTERRAINBASEDADJUSTMENT_H_ */
