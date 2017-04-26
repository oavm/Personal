/*
 * CTerrainEstimator.h
 *
 *  Created on: Sep 20, 2013
 *      Author: victor
 */

#ifndef CTERRAINESTIMATOR_H_
#define CTERRAINESTIMATOR_H_

#include <iostream>
#include <Eigen/Dense>
#include <iit/commons/dog/leg_data_map.h>
#include <iit/rbd/rbd.h>

class CTerrainEstimator {
public:
	CTerrainEstimator();
	virtual ~CTerrainEstimator();

	/**
	 * Compute estimates for the terrain roll and pitch angles
	 */
	void ComputeTerrainEstimation(double&, double&);

	/**
	 * Compute estimates for the terrain roll and pitch angles using some euristics to deal with rough terrain
	 */
	void ComputeTerrainEstimationRoughTerrain(const Eigen::Vector3d & old_terrain_normal, double& Estroll, double& EstPitch);

	bool planeEstimationLS(const iit::dog::LegDataMap<Eigen::Vector3d> & feet, double & LSerror, Eigen::Vector3d & plane_normal, double & d_param);

	bool planeEstimationEigenVector(const iit::dog::LegDataMap<Eigen::Vector3d> & feet, Eigen::Vector3d & plane_normal);

	/**
	 * Set foot positions described in the robot base frame
	 */
	void setFootPositionsBF(const Eigen::Matrix<double, 3,4>& foot_pos);

	/**
	 * Set vertical (Z axes) foot forces described in the robot base frame
	 */
	void setFootForcesBF(double LF_Force, double RF_Force,
						 double LH_Force, double RH_Force);

	/**
	 * Set a flag if one leg is in shin collision
	 */
	void setShinCollision(bool LF_shinFlag, bool RF_shinFlag,
						  bool LH_shinFlag, bool RH_shinFlag);
	/**
	 * Set force threshold to define when triggering each leg to stance status
	 */
	void setForceThreshold(double contactForce);

	/**
	 * Set filter time constant to filter the estimate changes
	 */
	void setFilter(double TaskSampleTime, double FilterTime);

	/**
	 * Set base angles according to the horizontal frame
	 */
	void setBaseAngles(double base_roll, double base_pitch);

	/**
	 * Turn on the terrain estimation
	 */
	void Enable(bool TurnOnEstimation=true);

	/**
	 * Turn off the terrain estimation
	 */
	void Disable();

	/**
	 * Set limits for maximum terrain roll and pitch estimates
	 */
	void setLimits(double mRoll, double mPitch);

	/**
	 * Reset terrain estimator
	 */
	void resetEstimator(void);


private:
	bool ChangesFlag, WarningFlag, TerrainEstimationFlag;
	double maxRoll, maxPitch;
	double ForceThreshold;
	double AlphaFilter;
	double LF_FootForce;
	double RF_FootForce;
	double LH_FootForce;
	double RH_FootForce;
	double baseRoll, basePitch;
	Eigen::Matrix<double, 8,2> Ag;
	Eigen::Matrix<double, 8,1> Bg;
	Eigen::Matrix<double, 3,3> R;
	Eigen::Matrix<double, 3,1> LF_pos;
	Eigen::Matrix<double, 3,1> RF_pos;
	Eigen::Matrix<double, 3,1> LH_pos;
	Eigen::Matrix<double, 3,1> RH_pos;
	Eigen::Matrix<double, 3,1> LF_pos_HF;
	Eigen::Matrix<double, 3,1> RF_pos_HF;
	Eigen::Matrix<double, 3,1> LH_pos_HF;
	Eigen::Matrix<double, 3,1> RH_pos_HF;
	Eigen::Matrix<double, 3,1> LF_foot_pos;
	Eigen::Matrix<double, 3,1> RF_foot_pos;
	Eigen::Matrix<double, 3,1> LH_foot_pos;
	Eigen::Matrix<double, 3,1> RH_foot_pos;
	Eigen::Matrix<double, 2,1> Relative_Angles;
	Eigen::Matrix<double, 2,1> Terrain_Angles;
	Eigen::Matrix<double, 2,1> Terrain_Angles_f1;
	Eigen::Matrix<double, 2,1> Terrain_Angles_f2;
	iit::dog::LegDataMap<bool> shinCollision;
	bool shinCollisionSet = false;
	void correctPlaneWithHeuristics(const iit::dog::LegDataMap<Eigen::Vector3d> & feet, const double sensitivity, const Eigen::Vector3d & old_terrain_normal, Eigen::Vector3d &plane_normal);
	Eigen::Vector3d computeTriangleNormal(const iit::dog::LegDataMap<Eigen::Vector3d> & feet, const Eigen::Vector3d & index);
	void checkStance();
};



inline void CTerrainEstimator::setForceThreshold(double contactForce) {

	ForceThreshold = contactForce;
}


inline void CTerrainEstimator::setBaseAngles(double base_roll, double base_pitch) {

	baseRoll = base_roll;
	basePitch = base_pitch;
}



inline void CTerrainEstimator::setFootForcesBF(double LF_Force, double RF_Force, double LH_Force, double RH_Force) {

	LF_FootForce = LF_Force;
	RF_FootForce = RF_Force;
	LH_FootForce = LH_Force;
	RH_FootForce = RH_Force;
}


inline void CTerrainEstimator::setFootPositionsBF(const Eigen::Matrix<double, 3,4>& foot_pos) {

	LF_foot_pos = foot_pos.block<3,1>(0,0);
	RF_foot_pos = foot_pos.block<3,1>(0,1);
	LH_foot_pos = foot_pos.block<3,1>(0,2);
	RH_foot_pos = foot_pos.block<3,1>(0,3);
}

inline void  CTerrainEstimator::setShinCollision(bool LF_shinFlag, bool RF_shinFlag, bool LH_shinFlag, bool RH_shinFlag)
{
	shinCollision[iit::dog::LF] = LF_shinFlag;
	shinCollision[iit::dog::RF] = RF_shinFlag;
	shinCollision[iit::dog::LH] = LH_shinFlag;
	shinCollision[iit::dog::RH] = RH_shinFlag;
	shinCollisionSet = true;
}



#endif /* CTERRAINESTIMATOR_H_ */
