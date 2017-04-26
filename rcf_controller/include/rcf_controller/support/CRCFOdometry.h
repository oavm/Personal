/*
 * CRCFOdometry.h
 *
 *  Created on: Feb 23, 2015
 *      Author: Victor Barasuol
 */

#ifndef CRCFODOMETRY_H_
#define CRCFODOMETRY_H_

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <iit/rbd/rbd.h>
#include <iit/commons/dog/declarations.h>

using namespace iit;
using namespace std;

class CRCFOdometry {
public:
	CRCFOdometry();
	virtual ~CRCFOdometry();
	/*
	 * Update trunk velocities (Xd, Yd, Zd) described in the horizontal frame.
	 */
	void setTrunkVelocitiesHF(const rbd::Vector3d& trunk_velocities);
	/*
	 * Update trunk angles (roll, pitch and yaw) using the RPY convention (Cardan).
	 * These angles are used to describe the displacement in the world frame.
	 */
	void setTrunkRollPitchYaw(const rbd::Vector3d& trunk_angles);
	/*
	 * Set task frequency in order to get the integration time step.
	 */
	void setTaskFrequency(const double& task_frequency);
	/*
	 * Reset the odometry outputs.
	 */
	void resetOdometry(void);
	/*
	 * Get the X coordinate of the trunk position in the world frame.
	 */
	double getXPosWF(void);
	/*
	 * Get trunk position in the world frame.
	 */
	rbd::Vector3d getTrunkPositionWF(void);
	/*
	 * Get trunk position in the world frame.
	 */
	rbd::Vector3d getTrunkPositionHF(void);
	/*
	 * Get trunk position in the world frame that is aligned to the yaw
	 * angle (Yaw0) at the moment of the start-up. The function resetOdometry()
	 * update such an initial yaw angle.
	 */
	rbd::Vector3d getTrunkPositionYaw0(void);
	/*
	 * Get the Y coordinate of the trunk position in the world frame.
	 */
	double getYPosWF(void);
	/*
	 * Get the Z coordinate of the trunk position in the world frame.
	 */
	double getZPosWF(void);
	/*
	 * Get the X coordinate of the trunk position in the horizontal frame.
	 */
	double getXPosHF(void);
	/*
	 * Get the Y coordinate of the trunk position in the horizontal frame.
	 */
	double getYPosHF(void);
	/*
	 * Get the Z coordinate of the trunk position in the horizontal frame.
	 */
	double getZPosHF(void);
	/*
	 * Get the X coordinate of the trunk position in the world frame aligned
	 * to the initial pose of the robot.
	 */
	double getXPosYaw0(void);
	/*
	 * Get the Y coordinate of the trunk position in the world frame aligned
	 * to the initial pose of the robot.
	 */
	double getYPosYaw0(void);
	/*
	 * Get the Z coordinate of the trunk position in the world frame aligned
	 * to the initial pose of the robot.
	 */
	double getZPosYaw0(void);
	/*
	 * Get the yaw at the moment the user resets the odometry.
	 * This yaw is used to provide the odometry aligned to the start-up
	 * robot pose.
	 */
	void setYaw0(void);
	/*
	 * Compute odometry (integrate the velocities according to time).
	 */
	void computeOdometry(void);
	/*
	 * Set the time constant of the velocity filter.
	 * The "filtered" odometry is not the standard output.
	 * Filtered outputs are addressed with "_f" at the end.
	 */
	void setFilterTimeConstant(const double& tf);
	/*
	 * Set flag to compute all the outputs based on filtered trunk velocity.
	 */
	void useFilteredOutputs(const bool& compute_filtered_outputs);


private:
	/*
	 * Return a 3x3 rotation matrix around the gravity vector (Z) for a given angle.
	 */
	rbd::Matrix33d getYawRotation(const double& angle);

public:
	rbd::Vector3d trunkPositionWF, trunkPositionWF_f;
	rbd::Vector3d trunkPositionHF, trunkPositionHF_f;
	rbd::Vector3d trunkPositionYaw0, trunkPositionYaw0_f;
	double XPosWF0, YPosWF0, ZPosWF0;
	double XPosWF0_f, YPosWF0_f, ZPosWF0_f;

private:
	bool computeFilteredOutputsFlag_;
	double dt_, yaw0_, alphaF_;
	rbd::Vector3d trunkVelHF_, trunkVelHF_f_, trunkAngles_;




};

#endif /* CRCFODOMETRY_H_ */
