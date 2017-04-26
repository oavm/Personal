/*
 * CPIDManager.h
 *
 *  Created on: Feb 17, 2015
 *      Author: victor
 */

#ifndef DLS_PIDMANAGER_H_
#define DLS_PIDMANAGER_H_

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>


class CPIDManager {
public:
	CPIDManager(int nJoints);
	virtual ~CPIDManager();
	//void setDefaultPIDGains(const Eigen::Matrix<double, 12, 3>& jointsGains);
	/*
	 * Set the PID gains to be used during the swing phase.
	 * Set the PID gains for one specific joint i (i = jointID).
	 */
	void setJointPIDGains(const int& jointID,
									 const double & kp,
									 const double & ki,
									 const double & kd);
	/*
	 * Set actual joint states (positions and velocities).
	 */
	void setJointStates(const std::vector<double>& joint_pos, const std::vector<double>& joint_vel);
	/*
	 * Set desired joint states (positions and velocities).
	 */
	void setJointDesiredStates(const std::vector<double>& joint_des_pos, const std::vector<double>& joint_des_vel,
		const std::vector<double>& joint_des_acc);
    /*
     * Function to compute joint output torque.
    */
	std::vector<double> computeOutputTorques(const int& leg_joint);

	static const int Kp = 0;
	static const int Ki = 1;
	static const int Kd = 2;
	enum {LF_HAA = 0, LF_HFE, LF_KFE,
			      RF_HAA, RF_HFE, RF_KFE,
			      LH_HAA, LH_HFE, LH_KFE,
			      RH_HAA, RH_HFE, RH_KFE};

private:
    /*
     * Function to compute joint output torque of a single joint.
    */
	double computeOutputTorque(const int& leg_joint);

	std::vector<double> jointPos_, jointDesPos_, jointVel_, jointDesVel_, jointDesAcc_;
	std::vector<double> lastJointPos_, lastJointDesPos_;
	std::vector<Eigen::Vector3d > jointsPIDGains;

};

#endif /* DLS_PIDMANAGER_H_ */
