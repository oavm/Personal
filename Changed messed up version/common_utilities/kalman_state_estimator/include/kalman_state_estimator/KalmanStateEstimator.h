/*
 * KalmanStateEstimator.h
 *
 *  Created on: Feb 25, 2014
 *      Author: Victor Barasuol
 */

#ifndef KALMANSTATEESTIMATOR_H_
#define KALMANSTATEESTIMATOR_H_

#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6,3> Matrix63d;

class KalmanStateEstimator {
public:
	KalmanStateEstimator();
	virtual ~KalmanStateEstimator();
	void setInSimulation(bool);
	void setConsiderGRF(bool);
	void setMinStanceLegs(int);
	void setImuNoiseCov(Eigen::Vector3d);
	void setImuAccelOffset(Eigen::Vector3d);
	void setKinematicsNoiseCov(Eigen::Vector3d);
	void setKinematicsVelOffset(Eigen::Vector3d);
	void setForceNoiseCov(Eigen::Vector3d);
	void setLegsJacobian(Matrix63d, Matrix63d, Matrix63d, Matrix63d);
	void setStanceFeet(const Eigen::Matrix<bool, 4,1>&);
	void setJointTorques(const Eigen::Matrix<double, 12, 1>& torques);
	void setJointVelocities(const Eigen::Matrix<double, 12, 1>& velocities);
	void setFeetPosition(const Eigen::Matrix<double, 3,4>& foot_pos);
	void setBodyAnglesPos(double, double, double);
	void setBodyAnglesVel(double, double, double);
	void setBodyAnglesAccel(double, double, double);
	void setBodyAccelFromIMU(double, double, double);
	void setBodyWeight(double);
	void setSamplingRate(double);
	Eigen::Vector3d trunkLinearVelocitiesInTheHF();
	bool getInSimulation() {return simulationFlag;}
	bool getConsiderGRF() {return considerGRF;}
	Eigen::Vector3d getImuNoiseCov()  {return imuNoiseCovariance;}
	Eigen::Vector3d getImuAccelOffset() {return imuAccelOffset;}
	Eigen::Vector3d getKinematicsNoiseCov() {return kinematicsNoiseCov;}
	Eigen::Vector3d getKinematicsVelOffset() {return kinematicsVelOffset;}

private:
	int minStanceLegs;
	bool simulationFlag;
	bool considerGRF;
	double roll, pitch, yaw, bodyWeight, samplingRate;
	Eigen::Matrix<double, 3,1> trunkLinearVelocity;
	Eigen::Vector3d trunkLinearVelocityPredicted;
	Eigen::Vector3d xbEstimatedWithKinematics;
	Eigen::Matrix<bool, 4,1> supportLegs;
	Eigen::Vector3d bodyAnglesVel;
	Eigen::Vector3d bodyAnglesAccel;
	Eigen::Vector3d imuNoiseCovariance;
	Eigen::Vector3d imuAccelOffset;
	Eigen::Vector3d imuAccel;
	Eigen::Vector3d imuAccelInTheHorizontalFrame;
	Eigen::Vector3d imuAccelInTheBaseFrame;
	Eigen::Vector3d kinematicsNoiseCov;
	Eigen::Vector3d kinematicsVelOffset;
	Eigen::Vector3d forceSensorCov;
	Eigen::Vector3d imuPosition;
	Eigen::Vector3d footPosLF, footPosRF, footPosLH, footPosRH;
	Eigen::Matrix<double, 12, 1> jointTorques;
	Eigen::Matrix<double, 12, 1> jointVel;
	Eigen::Matrix3d JLF, JRF, JLH, JRH, R;


	enum {LF_HAA = 0, LF_HFE, LF_KFE,
		      RF_HAA, RF_HFE, RF_KFE,
		      LH_HAA, LH_HFE, LH_KFE,
		      RH_HAA, RH_HFE, RH_KFE};
	enum {LF = 0, RF, LH, RH};		    
	enum Coords6D { AX=0, AY, AZ, LX, LY, LZ };
};


inline void KalmanStateEstimator::setLegsJacobian(Matrix63d J_LF,
		Matrix63d J_RF,
		Matrix63d J_LH,
		Matrix63d J_RH) {

	JLF = J_LF.block<3,3>(LX,0);
	JRF = J_RF.block<3,3>(LX,0);
	JLH = J_LH.block<3,3>(LX,0);
	JRH = J_RH.block<3,3>(LX,0);
}


inline void KalmanStateEstimator::setFeetPosition(const Eigen::Matrix<double, 3,4>& foot_pos) {
	footPosLF = foot_pos.block<3,1>(0,0);
	footPosRF = foot_pos.block<3,1>(0,1);
	footPosLH = foot_pos.block<3,1>(0,2);
	footPosRH = foot_pos.block<3,1>(0,3);
}


inline void KalmanStateEstimator::setBodyAnglesPos(double rollAngle, double pitchAngle, double yawAngle) {
	roll = rollAngle;
	pitch = pitchAngle;
	yaw = yawAngle;

	R(0,0) = cos(pitch);      R(0,1) = sin(pitch)*sin(roll);      R(0,2) = cos(roll) * sin(pitch);
	R(1,0) = 0.0;             R(1,1) = cos(roll);                  R(1,2) = -sin(roll);
	R(2,0) = -sin(pitch);     R(2,1) = cos(pitch) * sin(roll);    R(2,2) = cos(pitch) * cos(roll);

}


inline void KalmanStateEstimator::setBodyAnglesVel(double rollAngleVel, double pitchAngleVel, double yawAngleVel) {
	bodyAnglesVel << rollAngleVel, pitchAngleVel, yawAngleVel;
}


inline void KalmanStateEstimator::setBodyAnglesAccel(double rollAngleAccel, double pitchAngleAccel, double yawAngleAccel) {
	bodyAnglesAccel << rollAngleAccel, pitchAngleAccel, yawAngleAccel;
}

inline void KalmanStateEstimator::setBodyAccelFromIMU(double accelX, double accelY, double accelZ) {
	imuAccel << accelX, accelY, accelZ;
}


inline void KalmanStateEstimator::setStanceFeet(const Eigen::Matrix<bool, 4,1>& support_legs) {
	supportLegs = support_legs;
}


inline void KalmanStateEstimator::setJointTorques(const Eigen::Matrix<double, 12, 1>& torques){
	jointTorques = torques;
}

inline void KalmanStateEstimator::setJointVelocities(const Eigen::Matrix<double, 12, 1>& velocities){
	jointVel = velocities;
}


#endif /* KALMANSTATEESTIMATOR_H_ */
