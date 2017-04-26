/*
 * CPIDManager.cpp
 *
 *  Created on: Feb 17, 2015
 *      Author: victor
 */

#include <pid_manager/CPIDManager.h>

CPIDManager::CPIDManager(int nJoints):
    jointPos_(nJoints,0.0),
    jointVel_(nJoints,0.0),
    jointDesPos_(nJoints,0.0),
    jointDesVel_(nJoints,0.0),
    jointDesAcc_(nJoints,0.0),
    lastJointPos_(nJoints,0.0),
    lastJointDesPos_(nJoints,0.0),
    jointsPIDGains(nJoints)
{
    for(int i=0; i < jointsPIDGains.size(); i++)
    {
        jointsPIDGains[i] = Eigen::Vector3d().setZero();
    }
}

CPIDManager::~CPIDManager() {
	// TODO Auto-generated destructor stub
}


void CPIDManager::setJointPIDGains(const int& jointID,
											    const double & kp,
											    const double & ki,
											    const double & kd) {

	jointsPIDGains[jointID][Kp] = kp;
	jointsPIDGains[jointID][Ki] = ki;
	jointsPIDGains[jointID][Kd] = kd;
}


void CPIDManager::setJointStates(const std::vector<double>& joint_pos, const std::vector<double>& joint_vel)
{
    lastJointPos_ = jointPos_; //Store last position for Integral term
	jointPos_ = joint_pos;
	jointVel_ = joint_vel;
}

void CPIDManager::setJointDesiredStates(const std::vector<double>& des_pos, const std::vector<double>& des_vel,
    const std::vector<double>& des_acc)
{
    lastJointDesPos_ = jointDesPos_; //Store last position for Integral term
	jointDesPos_ = des_pos;
	jointDesVel_ = des_vel;
	jointDesAcc_ = des_acc;
}

double CPIDManager::computeOutputTorque(const int& leg_joint) {
    double lastPosError =  (lastJointDesPos_[leg_joint] - jointDesPos_[leg_joint]);
    double posError = (jointDesPos_[leg_joint] - jointPos_[leg_joint]);
    double velError = (jointDesVel_[leg_joint] - jointVel_[leg_joint]);

    return (jointsPIDGains[leg_joint][Kp] * posError) +
            (jointsPIDGains[leg_joint][Kd] * velError) + 
            (jointsPIDGains[leg_joint][Ki] * (lastPosError - posError));
}

std::vector<double> CPIDManager::computeOutputTorques(const int& leg_joint) {

    std::vector<double> outTau(jointsPIDGains.size(),0.0);

    for(int i = 0; i < jointsPIDGains.size(); i++)
    {
            outTau[i] = computeOutputTorque(i);
    }

    return outTau;
}
