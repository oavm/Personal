/*
 * CPIDManager.cpp
 *
 *  Created on: Feb 17, 2015
 *      Author: victor
 */

#include "CPIDManager.h"

CPIDManager::CPIDManager() {
    mode = 1;
    jointsDefaultPIDGains.setZero();
    jointPos_.setZero();
    jointVel_.setZero();
    jointDesPos_.setZero();
    jointDesVel_.setZero();
    jointDesAcc_.setZero();
    CPGPosRef_ = rbd::Vector3d::Zero();
    CPGVelRef_ = rbd::Vector3d::Zero();
    useCPGsSmooth_ = false;
    stepHeightThreshold_ = 0.05;
    smoothingGain_ = 2.0;
    inertialCompGain_ = 0.0;
    inertialCompJointGain.setOnes();
    inertiaMatrix_.setZero();
}

CPIDManager::~CPIDManager() {
    // TODO Auto-generated destructor stub
}

void CPIDManager::setLegStatus(const dog::LegDataMap<bool>& legStatus) {

    legStatus_ = legStatus;
}

//void CPIDManager::setDefaultPIDGains(const jointsPID& jointsGains) {
//
//	jointsDefaultPIDGains = jointsGains;
//}

void CPIDManager::setDefaultJointPIDGains(const int& jointID,
                                                const double & kp,
                                                const double & ki,
                                                const double & kd) {

    jointsDefaultPIDGains(jointID,Kp) = kp;
    jointsDefaultPIDGains(jointID,Ki) = ki;
    jointsDefaultPIDGains(jointID,Kd) = kd;
}

void CPIDManager::setStanceJointPIDGains(const int& jointID,
                                                const double & kp,
                                                const double & ki,
                                                const double & kd) {

    jointsStancePIDGains(jointID,Kp) = kp;
    jointsStancePIDGains(jointID,Ki) = ki;
    jointsStancePIDGains(jointID,Kd) = kd;
}

void CPIDManager::setStancePhasePIDGains(const jointsPID& jointGains) {

    jointsStancePIDGains = jointGains;
}

void CPIDManager::setJointStates(const nJoints& joint_pos, const nJoints& joint_vel) {

    jointPos_ = joint_pos;
    jointVel_ = joint_vel;
}

void CPIDManager::setJointDesiredStates(const nJoints& des_pos, const nJoints& des_vel, const nJoints& des_acc) {

    jointDesPos_ = des_pos;
    jointDesVel_ = des_vel;
    jointDesAcc_ = des_acc;
}

void CPIDManager::enablePIDManager(const bool& isEnable) {

    usePIDManager = isEnable;
}


void CPIDManager::setCPGPosReferences(const rbd::Vector3d& CPGRefs_LF,
                                            const rbd::Vector3d& CPGRefs_RF,
                                            const rbd::Vector3d& CPGRefs_LH,
                                            const rbd::Vector3d& CPGRefs_RH) {
    CPGPosRef_[dog::LF] = CPGRefs_LF;
    CPGPosRef_[dog::RF] = CPGRefs_RF;
    CPGPosRef_[dog::LH] = CPGRefs_LH;
    CPGPosRef_[dog::RH] = CPGRefs_RH;
}

void CPIDManager::setCPGPosReferences(const dog::LegDataMap<rbd::Vector3d>& CPGRefs) {
    CPGPosRef_ = CPGRefs;
}

void CPIDManager::setCPGVelReferences(const rbd::Vector3d& CPGRefs_LF,
                                            const rbd::Vector3d& CPGRefs_RF,
                                            const rbd::Vector3d& CPGRefs_LH,
                                            const rbd::Vector3d& CPGRefs_RH) {
    CPGVelRef_[dog::LF] = CPGRefs_LF;
    CPGVelRef_[dog::RF] = CPGRefs_RF;
    CPGVelRef_[dog::LH] = CPGRefs_LH;
    CPGVelRef_[dog::RH] = CPGRefs_RH;
}

void CPIDManager::setCPGVelReferences(const dog::LegDataMap<rbd::Vector3d>& CPGRefs) {
    CPGVelRef_ = CPGRefs;
}

void CPIDManager::setStepHeightThreshold(const double& stepHeight) {

    if (stepHeight > 0.0) {
        stepHeightThreshold_ = stepHeight;
    }
    else {
        std::cout << "The Step Height Threshold must be greater than zero!!!" << std::endl;
    }
}

double CPIDManager::getStepHeightThreshold(void) {

    return stepHeightThreshold_;
}

void CPIDManager::setSmoothingGain(const double& smoothingGain) {

    if (smoothingGain > 0.0) {
        smoothingGain_ = smoothingGain;
    }
    else {
        std::cout << "The Smoothing gain must be greater than zero!!!" << std::endl;
    }

}

void CPIDManager::setInertiaCompensationGain(const double& gain) {

    if ((gain >= 0.0) && (gain <= 1.0)) {
        inertialCompGain_ = gain;
    }
    else {
        std::cout << "This gain must be between 0 and 1!!!" << std::endl;
    }

}

void CPIDManager::setInertiaMatrix(const matrix12x12& iM) {

    inertiaMatrix_ = iM;
}

double CPIDManager::getSmoothingGain(void) {

    return smoothingGain_;
}

void CPIDManager::setMode(const int& modeNumber) {

    if(modeNumber == 1 || modeNumber == 2 || modeNumber == 3){
        mode = modeNumber;
    }
    else{
        std::cout <<"There is no mode " << modeNumber << "!!!" << std::endl;
    }
}

int CPIDManager::getMode(void) {

    return mode;
}


void CPIDManager::computeCompensatingTorques(void){

    outputTorques.setZero();
    tau_inertia_comp.setZero();

    if(usePIDManager) {

        if(!legStatus_[dog::LF] || !legStatus_[dog::RF] || !legStatus_[dog::LH] || !legStatus_[dog::RH]) {

            tau_inertia_comp = inertialCompGain_ * inertiaMatrix_ * jointDesAcc_;
            for(int joint = LF_HAA; joint <= RH_KFE; joint++) {
                tau_inertia_comp(joint) *= inertialCompJointGain(joint);
            }

            outputTorques = tau_inertia_comp;
        }


        switch(mode) {
        case 1:
            computeMode1();

            break;

        case 2:
            computeMode2();

            break;

        case 3:
            computeMode3();

            break;
        }

    }

}


void CPIDManager::computeMode1(void) {

    //If the leg is in stance phase then compute the torques to obtain the
    //desired impedance during the stance phase.
    if( legStatus_[dog::LF]) {// ||
        for(int joint = LF_HAA; joint <= LF_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
        }
    }

    if( legStatus_[dog::RF]){// ||
        for(int joint = RF_HAA; joint <= RF_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
        }
    }

    if( legStatus_[dog::LH]){// ||
        for(int joint = LH_HAA; joint <= LH_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
        }
    }


    if( legStatus_[dog::RH]){// ||
        for(int joint = RH_HAA; joint <= RH_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
        }
    }

}

void CPIDManager::computeMode2(void) {


    //If the leg is in stance phase then compute the torques to obtain the
    //desired impedance during the stance phase.
    if( legStatus_[dog::LF] || (CPGPosRef_[dog::LF](rbd::Z) < 0.0) && (CPGVelRef_[dog::LF](rbd::Z) < 0.0)) {
        for(int joint = LF_HAA; joint <= LF_KFE; joint++) {
            //If the flag useCPGs_ is true then, if the CPG phase corresponds to a
            //leg swing phase, the output torques are set to zero to recover the
            //desired impedance during the swing phase.
            if( (CPGPosRef_[dog::LF](rbd::Z) > 0.0) && (CPGVelRef_[dog::LF](rbd::Z) > 0.0)) {
                if(useCPGsSmooth_) {
                    outputTorques(joint) = computeOutputTorque(joint) * sig( CPGPosRef_[dog::LF](rbd::Z) )
                                                         + tau_inertia_comp(joint);
                }
                else {
                    outputTorques(joint) = computeOutputTorque(joint);
                }
            }
            else {
                outputTorques(joint) = computeOutputTorque(joint);
            }
        }
    }

    if( legStatus_[dog::RF] || (CPGPosRef_[dog::RF](rbd::Z) < 0.0) && (CPGVelRef_[dog::RF](rbd::Z) < 0.0)) {
        for(int joint = RF_HAA; joint <= RF_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
            if( (CPGPosRef_[dog::RF](rbd::Z) > 0.0) && (CPGVelRef_[dog::RF](rbd::Z) > 0.0)) {
                if(useCPGsSmooth_) {
                    outputTorques(joint) = computeOutputTorque(joint) * sig( CPGPosRef_[dog::RF](rbd::Z) )
                                                           + tau_inertia_comp(joint);
                }
                else {
                    outputTorques(joint) = computeOutputTorque(joint);
                }
            }
            else {
                outputTorques(joint) = computeOutputTorque(joint);
            }
        }
    }

    if( legStatus_[dog::LH] || (CPGPosRef_[dog::LH](rbd::Z) < 0.0) && (CPGVelRef_[dog::LH](rbd::Z) < 0.0)) {
        for(int joint = LH_HAA; joint <= LH_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
            if( (CPGPosRef_[dog::LH](rbd::Z) > 0.0) && (CPGVelRef_[dog::LH](rbd::Z) > 0.0)) {
                if(useCPGsSmooth_) {
                    outputTorques(joint) = computeOutputTorque(joint) * sig( CPGPosRef_[dog::LH](rbd::Z) )
                                                           + tau_inertia_comp(joint);
                }
                else {
                    outputTorques(joint) = computeOutputTorque(joint);
                }
            }
            else {
                outputTorques(joint) = computeOutputTorque(joint);
            }
        }
    }


    if( legStatus_[dog::RH] || (CPGPosRef_[dog::RH](rbd::Z) < 0.0) && (CPGVelRef_[dog::RH](rbd::Z) < 0.0)) {
        for(int joint = RH_HAA; joint <= RH_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
            if( (CPGPosRef_[dog::RH](rbd::Z) > 0.0) && (CPGVelRef_[dog::RH](rbd::Z) > 0.0)) {
                if(useCPGsSmooth_) {
                    outputTorques(joint) = computeOutputTorque(joint) * sig( CPGPosRef_[dog::RH](rbd::Z) )
                                                           + tau_inertia_comp(joint);
                }
                else {
                    outputTorques(joint) = computeOutputTorque(joint);
                }
            }
            else {
                outputTorques(joint) = computeOutputTorque(joint);
            }
        }
    }

}

void CPIDManager::computeMode3(void) {

    //If the leg is in stance phase or the foot is in downward motion then compute
    //the torques to obtain the desired impedance.
    if( legStatus_[dog::LF] || (CPGVelRef_[dog::LF](rbd::Z) < 0.0)) {
        for(int joint = LF_HAA; joint <= LF_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
        }
    }

    if( legStatus_[dog::RF] || (CPGVelRef_[dog::RF](rbd::Z) < 0.0)){
        for(int joint = RF_HAA; joint <= RF_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
        }
    }

    if( legStatus_[dog::LH] || (CPGVelRef_[dog::LH](rbd::Z) < 0.0)){
        for(int joint = LH_HAA; joint <= LH_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
        }
    }


    if( legStatus_[dog::RH] || (CPGVelRef_[dog::RH](rbd::Z) < 0.0)){
        for(int joint = RH_HAA; joint <= RH_KFE; joint++) {
            outputTorques(joint) = computeOutputTorque(joint);
        }
    }

}




double CPIDManager::computeOutputTorque(const int& leg_joint) {


    return (jointsStancePIDGains(leg_joint,Kp) - jointsDefaultPIDGains(leg_joint,Kp)) *
            (jointDesPos_(leg_joint) - jointPos_(leg_joint)) +
            (jointsStancePIDGains(leg_joint,Kd) - jointsDefaultPIDGains(leg_joint,Kd)) *
            (jointDesVel_(leg_joint) - jointVel_(leg_joint));

}

double CPIDManager::sig(const double& argf) {

    return exp(-smoothingGain_*argf/stepHeightThreshold_);
}
