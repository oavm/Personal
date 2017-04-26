/*
 * CKinematicAdjustment.cpp
 *
 *  Created on: Sep 24, 2013
 *      Author: victor
 */

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <kinematic_adjustment/CKinematicAdjustment.h>

using namespace std;

CKinematicAdjustment::CKinematicAdjustment() {

    KinematicAdjustmentFlag = false;
    BaseRoll = 0.0; BasePitch = 0.0;
    BaseRolld = 0.0; BasePitchd = 0.0;
    roll = 0.0; rolld = 0.0;
    pitch = 0.0; pitchd = 0.0;
    ExpCoefPos = 100; ExpCoefVel = 100;
    MaxRoll = 5*(3.14/180);  MaxPitch = 2.5*(3.14/180);
    RollCoef = 0.0; PitchCoef = 0.0;
    Rt.setIdentity();
    Rtd.setZero();
    desFootPosBF = rbd::Vector3d::Zero();
    desFootVelBF = rbd::Vector3d::Zero();
    desFootAccBF = rbd::Vector3d::Zero();
    desFootPosHF = rbd::Vector3d::Zero();
    desFootVelHF = rbd::Vector3d::Zero();
    desFootAccHF = rbd::Vector3d::Zero();

}

CKinematicAdjustment::~CKinematicAdjustment() {
    // TODO Auto-generated destructor stub
}


void CKinematicAdjustment::enable(bool TurnOnAdjustment) {

    KinematicAdjustmentFlag = TurnOnAdjustment;
    if(KinematicAdjustmentFlag){
        cout << "Kinematic adjustment ON!!!" << endl;
    }
    else{
        cout << "Kinematic adjustment OFF!!!" << endl;
    }

}

void CKinematicAdjustment::disable() {

    KinematicAdjustmentFlag = false;
    cout << "Kinematic Adjustment OFF!!!" << endl;
}



void CKinematicAdjustment::setSaturationRate(double expCoefPos, double expCoefVel) {

    if ((expCoefPos>=0.0) && (expCoefPos<=10000)) {
        ExpCoefPos = expCoefPos;
    }
    else {
        cout << "Coefficient out of range [0,10000]!!!" << endl;
    }

    if ((expCoefVel>=0.0) && (expCoefVel<=10000)) {
        ExpCoefVel = expCoefVel;
    }
    else {
        cout << "Coefficient out of range [0,10000]!!!" << endl;
    }
}



void CKinematicAdjustment::setKinAdjustment(double roll_adj_coef, double pitch_adj_coef) {

    if ((roll_adj_coef>=0.0) && (roll_adj_coef<=1.0)) {
        RollCoef = roll_adj_coef;
    }
    else {
        cout << "Roll coefficient out of range [0,1]!!!" << endl;
    }

    if ((pitch_adj_coef>=0.0) && (pitch_adj_coef<=1.0)) {
        PitchCoef = pitch_adj_coef;
    }
    else {
        cout << "Pitch coefficient out of range [0,1]!!!" << endl;
    }
}


void CKinematicAdjustment::setTaskFrequency(double taskFreq) {

    taskFrequency = taskFreq;
}


void CKinematicAdjustment::setDesiredFootPosInHF(iit::dog::LegDataMap<rbd::Vector3d>& desRef){

    desFootPosHF = desRef;
}

void CKinematicAdjustment::setDesiredFootVelInHF(iit::dog::LegDataMap<rbd::Vector3d>& desRef){

    desFootVelHF = desRef;
}

void CKinematicAdjustment::setDesiredFootAccInHF(iit::dog::LegDataMap<rbd::Vector3d>& desRef){

    desFootAccHF = desRef;
}




void CKinematicAdjustment::computeKinAdjustment(const Eigen::Matrix<double, 3,1> FootPosBaseFrame,
                                                      const Eigen::Matrix<double, 3,1> FootVelBaseFrame,
                                                      Eigen::Matrix<double, 3,1>& AdjFootPos,
                                                      Eigen::Matrix<double, 3,1>& AdjFootVel) {


    if (KinematicAdjustmentFlag) {

        //Kinematic Adjustment Level
        roll=RollCoef*BaseRoll;
        rolld=RollCoef*BaseRolld;
        pitch=PitchCoef*BasePitch;
        pitchd=PitchCoef*BasePitchd;


        //Saturation for roll velocity
        rolld=rolld*1/(exp(-ExpCoefVel*(MaxRoll-roll))+1)*1/(exp(ExpCoefVel*(-MaxRoll-roll))+1);
        //Saturation for roll position
        roll=roll+(MaxRoll-roll)*1/(exp(ExpCoefPos*(MaxRoll-roll))+1)+(-MaxRoll-roll)*1/(exp(-ExpCoefPos*(-MaxRoll-roll))+1);

        //Saturation for pitch velocity
        pitchd=pitchd*1/(exp(-ExpCoefVel*(MaxPitch-pitch))+1)*1/(exp(ExpCoefVel*(-MaxPitch-pitch))+1);
        //Saturation for pitch position
        pitch=pitch+(MaxPitch-pitch)*1/(exp(ExpCoefPos*(MaxPitch-pitch))+1)+(-MaxPitch-pitch)*1/(exp(-ExpCoefPos*(-MaxPitch-pitch))+1);




        /** Rotation matrix from Horizontal frame to base frame.
        * R(0,0) =  cos(pitch);
        * R(0,1) =  sin(pitch) * sin(roll);
        * R(0,2) =  cos(roll) * sin(pitch);
        * R(1,0) =  0.0;
        * R(1,1) =  cos(roll);
        * R(1,2) = -sin(roll);
        * R(2,0) = -sin(pitch);
        * R(2,1) =  cos(pitch) * sin(roll);
        * R(2,2) =  cos(pitch) * cos(roll);
        */

        Rt(0,0)= cos(pitch);                Rt(0,1)= 0;             Rt(0,2)=-sin(pitch) ;
        Rt(1,0)= sin(pitch) * sin(roll);    Rt(1,1)= cos(roll);     Rt(1,2)= cos(pitch) * sin(roll);
        Rt(2,0)= cos(roll) * sin(pitch);    Rt(2,1)=-sin(roll);     Rt(2,2)= cos(pitch) * cos(roll);


        Rtd(0,0) = -sin(pitch)*pitchd;
        Rtd(1,0) =  sin(pitch)*cos(roll)*rolld + cos(pitch)*sin(roll)*pitchd;
        Rtd(2,0) = -sin(roll)*sin(pitch)*rolld + cos(roll)*cos(pitch)*pitchd;
        Rtd(0,1) =  0.0;
        Rtd(1,1) = -sin(roll)*rolld;
        Rtd(2,1) = -cos(roll)*rolld;
        Rtd(0,2) = -cos(pitch)*pitchd;
        Rtd(1,2) =  cos(pitch)*cos(roll)*rolld - sin(pitch)*sin(roll)*pitchd;
        Rtd(2,2) = -cos(pitch)*sin(roll)*rolld - sin(pitch)*cos(roll)*pitchd;


        AdjFootPos = Rt*FootPosBaseFrame;
        AdjFootVel = Rt*FootVelBaseFrame + Rtd*FootPosBaseFrame;



    }
    else {
        AdjFootPos=FootPosBaseFrame;
        AdjFootVel=FootVelBaseFrame;
    }

}


void CKinematicAdjustment::computeKinAdjustment(void) {

    if (KinematicAdjustmentFlag) {

        Rt(0,0)= cos(pitch);                Rt(0,1)= 0;             Rt(0,2)=-sin(pitch) ;
        Rt(1,0)= sin(pitch) * sin(roll);    Rt(1,1)= cos(roll);     Rt(1,2)= cos(pitch) * sin(roll);
        Rt(2,0)= cos(roll) * sin(pitch);    Rt(2,1)=-sin(roll);     Rt(2,2)= cos(pitch) * cos(roll);

        Rtd(0,0) = -sin(pitch)*pitchd;
        Rtd(1,0) =  sin(pitch)*cos(roll)*rolld + cos(pitch)*sin(roll)*pitchd;
        Rtd(2,0) = -sin(roll)*sin(pitch)*rolld + cos(roll)*cos(pitch)*pitchd;
        Rtd(0,1) =  0.0;
        Rtd(1,1) = -sin(roll)*rolld;
        Rtd(2,1) = -cos(roll)*rolld;
        Rtd(0,2) = -cos(pitch)*pitchd;
        Rtd(1,2) =  cos(pitch)*cos(roll)*rolld - sin(pitch)*sin(roll)*pitchd;
        Rtd(2,2) = -cos(pitch)*sin(roll)*rolld - sin(pitch)*cos(roll)*pitchd;

        for(int leg = dog::LF; leg <= dog::RH; leg++) {
            desFootPosBF[leg] = Rt * desFootPosHF[leg];
            desFootVelBF[leg] = taskFrequency * (desFootPosBF[leg] - lastPos[leg]);
            desFootAccBF[leg] = taskFrequency * (desFootVelBF[leg] - lastVel[leg]);
            lastPos[leg] = desFootPosBF[leg];
            lastVel[leg] = desFootVelBF[leg];
        }

    }
    else {

        for(int leg = dog::LF; leg <= dog::RH; leg++) {
            desFootPosBF[leg] = desFootPosHF[leg];
            desFootVelBF[leg] = desFootVelHF[leg];
            desFootAccBF[leg] = desFootAccHF[leg];
            lastPos[leg] = desFootPosBF[leg];
            lastVel[leg] = desFootVelBF[leg];
        }
    }
}


iit::dog::LegDataMap<rbd::Vector3d> CKinematicAdjustment::getKinAdjustmentPos(void){

    return desFootPosBF;
}

iit::dog::LegDataMap<rbd::Vector3d> CKinematicAdjustment::getKinAdjustmentVel(void){

    return desFootVelBF;
}

iit::dog::LegDataMap<rbd::Vector3d> CKinematicAdjustment::getKinAdjustmentAcc(void){

    return desFootAccBF;
}
