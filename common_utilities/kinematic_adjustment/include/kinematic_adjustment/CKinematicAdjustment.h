/*
 * CKinematicAdjustment.h
 *
 *  Created on: Sep 24, 2013
 *      Author: victor
 */

#ifndef DLS_KINEMATICADJUSTMENT_H_
#define DLS_KINEMATICADJUSTMENT_H_

#include <iit/rbd/rbd.h>
#include <iit/commons/dog/leg_data_map.h>

using namespace iit;
using namespace dog;

class CKinematicAdjustment {
public:
    CKinematicAdjustment();
    virtual ~CKinematicAdjustment();

    void computeKinAdjustment(Eigen::Matrix<double, 3,1> FootPosBaseFrame,
                                 Eigen::Matrix<double, 3,1> FootVelBaseFrame,
                                 Eigen::Matrix<double, 3,1>& AdjFootPos,
                                 Eigen::Matrix<double, 3,1>& AdjFootVel);

    void computeKinAdjustment(void);

    iit::dog::LegDataMap<rbd::Vector3d> getKinAdjustmentPos(void);
    iit::dog::LegDataMap<rbd::Vector3d> getKinAdjustmentVel(void);
    iit::dog::LegDataMap<rbd::Vector3d> getKinAdjustmentAcc(void);
    void setDesiredFootPosInHF(iit::dog::LegDataMap<rbd::Vector3d>& desRef);
    void setDesiredFootVelInHF(iit::dog::LegDataMap<rbd::Vector3d>& desRef);
    void setDesiredFootAccInHF(iit::dog::LegDataMap<rbd::Vector3d>& desRef);

    void setTaskFrequency(double taskFreq);
    void setRollLimit(double max_roll);
    void setPitchLimit(double max_pitch);
    double getRollLimit();
    double getPitchLimit();
    double getRollKinAdjCoeff();
    double getPitchKinAdjCoeff();
    void setSaturationRate(double expCoefPos=100, double expCoefVel=100);
    void enable(bool TurnOnAdjustment = true);
    void disable();
    void setBaseAngles(double base_roll, double base_pitch, double base_roll_d, double base_pitch_d);
    void setKinAdjustment(double roll_adj_coef, double pitch_adj_coef);

private:
    double BaseRoll, BasePitch, BaseRolld, BasePitchd;
    double ExpCoefPos, ExpCoefVel;
    double MaxRoll, MaxPitch;
    double RollCoef, PitchCoef;
    double roll, rolld, pitch, pitchd;
    double taskFrequency;
    Eigen::Matrix<double, 3,3> Rt;
    Eigen::Matrix<double, 3,3> Rtd;
    iit::dog::LegDataMap<rbd::Vector3d> lastPos, lastVel;

public:
    bool KinematicAdjustmentFlag;
    iit::dog::LegDataMap<rbd::Vector3d> desFootPosBF, desFootVelBF, desFootAccBF;
    iit::dog::LegDataMap<rbd::Vector3d> desFootPosHF, desFootVelHF, desFootAccHF;
};



inline void CKinematicAdjustment::setRollLimit(double max_roll) {

    MaxRoll = max_roll;
}


inline void CKinematicAdjustment::setPitchLimit(double max_pitch) {

    MaxPitch = max_pitch;
}


inline double CKinematicAdjustment::getRollLimit() {

    return MaxRoll;
}


inline double CKinematicAdjustment::getPitchLimit() {

    return MaxPitch;
}


inline double CKinematicAdjustment::getRollKinAdjCoeff() {

    return RollCoef;
}


inline double CKinematicAdjustment::getPitchKinAdjCoeff() {

    return PitchCoef;
}


inline void CKinematicAdjustment::setBaseAngles(double base_roll, double base_pitch, double base_roll_d, double base_pitch_d) {

    BaseRoll = base_roll;
    BasePitch = base_pitch;
    BaseRolld = base_roll_d;
    BasePitchd = base_pitch_d;

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
}




#endif /* DLS_KINEMATICADJUSTMENT_H_ */
