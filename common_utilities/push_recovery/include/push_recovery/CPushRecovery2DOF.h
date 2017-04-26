/*
 * CPushRecovery2DOF.h
 *
 *  Created on: Sep 12, 2013
 *      Author: victor
 */

#ifndef CPUSHRECOVERY2DOF_H_
#define CPUSHRECOVERY2DOF_H_

#include <iostream>
#include <Eigen/Dense>
#include <iit/commons/dog/leg_data_map.h>
#include <iit/rbd/rbd.h>


using namespace std;
using namespace iit;
using namespace dog;

class CPushRecovery2DOF {
public:
    typedef Eigen::Matrix<double, 4,1> Vector4D;
    typedef Eigen::Matrix<double, 4,4> Matrix44D;
public:
    CPushRecovery2DOF();
    virtual ~CPushRecovery2DOF();

    void ComputeDeltas(double Vx,
                          double Vy,
                          double Psit,
                          double Body_Xd_HF,
                          double Body_Yd_HF,
                          double Body_Yawd,
                          Vector4D& Delta_X,
                          Vector4D& Delta_Y);

    void ComputeDeltas(double Vx,
                              double Vy,
                              double Psit,
                              double Body_Xd_HF,
                              double Body_Yd_HF,
                              double Body_Yawd);

    void Enable(bool TurnOnPushRecovery=true);
    void Disable();
    void setRecoveryGains(double X_RecoveryGain,
                    double Y_RecoveryGain,
                    double Yaw_RecoveryGain);


    void setStancePeriod(double);
    void setFeetHeight(double FeetHeight);
    void setDeltaLimits(double xLimit, double yLimit);
    void setTrunkParameters(double t_mass, double t_inertia, double t_length, double t_width);
    void setTaskFrequency(double sample_frequency_in_Hz = 250);
    void setOutputFilters(double cut_off_frequency_in_Hz = 10);
    void setLegOutputFilter(int leg = 0, double cut_off_frequency_in_Hz = 10);
    void Filter(Vector4D& DeltaX_f, Vector4D& DeltaY_f);
    void Filter(LegDataMap<rbd::Vector3d>& deltaF);
    void enablePushRecoveryX(bool);
    void enablePushRecoveryY(bool);
    void enablePushRecoveryPsi(bool);
    int getOnOffPushRecoveryFlagX(void);
    int getOnOffPushRecoveryFlagY(void);
    int getOnOffPushRecoveryFlagPsi(void);
    LegDataMap<rbd::Vector3d> getDeltaPos(void);
    LegDataMap<rbd::Vector3d> getDeltaVel(void);

private:
    enum {legLF = 0, legRF, legLH, legRH};
    double g0;
    double st_p;
    double Z0h;
    Vector4D Tf;
    Matrix44D alpha_filter;
    double alphaFilter, taskFrequency;
    double Ts;
    double t_mass, t_inertia, t_length, t_width;
    bool PushFlag;
    double K_pr_lx, K_pr_ly, K_pr_r;
    int onOff_x, onOff_y, onOff_psi;
    double maxDeltax, maxDeltay;
    LegDataMap<rbd::Vector3d> lastDelta;

public:
    LegDataMap<rbd::Vector3d> deltaPos;
    LegDataMap<rbd::Vector3d> deltaVel;

};

#endif /* CPUSHRECOVERY2DOF_H_ */





