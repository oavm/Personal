/*
 * CPG.h
 *
 *  Created on: Aug 29, 2014
 *      Author: victor
 */

#ifndef DLS_CPG_H_
#define DLS_CPG_H_

class CCPG {
public:
    CCPG();
    virtual ~CCPG();
    void computeCPG(void);
    void resetCPG(void);

    void setStepLength(const double& stepLength);
    void setStepHeight(const double& stepHeight);
    void setDutyFactor(const double& dutyFactor);
    void setAbsoluteVelocity(const double& absVelocity);
    void setVelocity(const Eigen::Vector3d& velocity);
    void setDefaultAngularModulationFlag(const bool& flag);
    void setOutPutFiltering(const bool& flag);
    void setDeltaOrigin(const Eigen::Vector3d& delta);
    void setRotationMatrix(const double& roll, const double& pitch, const double& yaw);
    void setTaskFrequency(const double& frequency);
    void setSlowerFactor(const double& slowerFactor);
    void set_bp(double& bp);
    void set_bf(double& bf);
    void set_bv(double& bv);
    void set_kc(double& kc);
    void set_kvf(double& kvf);
    void set_alpha(double& alpha_osc);
    void set_gamma(double& gamma_osc);
    void set_omega(double& omega_osc);
    void setStanceHs(const double& stance_Hs);
    void enableHeightModulation(const bool& height_mod_flag);
    double getAngularFrequency(void);

    void setMaxPlusModulation(const bool& flag); /* ADDED BY OCTAVIO */

private:
    void computeStandardModulation();
    void computeWithHeightModulation();
    double alpha, gamma, k_vf;
    double b_p, b_f, b_v, k_c;
    double omega, w_s;
    bool useHeightModulation;

	static const int X = 0;
	static const int Y = 1;
	static const int Z = 2;

public:
    double stanceHs;
    double couplingTerm;
    double z_td;
    Eigen::Vector3d Xp, dXp, Xf, dXf, last_dXf, last_dXp, V;
    int subSteps;
    double sigUpP, sigDownP, sigUpF, sigDownF;
    double Ls, Hs, Vs, Df;
    bool defaultModulation, useOutPutFilter, maxPlusModulation; /*maxPlusMOdulation added by OCTAVIO */
    double taskFrequency, rateSaturation, slowerFactor;
    Eigen::Vector3d deltaOrigin;
    //z_error_td: Positioning error in Z direction acquired at touch-down.
    //It is not used inside the class.
    double z_error_td;
    Eigen::Matrix3d R;

};

#endif /* DLS_CPG_H_ */
