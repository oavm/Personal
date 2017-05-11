/*
 * CPG.cpp
 *
 *  Created on: Aug 29, 2014
 *      Author: victor
 */

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <central_pattern_generator/CCPG.h>

CCPG::CCPG() {

   resetCPG(); //Set to default values

   subSteps = 10;
   alpha = 100.0; /* OCTAVIO Original value: 40 */
   gamma = 100.0; /* OCTAVIO Original value: 40 */
   k_vf = 1;
   b_p = 500;
   b_f = 200;
   b_v = 500;
   k_c = 60;

   sigUpP = 0.0;
   sigDownP = 0.0;
   sigUpF = 0.0;
   sigDownF = 0.0;
   defaultModulation = true;
   taskFrequency = 100.0;
   rateSaturation = 10.0;
   deltaOrigin.setZero();
   R.setIdentity();
   slowerFactor = 1.0;
   useOutPutFilter = true;
   z_error_td = 0.0;
   useHeightModulation = false;
   stanceHs = 0.001;
   w_s = 1.0;

   /* ADDED BY OCTAVIO */
   maxPlusModulation = false;

}

void CCPG::resetCPG() {
       omega = 1.0;
       couplingTerm = 0.0;
       z_td = 0.0;
       Xp << 0.001, 0.0, 0.001;
       dXp.setZero();
       Xf << 0.001, 0.0, 0.001;
       dXf.setZero();
       last_dXf.setZero();
       last_dXp.setZero();

       Ls = 0.001;
       Hs = 0.001;
       Vs = 0.001;
       V  << 0.001, 0.0, 0.0;

       Df = 0.5;
}

CCPG::~CCPG() {
    // TODO Auto-generated destructor stub
}


void CCPG::setStepLength(const double& stepLength) {
    if(stepLength > 0) {
        Ls = stepLength;
    }
    else {
        std::cout << "Step length must be greater than 0!!!" << std::endl << std::endl;
    }
}


void CCPG::setStepHeight(const double& stepHeight) {
    if(stepHeight > 0) {
        Hs = stepHeight;
    }
    else {
        std::cout << "Step height must be greater than 0!!!" << std::endl << std::endl;
    }
}


void CCPG::setDutyFactor(const double& dutyFactor) {
    if((dutyFactor > 0) && (dutyFactor < 1)) {
        Df = dutyFactor;
    }
    else {
        std::cout << "Duty factor must be between 0 and 1!!!" << std::endl << std::endl;
    }
}


void CCPG::setAbsoluteVelocity(const double& absVelocity) {
    Vs = absVelocity;
}


void CCPG::setVelocity(const Eigen::Vector3d& velocity) {
    V = velocity;
}


void CCPG::setDefaultAngularModulationFlag(const bool& flag) {
    defaultModulation = flag;
}


void CCPG::setMaxPlusModulation(const bool& flag) { /* ADDED BY OCTAVIO */
    maxPlusModulation = flag;
}


void CCPG::setOutPutFiltering(const bool& flag) {
    useOutPutFilter = flag;
}


void CCPG:: setDeltaOrigin(const Eigen::Vector3d& delta) {
    deltaOrigin = delta;
}


void CCPG::setRotationMatrix(const double& roll, const double& pitch, const double& yaw) {

    //First line
    R(0,0) = cos(yaw) * cos(pitch);
    R(0,1) = sin(yaw) * cos(pitch);
    R(0,2) = - sin(pitch);

    //Second line
    R(1,0) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
    R(1,1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
    R(1,2) = cos(pitch) * sin(roll);

    //Third line
    R(2,0) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
    R(2,1) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
    R(2,2) = cos(pitch) * cos(roll);

    R.transposeInPlace();
}


void CCPG::setTaskFrequency(const double& frequency) {
    if(frequency > 0) {
        taskFrequency = frequency;
    }
    else {
        std::cout << "Task frequency must be greater than 0!!!" << std::endl << std::endl;
    }
}


void CCPG::setSlowerFactor(const double& slower_factor) {
    if((slower_factor >= 0) && (slower_factor <= 1)) {
        slowerFactor = slower_factor;
    }
    else {
        std::cout << "Duty factor must be between [0,1]!!!" << std::endl << std::endl;
    }
}

void CCPG::set_bp(double& bp) {
    b_p = bp;
}


void CCPG::set_bf(double& bf) {
    b_f = bf;
}


void CCPG::set_bv(double& bv) {
    b_v = bv;
}


void CCPG::set_kc(double& kc) {
    k_c = kc;
}


void CCPG::set_kvf(double& kvf) {
    k_vf = kvf;
}


void CCPG::set_alpha(double& alpha_osc) {
    alpha = alpha_osc;
}


void CCPG::set_gamma(double& gamma_osc) {
    gamma = gamma_osc;
}


void CCPG::set_omega(double& omega_osc) {
    omega = omega_osc;
}

void CCPG::enableHeightModulation(const bool& height_mod_flag){
    useHeightModulation = height_mod_flag;
}

void CCPG::setStanceHs(const double& stance_Hs) {
    stanceHs = stance_Hs;
}

void CCPG::computeCPG() {

    if(useHeightModulation) {
        computeWithHeightModulation();
    }
    else {
        computeStandardModulation();
    }

}

double CCPG::getAngularFrequency() {

    return w_s;
}


void CCPG::computeStandardModulation() {

    double kv;

    for (int i = 1; i <= subSteps; i++)
        {

            //These switching functions adjust the transition between stance and swing phases. (EIGEN)
            sigUpP=1/(exp(-b_p * Xp(Z) / Hs) + 1);  sigDownP=1/(exp(b_p * Xp(Z) / Hs) + 1);


            //These switching functions adjust the transition between leg stance and swing behaviors.
            sigUpF=1/(exp(-b_f * (Xp(Z) - z_td) / Hs) + 1);      sigDownF=1/(exp(b_f * (Xp(Z) - z_td) / Hs) + 1);


            //These switching functions adjust the angular velocity of the limit cycle during swing phase.
            kv = k_vf / (exp(b_v * Xp(X) * 2 / Ls) + 1)  +  (k_vf / (2 * k_vf - 1)) / (exp(-b_v * Xp(X) * 2 / Ls)+1);


            //Coeficients that determine the angular velocity of the oscillators
            if(defaultModulation) {
                w_s = 3.14 * Vs / Ls * (Df/(1-Df) * sigUpP * kv + sigDownP) * slowerFactor;
            }
            else {
                w_s = omega;
            }


            // Master Oscillator
            dXp(X) = alpha * (1 - 4 * Xp(X)*Xp(X) / Ls / Ls - Xp(Z)*Xp(Z) / Hs / Hs) * Xp(X) + Xp(Z) * Ls / 2 / Hs * w_s;
            dXp(Y) = 0.0;
            if(maxPlusModulation){
              dXp(Z) = gamma * (1 - 4 * Xp(X)*Xp(X) / Ls / Ls - Xp(Z)*Xp(Z) / Hs / Hs) * Xp(Z) - Xp(X) * 2 * Hs / Ls * w_s; /*OCTAVIO*/
            }
            else{
              dXp(Z) = gamma * (1 - 4 * Xp(X)*Xp(X) / Ls / Ls - Xp(Z)*Xp(Z) / Hs / Hs) * Xp(Z) - Xp(X) * 2 * Hs / Ls * w_s + couplingTerm;
            }


            // Nonlinear filter
            dXf = sigUpF * (R * dXp + k_c * (R * Xp + deltaOrigin - Xf)) - sigDownF * V;


            // Safety checks
            for (int j = X; j <= Z; j++){
                if (dXf(j) > rateSaturation)
                    dXf(j) = rateSaturation;
                if(dXf(j) < -rateSaturation)
                    dXf(j) = -rateSaturation;
            }


            //Numerical Integration
            Xp = Xp + 0.5 * (dXp + last_dXp) * ((1/taskFrequency) / subSteps);

            // dWCPGb update for the next step of integration
            last_dXp = dXp;


            if(useOutPutFilter) {
                Xf = Xf + 0.5 * (dXf + last_dXf) * ((1/taskFrequency) / subSteps);
                last_dXf = dXf;
            }
            else {
                Xf = Xp;
                dXf = dXp;
                last_dXf = last_dXp;
            }

        }

}

void CCPG::computeWithHeightModulation() {

    double kv, w_s;

    for (int i = 1; i <= subSteps; i++)
            {

                //These switching functions adjust the transition between stance and swing phases. (EIGEN)
                sigUpP=1/(exp(-b_p * Xp(Z) / Hs) + 1);  sigDownP=1/(exp(b_p * Xp(Z) / Hs) + 1);


                //These switching functions adjust the transition between leg stance and swing behaviors.
                sigUpF=1/(exp(-b_f * (Xp(Z) - z_td) / Hs) + 1);      sigDownF=1/(exp(b_f * (Xp(Z) - z_td) / Hs) + 1);


                //These switching functions adjust the angular velocity of the limit cycle during swing phase.
                kv = k_vf / (exp(b_v * Xp(X) * 2 / Ls) + 1)  +  (k_vf / (2 * k_vf - 1)) / (exp(-b_v * Xp(X) * 2 / Ls)+1);


                //Coeficients that determine the angular velocity of the oscillators
                if(defaultModulation) {
                    w_s = 3.14 * Vs / Ls * (Df/(1-Df) * sigUpP * kv + sigDownP) * slowerFactor;
                }
                else {
                    w_s = omega;
                }


                // Master Oscillator
                dXp(X) = alpha * (1 - 4 * Xp(X)*Xp(X) / Ls / Ls - Xp(Z)*Xp(Z) / Hs / Hs) * Xp(X) + Xp(Z) * Ls / 2 / Hs * w_s;
                dXp(Y) = 0.0;
                if(maxPlusModulation){
                  dXp(Z) = gamma * (1 - 4 * Xp(X)*Xp(X) / Ls / Ls - Xp(Z)*Xp(Z) / Hs / Hs) * Xp(Z) - Xp(X) * 2 * Hs / Ls * w_s; /*OCTAVIO*/
                }
                else{
                  dXp(Z) = gamma * (1 - 4 * Xp(X)*Xp(X) / Ls / Ls - Xp(Z)*Xp(Z) / Hs / Hs) * Xp(Z) - Xp(X) * 2 * Hs / Ls * w_s + couplingTerm;
                }



                //Keep vertical rate
                if (Xp(Z) <= 0.0) {
                    dXf(Z) = dXp(Z) * stanceHs / Hs;
                }


                //Keep vertical rate
                V(Z) = - dXp(Z) * stanceHs / Hs;

                // Nonlinear filter
                dXf = sigUpF * (R * dXp + k_c * (R * Xp + deltaOrigin - Xf)) - sigDownF * V;

                //Keep vertical rate
                //if (Xp(Z) <= 0.0) {
                //    dXf(Z) = dXp(Z) * stanceHs / Hs;
                //}


                // Safety checks
                for (int j = X; j <= Z; j++){
                    if (dXf(j) > rateSaturation)
                        dXf(j) = rateSaturation;
                    if(dXf(j) < -rateSaturation)
                        dXf(j) = -rateSaturation;
                }


                //Numerical Integration
                Xp = Xp + 0.5 * (dXp + last_dXp) * ((1/taskFrequency) / subSteps);

                // dWCPGb update for the next step of integration
                last_dXp = dXp;


                if(useOutPutFilter) {
                    Xf = Xf + 0.5 * (dXf + last_dXf) * ((1/taskFrequency) / subSteps);
                    last_dXf = dXf;
                    //Keep vertical rate
                    if (Xp(Z) <= 0.0) {
                        Xf(Z) = Xp(Z) * stanceHs / Hs;
                    }

                }
                else {
                    Xf = Xp;
                    dXf = dXp;
                    last_dXf = last_dXp;
                }

            }


}
