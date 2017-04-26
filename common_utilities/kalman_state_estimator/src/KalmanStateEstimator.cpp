/*
 * KalmanStateEstimator.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: victor
 */


#include <kalman_state_estimator/KalmanStateEstimator.h>


KalmanStateEstimator::KalmanStateEstimator() {
    simulationFlag = false;
    considerGRF = false;
    minStanceLegs = 1;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    samplingRate = 100;
    bodyAnglesVel.setZero();
    bodyAnglesAccel.setZero();
    imuNoiseCovariance << 0.018, 0.0252, 0.0241;
    imuAccelOffset << 0.058, 0.036, 0.03;
    kinematicsNoiseCov << 0.0138, 0.0252, 0.25;
    kinematicsVelOffset.setZero();
    imuPosition << 0.5, 0.0, 0.09;
    imuAccel.setZero();
    imuAccelInTheHorizontalFrame.setZero();
    imuAccelInTheBaseFrame.setZero();
    forceSensorCov << 8, 8, 8;
    trunkLinearVelocity.setZero();
    xbEstimatedWithKinematics.setZero();
    bodyWeight = 83;
    footPosLF <<  0.5,  0.3, -0.5;
    footPosRF <<  0.5, -0.3, -0.5;
    footPosLH << -0.5,  0.3, -0.5;
    footPosRH << -0.5, -0.3, -0.5;
    supportLegs[LF] = false;
    supportLegs[RF] = false;
    supportLegs[LH] = false;
    supportLegs[RH] = false;
    JLF.setIdentity();
    JRF.setIdentity();
    JLH.setIdentity();
    JRH.setIdentity();
    R.setIdentity();
    jointTorques.setZero();
    jointVel.setZero();
}

KalmanStateEstimator::~KalmanStateEstimator() {
    // TODO Auto-generated destructor stub
}


//Set functions
void KalmanStateEstimator::setInSimulation(bool simulation) {
    simulationFlag = simulation;
}

void KalmanStateEstimator::setConsiderGRF(bool useGRF) {
    considerGRF = useGRF;
}

void KalmanStateEstimator::setMinStanceLegs(int min) {
    if (min >= 1) {
        minStanceLegs = min;
    }
    else {
        std::cout << "Minimun stance legs to use data from kinematics is 1!!!" << std::endl;
    }
}

void KalmanStateEstimator::setImuNoiseCov(Eigen::Vector3d imuCov) {
    imuNoiseCovariance = imuCov;
}

void KalmanStateEstimator::setImuAccelOffset(Eigen::Vector3d imuOffset) {
    imuAccelOffset = imuOffset;
}

void KalmanStateEstimator::setKinematicsNoiseCov(Eigen::Vector3d kinCov) {
    kinematicsNoiseCov = kinCov;
}

void KalmanStateEstimator::setKinematicsVelOffset(Eigen::Vector3d kinOffset) {
    kinematicsVelOffset = kinOffset;
}

void KalmanStateEstimator::setForceNoiseCov(Eigen::Vector3d forceCov) {
    forceSensorCov = forceCov;
}

void KalmanStateEstimator::setBodyWeight(double weight) {
    bodyWeight = weight;
}

void KalmanStateEstimator::setSamplingRate(double sampling) {
    if (sampling > 0) {
        samplingRate = sampling;
    }
    else {
        std::cout << "Sampling rate must be greater then zero!!!" << std::endl;
    }
}


//Estimation function
Eigen::Vector3d KalmanStateEstimator::trunkLinearVelocitiesInTheHF() {


     static Eigen::Matrix<double, 3,1> Xb1_est_HF;
     static Eigen::Matrix<double, 3,1> Xb2_est_HF;
     static Eigen::Matrix<double, 3,1> Xb3_est_HF;
     static Eigen::Matrix<double, 3,1> Xb4_est_HF;
     static Eigen::Matrix<double, 4,1> countIn;

     static Eigen::Matrix<double, 3,3> P_k_k = Eigen::Matrix<double, 3,3>::Identity();
     static Eigen::Matrix<double, 3,3> P_k_k1 = Eigen::Matrix<double, 3,3>::Identity();
     static Eigen::Matrix<double, 3,3> P_k1_k1 = Eigen::Matrix<double, 3,3>::Identity();
     static Eigen::Matrix<double, 3,3> Q_k = Eigen::Matrix<double, 3,3>::Identity();
     static Eigen::Matrix<double, 3,3> R_k = Eigen::Matrix<double, 3,3>::Identity();
     static Eigen::Matrix<double, 3,3> S_k;
     static Eigen::Matrix<double, 3,3> K_k;

     static Eigen::Matrix<double, 3,1> forcesOnTheBody;
     static Eigen::Matrix<double, 3,1> xbTilde;

     static Eigen::Matrix<double, 3,3> dRdPhi;
     static Eigen::Matrix<double, 3,3> dRdTheta;
     static Eigen::Matrix<double, 3,3> dRdPsi;
     static Eigen::Matrix<double, 3,3> M1;
     static Eigen::Matrix<double, 3,3> M2;
     static Eigen::Matrix<double, 3,3> M3;
     static Eigen::Matrix<double, 3,3> M4;
     static Eigen::Matrix<double, 3,3> Xf11, Xf12, Xf13;
     static Eigen::Matrix<double, 3,3> Xf21, Xf22, Xf23;
     static Eigen::Matrix<double, 3,3> Xf31, Xf32, Xf33;
     static Eigen::Matrix<double, 3,3> Xf41, Xf42, Xf43;

     static Eigen::Matrix<double, 3,1> legForce_LF;
     static Eigen::Matrix<double, 3,1> legForce_RF;
     static Eigen::Matrix<double, 3,1> legForce_LH;
     static Eigen::Matrix<double, 3,1> legForce_RH;


     static Eigen::Matrix3d skew_sim_IMU_lever_arm;
     static Eigen::Vector3d delta_vel_RPY =  Eigen::Matrix<double, 3,1>::Zero();
     static Eigen::Vector3d lastBodyAnglesVel =  Eigen::Matrix<double, 3,1>::Zero();
     static Eigen::Vector3d delta_Xb_from_IMU;

     static Eigen::Matrix3d I = Eigen::Matrix<double, 3,3>::Identity();


    double dt = 1/samplingRate;

    Xb1_est_HF.setZero();
    Xb2_est_HF.setZero();
    Xb3_est_HF.setZero();
    Xb4_est_HF.setZero();

    legForce_LF.setZero();
    legForce_RF.setZero();
    legForce_LH.setZero();
    legForce_RH.setZero();


    double nStanceLegs=0;
    if(supportLegs[0]){nStanceLegs++;}
    if(supportLegs[1]){nStanceLegs++;}
    if(supportLegs[2]){nStanceLegs++;}
    if(supportLegs[3]){nStanceLegs++;}


    if (nStanceLegs >= minStanceLegs) {

        double average_coef=1/nStanceLegs;

        //Interface SL to Eigen math
        Xf11 << footPosLF(0), 0, 0, footPosLF(1), 0, 0, footPosLF(2), 0, 0;
        Xf12 << 0, footPosLF(0), 0, 0, footPosLF(1), 0, 0, footPosLF(2), 0;
        Xf13 << 0, 0, footPosLF(0), 0, 0, footPosLF(1), 0, 0, footPosLF(2);

        Xf21 << footPosRF(0), 0, 0, footPosRF(1), 0, 0, footPosRF(2), 0, 0;
        Xf22 << 0, footPosRF(0), 0, 0, footPosRF(1), 0, 0, footPosRF(2), 0;
        Xf23 << 0, 0, footPosRF(0), 0, 0, footPosRF(1), 0, 0, footPosRF(2);

        Xf31 << footPosLH(0), 0, 0, footPosLH(1), 0, 0, footPosLH(2), 0, 0;
        Xf32 << 0, footPosLH(0), 0, 0, footPosLH(1), 0, 0, footPosLH(2), 0;
        Xf33 << 0, 0, footPosLH(0), 0, 0, footPosLH(1), 0, 0, footPosLH(2);

        Xf41 << footPosRH(0), 0, 0, footPosRH(1), 0, 0, footPosRH(2), 0, 0;
        Xf42 << 0, footPosRH(0), 0, 0, footPosRH(1), 0, 0, footPosRH(2), 0;
        Xf43 << 0, 0, footPosRH(0), 0, 0, footPosRH(1), 0, 0, footPosRH(2);

        // R11=cos(theta)*cos(psi)
        double dR11dPhi   = 0.0;
        double dR11dTheta = -sin(pitch);
        double dR11dPsi   = 0.0;

        // R12=-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi)
        double dR12dPhi   = cos(roll)*sin(pitch);
        double dR12dTheta = sin(roll)*cos(pitch);
        double dR12dPsi   = -cos(roll);

        // R13=sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos*(psi)
        double dR13dPhi   = -sin(roll)*sin(pitch);
        double dR13dTheta = cos(roll)*cos(pitch);
        double dR13dPsi   = sin(roll);

        // R21=cos(theta)*sin(psi)
        double dR21dPhi   = 0.0;
        double dR21dTheta = 0.0;
        double dR21dPsi   = cos(pitch);

        // R22=cos(phi)*cos(psi)+sin(psi)*sin(theta)*sin(psi)
        double dR22dPhi   = -sin(roll);
        double dR22dTheta = 0.0;
        double dR22dPsi   = sin(roll)*sin(pitch);

        // R23=-sin(phi)*cos(psi)+cos(psi)*sin(theta)*sin(psi)
        double dR23dPhi   = -cos(roll);
        double dR23dTheta = 0.0;
        double dR23dPsi   = cos(roll)*sin(pitch);

        // R31=-sin(theta)
        double dR31dPhi   = 0.0;
        double dR31dTheta = -cos(pitch);
        double dR31dPsi   = 0.0;

        // R32=sin(psi)*cos(theta)
        double dR32dPhi = cos(roll)*cos(pitch);
        double dR32dTheta = -sin(roll)*sin(pitch);
        double dR32dPsi = 0;

        // R33=cos(psi)*cos(theta)
        double dR33dPhi= -sin(roll)*cos(pitch);
        double dR33dTheta= -cos(roll)*sin(pitch);
        double dR33dPsi = 0;


        dRdPhi << dR11dPhi, dR12dPhi, dR13dPhi,
                dR21dPhi, dR22dPhi, dR23dPhi,
                dR31dPhi, dR32dPhi, dR33dPhi;

        dRdTheta << dR11dTheta, dR12dTheta, dR13dTheta,
                dR21dTheta, dR22dTheta, dR23dTheta,
                dR31dTheta, dR32dTheta, dR33dTheta;

        dRdPsi << dR11dPsi, dR12dPsi, dR13dPsi,
                dR21dPsi, dR22dPsi, dR23dPsi,
                dR31dPsi, dR32dPsi, dR33dPsi;

        M1 = dRdPhi*Xf11 + dRdTheta*Xf12 + dRdPsi*Xf13;
        M2 = dRdPhi*Xf21 + dRdTheta*Xf22 + dRdPsi*Xf23;
        M3 = dRdPhi*Xf31 + dRdTheta*Xf32 + dRdPsi*Xf33;
        M4 = dRdPhi*Xf41 + dRdTheta*Xf42 + dRdPsi*Xf43;


        countIn.setZero();

        //State measurement

        if(supportLegs[LF]) {
            legForce_LF=-(R * JLF).transpose().inverse() * jointTorques.block<3,1>(LF_HAA,0);

            Xb1_est_HF = -(R * JLF * jointVel.block<3,1>(LF_HAA,0) + M1 * bodyAnglesVel);

            countIn(0)=1.0;

        }

        if(supportLegs[RF]) {
            legForce_RF=-(R * JRF).transpose().inverse() * jointTorques.block<3,1>(RF_HAA,0);

            Xb2_est_HF = -(R * JRF * jointVel.block<3,1>(RF_HAA,0) + M2 * bodyAnglesVel);

            countIn(1)=1.0;

        }

        if(supportLegs[LH]) {
            legForce_LH=-(R * JLH).transpose().inverse() * jointTorques.block<3,1>(LH_HAA,0);

            Xb3_est_HF = -(R * JLH * jointVel.block<3,1>(LH_HAA,0) + M3 * bodyAnglesVel);

            countIn(2)=1.0;

        }

        if(supportLegs[RH]) {
            legForce_RH=-(R * JRH).transpose().inverse() * jointTorques.block<3,1>(RH_HAA,0);

            Xb4_est_HF = -(R * JRH * jointVel.block<3,1>(RH_HAA,0) + M4 * bodyAnglesVel);

            countIn(3)=1.0;

        }


        // Computing estimated trunk linear velocities from joint velocities.
            average_coef = 1/(countIn(0)+countIn(1)+countIn(2)+countIn(3));

            xbEstimatedWithKinematics = average_coef * (countIn(0)*Xb1_est_HF + countIn(1)*Xb2_est_HF +
                                                        countIn(2)*Xb3_est_HF + countIn(3)*Xb4_est_HF);

        //Computing forces on the body with respect to the horizontal frame.
        forcesOnTheBody = legForce_LF + legForce_RF + legForce_LH + legForce_RH;

        //Measurement covariance matrix.
        R_k(0,0) = kinematicsNoiseCov(0) * kinematicsNoiseCov(0);
        R_k(1,1) = kinematicsNoiseCov(1) * kinematicsNoiseCov(1);
        R_k(2,2) = kinematicsNoiseCov(2) * kinematicsNoiseCov(2);


        //Covariance matrix for state prediction
        Q_k(0,0) = (dt * dt) * imuNoiseCovariance(0) * imuNoiseCovariance(0);
        Q_k(1,1) = (dt * dt) * imuNoiseCovariance(1) * imuNoiseCovariance(1);

        if (considerGRF) {
            Q_k(2,2) = (dt * dt) * imuNoiseCovariance(2) * imuNoiseCovariance(2) +
                       (1 / bodyWeight / bodyWeight) * forceSensorCov(2) * forceSensorCov(2);
        }
        else {
            Q_k(2,2) = (dt * dt) * imuNoiseCovariance(2) * imuNoiseCovariance(2);
        }

        if(simulationFlag) {
            //In simulation is not needed to compensate for the IMU position (IMU is base-centered by default).
            imuAccelInTheHorizontalFrame = R * imuAccel;
            delta_Xb_from_IMU = dt * (imuAccelInTheHorizontalFrame - imuAccelOffset);

        }
        else {
            //Computing IMU accelerations due to IMU mounting lever arm.

            skew_sim_IMU_lever_arm <<       0.0,         -imuPosition(2),        imuPosition(1),
                                       imuPosition(2),        0.0,              -imuPosition(0),
                                      -imuPosition(1),   imuPosition(0),              0.0       ;


            delta_vel_RPY = bodyAnglesVel - lastBodyAnglesVel;
            lastBodyAnglesVel = bodyAnglesVel;


            imuAccelInTheBaseFrame = imuAccel - (-skew_sim_IMU_lever_arm * bodyAnglesAccel);
            imuAccelInTheHorizontalFrame = R * imuAccelInTheBaseFrame - imuAccelOffset;

            delta_Xb_from_IMU = dt * imuAccelInTheHorizontalFrame;
        }


        // State Prediction (a priori, X_k = A * X_K-1 + B * u, with A = I and B = dT/body_mass)
        trunkLinearVelocityPredicted = trunkLinearVelocity + delta_Xb_from_IMU; // xd = xd + dt*a

        //Considering the acceleration caused by ground reaction forces (needed when the acceleration signal includes gravity acceleration)
        if(considerGRF)
        trunkLinearVelocityPredicted(2) -= dt * (forcesOnTheBody(2) / bodyWeight); //negative sign because IMU considers positive gravity acceleration


        // Predicted estimate covariance (a priori)
        // P_k|k-1 = P_k-1|k-1 + Q_k   (P_k|k-1 = F_k * P_k-1|k-1 * F_k^T + Q_k with F_k = I)
        P_k_k1 =  P_k1_k1 + Q_k;

        // Inovation error
        xbTilde = xbEstimatedWithKinematics - trunkLinearVelocityPredicted;

        // Inovation covariance (S_k = P_k|k-1 + R_k, where R_k is the measurement covariance)
        S_k = P_k_k1 + R_k;

        // Kalman gain (K_k = P_k|k-1 * S_k^-1)
        K_k = P_k_k1 * S_k.inverse();


        // State Prediction (a posteriori)
        trunkLinearVelocity = trunkLinearVelocityPredicted + K_k * xbTilde;

        // Predicted estimate covariance (a posteriori, P_k = (I-K_k) * P_k|k-1)
        P_k_k = (I-K_k) * P_k_k1;

        //Updating for the next loop
        P_k1_k1 = P_k_k;

    }
    else{

        // Prediction
        forcesOnTheBody.setZero();

        // Including gravitational force into the force_on_the_body vector
        forcesOnTheBody(2) += -9.81 * bodyWeight;

        //Estimation
        trunkLinearVelocity = trunkLinearVelocity + dt * forcesOnTheBody / bodyWeight ;
    }


    return trunkLinearVelocity;
}
