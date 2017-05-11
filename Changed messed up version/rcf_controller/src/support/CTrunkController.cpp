/*
 * CTrunkController.cpp
 *
 *  Created on: Nov 5, 2013
 *      Author: Victor Barasuol
 */

//#include <stdio.h>
//#include <iit/robots/hyq/leg_data_map.h>
//#include <iit/robots/hyq2max/declarations.h>
#include "CTrunkController.h"

using namespace iit;

CTrunkController::CTrunkController() {

	minNumberStanceLegs = 1;
	trunkRoll = 0.0;
	trunkPitch = 0.0;
	trunkControllerOption = 1;
	trunkWrench.setZero();
	jointTorques.setZero();
	stanceLegs = false;
}


CTrunkController::~CTrunkController() {
	// TODO Auto-generated destructor stub
}


void CTrunkController::setActualFeetPos(const rbd::Vector3d& actualFootPosLF,
			  	  	  	  	  	           const rbd::Vector3d& actualFootPosRF,
			  	  	  	  	  	           const rbd::Vector3d& actualFootPosLH,
			  	  	  	  	  	           const rbd::Vector3d& actualFootPosRH) {

	actualFootPos[LF] = actualFootPosLF;
	actualFootPos[RF] = actualFootPosRF;
	actualFootPos[LH] = actualFootPosLH;
	actualFootPos[RH] = actualFootPosRH;
}

void CTrunkController::setActualFeetPos(const dog::LegDataMap<rbd::Vector3d>& actualFootPosition) {

    actualFootPos = actualFootPosition;
}


void CTrunkController::setFeetJacobians(const legJac& JFootLF,
		                                    const legJac& JFootRF,
		                                    const legJac& JFootLH,
		                                    const legJac& JFootRH) {

	J_Leg[LF] = JFootLF.block<3,3>(rbd::LX,0);
	J_Leg[RF] = JFootRF.block<3,3>(rbd::LX,0);
	J_Leg[LH] = JFootLH.block<3,3>(rbd::LX,0);
	J_Leg[RH] = JFootRH.block<3,3>(rbd::LX,0);
}

void CTrunkController::setFeetJacobians(const dog::LegDataMap<legJac>& allJ_Leg) {

    J_Leg[LF] = allJ_Leg[LF].block<3,3>(rbd::LX,0);
    J_Leg[RF] = allJ_Leg[RF].block<3,3>(rbd::LX,0);
    J_Leg[LH] = allJ_Leg[LH].block<3,3>(rbd::LX,0);
    J_Leg[RH] = allJ_Leg[RH].block<3,3>(rbd::LX,0);
}


void CTrunkController::setTrunkAttitude(const double& t_roll, const double& t_pitch) {

	trunkRoll = t_roll;
	trunkPitch = t_pitch;

}

void CTrunkController::setMinStanceLegs(const double& minNumberOfStanceLegs) {

	minNumberStanceLegs = minNumberOfStanceLegs;

}

void CTrunkController::setTrunkControllerOption(const int& controller_option) {

	trunkControllerOption = controller_option;

}

void CTrunkController::setStanceLegs(const dog::LegDataMap<bool>& stance) {

    stanceLegs = stance;

}

void CTrunkController::setTrunkWrench(const Eigen::Matrix<double, 6,1>& trunkW) {

    trunkWrench = trunkW;
}



void CTrunkController::computeOutputTorques() {

    int nStanceLegs = 0;
    jointTorques.setZero();

    switch (trunkControllerOption) {
    case 1:

        if (stanceLegs[LF]) {
            nStanceLegs++;
        }
        if (stanceLegs[RF]) {
            nStanceLegs++;
        }
        if (stanceLegs[LH]) {
            nStanceLegs++;
        }
        if (stanceLegs[RH]) {
            nStanceLegs++;
        }

        if (nStanceLegs >= minNumberStanceLegs) {

            double n_joints = 12;
            Eigen::Matrix<double, 3, 18> Jh1;
            Eigen::Matrix<double, 3, 18> Jh2;
            Eigen::Matrix<double, 3, 18> Jh3;
            Eigen::Matrix<double, 3, 18> Jh4;
            Eigen::Matrix<double, 12, 18> Jh;

            Eigen::Matrix<double, 18, 1> tau_desired;
            Eigen::Matrix<double, 18, 1> tau_control;
            Eigen::Matrix<double, 18, 12> pseudo_Jh;
            Eigen::Matrix<double, 18, 12> pseudo_Jh_aux;
            Eigen::Matrix<double, 18, 18> Projector;
            Eigen::Matrix<double, 3, 3> dRdPhi;
            Eigen::Matrix<double, 3, 3> dRdTheta;
            Eigen::Matrix<double, 3, 3> dRdPsi;
            Eigen::Matrix<double, 3, 3> M1;
            Eigen::Matrix<double, 3, 3> M2;
            Eigen::Matrix<double, 3, 3> M3;
            Eigen::Matrix<double, 3, 3> M4;
            Eigen::Matrix<double, 3, 3> Xf11, Xf12, Xf13;
            Eigen::Matrix<double, 3, 3> Xf21, Xf22, Xf23;
            Eigen::Matrix<double, 3, 3> Xf31, Xf32, Xf33;
            Eigen::Matrix<double, 3, 3> Xf41, Xf42, Xf43;
            Eigen::Matrix<double, 3, 3> R;
            Eigen::Matrix<double, 18, 18> W;

            Jh.setZero();
            Jh1.setZero();
            Jh2.setZero();
            Jh3.setZero();
            Jh4.setZero();
            pseudo_Jh.setZero();
            pseudo_Jh_aux.setZero();
            tau_desired.setZero();
            tau_control.setZero();

            W = Eigen::Matrix<double, 18, 18>::Identity();

            Xf11 << actualFootPos[LF](rbd::X), 0, 0, actualFootPos[LF](rbd::Y), 0, 0, actualFootPos[LF](
                    rbd::Z), 0, 0;
            Xf12 << 0, actualFootPos[LF](rbd::X), 0, 0, actualFootPos[LF](
                    rbd::Y), 0, 0, actualFootPos[LF](rbd::Z), 0;
            Xf13 << 0, 0, actualFootPos[LF](rbd::X), 0, 0, actualFootPos[LF](
                    rbd::Y), 0, 0, actualFootPos[LF](rbd::Z);

            Xf21 << actualFootPos[RF](rbd::X), 0, 0, actualFootPos[RF](rbd::Y), 0, 0, actualFootPos[RF](
                    rbd::Z), 0, 0;
            Xf22 << 0, actualFootPos[RF](rbd::X), 0, 0, actualFootPos[RF](
                    rbd::Y), 0, 0, actualFootPos[RF](rbd::Z), 0;
            Xf23 << 0, 0, actualFootPos[RF](rbd::X), 0, 0, actualFootPos[RF](
                    rbd::Y), 0, 0, actualFootPos[RF](rbd::Z);

            Xf31 << actualFootPos[LH](rbd::X), 0, 0, actualFootPos[LH](rbd::Y), 0, 0, actualFootPos[LH](
                    rbd::Z), 0, 0;
            Xf32 << 0, actualFootPos[LH](rbd::X), 0, 0, actualFootPos[LH](
                    rbd::Y), 0, 0, actualFootPos[LH](rbd::Z), 0;
            Xf33 << 0, 0, actualFootPos[LH](rbd::X), 0, 0, actualFootPos[LH](
                    rbd::Y), 0, 0, actualFootPos[LH](rbd::Z);

            Xf41 << actualFootPos[RH](rbd::X), 0, 0, actualFootPos[RH](rbd::Y), 0, 0, actualFootPos[RH](
                    rbd::Z), 0, 0;
            Xf42 << 0, actualFootPos[RH](rbd::X), 0, 0, actualFootPos[RH](
                    rbd::Y), 0, 0, actualFootPos[RH](rbd::Z), 0;
            Xf43 << 0, 0, actualFootPos[RH](rbd::X), 0, 0, actualFootPos[RH](
                    rbd::Y), 0, 0, actualFootPos[RH](rbd::Z);

            //Rotation matrix
            R(0, 0) = cos(trunkPitch);
            R(0, 1) = sin(trunkPitch) * sin(trunkRoll);
            R(0, 2) = cos(trunkRoll) * sin(trunkPitch);
            R(1, 0) = 0.0;
            R(1, 1) = cos(trunkRoll);
            R(1, 2) = -sin(trunkRoll);
            R(2, 0) = -sin(trunkPitch);
            R(2, 1) = cos(trunkPitch) * sin(trunkRoll);
            R(2, 2) = cos(trunkPitch) * cos(trunkRoll);

            // Rt11=cos(theta)*cos(psi)
            double dR11dPhi = 0.0;
            double dR11dTheta = -sin(trunkPitch);
            double dR11dPsi = 0.0;

            // R12=-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi)
            double dR12dPhi = cos(trunkRoll) * sin(trunkPitch);
            double dR12dTheta = sin(trunkRoll) * cos(trunkPitch);
            double dR12dPsi = -cos(trunkRoll);

            // R13=sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos*(psi)
            double dR13dPhi = -sin(trunkRoll) * sin(trunkPitch);
            double dR13dTheta = cos(trunkRoll) * cos(trunkPitch);
            double dR13dPsi = sin(trunkRoll);

            // R21=cos(theta)*sin(psi)
            double dR21dPhi = 0.0;
            double dR21dTheta = 0.0;
            double dR21dPsi = cos(trunkPitch);

            // R22=cos(phi)*cos(psi)+sin(psi)*sin(theta)*sin(psi)
            double dR22dPhi = -sin(trunkRoll);
            double dR22dTheta = 0.0;
            double dR22dPsi = sin(trunkRoll) * sin(trunkPitch);

            // R23=-sin(phi)*cos(psi)+cos(psi)*sin(theta)*sin(psi)
            double dR23dPhi = -cos(trunkRoll);
            double dR23dTheta = 0.0;
            double dR23dPsi = cos(trunkRoll) * sin(trunkPitch);

            // R31=-sin(theta)
            double dR31dPhi = 0.0;
            double dR31dTheta = -cos(trunkPitch);
            double dR31dPsi = 0.0;

            // R32=sin(psi)*cos(theta)
            double dR32dPhi = cos(trunkRoll) * cos(trunkPitch);
            double dR32dTheta = -sin(trunkRoll) * sin(trunkPitch);
            double dR32dPsi = 0;

            // R33=cos(psi)*cos(theta)
            double dR33dPhi = -sin(trunkRoll) * cos(trunkPitch);
            double dR33dTheta = -cos(trunkRoll) * sin(trunkPitch);
            double dR33dPsi = 0;

            dRdPhi << dR11dPhi, dR12dPhi, dR13dPhi, dR21dPhi, dR22dPhi, dR23dPhi, dR31dPhi, dR32dPhi, dR33dPhi;

            dRdTheta << dR11dTheta, dR12dTheta, dR13dTheta, dR21dTheta, dR22dTheta, dR23dTheta, dR31dTheta, dR32dTheta, dR33dTheta;

            dRdPsi << dR11dPsi, dR12dPsi, dR13dPsi, dR21dPsi, dR22dPsi, dR23dPsi, dR31dPsi, dR32dPsi, dR33dPsi;

            M1 = dRdPhi * Xf11 + dRdTheta * Xf12 + dRdPsi * Xf13;
            M2 = dRdPhi * Xf21 + dRdTheta * Xf22 + dRdPsi * Xf23;
            M3 = dRdPhi * Xf31 + dRdTheta * Xf32 + dRdPsi * Xf33;
            M4 = dRdPhi * Xf41 + dRdTheta * Xf42 + dRdPsi * Xf43;

            Jh1.block<3, 3>(0, 0) = R * J_Leg[LF];
            Jh2.block<3, 3>(0, 3) = R * J_Leg[RF];
            Jh3.block<3, 3>(0, 6) = R * J_Leg[LH];
            Jh4.block<3, 3>(0, 9) = R * J_Leg[RH];

            Jh1.block<3, 3>(0, 12).setIdentity();
            Jh2.block<3, 3>(0, 12).setIdentity();
            Jh3.block<3, 3>(0, 12).setIdentity();
            Jh4.block<3, 3>(0, 12).setIdentity();

            Jh1.block<3, 3>(0, 15) = M1;
            Jh2.block<3, 3>(0, 15) = M2;
            Jh3.block<3, 3>(0, 15) = M3;
            Jh4.block<3, 3>(0, 15) = M4;

            double W1 = 1000;
            W.block<6, 6>(12, 12) = W1
                    * Eigen::Matrix<double, 6, 6>::Identity();

            double count = 0;
            if (stanceLegs[LF]) {
                Jh.block<3, 18>(0, 0) = Jh1;
                count++;
            }
            if (stanceLegs[RF]) {
                Jh.block<3, 18>(3 * count, 0) = Jh2;
                count++;
            }
            if (stanceLegs[LH]) {
                Jh.block<3, 18>(3 * count, 0) = Jh3;
                count++;
            }
            if (stanceLegs[RH]) {
                Jh.block<3, 18>(3 * count, 0) = Jh4;
                count++;
            }

            //Computing controller action.
            Eigen::Matrix<double, 12, 12> PartialPseudoInverse;
            Eigen::Matrix<double, 12, 12> ridgeFactor;
            ridgeFactor = 0.0001 * Eigen::Matrix<double, 12, 12>::Identity();
            pseudo_Jh.leftCols(3 * count) =
                    W * Jh.topRows(3 * count).transpose()
                            * (Jh.topRows(3 * count) * W
                                    * Jh.topRows(3 * count).transpose()
                                    + ridgeFactor.block(0, 0, 3 * count,
                                            3 * count)).inverse();

            Projector = (Eigen::Matrix<double, 18, 18>::Identity()
                    - pseudo_Jh * Jh).transpose();

            tau_desired.block<6, 1>(12, 0) = trunkWrench;
            jointTorques = (Projector * tau_desired).topRows(n_joints);
        }

        break;


    case 2:

        if (stanceLegs[LF]) {
            nStanceLegs++;
        }
        if (stanceLegs[RF]) {
            nStanceLegs++;
        }
        if (stanceLegs[LH]) {
            nStanceLegs++;
        }
        if (stanceLegs[RH]) {
            nStanceLegs++;
        }

        if (nStanceLegs >= minNumberStanceLegs) {

            Eigen::Matrix<double, 12, 12> RJb;
            Eigen::Matrix<double, 3, 6> Jc1;
            Eigen::Matrix<double, 3, 6> Jc2;
            Eigen::Matrix<double, 3, 6> Jc3;
            Eigen::Matrix<double, 3, 6> Jc4;
            Eigen::Matrix<double, 12, 6> Jc;
            Eigen::Matrix<double, 6, 12> pseudo_Jc;

            Eigen::Matrix<double, 3, 3> dRdPhi;
            Eigen::Matrix<double, 3, 3> dRdTheta;
            Eigen::Matrix<double, 3, 3> dRdPsi;
            Eigen::Matrix<double, 3, 3> M1;
            Eigen::Matrix<double, 3, 3> M2;
            Eigen::Matrix<double, 3, 3> M3;
            Eigen::Matrix<double, 3, 3> M4;
            Eigen::Matrix<double, 3, 3> Xf11, Xf12, Xf13;
            Eigen::Matrix<double, 3, 3> Xf21, Xf22, Xf23;
            Eigen::Matrix<double, 3, 3> Xf31, Xf32, Xf33;
            Eigen::Matrix<double, 3, 3> Xf41, Xf42, Xf43;
            Eigen::Matrix<double, 3, 3> R;
            Eigen::Matrix<double, 18, 1> tau_desired;
            Eigen::Matrix<double, 18, 1> tau_control;

            RJb.setZero();
            Jc.setZero();
            pseudo_Jc.setZero();
            tau_desired.setZero();
            tau_control.setZero();

            Xf11 << actualFootPos[LF](rbd::X), 0, 0, actualFootPos[LF](rbd::Y), 0, 0, actualFootPos[LF](
                    rbd::Z), 0, 0;
            Xf12 << 0, actualFootPos[LF](rbd::X), 0, 0, actualFootPos[LF](
                    rbd::Y), 0, 0, actualFootPos[LF](rbd::Z), 0;
            Xf13 << 0, 0, actualFootPos[LF](rbd::X), 0, 0, actualFootPos[LF](
                    rbd::Y), 0, 0, actualFootPos[LF](rbd::Z);

            Xf21 << actualFootPos[RF](rbd::X), 0, 0, actualFootPos[RF](rbd::Y), 0, 0, actualFootPos[RF](
                    rbd::Z), 0, 0;
            Xf22 << 0, actualFootPos[RF](rbd::X), 0, 0, actualFootPos[RF](
                    rbd::Y), 0, 0, actualFootPos[RF](rbd::Z), 0;
            Xf23 << 0, 0, actualFootPos[RF](rbd::X), 0, 0, actualFootPos[RF](
                    rbd::Y), 0, 0, actualFootPos[RF](rbd::Z);

            Xf31 << actualFootPos[LH](rbd::X), 0, 0, actualFootPos[LH](rbd::Y), 0, 0, actualFootPos[LH](
                    rbd::Z), 0, 0;
            Xf32 << 0, actualFootPos[LH](rbd::X), 0, 0, actualFootPos[LH](
                    rbd::Y), 0, 0, actualFootPos[LH](rbd::Z), 0;
            Xf33 << 0, 0, actualFootPos[LH](rbd::X), 0, 0, actualFootPos[LH](
                    rbd::Y), 0, 0, actualFootPos[LH](rbd::Z);

            Xf41 << actualFootPos[RH](rbd::X), 0, 0, actualFootPos[RH](rbd::Y), 0, 0, actualFootPos[RH](
                    rbd::Z), 0, 0;
            Xf42 << 0, actualFootPos[RH](rbd::X), 0, 0, actualFootPos[RH](
                    rbd::Y), 0, 0, actualFootPos[RH](rbd::Z), 0;
            Xf43 << 0, 0, actualFootPos[RH](rbd::X), 0, 0, actualFootPos[RH](
                    rbd::Y), 0, 0, actualFootPos[RH](rbd::Z);

            //Rotation matrix
            R(0, 0) = cos(trunkPitch);
            R(0, 1) = sin(trunkPitch) * sin(trunkRoll);
            R(0, 2) = cos(trunkRoll) * sin(trunkPitch);
            R(1, 0) = 0.0;
            R(1, 1) = cos(trunkRoll);
            R(1, 2) = -sin(trunkRoll);
            R(2, 0) = -sin(trunkPitch);
            R(2, 1) = cos(trunkPitch) * sin(trunkRoll);
            R(2, 2) = cos(trunkPitch) * cos(trunkRoll);

            // Rt11=cos(theta)*cos(psi)
            double dR11dPhi = 0.0;
            double dR11dTheta = -sin(trunkPitch);
            double dR11dPsi = 0.0;

            // R12=-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi)
            double dR12dPhi = cos(trunkRoll) * sin(trunkPitch);
            double dR12dTheta = sin(trunkRoll) * cos(trunkPitch);
            double dR12dPsi = -cos(trunkRoll);

            // R13=sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos*(psi)
            double dR13dPhi = -sin(trunkRoll) * sin(trunkPitch);
            double dR13dTheta = cos(trunkRoll) * cos(trunkPitch);
            double dR13dPsi = sin(trunkRoll);

            // R21=cos(theta)*sin(psi)
            double dR21dPhi = 0.0;
            double dR21dTheta = 0.0;
            double dR21dPsi = cos(trunkPitch);

            // R22=cos(phi)*cos(psi)+sin(psi)*sin(theta)*sin(psi)
            double dR22dPhi = -sin(trunkRoll);
            double dR22dTheta = 0.0;
            double dR22dPsi = sin(trunkRoll) * sin(trunkPitch);

            // R23=-sin(phi)*cos(psi)+cos(psi)*sin(theta)*sin(psi)
            double dR23dPhi = -cos(trunkRoll);
            double dR23dTheta = 0.0;
            double dR23dPsi = cos(trunkRoll) * sin(trunkPitch);

            // R31=-sin(theta)
            double dR31dPhi = 0.0;
            double dR31dTheta = -cos(trunkPitch);
            double dR31dPsi = 0.0;

            // R32=sin(psi)*cos(theta)
            double dR32dPhi = cos(trunkRoll) * cos(trunkPitch);
            double dR32dTheta = -sin(trunkRoll) * sin(trunkPitch);
            double dR32dPsi = 0;

            // R33=cos(psi)*cos(theta)
            double dR33dPhi = -sin(trunkRoll) * cos(trunkPitch);
            double dR33dTheta = -cos(trunkRoll) * sin(trunkPitch);
            double dR33dPsi = 0;

            dRdPhi << dR11dPhi, dR12dPhi, dR13dPhi, dR21dPhi, dR22dPhi, dR23dPhi, dR31dPhi, dR32dPhi, dR33dPhi;

            dRdTheta << dR11dTheta, dR12dTheta, dR13dTheta, dR21dTheta, dR22dTheta, dR23dTheta, dR31dTheta, dR32dTheta, dR33dTheta;

            dRdPsi << dR11dPsi, dR12dPsi, dR13dPsi, dR21dPsi, dR22dPsi, dR23dPsi, dR31dPsi, dR32dPsi, dR33dPsi;

            M1 = dRdPhi * Xf11 + dRdTheta * Xf12 + dRdPsi * Xf13;
            M2 = dRdPhi * Xf21 + dRdTheta * Xf22 + dRdPsi * Xf23;
            M3 = dRdPhi * Xf31 + dRdTheta * Xf32 + dRdPsi * Xf33;
            M4 = dRdPhi * Xf41 + dRdTheta * Xf42 + dRdPsi * Xf43;

            double count = 0;
            if (stanceLegs[LF]) {
                RJb.block<3, 3>(0, 0) = R * J_Leg[LF];
                Jc.block<3, 3>(0, 0).setIdentity();
                Jc.block<3, 3>(0, 3) = M1;
                count++;
            }
            if (stanceLegs[RF]) {
                RJb.block<3, 3>(3 * count, 3) = R * J_Leg[RF];
                Jc.block<3, 3>(3 * count, 0).setIdentity();
                Jc.block<3, 3>(3 * count, 3) = M2;
                count++;
            }
            if (stanceLegs[LH]) {
                RJb.block<3, 3>(3 * count, 6) = R * J_Leg[LH];
                Jc.block<3, 3>(3 * count, 0).setIdentity();
                Jc.block<3, 3>(3 * count, 3) = M3;
                count++;
            }
            if (stanceLegs[RH]) {
                RJb.block<3, 3>(3 * count, 9) = R * J_Leg[RH];
                Jc.block<3, 3>(3 * count, 0).setIdentity();
                Jc.block<3, 3>(3 * count, 3) = M4;
                count++;
            }

            //Computing controller action.
            Eigen::Matrix<double, 12, 12> ridgeFactor;
            ridgeFactor = 0.0001 * Eigen::Matrix<double, 12, 12>::Identity();

            pseudo_Jc.leftCols(3 * count) =
                    Jc.topRows(3 * count).transpose()
                            * (Jc.topRows(3 * count)
                                    * Jc.topRows(3 * count).transpose()
                                    + ridgeFactor.block(0, 0, 3 * count,
                                            3 * count)).inverse();

            jointTorques = -(pseudo_Jc * RJb).transpose() * trunkWrench;
        }

        break;
    }
}
