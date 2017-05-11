/*
 * hyqWorkspaceCheck.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: victor
 */

#include "HyQ2MaxWorkspaceCheck.h"

using namespace std;

HyQ2MaxWorkspaceCheck::HyQ2MaxWorkspaceCheck() {

	a1 = 0.35;
	a2 = 0.33;
	r = 0.5;
	gamma = 3 / 2 * 3.1415;
	gamma_max = 3 / 2 * 3.1415;
	gamma_min = 3 / 2 * 3.1415;
	gamma_prime = 3.1415 / 4;
	haa_max = 0.5236 - 0.2;
	haa_min = -1.5708 + 0.2;
	hfe_max_bf = 1.2217;
	hfe_min_bf = -0.8727;
	hfe_max_br = 0.8727;
	hfe_min_br = -1.2217;
	kfe_max_bf = -0.6; // (from config/SensorOffset.cf: -0.3491. From real robot: -0.538.)
	kfe_min_bf = -2.38; // (from config/SensorOffset.cf: -2.4435. From real robot: -2.412.)
	haa = 0.0;
	dx = 0.366;
	dy = 0.207;
	dz = 0.0;
	d_haa_hfe = 0.08;
	ratio = 1;
	coefAdj = 10000;
	memoryPosition << 0, 0, -0.5;
	check = true;
	r_max = d_haa_hfe + sqrt(a1 * a1 + a2 * a2 - 2 * a1 * a2 * cos(3.1415 + kfe_max_bf));
	r_min = d_haa_hfe + sqrt(a1 * a1 + a2 * a2 - 2 * a1 * a2 * cos(3.1415 + kfe_min_bf));
	positionOffset.setZero();
	diff.setZero();
	deltaAngles.setZero();

}

HyQ2MaxWorkspaceCheck::~HyQ2MaxWorkspaceCheck() {
	// TODO Auto-generated destructor stub
}

void HyQ2MaxWorkspaceCheck::setMaxHaaAngle(const double angle) {
	haa_max = angle;
}

void HyQ2MaxWorkspaceCheck::setMinHaaAngle(const double angle) {
	haa_min = angle;
}

void HyQ2MaxWorkspaceCheck::setMaxHfeAngleFrontLegs(const double angle) {
	hfe_max_bf = angle;
}

void HyQ2MaxWorkspaceCheck::setMinHfeAngleFrontLegs(const double angle) {
	hfe_min_bf = angle;
}

void HyQ2MaxWorkspaceCheck::setMaxHfeAngleRearLegs(const double angle) {
	hfe_max_br = angle;
}

void HyQ2MaxWorkspaceCheck::setMinHfeAngleRearLegs(const double angle) {
	hfe_min_br = angle;
}

void HyQ2MaxWorkspaceCheck::setMaxKfeAngleFrontLegs(const double angle) {
	kfe_max_bf = angle;
}

void HyQ2MaxWorkspaceCheck::setMinKfeAngleFrontLegs(const double angle) {
	kfe_min_bf = angle;
}

void HyQ2MaxWorkspaceCheck::setAdjustmentCoefficient(const double coefficient) {
	coefAdj = coefficient;
}

void HyQ2MaxWorkspaceCheck::legWorkspaceCheck(const int leg_id,
										        iit::rbd::Vector3d& reachableFootPosition,
										        iit::rbd::Vector3d& adjustedFootVelocity) {

    iit::rbd::Vector3d desFootPosition;
    iit::rbd::Vector3d desFootVelocity;

    desFootPosition = reachableFootPosition;
    desFootVelocity = adjustedFootVelocity;


	double zMemory = 0.0;
	double deltaZ = 0.0;
	double deltaY = 0.0;
	//double deltaX = 0.0; Not used if HFE is not checked.
	//double auxAngle = 0.0; Not used if HFE is not checked.
	iit::rbd::Vector3d auxPosMemory;
	auxPosMemory = desFootPosition;


	//Default state when no adjustment is needed
	adjustedFootVelocity = desFootVelocity;
	diff.setZero();
	//Velocity adjustment is done when the desired foot position crosses the leg workspace limit


	deltaAngles.setZero();

	ratio = 1;


	switch (leg_id) {

	case 0:

		//Removing hip offset
		positionOffset << dx, dy, dz;
		desFootPosition = desFootPosition - positionOffset;
		memoryPosition = desFootPosition;

		//*****************************  KFE  *************************************//
		//REGARDING THE KFE JOINT LIMITS
		r = sqrt(desFootPosition(0) * desFootPosition(0) + desFootPosition(1) * desFootPosition(1) + desFootPosition(2) * desFootPosition(2));

		if (r > r_max)
			ratio = r_max / r;

		if (r < r_min)
			ratio = r_min / r;

		desFootPosition = ratio * desFootPosition;

		//*****************************  HAA  *************************************//
		//REGARDING THE HAA JOINT LIMITS (haa = atan(Py,Pz))
		zMemory = desFootPosition(2);

		haa = atan2(desFootPosition(1),desFootPosition(2));

		//Adjust to HyQ joint convention
		if(haa>0) {
			haa = haa -3.1415;
		}
		else {
			haa = haa +3.1415;
		}

		//Computing adjustment angle
		if (haa > haa_max)
			deltaAngles(0) = -(haa_max - haa);

		if (haa < haa_min)
			deltaAngles(0) = -(haa_min - haa);

		desFootPosition = Rxyz(deltaAngles) * desFootPosition;

		deltaZ = desFootPosition(2) - zMemory;

		desFootPosition(2) -= deltaZ;

		if (haa >= 0)
		{
			deltaY = deltaZ * tan(haa_max);
		}
		else
		{
			deltaY = deltaZ * tan(haa_min);
		}

		desFootPosition(1) -= deltaY;


		//*****************************  HFE  *************************************//
		//REGARDING THE HFE JOINT LIMITS

		/*
		//Reseting adjustment angles
		deltaAngles.setZero();


		auxPosMemory = desFootPosition;

		//Removing HAA angle so that the fitting does not affect it.
		//Note: the rotation angle related to roll inside deltaAngles (i.e. deltaAngles(0))
		//has opposite convention with respect to HAA angle. Therefore, deltaAngles is assigned
		//with the same sign of HAA angle for the rotation. This happens for the left-front and
		//left-hind legs.
		if ((haa >= haa_min) && (haa <= haa_max))
			deltaAngles(0) = haa;
		if (haa > haa_max)
			deltaAngles(0) = haa_max;
		if (haa < haa_min)
			deltaAngles(0) = haa_min;

		desFootPosition = Rxyz(deltaAngles) * desFootPosition;


		zMemory = desFootPosition(2);


		gamma = atan2(desFootPosition(2),desFootPosition(0));
		//cout << "gamma: " << gamma << endl;

		//Pre-processing
		if(gamma > 0) {
			gamma = gamma - 3.1415;
		}
		else {
			gamma = gamma + 3.1415;
		}
		//cout << "gamma: " << gamma << endl;

		r = sqrt(desFootPosition(0) * desFootPosition(0) + desFootPosition(1) * desFootPosition(1) + desFootPosition(2) * desFootPosition(2));
		//cout << "r: " << r << endl;

		gamma_prime = acos((a1 * a1 + r * r - a2 * a2) / 2 / a1 / r);
		//cout << "gamma_prime: " << gamma_prime << endl;

		gamma_max = hfe_max_bf - gamma_prime + 3.1415 / 2;
		//cout << "gamma_max: " << gamma_max << endl;

		gamma_min = hfe_min_bf - gamma_prime + 3.1415 / 2;
		//cout << "gamma_min: " << gamma_min << endl;

		deltaAngles.setZero();

		if (gamma > gamma_max)
			deltaAngles(1) = -(gamma_max - gamma);

		if (gamma < gamma_min)
			deltaAngles(1) = -(gamma_min - gamma);

		desFootPosition = Rxyz(deltaAngles) * desFootPosition;
		*/

		/*
		deltaZ = desFootPosition(2) - zMemory;

		desFootPosition(2) -= deltaZ;

		if (gamma >= 0)
		{
			deltaX = deltaZ * tan(gamma_max);
		}
		else
		{
			deltaX = deltaZ * tan(gamma_min);
		}

		desFootPosition(0) -= deltaX;
		 */

		/*
		//Adding HAA angle removed at the beginning of the fitting.
		//Note: the rotation angle related to roll inside deltaAngles (i.e. deltaAngles(0))
		//has opposite convention with respect to HAA angle.
		deltaAngles.setZero();

		if ((haa > haa_min) && (haa < haa_max))
			deltaAngles(0) = -haa;
		if (haa >= haa_max)
			deltaAngles(0) = -haa_max;
		if (haa <= haa_min)
			deltaAngles(0) = -haa_min;

		desFootPosition = Rxyz(deltaAngles) * desFootPosition;
		*/

		//**********************  FINAL ASSIGNMENT  *******************************//

		reachableFootPosition = desFootPosition + positionOffset;

		diff = memoryPosition - (reachableFootPosition - positionOffset);
		break;


	case 1:

		//Removing hip offset
		positionOffset << dx, -dy, dz;
		desFootPosition = desFootPosition - positionOffset;
		memoryPosition  = desFootPosition;


		//*****************************  KFE  *************************************//
		//REGARDING THE KFE JOINT LIMITS
		r = sqrt(desFootPosition(0) * desFootPosition(0) + desFootPosition(1) * desFootPosition(1) + desFootPosition(2) * desFootPosition(2));

		if (r > r_max)
			ratio = r_max / r;

		if (r < r_min)
			ratio = r_min / r;

		desFootPosition = ratio * desFootPosition;


		//*****************************  HAA  *************************************//
		//REGARDING THE HAA JOINT LIMITS (haa = atan(Py,Pz))
		zMemory = desFootPosition(2);

		haa = atan2(desFootPosition(1),desFootPosition(2));

		//Adjust to HyQ joint convention
		if(haa>0) {
			haa = -(haa -3.1415);
		}
		else {
			haa = -(haa +3.1415);
		}

		//Computing adjustment angle
		if (haa > haa_max)
			deltaAngles(0) = (haa_max - haa);

		if (haa < haa_min)
			deltaAngles(0) = (haa_min - haa);

		desFootPosition = Rxyz(deltaAngles) * desFootPosition;

		deltaZ = desFootPosition(2) - zMemory;

		desFootPosition(2) -= deltaZ;

		if (haa >= 0)
		{
			deltaY = -deltaZ * tan(haa_max);
		}
		else
		{
			deltaY = -deltaZ * tan(haa_min);
		}

		desFootPosition(1) -= deltaY;


		//*****************************  HFE  *************************************//
		//REGARDING THE HFE JOINT LIMITS

		/*
		//Reseting adjustment angles
		deltaAngles.setZero();


		//Regarding the hfe joint limits
		double alpha = atan2(position(0),abs(position(2)));
		double hfe = alpha;

		if (hfe > hfe_max_bf)
			deltaAngles(1) = hfe_max_bf - hfe;
		//deltaAngles.a2 = hfe_max - hfe;

		if (hfe < hfe_min_bf)
			deltaAngles(1) = hfe_min_bf - hfe;
		//deltaAngles.a2 = hfe_min - hfe;

		position = Rxyz(deltaAngles) * position;
		*/


		//**********************  FINAL ASSIGNMENT  *******************************//

		reachableFootPosition = desFootPosition + positionOffset;

		diff = memoryPosition - (reachableFootPosition - positionOffset);

		break;



	case 2:

		//Removing hip offset
		positionOffset << -dx, dy, dz;
		desFootPosition = desFootPosition - positionOffset;
		memoryPosition = desFootPosition;

		//*****************************  KFE  *************************************//
		//REGARDING THE KFE JOINT LIMITS
		r = sqrt(desFootPosition(0) * desFootPosition(0) + desFootPosition(1) * desFootPosition(1) + desFootPosition(2) * desFootPosition(2));

		if (r > r_max)
			ratio = r_max / r;

		if (r < r_min)
			ratio = r_min / r;

		desFootPosition = ratio * desFootPosition;


		//*****************************  HAA  *************************************//
		//REGARDING THE HAA JOINT LIMITS (haa = atan(Py,Pz))
		zMemory = desFootPosition(2);

		haa = atan2(desFootPosition(1),desFootPosition(2));

		//Adjust to HyQ joint convention
		if(haa>0) {
			haa = haa -3.1415;
		}
		else {
			haa = haa +3.1415;
		}

		//Computing adjustment angle
		if (haa > haa_max)
			deltaAngles(0) = -(haa_max - haa);

		if (haa < haa_min)
			deltaAngles(0) = -(haa_min - haa);

		desFootPosition = Rxyz(deltaAngles) * desFootPosition;

		deltaZ = desFootPosition(2) - zMemory;

		desFootPosition(2) -= deltaZ;

		if (haa >= 0)
		{
			deltaY = deltaZ * tan(haa_max);
		}
		else
		{
			deltaY = deltaZ * tan(haa_min);
		}

		desFootPosition(1) -= deltaY;

		//*****************************  HFE  *************************************//
		//REGARDING THE HFE JOINT LIMITS

		/*
		//Reseting adjustment angles
		deltaAngles.setZero();


		        //Regarding the hfe joint limits
		        double alpha = atan2(position(0),abs(position(2)));
		        double hfe = alpha;

		        if (hfe > hfe_max_bf)
		            deltaAngles(1) = hfe_max_bf - hfe;
		            //deltaAngles.a2 = hfe_max - hfe;

		        if (hfe < hfe_min_bf)
		            deltaAngles(1) = hfe_min_bf - hfe;
		            //deltaAngles.a2 = hfe_min - hfe;

		        position = Rxyz(deltaAngles) * position;
		 */



		//**********************  FINAL ASSIGNMENT  *******************************//

		reachableFootPosition = desFootPosition + positionOffset;

		diff = memoryPosition - (reachableFootPosition - positionOffset);

		break;



	case 3:

		//Removing hip offset
		positionOffset << -dx, -dy, dz;
		desFootPosition = desFootPosition - positionOffset;
		memoryPosition = desFootPosition;

		//*****************************  KFE  *************************************//
		//REGARDING THE KFE JOINT LIMITS
		r = sqrt(desFootPosition(0) * desFootPosition(0) + desFootPosition(1) * desFootPosition(1) + desFootPosition(2) * desFootPosition(2));

		if (r > r_max)
			ratio = r_max / r;

		if (r < r_min)
			ratio = r_min / r;

		desFootPosition = ratio * desFootPosition;


		//*****************************  HAA  *************************************//
		//REGARDING THE HAA JOINT LIMITS (haa = atan(Py,Pz))
		zMemory = desFootPosition(2);

		haa = atan2(desFootPosition(1),desFootPosition(2));

		//Adjust to HyQ joint convention
		if(haa>0) {
			haa = -(haa -3.1415);
		}
		else {
			haa = -(haa +3.1415);
		}

		//Computing adjustment angle
		if (haa > haa_max)
			deltaAngles(0) = (haa_max - haa);

		if (haa < haa_min)
			deltaAngles(0) = (haa_min - haa);

		desFootPosition = Rxyz(deltaAngles) * desFootPosition;

		deltaZ = desFootPosition(2) - zMemory;

		desFootPosition(2) -= deltaZ;

		if (haa >= 0)
		{
			deltaY = -deltaZ * tan(haa_max);
		}
		else
		{
			deltaY = -deltaZ * tan(haa_min);
		}

		desFootPosition(1) -= deltaY;


		//*****************************  HFE  *************************************//
		//REGARDING THE HFE JOINT LIMITS

		/*
		//Reseting adjustment angles
		deltaAngles.setZero();


		        //Regarding the hfe joint limits
		        double alpha = atan2(position(0),abs(position(2)));
		        double hfe = alpha;

		        if (hfe > hfe_max_bf)
		            deltaAngles(1) = hfe_max_bf - hfe;
		            //deltaAngles.a2 = hfe_max - hfe;

		        if (hfe < hfe_min_bf)
		            deltaAngles(1) = hfe_min_bf - hfe;
		            //deltaAngles.a2 = hfe_min - hfe;

		        position = Rxyz(deltaAngles) * position;
		 */


		//**********************  FINAL ASSIGNMENT  *******************************//

		reachableFootPosition = desFootPosition + positionOffset;

		diff = memoryPosition - (reachableFootPosition - positionOffset);

		break;
	}

	adjustedFootVelocity(0) = desFootVelocity(0) * exp(-coefAdj * diff(0) * diff(0));
	adjustedFootVelocity(1) = desFootVelocity(1) * exp(-coefAdj * diff(1) * diff(1));
	adjustedFootVelocity(2) = desFootVelocity(2) * exp(-coefAdj * diff(2) * diff(2));

}


	iit::rbd::Matrix33d HyQ2MaxWorkspaceCheck::Rxyz(iit::rbd::Vector3d& angles) {

		iit::rbd::Matrix33d Rout;

		Rout << cos(angles(1)), sin(angles(1))*sin(angles(0)), cos(angles(0)) * sin(angles(1)),
				0, cos(angles(0)), -sin(angles(0)),
				-sin(angles(1)), cos(angles(1)) * sin(angles(0)), cos(angles(1)) * cos(angles(0));

		return Rout;
	}


