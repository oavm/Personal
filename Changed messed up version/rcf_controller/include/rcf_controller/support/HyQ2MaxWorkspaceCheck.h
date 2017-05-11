/*
 * hyqWorkspaceCheck.h
 *
 *  Created on: Mar 18, 2014
 *      Author: victor
 */

#ifndef HYQ2MAXWORKSPACECHECK_H_
#define HYQ2MAXWORKSPACECHECK_H_

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <iit/rbd/rbd.h>
#include <iit/robots/hyq2max/declarations.h>

class HyQ2MaxWorkspaceCheck {
public:
	HyQ2MaxWorkspaceCheck();
	virtual ~HyQ2MaxWorkspaceCheck();

	void legWorkspaceCheck(const int leg_id,
						      iit::rbd::Vector3d& reachableFootPosition,
						      iit::rbd::Vector3d& adjustedFootVelocity);

	iit::rbd::Matrix33d Rxyz(iit::rbd::Vector3d& angles);

	double getMaxHaaAngle(){return haa_max;}
	double getMinHaaAngle(){return haa_min;}
	double getMaxHfeAngleFrontLegs(){return hfe_max_bf;}
	double getMinHfeAngleFrontLegs(){return hfe_min_bf;}
	double getMaxHfeAngleRearLegs(){return hfe_max_br;}
	double getMinHfeAngleRearLegs(){return hfe_min_br;}
	double getMaxKfeAngleFrontLegs(){return kfe_max_bf;}
	double getMinKfeAngleFrontLegs(){return kfe_min_bf;}
	double getAdjustmentCoefficient(){return coefAdj;}


	void setMaxHaaAngle(const double);
	void setMinHaaAngle(const double);
	void setMaxHfeAngleFrontLegs(const double);
	void setMinHfeAngleFrontLegs(const double);
	void setMaxHfeAngleRearLegs(const double);
	void setMinHfeAngleRearLegs(const double);
	void setMaxKfeAngleFrontLegs(const double);
	void setMinKfeAngleFrontLegs(const double);
	void setAdjustmentCoefficient(const double);



private:
	double a1, a2;
	double r, gamma, gamma_max, gamma_min, gamma_prime;
	double haa_max, haa_min, hfe_max_bf, hfe_min_bf, hfe_max_br, hfe_min_br, kfe_max_bf, kfe_min_bf;
	double haa;
	double dx, dy, dz, d_haa_hfe;
	double coefAdj;
	double ratio;
	bool check;
	double r_max, r_min;
	iit::rbd::Vector3d positionOffset;
	iit::rbd::Vector3d memoryPosition;
	iit::rbd::Vector3d diff;
	iit::rbd::Vector3d deltaAngles;

};

#endif /* HYQ2MAXWORKSPACECHECK_H_ */
