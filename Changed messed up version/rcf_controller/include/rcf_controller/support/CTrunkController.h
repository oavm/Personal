/*
 * CTrunkController.h
 *
 *  Created on: Nov 5, 2013
 *      Author: Victor Barasuol
 */

#ifndef CTRUNKCONTROLLER_H_
#define CTRUNKCONTROLLER_H_

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <iit/rbd/rbd.h>
#include <iit/commons/dog/leg_data_map.h>
#include <iit/commons/dog/joint_id_declarations.h>

using namespace std;
using namespace iit;

typedef Eigen::Matrix<double, 6, 3> legJac;

class CTrunkController {

public:
	CTrunkController();
	virtual ~CTrunkController();
	void setActualFeetPos(const Eigen::Matrix<double, 3, 1>& actualFootPosLF,
	  	  	  	   	   	  const Eigen::Matrix<double, 3, 1>& actualFootPosRF,
	  	  	  	   	   	  const Eigen::Matrix<double, 3, 1>& actualFootPosLH,
	  	  	  	   	   	  const Eigen::Matrix<double, 3, 1>& actualFootPosRH);
	void setActualFeetPos(const dog::LegDataMap<rbd::Vector3d>& actualFootPosition);
	void setFeetJacobians(const legJac& JFootLF, const legJac& JFootRF,
			                 const legJac& JFootLH, const legJac& JFootRH);
	void setFeetJacobians(const dog::LegDataMap<legJac>& allJ_Leg);
	void setTrunkAttitude(const double& tRoll, const double& tPitch);
	void setMinStanceLegs(const double&  minNumberOfStanceLegs);
	void setStanceLegs(const dog::LegDataMap<bool>& stance);
	void setTrunkWrench(const Eigen::Matrix<double, 6,1>& trunkW);
	void setTrunkControllerOption(const int& controllerOption);
	void computeOutputTorques();


private:
	double trunkPitch, trunkRoll;
	dog::LegDataMap<rbd::Vector3d> actualFootPos;
	dog::LegDataMap<rbd::Matrix33d> J_Leg;
	dog::LegDataMap<bool> stanceLegs;
	enum legID {LF=0, RF, LH, RH};

public:
	Eigen::Matrix<double, 6,1> trunkWrench;
	Eigen::Matrix<double, 12,1> jointTorques;
	double minNumberStanceLegs;
	int trunkControllerOption;

};

#endif /* CTRUNKCONTROLLER_H_ */
