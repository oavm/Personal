/*
 * CTerrainEstimator.cpp
 *
 *  Created on: Sep 20, 2013
 *      Author: victor
 */

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <terrain_estimator/CTerrainEstimator.h>
#include <math_utils/utils.h>

using namespace Eigen;
using namespace iit;
using namespace dog;

CTerrainEstimator::CTerrainEstimator() {

	ForceThreshold = 50;
	AlphaFilter = 1.0;
	ChangesFlag = false;
	WarningFlag = false;
	TerrainEstimationFlag = true;
	baseRoll = 0;
	basePitch = 0;
	maxRoll = 60*3.1415/180;
	maxPitch = 60*3.1415/180;
	LF_FootForce = 0.0;  RF_FootForce = 0.0;
	LH_FootForce = 0.0;  RH_FootForce = 0.0;
    Ag << 1.0, 0.0,
    	  1.0, 0.0,
    	  1.0, 0.0,
    	  1.0, 0.0,
    	  0.0, 0.5,
    	  0.0, 0.5,
    	  0.0, 0.5,
    	  0.0, 0.5;
    Bg.setZero();
    R.setIdentity();
    Relative_Angles.setZero();
	Terrain_Angles.setZero();
	Terrain_Angles_f1.setZero();
	Terrain_Angles_f2.setZero();

	LF_foot_pos << 0.5, 0.25, 0;
	RF_foot_pos << 0.5, -0.25, 0;
	LH_foot_pos << -0.5, 0.25, 0;
	RH_foot_pos << -0.5, -0.25, 0;
	LF_pos = LF_foot_pos;
	RF_pos = RF_foot_pos;
	LH_pos = LH_foot_pos;
	RH_pos = RH_foot_pos;

}

CTerrainEstimator::~CTerrainEstimator() {

}



void CTerrainEstimator::Enable(bool TurnOnEstimation) {

	TerrainEstimationFlag = TurnOnEstimation;
	if(TerrainEstimationFlag){
		std::cout << "Terrain Estimation ON!!!" << std::endl;
	}
	else{
		Terrain_Angles.setZero();
		std::cout << "Terrain Estimation OFF!!!" << std::endl;
	}

}


void CTerrainEstimator::Disable() {

	TerrainEstimationFlag = false;
	Terrain_Angles.setZero();
	std::cout << "Terrain Estimation OFF!!!" << std::endl;
}


void CTerrainEstimator::setLimits(double mRoll, double mPitch) {

	if(mRoll>0 && mPitch>0) {
	maxRoll = mRoll;
	maxPitch = mPitch;
	}
	else {
	std::cout << "Values must be greater than zero!!!" << std::endl;
	}
}


void CTerrainEstimator::setFilter(double TaskSampleTime, double FilterTime) {

	if (TaskSampleTime>0 && FilterTime>0) {
	AlphaFilter=TaskSampleTime/(FilterTime+TaskSampleTime);
	}
	else {
	std::cout << "SetFilter Message:" << std::endl << "Values must be greater than zero!!!" << std::endl;
	}
}

void CTerrainEstimator::resetEstimator(void) {

    Relative_Angles.setZero();
    Terrain_Angles.setZero();
    Terrain_Angles_f1.setZero();
    Terrain_Angles_f2.setZero();
}



void CTerrainEstimator::ComputeTerrainEstimation(double& EstimatedRoll, double& EstimatedPitch) {

	if(TerrainEstimationFlag) {

		checkStance();


		//Compute estimation
		if(ChangesFlag==true) {

			R(0,0) = cos(basePitch);		R(0,1) = sin(basePitch)*sin(baseRoll);		R(0,2) = cos(baseRoll) * sin(basePitch);
			R(1,0) = 0.0;					R(1,1) = cos(baseRoll);						R(1,2) = -sin(baseRoll);
			R(2,0) = -sin(basePitch);		R(2,1) = cos(basePitch) * sin(baseRoll);	R(2,2) = cos(basePitch) * cos(baseRoll);

			LF_pos_HF = R*LF_pos; RF_pos_HF = R*RF_pos; LH_pos_HF = R*LH_pos; RH_pos_HF = R*RH_pos;

			Ag(0,0) = LF_pos_HF(1)-RF_pos_HF(1);  Ag(4,1) = LF_pos_HF(0)-RH_pos_HF(0);
			Ag(1,0) = LF_pos_HF(1)-RH_pos_HF(1);  Ag(5,1) = LF_pos_HF(0)-LH_pos_HF(0);
			Ag(2,0) = LH_pos_HF(1)-RF_pos_HF(1);  Ag(6,1) = RF_pos_HF(0)-RH_pos_HF(0);
			Ag(3,0) = LH_pos_HF(1)-RH_pos_HF(1);  Ag(7,1) = RF_pos_HF(0)-LH_pos_HF(0);
			Bg(0) = LF_pos_HF(2)-RF_pos_HF(2);	Bg(4) = -(LF_pos_HF(2)-RH_pos_HF(2));
			Bg(1) = LF_pos_HF(2)-RH_pos_HF(2);	Bg(5) = -(LF_pos_HF(2)-LH_pos_HF(2));
			Bg(2) = LH_pos_HF(2)-RF_pos_HF(2);	Bg(6) = -(RF_pos_HF(2)-RH_pos_HF(2));
			Bg(3) = LH_pos_HF(2)-RH_pos_HF(2);	Bg(7) = -(RF_pos_HF(2)-LH_pos_HF(2));
			//std::cout << atan(Bg(0)/Ag(0,0)) << std::endl;
			//std::cout << atan(Bg(1)/Ag(1,0)) << std::endl;
			//std::cout << atan(Bg(2)/Ag(2,0)) << std::endl;
			//std::cout << atan(Bg(3)/Ag(3,0)) << std::endl;
			//std::cout << atan(Bg(4)/Ag(4,1)) << std::endl;
			//std::cout << atan(Bg(5)/Ag(5,1)) << std::endl;
			//std::cout << atan(Bg(6)/Ag(6,1)) << std::endl;
			//std::cout << atan(Bg(7)/Ag(7,1)) << std::endl << std::endl;

			//Compute terrain inclination according to the robot base
			if((Ag.transpose()*Ag).determinant()!=0.0) {
				Relative_Angles = (Ag.transpose()*Ag).inverse()*Ag.transpose()*Bg;
				WarningFlag=false;
			}
			else {
				if(WarningFlag==false){
					std::cout << "Singular Matrix!!!" << std::endl;
					WarningFlag=true;
				}
			}

			Relative_Angles(0) = atan(Relative_Angles(0));
			Relative_Angles(1) = atan(Relative_Angles(1));

			//Compute terrain inclination according to the horizontal frame
			Terrain_Angles(0) = Relative_Angles(0);//+baseRoll;
			Terrain_Angles(1) = Relative_Angles(1);//+basePitch;
			ChangesFlag=false;
		}

	}


	//Compute output filtering
	Terrain_Angles_f1 = (1-AlphaFilter)*Terrain_Angles_f1 + AlphaFilter*Terrain_Angles;
	Terrain_Angles_f2 = (1-AlphaFilter)*Terrain_Angles_f2 + AlphaFilter*Terrain_Angles_f1;

	//Check output limits
	if(Terrain_Angles_f2(0)>-maxRoll && Terrain_Angles_f2(0)<maxRoll && Terrain_Angles_f2(1)>-maxPitch && Terrain_Angles_f2(1)<maxPitch) {
	EstimatedRoll = Terrain_Angles_f2(0);
	EstimatedPitch = Terrain_Angles_f2(1);
	}


}

void CTerrainEstimator::ComputeTerrainEstimationRoughTerrain(const Eigen::Vector3d & old_terrain_normal, double& EstimatedRoll, double& EstimatedPitch)
{

	if(TerrainEstimationFlag) {

		checkStance();
		//Compute estimation
		if(ChangesFlag==true) {

			R(0,0) = cos(basePitch);		R(0,1) = sin(basePitch)*sin(baseRoll);		R(0,2) = cos(baseRoll) * sin(basePitch);
			R(1,0) = 0.0;					R(1,1) = cos(baseRoll);						R(1,2) = -sin(baseRoll);
			R(2,0) = -sin(basePitch);		R(2,1) = cos(basePitch) * sin(baseRoll);	R(2,2) = cos(basePitch) * cos(baseRoll);

			dog::LegDataMap<Vector3d> feetHF;
			double LS_error = 0.0, d_param;
			bool planeEstOk;
			Vector3d plane_normal;

			feetHF[dog::LF] = R*LF_pos; feetHF[dog::RF] = R*RF_pos; feetHF[dog::LH] = R*LH_pos; feetHF[dog::RH] = R*RH_pos;
			//use this for the LS error computation
			planeEstOk = planeEstimationLS(feetHF, LS_error, plane_normal, d_param);
			//use this for the normal
			planeEstOk = planeEstimationEigenVector(feetHF, plane_normal);
			if (planeEstOk)
			{
				if (LS_error > 0.002){ //start correction
					//prt("correcting")
					double sensitivity = (40000)/0.008*LS_error;
					correctPlaneWithHeuristics(feetHF, sensitivity, old_terrain_normal, plane_normal);
				}

				//normal = (Rx(roll)*Ry(pith))*[0;0;1];
				//normal = [ cos(roll)*sin(pitch), -sin(roll), cos(roll)*cos(pitch)]
				//update roll/pitch only if it is meaningful compute roll/pitch from normal
				Relative_Angles(1) = atan(plane_normal(0)/plane_normal(2));//pitch
				Relative_Angles(0) = atan(-plane_normal(1)*sin(Relative_Angles(1))/ (plane_normal(0)) ); //roll
			}
			//prt(LS_error)
			//prt(plane_normal.transpose())
			//Compute terrain inclination according to the horizontal frame
			Terrain_Angles(0) = Relative_Angles(0);//+baseRoll;
			Terrain_Angles(1) = Relative_Angles(1);//+basePitch;
			ChangesFlag=false;
		}


	}
	//Compute output filtering
	Terrain_Angles_f1 = (1-AlphaFilter)*Terrain_Angles_f1 + AlphaFilter*Terrain_Angles;
	Terrain_Angles_f2 = (1-AlphaFilter)*Terrain_Angles_f2 + AlphaFilter*Terrain_Angles_f1;

	//Check output limits
	if(Terrain_Angles_f2(0)>-maxRoll && Terrain_Angles_f2(0)<maxRoll && Terrain_Angles_f2(1)>-maxPitch && Terrain_Angles_f2(1)<maxPitch) {
		//to do one shoot
		//EstimatedRoll = Terrain_Angles(0);
		//EstimatedPitch = Terrain_Angles(1);
		EstimatedRoll = Terrain_Angles_f2(0);
		EstimatedPitch = Terrain_Angles_f2(1);
	}

}
void CTerrainEstimator::checkStance()
{
	//Updating check
		if(ForceThreshold>=0) {
			if (shinCollisionSet)
			{
				if((LF_FootForce>ForceThreshold) && (!shinCollision[dog::LF])) {
					LF_pos = LF_foot_pos;
					ChangesFlag=true;
				}

				if((RF_FootForce>ForceThreshold) && (!shinCollision[dog::RF]))  {
					RF_pos = RF_foot_pos;
					ChangesFlag=true;
				}

				if((LH_FootForce>ForceThreshold) && (!shinCollision[dog::LH])) 	{
					LH_pos = LH_foot_pos;
					ChangesFlag=true;
				}

				if((RH_FootForce>ForceThreshold) && (!shinCollision[dog::RH]))  {
					RH_pos = RH_foot_pos;
					ChangesFlag=true;
				}
				shinCollisionSet = false;
			} else {
				if(LF_FootForce>ForceThreshold) {
					LF_pos = LF_foot_pos;
					ChangesFlag=true;
				}

				if(RF_FootForce>ForceThreshold) {
					RF_pos = RF_foot_pos;
					ChangesFlag=true;
				}

				if(LH_FootForce>ForceThreshold)	{
					LH_pos = LH_foot_pos;
					ChangesFlag=true;
				}

				if(RH_FootForce>ForceThreshold) {
					RH_pos = RH_foot_pos;
					ChangesFlag=true;
				}
			}
		}
		else {
			if(LF_FootForce<ForceThreshold) {
				LF_pos = LF_foot_pos;
				ChangesFlag=true;
			}

			if(RF_FootForce<ForceThreshold) {
				RF_pos = RF_foot_pos;
				ChangesFlag=true;
			}

			if(LH_FootForce<ForceThreshold)	{
				LH_pos = LH_foot_pos;
				ChangesFlag=true;
			}

			if(RH_FootForce<ForceThreshold) {
				RH_pos = RH_foot_pos;
				ChangesFlag=true;
			}
		}
}

bool CTerrainEstimator::planeEstimationLS(const iit::dog::LegDataMap<Eigen::Vector3d> & feet, double & LSerror, Vector3d & plane_normal, double & d_param)
{

	bool planeEst = false;
	//fitting planes LS on Z (does not work for vertical planes)
	//find the plane ax + by + d = -z c=1
	Matrix <double, 4,3> Ag;
	Matrix <double, 4,1>  Bg;
	Vector3d solution;
	Ag << feet[dog::LF](rbd::X), feet[dog::LF](rbd::Y), 1,
			feet[dog::RF](rbd::X), feet[dog::RF](rbd::Y), 1,
			feet[dog::LH](rbd::X), feet[dog::LH](rbd::Y), 1,
			feet[dog::RH](rbd::X), feet[dog::RH](rbd::Y), 1;
	Bg(0) =  -feet[dog::LF](rbd::Z);
	Bg(1) =  -feet[dog::RF](rbd::Z);
	Bg(2) =  -feet[dog::LH](rbd::Z);
	Bg(3) =  -feet[dog::RH](rbd::Z);

	if((Ag.transpose()*Ag).determinant()!=0.0) {
		solution = (Ag.transpose()*Ag).inverse()*Ag.transpose()*Bg;
		planeEst = true;
	}
	else {

		std::cout << "Singular Matrix!!!" << std::endl;
	}

	//we fixed the c director to be 1  x =[a b d]
	plane_normal << solution(0), solution(1),  1;
	plane_normal.normalize(); // to plot the plane you should not normalize
	d_param = solution(2);
	//compute LS error
	LSerror = (Ag*solution - Bg).transpose()*(Ag*solution - Bg);

	return planeEst;
}


bool CTerrainEstimator::planeEstimationEigenVector(const iit::dog::LegDataMap<Eigen::Vector3d> & feet, Vector3d & plane_normal)
{
	//from affine_fit.m by  by Adrien Leygue
	bool planeEst = false;
	Vector3d pointPlane;pointPlane.setZero();
	for (int i = 0; i<dog::_LEGS_COUNT; i++){
		pointPlane+=feet[dog::LegID(i)];
	}
    //the mean of the samples belongs to the plane
	pointPlane /=4;
    //The samples are reduced:
	Matrix <double, 4,3> R;
	R << 	(feet[dog::LF] - pointPlane).transpose(),
			(feet[dog::RF] - pointPlane).transpose(),
			(feet[dog::LH] - pointPlane).transpose(),
			(feet[dog::RH] - pointPlane).transpose();
	Matrix3d RR;
	RR = R.transpose()*R;
	FullPivLU<Matrix3d> lu_decomp(RR);
	auto rank = lu_decomp.rank();
	if (rank > 1)
	{
		SelfAdjointEigenSolver<Matrix3d> eigensolver(RR);
		//		std::cout << "The eigenvalues of A are:\n" << eigensolver.eigenvalues() << std::endl;
		//		std::cout << "Here's a matrix whose columns are eigenvectors of A \n"
		//				<< "corresponding to these eigenvalues:\n"
		//				<< eigensolver.eigenvectors() << std::endl;
		//NB The eigenvectors are normalized to have (Euclidean) norm equal to one.
		plane_normal =  eigensolver.eigenvectors().col(0);
		planeEst = true;
	}
	else
	{
		prt("you have 3 feet aligned, rank is 1, I cannot fit a plane!")
	}
	return planeEst;
}

void CTerrainEstimator::correctPlaneWithHeuristics(const iit::dog::LegDataMap<Eigen::Vector3d> & feet,const double sensitivity, const Eigen::Vector3d & old_terrain_normal,  Vector3d &plane_normal)
{

	std::vector<Vector3d> normals;
	std::vector<double> weights;
	double sumOfWeights = 0.0;
	Vector3d normal_average;
	//how many  triangles ? (4 combinations for set of 3 elements out of 4)
	Matrix<double, 4,3>  combinations;
	combinations  <<LF,  RF  ,  LH,
			LF ,  RF  ,  RH ,
			LF ,  LH  ,  RH ,
			RF ,  LH  ,  RH ;
	//compute the weight for each trangle according on how close is to gravity
	for (int i=0; i<4;i++){
		//compute the normal of each triangle
		normals.push_back(computeTriangleNormal(feet, combinations.row(i)));
		double arg = normals[i].dot(old_terrain_normal);
		//compute weights according on how each normal is "far away" from gravity
		weights.push_back(1/(1+sensitivity*pow(arg-1,2)));
		sumOfWeights +=weights[i];
	}
	//compute average orientation with geodesics
	normal_average = normals[0];
	for (int i=1; i<normals.size(); i++)
	{
		//update recursively average directions
		//1 - get the angle bw the vector  des_normal and the avg
		double angle = acos(normal_average.dot(normals[i]));

		//get the angle on the geodesic weighted towards the avg and update number of samples
		angle *= std::accumulate(weights.begin(), (weights.begin() + i), 0.0)/std::accumulate(weights.begin(), (weights.begin() + i+1), 0.0); //this is the weight of the average
		//get the axis to rotate des_normal towards des_normal_avg
		Eigen::Vector3d axis = (normals[i].cross(normal_average)).normalized(); //this will be in base frame!
		//rotate v towards avg to get the new avg
		normal_average = commons::rotVecToRotMat(angle, axis).transpose()*normals[i];
	}
	plane_normal = normal_average;

}

Vector3d CTerrainEstimator::computeTriangleNormal(const iit::dog::LegDataMap<Eigen::Vector3d> & feet, const Vector3d & index)
{

	Vector3d l1 = feet[index(1)] - feet[index(0)];
	Vector3d l2 = feet[index(2)] - feet[index(1)];
	Vector3d normal = l1.cross(l2); normal.normalize(); //fundamental, you need to normalize to compute the projection on gravity
	if (normal(rbd::Z)<0)
	{
		normal = -1*normal;
	}
	return normal;
}



