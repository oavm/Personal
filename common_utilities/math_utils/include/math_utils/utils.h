/*
 * utils.h
 *
 *  Created on: Sep 20, 2013
 *      Author: mfocchi
 */

#ifndef COMMON_MATH_UTILS_H_
#define COMMON_MATH_UTILS_H_


#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <iostream>
#include <algorithm>    // std::max
#include <math.h>
#include <string>


#define prt(x) std::cout << #x " = \n" << x << "\n" << std::endl;
#define prt_vec(x) for( int i = 0; i < x.size(); i++) {std::cout << x[i] << " \n";};

namespace iit {
namespace commons {

typedef Eigen::Quaterniond Quaternion;
typedef Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > MatrixXR;

inline bool isnan(const Eigen::Vector3d & vector)
{
	if (std::isnan(vector(0)) || std::isnan(vector(1)) || std::isnan(vector(2)))
		return true;
	else
		return false;
}

template <typename Scalar>
struct CwiseAlmostZeroOp {
	CwiseAlmostZeroOp(const Scalar& threshold) : thresh(threshold) {}
	const Scalar operator()(const Scalar& x) const {
		return std::abs(x) < thresh ? 0 : x;
	}
private:
	Scalar thresh;
};


//Compare dense vectors (matrices) using both relative and absolute tolerances. Equivalent of all close in python
//If the following equation is element-wise True, then allclose returns True.   absolute(a - b) <= (atol + rtol * absolute(b))
//Returns True if two arrays are element-wise equal within a tolerance.
//The tolerance values are positive, typically very small numbers.
//The relative difference (rtol * abs(b)) and the absolute difference atol are added together to compare against the absolute difference between a and b.
template<typename DerivedA, typename DerivedB>
bool allclose(const Eigen::DenseBase<DerivedA>& a,
              const Eigen::DenseBase<DerivedB>& b,
              const typename DerivedA::RealScalar& rtol = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
              const typename DerivedA::RealScalar& atol = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
  return ((a.derived() - b.derived()).array().abs()
          <= (atol + rtol * b.derived().array().abs())).all();
}




// SVD method

/**
 * WARNING: the second parameter is not really const. It gets modified (it contains the result)
 *          see http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
 */
template <typename MxIN, typename MxOut> inline
void psdInv(const Eigen::MatrixBase<MxIN>& a, const Eigen::MatrixBase<MxOut>& result) {

  double epsilon = Eigen::NumTraits<double>::epsilon();
  Eigen::JacobiSVD<MatrixXR> svd;

  svd.compute(a, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // As done in Matlab:
  // http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse t = ε•max(m,n)•max(Σ)
  double tolerance = epsilon; //std::max(a.cols(), (int)a.rows()) * svd.singularValues().cwiseAbs().maxCoeff(); TODO

  // Build a diagonal matrix with the Inverted Singular values
  // The pseudo inverted singular matrix is easy to compute :
  // is formed by replacing every nonzero entry by its reciprocal (inversing).
  Eigen::Matrix<double, Eigen::Dynamic, 1> singularValuesVector (svd.matrixV().cols(),1); // size of E has same size as columns of V

  singularValuesVector = (svd.singularValues().array()>tolerance).select(svd.singularValues().array().inverse(), 0);

  //select only the first r rows (not used)
  //  svdA.matrixU().transpose() = svdA.matrixU().transpose().block(0,0,singularValuesVector.rows(),svd.matrixU().transpose().cols())

//  prt(svd.singularValues())
//  prt(svd.matrixU())
//  prt(svd.matrixV())

  // Pseudo-Inversion : V * S * U'
  const_cast< Eigen::MatrixBase<MxOut>& >(result) = svd.matrixV() *  singularValuesVector.asDiagonal() * svd.matrixU().transpose();
  //prt(result)

}

template <typename MxIN, typename MxOut> inline
void psdInv(const Eigen::MatrixBase<MxIN>& a, const Eigen::MatrixBase<MxOut>& result, double tolerance) {

  double epsilon = Eigen::NumTraits<double>::epsilon();
  Eigen::JacobiSVD<MatrixXR> svd;

  svd.compute(a, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Build a diagonal matrix with the Inverted Singular values
  // The pseudo inverted singular matrix is easy to compute :
  // is formed by replacing every nonzero entry by its reciprocal (inversing).
  Eigen::Matrix<double, Eigen::Dynamic, 1> singularValuesVector (svd.matrixV().cols(),1); // size of E has same size as columns of V

  singularValuesVector = (svd.singularValues().array()>tolerance).select(svd.singularValues().array().inverse(), 0);

  //select only the first r rows (not used)
  //  svdA.matrixU().transpose() = svdA.matrixU().transpose().block(0,0,singularValuesVector.rows(),svd.matrixU().transpose().cols())

//  prt(svd.singularValues())
//  prt(svd.matrixU())
//  prt(svd.matrixV())

  // Pseudo-Inversion : V * S * U'
  const_cast< Eigen::MatrixBase<MxOut>& >(result) = svd.matrixV() *  singularValuesVector.asDiagonal() * svd.matrixU().transpose();
  //prt(result)

}

inline MatrixXR  psdInv(const MatrixXR & in, double tolerance) {

  Eigen::JacobiSVD<MatrixXR> svd;
  Eigen::MatrixXd out(in.cols(),in.rows());

  svd.compute(in, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // As done in Matlab:
  // http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse t = ε•max(m,n)•max(Σ)
  //double tolerance = epsilon; //std::max(a.cols(), (int)a.rows()) * svd.singularValues().cwiseAbs().maxCoeff(); TODO

  // Build a diagonal matrix with the Inverted Singular values
  // The pseudo inverted singular matrix is easy to compute :
  // is formed by replacing every nonzero entry by its reciprocal (inversing).
  Eigen::Matrix<double, Eigen::Dynamic, 1> singularValuesVector (svd.matrixV().cols(),1); // size of E has same size as columns of V

  singularValuesVector = (svd.singularValues().array()>tolerance).select(svd.singularValues().array().inverse(), 0);

  //select only the first r rows (not used)
  //  svdA.matrixU().transpose() = svdA.matrixU().transpose().block(0,0,singularValuesVector.rows(),svd.matrixU().transpose().cols())

//  prt(svd.singularValues())
//  prt(svd.matrixU())
//  prt(svd.matrixV())

  // Pseudo-Inversion : V * S * U'
  out =  (svd.matrixV() *  singularValuesVector.asDiagonal() * svd.matrixU().transpose());
  return (out);
  //prt(result)
}

inline MatrixXR psdInvW(const MatrixXR & in, const MatrixXR & W, double tolerance) {
MatrixXR out(in.cols(),in.rows());
if (W.rows()!= in.rows())
	prt("weighting matrix has the wrong size!!")

	if (in.cols()>=in.rows())
	{		//fat  A#w = WA^T (A W A^T)^{#} this works also for rank deficient matrix
		out = W*in.transpose()*psdInv(in*W*in.transpose(), tolerance);
	}else{//skinny A#w =  (A^T W A)^{#}A^T*W
		out = psdInv(in.transpose()*W*in, tolerance)*in.transpose()*W;
	}
return out;
}

//return the full QR decomposition for the matrix Q and just the reduced one for the matrix R
inline void computeQR(const MatrixXR & in,  MatrixXR & Q, MatrixXR & R, double tolerance) {
//compute the QR decomposition of the Jacobian
//Q = U ; R = EPS*V^T
Eigen::JacobiSVD<MatrixXR> svd;

svd.compute(in, Eigen::ComputeFullU | Eigen::ComputeFullV);

Q.resize(in.rows(),in.rows());
Q = svd.matrixU();

R.resize(in.cols(),in.cols());

MatrixXR S(in.cols(),in.cols());
S= svd.singularValues().segment(0,in.cols()).asDiagonal();

R = S * svd.matrixV().transpose();

}


//////////////////////////////
/* Function to compute the rotation matrix which express a vector of the fixed frame A into the rotated frame B
 * according to the ZYX convention (subsequent rotation)
 * considering right hand coordinate systems (counter clockwise convention)
 */
//[                            cos(psi)*cos(theta),                              cos(theta)*sin(psi),         -sin(theta)]
//[ cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(phi)]
//[ sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), cos(phi)*cos(theta)]
//the transpose of this matrix has as director cosines (columns) the axis of the rotated frame expressed in the fixed frame which will be multiplied
//for the component of the vector in the rotated frame B to get the components in the fixed frame A

Eigen::Matrix3d inline rpyToRot(const Eigen::Vector3d & rpy){

Eigen::Matrix3d Rx, Ry, Rz;
double roll, pitch, yaw;

roll = rpy(0);
pitch = rpy(1);
yaw = rpy(2);

Rx <<	   1   ,    0     	  ,  	  0,
           0   ,    cos(roll) ,  sin(roll),
           0   ,    -sin(roll),  cos(roll);


Ry << cos(pitch) 	,	 0  ,   -sin(pitch),
            0       ,    1  ,   0,
      sin(pitch) 	,	0   ,  cos(pitch);

Rz << cos(yaw)  ,  sin(yaw) ,		0,
      -sin(yaw) ,  cos(yaw) ,  		0,
         0      ,     0     ,       1;


return Rx*Ry*Rz;

}

//according to ZYX convention
Eigen::Vector3d inline rotTorpy(Eigen::Matrix3d R)
{
	Eigen::Vector3d rpy;
	rpy(0) = atan2f((float) R(1,2), (float) R(2,2));
	rpy(1) = -asinf((float) R(0,2));
	rpy(2) = atan2f((float) R(0,1), (float) R(0,0));

	return rpy;
}

//////////////////////////////
/* Function to compute the linear tranformation matrix between euler rates (in ZYX convention) and omega vector
 * where omega is expressed in world coordinates*/
//to get the component expressed in the world ortogonal frame I need to
//multiply  the components of the vector of euler rate which is expressed
//the rpy (non orthogonal) by the roll pitch yaw axis expressed in the world frame
//I need to express the yaw/pitch/roll  axis in world frame, since we do
//first the rotation in the z axis, wz = yaw_d therefore z = z' =[0;0;1]
// then we rotate about pitch so we have component for this rotation in wy
// and -wx (y'= [-sin(yaw; cos(yaw) ;0])
//if we consider roll after the pitch we will have the roll axis after yaw and pitch rotation to be x'' = cos(pitch)*x'
//-sin(pitch)*[0;0;1] where x' = [cos(yaw);sin(yaw);0]

Eigen::Matrix3d inline rpyToEarInv(const Eigen::Vector3d & rpy){

Eigen::Matrix3d EarInv;
double roll = rpy(0);
double pitch = rpy(1);
double yaw = rpy(2);

EarInv<< cos(pitch)*cos(yaw), -sin(yaw), 0,
		 cos(pitch)*sin(yaw),   cos(yaw),    0,
	    	      -sin(pitch),         0,    1;

return EarInv;
}

//////////////////////////////
/* Function to compute the linear tranformation matrix between euler rates (in ZYX convention) and omega vector
 * where omega is expressed in base coordinates (is R*EarInv) */


Eigen::Matrix3d inline  rpyToEar(const Eigen::Vector3d & rpy){

Eigen::Matrix3d Ear;
double roll = rpy(0);
double pitch = rpy(1);
double yaw = rpy(2);

Ear<< 1,         0,         -sin(pitch),
	  0,  cos(roll), cos(pitch)*sin(roll),
	  0, -sin(roll), cos(pitch)*cos(roll);


return Ear;
}

//dervative of rpyToEarInv
Eigen::Matrix3d inline rpyToEarInv_dot(const Eigen::Vector3d & rpy, const Eigen::Vector3d & rpyd){

Eigen::Matrix3d EarInv_dot;
double roll = rpy(0);
double pitch = rpy(1);
double yaw = rpy(2);
double rolld = rpyd(0);
double pitchd = rpyd(1);
double yawd = rpyd(2);

EarInv_dot << - cos(yaw)*sin(pitch)*pitchd - cos(pitch)*sin(yaw)*yawd, - cos(yaw)*yawd, 0,
			    cos(yaw)*cos(pitch)*yawd - sin(yaw)*sin(pitch)*pitchd, -sin(yaw)*yawd, 0,
               -cos(pitch)*pitchd,  0, 0;

return EarInv_dot;
}



//find the rotation matrix that rotates vec1 into vec2 so that vec2= R* vec1
//NB the matrix orientation of the rotated frame is the transposed of the rotation matrix which
//rotates the vector in the fixed frame to align with the rotated frame

Eigen::Matrix3d inline twovecToRot(const Eigen::Vector3d & vec1, const Eigen::Vector3d & vec2){


	Eigen::Vector3d axis;
	Eigen::Matrix3d base;
	Eigen::Matrix3d coordChangeMat;

	//check if there is a singulatiry in the cross product
	if (vec1 == vec2) {

		return Eigen::Matrix3d::Identity();
	}
	else {
		//do cross product to find the axis
		axis=vec1.cross(vec2);
		axis.normalize(); //it is fundamental to normalize it

		//build the new basis formed by the 3 vectors

		base.col(0) = vec1.normalized();
		base.col(1) = vec2.normalized();
		base.col(2) = axis;

	    //rotation is like a change of coord between v1 and v2 leaving the axis unchanged

		coordChangeMat << 0, 1, 0,
				1, 0, 0,
				0, 0, 1;
		//the rotation matrix is in the original space

		return base * coordChangeMat * base.inverse();
	}
}

Quaternion inline rpyToquat(Eigen::Vector3d rpy) {
//using ZYX convention for rpy
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  Quaternion q;
  double roll, pitch, yaw;

  roll = rpy(0);
  pitch = rpy(1);
  yaw = rpy(2);

  q.w() = cos(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0)+sin(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
  q.x() = sin(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0)-cos(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
  q.y() = cos(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0)+sin(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0);
  q.z() = cos(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0)-sin(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0);


  return q;
}


Eigen::Matrix3d inline  skew_sim(const Eigen::Vector3d& w) {

Eigen::Matrix3d S;
	if (w.size() != 3){
		std::cout<<"SCREWS:skew,vector must be 3x1"<<std::endl;
}

S.setZero();


S(0,1) = -w(2);
S(0,2) =  w(1);
S(1,2) = -w(0);

S(1,0) =  w(2);
S(2,0) = -w(1);
S(2,1) =  w(0);

return S;

}

Eigen::Vector3d inline  skew_simToVec(const Eigen::Matrix3d& R) {

Eigen::Vector3d v;
	if (R.rows() != 3){
		std::cout<<"SCREWS:skew,matrix must be 3x3"<<std::endl;
}
v.setZero();
v(0) = 0.5 *  (R(2,1) - R(1,2));
v(1) = 0.5 *  (R(0,2) - R(2,0));
v(2) = 0.5 *  (R(1,0) - R(0,1));

return v;
}



Eigen::Vector4d inline castQuatToVector (Quaternion q){
	Eigen::Vector4d q_v;
	q_v(0) = q.w();
	q_v.segment(1,3) = q.vec();
	return q_v;
}

/*!*****************************************************************************
 *******************************************************************************
\note  quatToAngularVelocity
\date  July 2005
\remarks

 computes the angular velocity from quaternian position and derivatives (in body coordinates)

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output


 ******************************************************************************/
Eigen::Vector3d inline quatToOmega(Quaternion q, Quaternion qd)
{

	Eigen::Matrix<double,3,4> W;

	W(0,0) = -q.x();
	W(0,1) = q.w();
	W(0,2) = q.z();
	W(0,3) = -q.y();

	W(1,0) = -q.y();
	W(1,1) = -q.z();
	W(1,2) = q.w();
	W(1,3) = q.x();

	W(2,0) = -q.z();
	W(2,1) = q.y();
	W(2,2) = -q.x();
	W(2,3) = q.w();


  return 2.0*W*castQuatToVector(qd);

}



/*!*****************************************************************************
 *******************************************************************************
\note  quatToRotMat
\date  July 2005
\remarks

 converts a quaternion into a rotation matrix, where the rotation matrix
 is the transformation from global to local coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     q   : structure containing the quaternion
 \param[out]    R   : 3 by 3 rotation matrix

//uses formula from atttitude document [1 0^T; 0 R ]= Qr(q)^t*Ql(q)
 *
 ******************************************************************************/
inline Eigen::Matrix3d  quatToRotMat(const Quaternion & q)
{
//TODO check which is the fastest implementation
//	Eigen::Vector3d v = q.vec();
//	return  (q.w()*q.w() - v.transpose()*v)*Eigen::Matrix3d::Identity()  + 2*v*v.transpose()  - 2*q.w()*skew_sim(v);


	Eigen::Matrix3d R;
	R(0,0) = -1.0 + 2.0*(q.w()*q.w()) + 2.0*(q.x()*q.x());
	R(1,1) = -1.0 + 2.0*(q.w()*q.w()) + 2.0*(q.y()*q.y());
	R(2,2) = -1.0 + 2.0*(q.w()*q.w()) + 2.0*(q.z()*q.z());
	R(0,1) = 2.0 * (q.x()*q.y() + q.w()*q.z());
	R(0,2) = 2.0 * (q.x()*q.z() - q.w()*q.y());
	R(1,0) = 2.0 * (q.x()*q.y() - q.w()*q.z());
	R(1,2) = 2.0 * (q.y()*q.z() + q.w()*q.x());
	R(2,0) = 2.0 * (q.x()*q.z() + q.w()*q.y());
	R(2,1) = 2.0 * (q.y()*q.z() - q.w()*q.x());

  return R;
}



/*! Limits the norm of a rotation vector to pi
 * @return 	rescaled rotation vector
 * @param[in] 	v	rotation vector
 */
inline Eigen::Vector3d rangePi(const Eigen::Vector3d& v){
	const double a = v.norm();
	if (a<=M_PI){
		return v;
	} else {
		const double a2 = -2.0*M_PI*floor((a+M_PI)/(2*M_PI))+a;
		return v/a*a2;
	}
}



/*! Converts a quaternion to a rotation vector
 * @return 	corresponding rotation vector
 * @param[in]	q	quaternion
 */
inline Eigen::Vector3d quatToRotVec(const Quaternion & q){
	Eigen::Vector3d v;
	const double c = q.w();
	v = q.vec();
	const double s = v.norm();
	if(fabs(s) >= 1e-10){
		const double a = 2*atan2(s,c);
		return v*a/s;
	} else {
		return v*2;
	}
}

/*! Converts a quaternion to a rpy ZYX convention
 * @return 	corresponding rpy vector
 * @param[in]	q	quaternion
 */
inline Eigen::Vector3d quatToRPY(const Quaternion & q){
	Eigen::Vector3d rpy;
	rpy = rotTorpy(quatToRotMat(q));
	return rpy;
}

/*! Converts a rotation vector to a quaternion
 * @return 	corresponding quaternion
 * @param[in]	v	rotation vector
 */
inline Quaternion rotVecToQuat(const Eigen::Vector3d& v){
	Quaternion q;
	const double a = v.norm();
	q.w() = cos(a/2);
	if(a >= 1e-10){
		q.vec() = sin(a/2)/a*v;
	} else {
		q.vec() = v/2;
	}
	q.normalize();
	return q;
}

//compute the rotation matrix associated to the rot_vector.
//the rotation matrix maps a vector in a frame which is rotated about the rot_vector wrt the original frame
//if you want to rotate the vector in the original frame you should multiply by the transpose of this matrix
inline Eigen::Matrix3d rotVecToRotMat(const Eigen::Vector3d& v){
	Eigen::Matrix3d R;

	double angle = v.norm();
	if (fabs(angle) >= 1e-10) {
		Eigen::Vector3d axis = v.normalized();
		//using rodriguez formula Matrix3f R; R = AngleAxisf(0.33*M_PI, Vector3f::UnitZ());
		R = cos(angle) *Eigen::Matrix3d::Identity() + (1-cos(angle))*axis*axis.transpose() - sin(angle)*skew_sim(axis);
	} else {
		R.setIdentity();
	}
	return R;
}

inline Eigen::Matrix3d rotVecToRotMat(const double angle, Eigen::Vector3d  axis){

	if (fabs(angle) >= 1e-10)
	{
		axis.normalize();

		return rotVecToRotMat(angle*axis);
	}
	else
		return Eigen::Matrix3d::Identity();
}

inline Eigen::Vector3d rotMatToRotVec(const Eigen::Matrix3d & R)
{
//this formula depends on how you define R
Eigen::Vector3d axis;
Eigen::Matrix3d Rskew;
double angle;


//angle = acos( 0.5*( R(0,0)+ R(1,1)+ R(2,2)-1));
//axis(0) =  R(1,2)-R(2,1);
//axis(1) =  R(2,0) -R(0,2);
//axis(2) =  R(0,1) - R(1,0);
//axis *=0.5/sin(angle);
//
//return  angle*axis.normalized();

double c = 0.5*( R(0,0)+ R(1,1)+ R(2,2)-1);
Eigen::Vector3d w = - skew_simToVec(R);
double s = w.norm(); //w = sin(theta)*axis

if (fabs(s) <= 1e-10)
	return Eigen::Vector3d::Zero();

else {
	angle = atan2(s,c);
	axis = w/s;
	return angle*axis;}

////more robust implementation (Roy)
//double c = 0.5*( R(0,0)+ R(1,1)+ R(2,2)-1);
//Eigen::Vector3d w = - skew_simToVec(R);
//double s = w.norm();
//angle = atan2(s,c);
//
//if (s == 0)
//	return Eigen::Vector3d::Zero();
//
//else {
//	if  (angle< 0.9*M_PI){ //			% a somewhat arbitrary threshold
//		axis = w/s;
//		return angle*axis;}
//	else
//	{				//% extract v*v' component from R and
//		Rskew = R - c * Eigen::Matrix3d::Identity();// pick biggest column (best chance
//		Rskew = Rskew + Rskew.transpose();				// to get sign right)
//
//		if ((Rskew(0,0) >= Rskew(1,1)) && (Rskew(0,0) >= Rskew(2,2)))
//		{
//			if (w(0) >= 0)
//				axis = R.col(0);
//			else
//				axis = -R.col(0);
//		}
//
//		else
//		{
//			if (Rskew(1,1) >= Rskew(2,2))
//			{
//				if (w(1) >= 0)
//					axis = R.col(1);
//				else
//					axis = R.col(1);
//			}
//			else
//			{
//				if  (w(2) >= 0)
//					axis = R.col(2);
//				else
//					axis = -R.col(2);
//			}
//		}
//
//		return  angle*axis.normalized();
//	}
//
//
//}

}

/*! Computes the inverse of a quaternion
 * @return 	corresponding quaternion inverse
 * @param[in]	q	quaternion
 */
inline Quaternion quatInverse(const Quaternion& q){
	Quaternion q2;
	q2.vec() = -q.vec();
	q2.w() = q.w();
	return q2;
}


/*! Computes the left-hand multiplication matrix from a given quaternion
 * @return 	left-hand multiplication matrix q2 * q1 = Ql(q2)*q1 NB Q(q_inv) = Q(q)^T
 * @param[in]	q	quaternion
 */
inline Eigen::Matrix<double,4,4> quatL(const Quaternion & q){
	Eigen::Matrix<double,4,4> M;

	M.setIdentity();
	M*=q.w();

	M(1,0) = q.x();
	M(2,0) = q.y();
	M(3,0) = q.z();
	M(0,1) = -q.x();
	M(0,2) = -q.y();
	M(0,3) = -q.z();


	M(1,2) = q.z();
	M(1,3) = -q.y();
	M(2,1) = -q.z();
	M(2,3) = q.x();
	M(3,1) = q.y();
	M(3,2) = -q.x();
	return M;
}

/*! Computes the right-hand multiplication matrix from a given quaternion
 * @return 	right-hand multiplication matrix q2 * q1 = Qr(q1)*q2 (Qnot conjugate quaternion matrix)
 * @param[in]	q	quaternion
 */
inline Eigen::Matrix<double,4,4> quatR(const Quaternion& q){
	Eigen::Matrix<double,4,4> M;

	M.setIdentity();
	M*=q.w();

	M(1,0) = q.x();
	M(2,0) = q.y();
	M(3,0) = q.z();
	M(0,1) = -q.x();
	M(0,2) = -q.y();
	M(0,3) = -q.z();

	M(1,2) = -q.z();
	M(1,3) = q.y();
	M(2,1) = q.z();
	M(2,3) = -q.x();
	M(3,1) = -q.y();
	M(3,2) = q.x();
	return M;

}

inline Quaternion  quatMult(const Quaternion & q2, const Quaternion & q1)
{
//computes qout= q2xq1
Quaternion q_e;

//compute angular error qe = qd x q1_inv
q_e.w() = q1.w()*q2.w() - q1.vec().transpose()*q2.vec();
//compute rotation axis
q_e.vec() = q1.w()*q2.vec() + q2.w()*q1.vec() + q1.vec().cross(q2.vec());

return q_e;
}

//using formula from attitude document the input matrix should be a_R_w where a is the rotated frame
inline Quaternion  rotMatToQuat(const Eigen::Matrix3d & R) {
Quaternion quat;

////compute the trace of R
//double T = 1 + R(0,0) + R(1,1)  + R(2,2);
////If the trace of the matrix is greater than zero, then
////perform an "instant" calculation.
//
//if (T>1e-03) {
//	quat.w() = 0.5* sqrt(T);
//	quat.x() = 0.5*(R(1,2) - R(1,2))/sqrt(T);
//	quat.y() = 0.5*(R(2,0) - R(2,0))/sqrt(T);
//	quat.z() = 0.5*(R(0,1) - R(0,1))/sqrt(T);
//} else {
//	std::cout<<"cannot compute quaternion";
//}

double tr = R.trace();
double sqrt_tr;

if (tr > 1e-06)
{
   	sqrt_tr = sqrt(R.trace() + 1);
	quat.w() = 0.5*sqrt_tr;
	quat.x() = (R(1,2) - R(2,1))/(2.0*sqrt_tr);
	quat.y() = (R(2,0) - R(0,2))/(2.0*sqrt_tr);
	quat.z() = (R(0,1) - R(1,0))/(2.0*sqrt_tr);
}
else  if ((R(1,1) > R(0,0)) && (R(1,1) > R(2,2)))
			{
	// max value at R(1,1)
	sqrt_tr = sqrt(R(1,1) - R(0,0) - R(2,2) + 1.0 );

	quat.y() = 0.5*sqrt_tr;

	if ( sqrt_tr > 1e-06 ) sqrt_tr = 0.5/sqrt_tr;


	quat.w() = (R(2, 0) - R(0, 2))*sqrt_tr;
	quat.x() = (R(0, 1) + R(1, 0))*sqrt_tr;
	quat.z() = (R(1, 2) + R(2, 1))*sqrt_tr;

			}
else if (R(2,2) > R(0,0))
{
	// max value at R(2,2)
	sqrt_tr = sqrt(R(2,2) - R(0,0) - R(1,1) + 1.0 );

	quat.z() = 0.5*sqrt_tr;

	if ( sqrt_tr > 1e-06 ) sqrt_tr = 0.5/sqrt_tr;

	quat.w() = (R(0, 1) - R(1, 0))*sqrt_tr;
	quat.x() = (R(2, 0) + R(0, 2))*sqrt_tr;
	quat.y() = (R(1, 2) + R(2, 1))*sqrt_tr;
}
else {
	// max value at dcm(0,0)
	sqrt_tr = sqrt(R(0,0) - R(1,1) - R(2,2) + 1.0 );

	quat.x() = 0.5*sqrt_tr;

	if ( sqrt_tr > 1e-06 ) sqrt_tr = 0.5/sqrt_tr;

	quat.w() = (R(1, 2) - R(2, 1))*sqrt_tr;
	quat.y() = (R(0, 1) + R(1, 0))*sqrt_tr;
	quat.z() = (R(2, 0) + R(0, 2))*sqrt_tr;
}

return quat;

}
//the rotation matrix should be defined act_R_w des_R_w
Eigen::Vector3d  inline  computeOrientError(const Eigen::Vector3d & des_orient, const Eigen::Vector3d & actual_orient)
{

	Eigen::Matrix3d Ract = commons::rpyToRot(actual_orient);
	Eigen::Matrix3d Rdes = commons::rpyToRot(des_orient);
	Eigen::Vector3d err;

	//compute the rotation matrix that represent the relative orientation of des_orient w.r.t actual_orient
	Eigen::Matrix3d Re = Rdes*Ract.transpose();
	//compute the angle-axis representation
	err = rotMatToRotVec(Re);

	return err;
}


//compute orientation error based on rotation matrix
//the rotation matrix should be defined act_R_w des_R_w

Eigen::Vector3d  inline  computeOrientError(const Eigen::Matrix3d & Rdes, const Eigen::Matrix3d & Ract)
{

	Eigen::Vector3d err;

	//compute the rotation matrix that represent the relative orientation of des_orient w.r.t actual_orient
	Eigen::Matrix3d Re = Rdes*Ract.transpose();
	//compute the angle-axis representation
	err = rotMatToRotVec(Re);

	return err;
}

//compute orientation error based on quaternions

Quaternion  inline  computeOrientError(Quaternion q_des, Quaternion q)
{

	Quaternion q_e;

	// to the test to take the shortest path in the quaternion
	if (q.dot(q_des) < 0) {
		q.w() =-q.w();
		q.vec() =-q.vec();
	}
	//compute angular error qe = qd x q1_inv
	q_e.w() = q.w()*q_des.w() + q.vec().transpose()*q_des.vec();

	//compute rotation axis
	q_e.vec() = q.w()*q_des.vec() - q_des.w()*q.vec() - q.vec().cross(q_des.vec());


	return q_e;

}


}
}

#endif /* COMMON_MATH_UTILS_H_ */
