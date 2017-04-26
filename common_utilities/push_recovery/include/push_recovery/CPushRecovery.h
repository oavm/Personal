/*
 * CPushRecovery.h
 *
 *  Created on: Sep 12, 2013
 *      Author: victor
 */

#ifndef CPUSHRECOVERY_H_
#define CPUSHRECOVERY_H_



class CPushRecovery {
public:
	typedef Eigen::Matrix<double, 4,1> Vector4D;
public:
	CPushRecovery();
	virtual ~CPushRecovery();

	void ComputeDeltas(double Vx,
		   	   	   	   	  double Vy,
		   	   	   	   	  double Psit,
		   	   	   	   	  double Body_Xd_HF,
		   	   	   	   	  double Body_Yd_HF,
		   	   	   	   	  double Body_Yawd,
		   	   	   	   	  Vector4D& Delta_X,
		   	   	   	   	  Vector4D& Delta_Y);

	void Enable(bool TurnOnPushRecovery=true);
	void Disable();
	void setRecoveryGains(double X_RecoveryGain,
					double Y_RecoveryGain,
					double Yaw_RecoveryGain);


	void setStancePeriod(double);
	void setFeetHeight(double FeetHeight);
	void setDeltaLimits(double xLimit, double yLimit);
	void setTrunkParameters(double t_mass, double t_inertia, double t_length, double t_width);
	void setOutputFilter(double sample_frequency_in_Hz = 250, double cut_off_frequency_in_Hz = 1000);
	void Filter(Vector4D& DeltaX_f, Vector4D& DeltaY_f);

private:
	double g0;
	double st_p;
	double Z0h;
	double Tf, Ts, alpha_filter;
	double t_mass, t_inertia, t_length, t_width;
	bool PushFlag;
	double K_pr_lx, K_pr_ly, K_pr_r;
	double maxDeltax, maxDeltay;
};

#endif /* CPUSHRECOVERY_H_ */





