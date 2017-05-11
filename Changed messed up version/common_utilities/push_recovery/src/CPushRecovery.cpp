/*
 * CPushRecovery.cpp
 *
 *  Created on: Sep 12, 2013
 *      Author: Victor Barasuol
 */

#include <Eigen/Dense>
#include <stdio.h>
#include <push_recovery/CPushRecovery.h>

/**
 *  Push Recovery Details
 *
 * \param PushFlag = If TRUE it enables push recovery algorithm. If PushFlag=FALSE the Delta's are zero.
 * \param XFlag = If TRUE it enables push recovery in the longitudinal direction.
 * \param YFlag = If TRUE it enables push recovery in the lateral direction.
 * \param YawFlag = If TRUE it enables push recovery for yaw motion.
 * \param Vx = Desired longitudinal velocity for the body.
 * \param Vy = Desired lateral velocity for the body.
 * \param Psit = Desired yaw rotation for the body.
 * \param trunk_mass = trunk mass  //  trunk_length = trunk length  //  trunk_width = trunk width  //
 * \param trunk_inertia = trunk inertia with respect to the horizontal frame z axes.
 * \param st_p = stance period  // Z0h = Origin of primitives according to the horizontal frame
 * \param Body_Xd_HF = Estimated body longitudinal velocity  //  Body_Xd_HF = Estimated body lateral velocity.
 * \param Body_Yawd = Sensored body yaw angular velocity.
 * \param maxDeltax = Maximus displacement (+ and -) allowed for the x outputs in order to avoid reaching leg worspace limit
 * \param maxDeltay = Maximus displacement (+ and -) allowed for the y outputs in order to avoid reaching leg worspace limit
 */

CPushRecovery::CPushRecovery() {

	PushFlag = false;
	g0=9.81;
	Ts = 1/250; Tf = 1/1000; alpha_filter = 0.0;
	K_pr_lx = 0; K_pr_ly = 0; K_pr_r = 0;
	t_mass=1.0; t_inertia=1.0; t_length=1.0; t_width=1.0;
	st_p=1.0; Z0h=1.0;
	maxDeltax=0.1; maxDeltay=0.1;
}


CPushRecovery::~CPushRecovery() {
	// TODO Auto-generated destructor stub
}


void CPushRecovery::Enable(bool TurnOnPushRecovery) {

	PushFlag = TurnOnPushRecovery;
	if(PushFlag){
		printf("\nPush Recovery ON!!!\n\n");
	}
	else{
		printf("\nPush Recovery OFF!!!\n\n");
	}

}


void CPushRecovery::Disable() {

	PushFlag = false;
	printf("\nPush Recovery OFF\n\n!!!");
}


void CPushRecovery::setOutputFilter(double sample_frequency_in_Hz, double cut_off_frequency_in_Hz) {

	if(sample_frequency_in_Hz>0) {
		Ts = 1/sample_frequency_in_Hz;
	}
	else {
		printf("\n Sample frequency must be greater than 0!!! \n\n");
	}

	if(cut_off_frequency_in_Hz>0) {
		Tf = 1/cut_off_frequency_in_Hz;
	}
	else {
		printf("\n Sample frequency must be greater than 0!!! \n\n");

	}

	alpha_filter = Ts/(Ts+Tf);

}

void CPushRecovery::Filter(Vector4D& DeltaX_f, Vector4D& DeltaY_f) {

	static Vector4D DeltaX_filter = Vector4D::Zero();
	static Vector4D DeltaY_filter = Vector4D::Zero();

	DeltaX_filter = (1-alpha_filter)*DeltaX_filter + alpha_filter*DeltaX_f;
	DeltaY_filter = (1-alpha_filter)*DeltaY_filter + alpha_filter*DeltaY_f;
	DeltaX_f=DeltaX_filter;
	DeltaY_f=DeltaY_filter;

}

void CPushRecovery::setRecoveryGains(double X_PushRecovery, double Y_PushRecovery, double Yaw_PushRecovery) {

	if ((X_PushRecovery>=0) && (X_PushRecovery<5) && (Y_PushRecovery>=0) && (Y_PushRecovery<5) && (Yaw_PushRecovery>=0) && (Yaw_PushRecovery<5)) {
	K_pr_lx = X_PushRecovery;
	K_pr_ly = Y_PushRecovery;
	K_pr_r = Yaw_PushRecovery;
	}
	else{
		printf("\n Unsafe values!!!\n\n");
	}
}


void CPushRecovery::setDeltaLimits(double xLimit, double yLimit) {

	if (xLimit>0 && yLimit>0){
		maxDeltax=xLimit;
		maxDeltay=yLimit;
	}
	else{
		printf("\nWrong values!!! This function imposes symmetric saturations for the relative displacements.\n\n");
	}
}


void CPushRecovery::setStancePeriod(double stance_period) {

	if(stance_period>0){
		st_p=stance_period;
	}
	else{
		printf("\nThe stance period must be a positive value!!!\n\n");
	}
}


void CPushRecovery::setFeetHeight(double FeetHeight) {

	if(FeetHeight>0){
		Z0h=FeetHeight;
	}
	else{
		printf("\nThe equivalent height for all feet must be a positive value!!!");
	}
}







void CPushRecovery::setTrunkParameters(double trunk_mass, double trunk_inertia, double trunk_length, double trunk_width) {

	if (trunk_mass>0 && trunk_inertia>0 && trunk_length>0 && trunk_width>0){
		t_mass=trunk_mass;
		t_inertia=trunk_inertia;
		t_length=trunk_length;
		t_width=trunk_width;
	}
	else{
		printf("\nThe parameters must have positive values!!!\n\n");
	}
}



void CPushRecovery::ComputeDeltas(double Vx,
								   	   double Vy,
								   	   double Psit,
								   	   double Body_Xd_HF,
		 	 	 	 	 	 	 	   double Body_Yd_HF,
		 	 	 	 	 	 	 	   double Body_Yawd,
		 	 	 	 	 	 	 	   Vector4D& Delta_X,
		 	 	 	 	 	 	 	   Vector4D& Delta_Y) {
	/**
	 * Commands to print inputs
	 *
		printf("Vx: %f\n",Vx);
		printf("Vy: %f\n",Vy);
		printf("Psit_push: %f\n",Psit_push);
		printf("t_mass: %f\n",t_mass);
		printf("t_inertia: %f\n",t_inertia);
		printf("st_p: %f\n",st_p);
		printf("Z0h: %f\n\n",Z0h);
		printf("maxDeltax: %f\n",maxDeltax);
		printf("maxDeltay: %f\n",maxDeltay);
		printf("Body_Xd_HF: %f\n",Body_Xd_HF);
		printf("Body_Yd_HF: %f\n",Body_Yd_HF);
		printf("Body_Yawd: %f\n",Body_Yawd);
		*/


		// Desired Body Motion Input Check
		if(Vx<-10 || Vx>10 || Vy<-10 || Vy>10 || Psit<-10 || Psit>10){
			PushFlag=false;}

		//Body Motion Input Check
		if(Body_Xd_HF<-10 || Body_Xd_HF>10 || Body_Yd_HF<-10 || Body_Yd_HF>10 || Body_Yawd<-10 || Body_Yawd>10){
			PushFlag=false;}


		if (PushFlag){

			double Sr=sin(atan(t_width/t_length));
			double Cr=cos(atan(t_width/t_length));
			double C1_pr=Z0h/g0/st_p; //C1_pr should be positive
			double C2_pr=Sr*Sr*t_inertia/t_width/t_mass; //C2_pr is a positive coefficient
			double C3_pr=Cr*Cr*t_inertia/t_length/t_mass; //C3_pr is a positive coefficient


			Delta_X(0)=C1_pr*(K_pr_lx*(Body_Xd_HF-Vx)-K_pr_r*C2_pr*(Body_Yawd-Psit));
				if(Delta_X(0)>maxDeltax){Delta_X(0)=maxDeltax;}
				if(Delta_X(0)<-maxDeltax){Delta_X(0)=-maxDeltax;}

			Delta_Y(0)=C1_pr*(K_pr_ly*(Body_Yd_HF-Vy)+K_pr_r*C3_pr*(Body_Yawd-Psit));
				if(Delta_Y(0)>maxDeltay){Delta_Y(0)=maxDeltay;}
				if(Delta_Y(0)<-maxDeltay){Delta_Y(0)=-maxDeltay;}

			Delta_X(1)=C1_pr*(K_pr_lx*(Body_Xd_HF-Vx)+K_pr_r*C2_pr*(Body_Yawd-Psit));
				if(Delta_X(1)>maxDeltax){Delta_X(1)=maxDeltax;}
				if(Delta_X(1)<-maxDeltax){Delta_X(1)=-maxDeltax;}

			Delta_Y(1)=C1_pr*(K_pr_ly*(Body_Yd_HF-Vy)+K_pr_r*C3_pr*(Body_Yawd-Psit));
				if(Delta_Y(1)>maxDeltay){Delta_Y(1)=maxDeltay;}
				if(Delta_Y(1)<-maxDeltay){Delta_Y(1)=-maxDeltay;}

			Delta_X(2)=C1_pr*(K_pr_lx*(Body_Xd_HF-Vx)-K_pr_r*C2_pr*(Body_Yawd-Psit));
				if(Delta_X(2)>maxDeltax){Delta_X(2)=maxDeltax;}
				if(Delta_X(2)<-maxDeltax){Delta_X(2)=-maxDeltax;}

			Delta_Y(2)=C1_pr*(K_pr_ly*(Body_Yd_HF-Vy)-K_pr_r*C3_pr*(Body_Yawd-Psit));
				if(Delta_Y(2)>maxDeltay){Delta_Y(2)=maxDeltay;}
				if(Delta_Y(2)<-maxDeltay){Delta_Y(2)=-maxDeltay;}

			Delta_X(3)=C1_pr*(K_pr_lx*(Body_Xd_HF-Vx)+K_pr_r*C2_pr*(Body_Yawd-Psit));
				if(Delta_X(3)>maxDeltax){Delta_X(3)=maxDeltax;}
				if(Delta_X(3)<-maxDeltax){Delta_X(3)=-maxDeltax;}

			Delta_Y(3)=C1_pr*(K_pr_ly*(Body_Yd_HF-Vy)-K_pr_r*C3_pr*(Body_Yawd-Psit));
				if(Delta_Y(3)>maxDeltay){Delta_Y(3)=maxDeltay;}
				if(Delta_Y(3)<-maxDeltay){Delta_Y(3)=-maxDeltay;}
			}
		else
			{
			Delta_X.setZero();
			Delta_Y.setZero();
			}

		Filter(Delta_X, Delta_Y);

}


//void CPushRecovery::setFlags(const cfg_flags& f) {
//	my_flags.pushFlag = f.pushFlag;
//...
//}

