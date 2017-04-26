/*
 * CPushRecovery2DOF.cpp
 *
 *  Created on: July 1, 2013
 *      Author: Victor Barasuol
 */

#include <Eigen/Dense>
#include <stdio.h>
#include <push_recovery/CPushRecovery2DOF.h>

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

CPushRecovery2DOF::CPushRecovery2DOF() {

    PushFlag = false;
    g0=9.81;
    Ts = 1/250;
    //legLF = 0;
    //legRF = 1;
    //legLH = 2;
    //legRH = 3;
    Tf(legLF) = 1/1000;
    Tf(legRF) = 1/1000;
    Tf(legLH) = 1/1000;
    Tf(legRH) = 1/1000;
    alpha_filter.setZero();
    K_pr_lx = 0; K_pr_ly = 0; K_pr_r = 0;
    t_mass=1.0; t_inertia=1.0; t_length=1.0; t_width=1.0;
    st_p=1.0; Z0h=1.0;
    maxDeltax=0.1; maxDeltay=0.1;
    onOff_x = 1; onOff_y = 1; onOff_psi = 1;

    for (int leg = dog::LF; leg <= dog::RH; leg++){
        deltaPos[leg].setZero();
        deltaVel[leg].setZero();
    }
}


CPushRecovery2DOF::~CPushRecovery2DOF() {
    // TODO Auto-generated destructor stub
}


void CPushRecovery2DOF::Enable(bool TurnOnPushRecovery) {

    PushFlag = TurnOnPushRecovery;
    if(PushFlag){
        printf("\nPush Recovery ON!!!\n\n");
    }
    else{
        printf("\nPush Recovery OFF!!!\n\n");
    }

}


void CPushRecovery2DOF::Disable() {

    PushFlag = false;
    printf("\nPush Recovery OFF\n\n!!!");
}


void CPushRecovery2DOF::setTaskFrequency(double sample_frequency_in_Hz) {

    if(sample_frequency_in_Hz>0) {
        Ts = 1/sample_frequency_in_Hz;
    }
    else {
        printf("\n Sample frequency must be greater than 0!!! \n\n");
    }

}

void CPushRecovery2DOF::setOutputFilters(double cut_off_frequency_in_Hz) {

    if(cut_off_frequency_in_Hz > 0) {
        Tf(legLF) = 1/cut_off_frequency_in_Hz;
        Tf(legRF) = 1/cut_off_frequency_in_Hz;
        Tf(legLH) = 1/cut_off_frequency_in_Hz;
        Tf(legRH) = 1/cut_off_frequency_in_Hz;
        alpha_filter(legLF,legLF) = Ts/(Ts+Tf(legLF));
        alpha_filter(legRF,legRF) = Ts/(Ts+Tf(legRF));
        alpha_filter(legLH,legLH) = Ts/(Ts+Tf(legLH));
        alpha_filter(legRH,legRH) = Ts/(Ts+Tf(legRH));
        alphaFilter = Ts / (Ts + (1/cut_off_frequency_in_Hz));
    }
    else {
        printf("\n Sample frequency must be greater than 0!!! \n\n");
    }

}

void CPushRecovery2DOF::enablePushRecoveryX(bool enableFlag) {

    onOff_x = 0;
    if(enableFlag)
        onOff_x = 1;
}
void CPushRecovery2DOF::enablePushRecoveryY(bool enableFlag) {

    onOff_y = 0;
    if(enableFlag)
        onOff_y = 1;
}
void CPushRecovery2DOF::enablePushRecoveryPsi(bool enableFlag){

    onOff_psi = 0;
    if(enableFlag)
        onOff_psi = 1;
}


int CPushRecovery2DOF::getOnOffPushRecoveryFlagX(void) {

    return onOff_x;
}

int CPushRecovery2DOF::getOnOffPushRecoveryFlagY(void) {

    return onOff_y;
}

int CPushRecovery2DOF::getOnOffPushRecoveryFlagPsi(void) {

    return onOff_psi;
}



void CPushRecovery2DOF::setLegOutputFilter(int legNumber, double cut_off_frequency_in_Hz) {

    switch(legNumber) {
    case legLF:
        if(cut_off_frequency_in_Hz > 0) {
            Tf(legLF) = 1/cut_off_frequency_in_Hz;
            alpha_filter(legLF,legLF) = Ts/(Ts+Tf(legLF));
        }
        else {
            printf("\n LF Sample frequency must be greater than 0!!! \n\n");
        }
        break;

    case legRF:
        if(cut_off_frequency_in_Hz > 0) {
            Tf(legRF) = 1/cut_off_frequency_in_Hz;
            alpha_filter(legRF,legRF) = Ts/(Ts+Tf(legRF));

        }
        else {
            printf("\n RF Sample frequency must be greater than 0!!! \n\n");

        }
        break;

    case legLH:
        if(cut_off_frequency_in_Hz > 0) {
            Tf(legLH) = 1/cut_off_frequency_in_Hz;
            alpha_filter(legLH,legLH) = Ts/(Ts+Tf(legLH));
        }
        else {
            printf("\n LH Sample frequency must be greater than 0!!! \n\n");

        }
        break;

    case legRH:
        if(cut_off_frequency_in_Hz > 0) {
            Tf(legRH) = 1/cut_off_frequency_in_Hz;
            alpha_filter(legRH,legRH) = Ts/(Ts+Tf(legRH));
        }
        else {
            printf("\n RH Sample frequency must be greater than 0!!! \n\n");

        }
        break;
    }

}


void CPushRecovery2DOF::Filter(Vector4D& DeltaX_f, Vector4D& DeltaY_f) {

    static Vector4D DeltaX_filter = Vector4D::Zero();
    static Vector4D DeltaY_filter = Vector4D::Zero();

    DeltaX_filter = (Matrix44D::Identity()-alpha_filter)*DeltaX_filter + alpha_filter*DeltaX_f;
    DeltaY_filter = (Matrix44D::Identity()-alpha_filter)*DeltaY_filter + alpha_filter*DeltaY_f;
    DeltaX_f = DeltaX_filter;
    DeltaY_f = DeltaY_filter;

}

void CPushRecovery2DOF::setRecoveryGains(double break_coefficient_x, double break_coefficient_y, double break_coefficient_psi) {

    if ((break_coefficient_x>=0) && (break_coefficient_x<=1)) {
    K_pr_lx = break_coefficient_x;
    }
    else {
        printf("\n Unsafe break coefficient for X direction!!!\n\n");
    }

    if ((break_coefficient_y>=0) && (break_coefficient_y<=1)) {
    K_pr_ly = break_coefficient_y;
    }
    else {
        printf("\n Unsafe break coefficient for Y direction!!!\n\n");
    }

    if((break_coefficient_psi>=0) && (break_coefficient_psi<=1)) {
    K_pr_r = break_coefficient_psi;
    }
    else {
        printf("\n Unsafe heading break coefficient!!!\n\n");
    }
}


void CPushRecovery2DOF::setDeltaLimits(double xLimit, double yLimit) {

    if (xLimit>0 && yLimit>0){
        maxDeltax = xLimit;
        maxDeltay = yLimit;
    }
    else{
        printf("\nWrong values!!! This function imposes symmetric saturations for the relative displacements.\n\n");
    }
}


void CPushRecovery2DOF::setStancePeriod(double stance_period) {

    if(stance_period>0){
        st_p = stance_period;
    }
    else{
        printf("\nThe stance period must be a positive value!!!\n\n");
    }
}


void CPushRecovery2DOF::setFeetHeight(double FeetHeight) {

    if(FeetHeight>0){
        Z0h=FeetHeight;
    }
    else{
        printf("\nThe equivalent height for all feet must be a positive value!!!");
    }
}







void CPushRecovery2DOF::setTrunkParameters(double trunk_mass, double trunk_inertia, double trunk_length, double trunk_width) {

    if (trunk_mass>0 && trunk_inertia>0 && trunk_length>0 && trunk_width>0){
        t_mass = trunk_mass;
        t_inertia = trunk_inertia;
        t_length = trunk_length;
        t_width = trunk_width;
    }
    else{
        printf("\nThe parameters must have positive values!!!\n\n");
    }
}

LegDataMap<rbd::Vector3d> CPushRecovery2DOF::getDeltaPos(void){

    return deltaPos;
}

LegDataMap<rbd::Vector3d> CPushRecovery2DOF::getDeltaVel(void){

    return deltaVel;
}



void CPushRecovery2DOF::Filter(LegDataMap<rbd::Vector3d>& deltaF) {

    static LegDataMap<rbd::Vector3d> deltaFilter(rbd::Vector3d::Zero());

    for (int leg = dog::LF; leg <= dog::RH; leg++){

        deltaFilter[leg] = (1.0 - alpha_filter(leg,leg)) * deltaFilter[leg]
                           + alpha_filter(leg,leg) * deltaF[leg];
    }

    deltaF = deltaFilter;
}



void CPushRecovery2DOF::ComputeDeltas(double Vx,
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
            PushFlag = false;}

        //Body Motion Input Check
        if(Body_Xd_HF<-10 || Body_Xd_HF>10 || Body_Yd_HF<-10 || Body_Yd_HF>10 || Body_Yawd<-10 || Body_Yawd>10){
            PushFlag = false;}


        if (PushFlag){

            double r = sqrt(t_length*t_length + t_width*t_width)/2;
            double rx = t_length/2;
            double ry = t_width/2;
            double tau_t = sqrt(Z0h/g0);
            double tau_r = sqrt(Z0h*t_inertia/t_mass/g0/(r*r));
            double Cx_pr = onOff_x * tau_t * ( cosh(st_p/tau_t) / sinh(st_p/tau_t) - (1 - K_pr_lx) / sinh(st_p/tau_t) );
            double Cy_pr = onOff_y * tau_t * ( cosh(st_p/tau_t) / sinh(st_p/tau_t) - (1 - K_pr_ly) / sinh(st_p/tau_t) );
            double Cr_pr = onOff_psi * tau_r * ( cosh(st_p/tau_r) / sinh(st_p/tau_r) - (1 - K_pr_r) / sinh(st_p/tau_r) );


            Delta_X(0) = Cx_pr * (Body_Xd_HF-Vx) - Cr_pr * ry * (Body_Yawd-Psit);
                if(Delta_X(0) > maxDeltax){Delta_X(0) = maxDeltax;}
                if(Delta_X(0) < -maxDeltax){Delta_X(0) = -maxDeltax;}

            Delta_Y(0) = Cy_pr * (Body_Yd_HF-Vy) + Cr_pr * rx * (Body_Yawd-Psit);
                if(Delta_Y(0) > maxDeltay){Delta_Y(0) = maxDeltay;}
                if(Delta_Y(0) < -maxDeltay){Delta_Y(0) = -maxDeltay;}

            Delta_X(1) = Cx_pr * (Body_Xd_HF-Vx) + Cr_pr * ry * (Body_Yawd-Psit);
                if(Delta_X(1) > maxDeltax){Delta_X(1) = maxDeltax;}
                if(Delta_X(1) < -maxDeltax){Delta_X(1) = -maxDeltax;}

            Delta_Y(1) = Cy_pr * (Body_Yd_HF-Vy) + Cr_pr * rx * (Body_Yawd-Psit);
                if(Delta_Y(1) > maxDeltay){Delta_Y(1) = maxDeltay;}
                if(Delta_Y(1) < -maxDeltay){Delta_Y(1) = -maxDeltay;}

            Delta_X(2) = Cx_pr * (Body_Xd_HF-Vx) - Cr_pr * ry * (Body_Yawd-Psit);
                if(Delta_X(2) > maxDeltax){Delta_X(2) = maxDeltax;}
                if(Delta_X(2) < -maxDeltax){Delta_X(2) = -maxDeltax;}

            Delta_Y(2) = Cy_pr * (Body_Yd_HF-Vy) - Cr_pr * rx * (Body_Yawd-Psit);
                if(Delta_Y(2) > maxDeltay){Delta_Y(2) = maxDeltay;}
                if(Delta_Y(2) < -maxDeltay){Delta_Y(2) = -maxDeltay;}

            Delta_X(3) = Cx_pr * (Body_Xd_HF-Vx) + Cr_pr * ry * (Body_Yawd-Psit);
                if(Delta_X(3) > maxDeltax){Delta_X(3) = maxDeltax;}
                if(Delta_X(3) < -maxDeltax){Delta_X(3) = -maxDeltax;}

            Delta_Y(3) = Cy_pr * (Body_Yd_HF-Vy) - Cr_pr * rx * (Body_Yawd-Psit);
                if(Delta_Y(3) > maxDeltay){Delta_Y(3) = maxDeltay;}
                if(Delta_Y(3) < -maxDeltay){Delta_Y(3) = -maxDeltay;}
            }
        else
            {
            Delta_X.setZero();
            Delta_Y.setZero();
            }

        Filter(Delta_X, Delta_Y);

}


void CPushRecovery2DOF::ComputeDeltas(double Vx,
                                       double Vy,
                                       double Psit,
                                       double Body_Xd_HF,
                                       double Body_Yd_HF,
                                       double Body_Yawd) {

        // Desired Body Motion Input Check
        if(Vx<-10 || Vx>10 || Vy<-10 || Vy>10 || Psit<-10 || Psit>10){
            PushFlag = false;}

        //Body Motion Input Check
        if(Body_Xd_HF<-10 || Body_Xd_HF>10 || Body_Yd_HF<-10 || Body_Yd_HF>10 || Body_Yawd<-10 || Body_Yawd>10){
            PushFlag = false;}


        if (PushFlag){

            double r = sqrt(t_length*t_length + t_width*t_width)/2;
            double rx = t_length/2;
            double ry = t_width/2;
            double tau_t = sqrt(Z0h/g0);
            double tau_r = sqrt(Z0h*t_inertia/t_mass/g0/(r*r));
            double Cx_pr = onOff_x * tau_t * ( cosh(st_p/tau_t) / sinh(st_p/tau_t) - (1 - K_pr_lx) / sinh(st_p/tau_t) );
            double Cy_pr = onOff_y * tau_t * ( cosh(st_p/tau_t) / sinh(st_p/tau_t) - (1 - K_pr_ly) / sinh(st_p/tau_t) );
            double Cr_pr = onOff_psi * tau_r * ( cosh(st_p/tau_r) / sinh(st_p/tau_r) - (1 - K_pr_r) / sinh(st_p/tau_r) );


            deltaPos[dog::LF](rbd::X) = Cx_pr * (Body_Xd_HF-Vx) - Cr_pr * ry * (Body_Yawd-Psit);
                if(deltaPos[dog::LF](rbd::X) > maxDeltax){
                    deltaPos[dog::LF](rbd::X) = maxDeltax;}
                if(deltaPos[dog::LF](rbd::X) < -maxDeltax){
                    deltaPos[dog::LF](rbd::X) = -maxDeltax;}

                deltaPos[dog::LF](rbd::Y) = Cy_pr * (Body_Yd_HF-Vy) + Cr_pr * rx * (Body_Yawd-Psit);
                if(deltaPos[dog::LF](rbd::Y) > maxDeltay){
                    deltaPos[dog::LF](rbd::Y) = maxDeltay;}
                if(deltaPos[dog::LF](rbd::Y) < -maxDeltay){
                    deltaPos[dog::LF](rbd::Y) = -maxDeltay;}

                deltaPos[dog::RF](rbd::X) = Cx_pr * (Body_Xd_HF-Vx) + Cr_pr * ry * (Body_Yawd-Psit);
                if(deltaPos[dog::RF](rbd::X) > maxDeltax){
                    deltaPos[dog::RF](rbd::X) = maxDeltax;}
                if(deltaPos[dog::RF](rbd::X) < -maxDeltax){
                    deltaPos[dog::RF](rbd::X) = -maxDeltax;}

                deltaPos[dog::RF](rbd::Y) = Cy_pr * (Body_Yd_HF-Vy) + Cr_pr * rx * (Body_Yawd-Psit);
                if(deltaPos[dog::RF](rbd::Y) > maxDeltay){
                    deltaPos[dog::RF](rbd::Y) = maxDeltay;}
                if(deltaPos[dog::RF](rbd::Y) < -maxDeltay){
                    deltaPos[dog::RF](rbd::Y) = -maxDeltay;}

                deltaPos[dog::LH](rbd::X) = Cx_pr * (Body_Xd_HF-Vx) - Cr_pr * ry * (Body_Yawd-Psit);
                if(deltaPos[dog::LH](rbd::X) > maxDeltax){
                    deltaPos[dog::LH](rbd::X) = maxDeltax;}
                if(deltaPos[dog::LH](rbd::X) < -maxDeltax){
                    deltaPos[dog::LH](rbd::X) = -maxDeltax;}

            deltaPos[dog::LH](rbd::Y) = Cy_pr * (Body_Yd_HF-Vy) - Cr_pr * rx * (Body_Yawd-Psit);
                if(deltaPos[dog::LH](rbd::Y) > maxDeltay){
                    deltaPos[dog::LH](rbd::Y) = maxDeltay;}
                if(deltaPos[dog::LH](rbd::Y) < -maxDeltay){
                    deltaPos[dog::LH](rbd::Y) = -maxDeltay;}

            deltaPos[dog::RH](rbd::X) = Cx_pr * (Body_Xd_HF-Vx) + Cr_pr * ry * (Body_Yawd-Psit);
                if(deltaPos[dog::RH](rbd::X) > maxDeltax){
                    deltaPos[dog::RH](rbd::X) = maxDeltax;}
                if(deltaPos[dog::RH](rbd::X) < -maxDeltax){
                    deltaPos[dog::RH](rbd::X) = -maxDeltax;}

            deltaPos[dog::RH](rbd::Y) = Cy_pr * (Body_Yd_HF-Vy) - Cr_pr * rx * (Body_Yawd-Psit);
                if(deltaPos[dog::RH](rbd::Y) > maxDeltay){
                    deltaPos[dog::RH](rbd::Y) = maxDeltay;}
                if(deltaPos[dog::RH](rbd::Y) < -maxDeltay){
                    deltaPos[dog::RH](rbd::Y) = -maxDeltay;}
            }
        else
            {
            deltaPos = rbd::Vector3d::Zero();
            deltaVel = rbd::Vector3d::Zero();
            }


        //Reaction filtering (if set by the user)
        Filter(deltaPos);


        //Compute delta variation (velocity) by discrete time difference
        for (int leg = dog::LF; leg <= dog::RH; leg++){
            if(PushFlag){
                deltaVel[leg] = 1.0 / Ts * (deltaPos[leg] - lastDelta[leg]);                
            }else{
                deltaVel[leg].setZero();
            }
            lastDelta[leg] = deltaPos[leg];
        }


}


//void CPushRecovery::setFlags(const cfg_flags& f) {
//  my_flags.pushFlag = f.pushFlag;
//...
//}

