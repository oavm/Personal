/*
 * ForceController.cpp
 *
 *  Created on: Jun 6, 2013
 *      Author: victor
 */

#include "../include/force_controller/ForceController.h"

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>

ForceController::ForceController() {

    Kva   = 0.0;
    Kvb   = 0.0;
    KvlkA = 0.0;
    KvlkB = 0.0;
    ps    = 165E5;
    pt    = 0.0;
    up_dz = 0.0;
    lw_dz = 0.0;
    beta  = 0.0;
    Va0   = 0.0;
    Vb0   = 0.0;
    Aa    = 0.000206;
    Ab    = 0.000122;
    L     = 0.0;
    leak  = 0.0;
    actuatorEffort = 0.0;
    actuatorPosition = 0.0;
    actuatorVelocity = 0.0;
    taskPeriod = 0.001;

    vd = 0.0;

    controlAction = 0.0;
    controllerType = 0;
    forceError = 0.0;
    forceErrorInt = 0.0;
    maxInt = 10000;
    desiredActuatorEffort = 0.0;
    Kp = 0.0; Kd = 0.0; Ki = 0.0;
    Kpc = 0.0; Kvc = 0.0;
    Pa = 0.0; Pb = 0.0; dPa = 0.0; dPb = 0.0;
}


ForceController::~ForceController() {

}


void ForceController::setControllerType(const int& contType){

    controllerType = contType;
}


double ForceController::getPressureA(){

    return Pa; //[Pa]
}


double ForceController::getPressureB(){

    return Pb; //[Pa]
}


double ForceController::getAreaBore(){ return Aa;}


double ForceController::getAreaRod(){ return Ab;}


void ForceController::resetIntegralError(){

    forceErrorInt = 0.0;
}

void ForceController::resetIntegralError(const double& intError){

    forceErrorInt = intError;
}


void ForceController::setIntegralErrorSaturation(const double& intSat) {

    maxInt = intSat;
}


void ForceController::computeControlAction() {

    //Force error
    forceError = desiredActuatorEffort - actuatorEffort;

    //Error integration
    forceErrorInt += forceError;

    //Safety check to avoid a command peak when Ki becomes greater than 0
    if(Ki == 0.0){
        forceErrorInt = 0.0;
    }


//    //Integral error saturation
//    if(forceErrorInt > maxInt){
//        forceErrorInt = maxInt;
//    }
//
//    if(forceErrorInt < -maxInt){
//        forceErrorInt = -maxInt;
//    }

    switch(controllerType){

    //PID
    case 0:

        if(actuatorVelocity >= 0) {
            controlAction =  Kp * forceError + Ki * forceErrorInt +
                    Kvc * Aa * actuatorVelocity;
        }
        else{
            controlAction =  Kp * forceError + Ki * forceErrorInt +
                    Kvc * Ab * actuatorVelocity;
        }

        break;


    //FL+PID
    case 1:

        controlAction = 0.0;
        break;

    //TDC
    case 2:

        controlAction = 0.0;
        break;
    }


}


/*
 * SATURATION of the valve
 */
double ForceController::sat(double val,double min, double max){

    if (val < min) return min;
    else if (val > max) return max;
    else return val;
}

/*
 * SIGNUM function
 */
int ForceController::sgn(double val) {

    if (val < 0) return -1;
    if (val == 0) {
        return 0;
    }
    else {
        return 1;
    }

}

/*
 * ABSOLUTE value function
 */
double ForceController::abso(double x){

    if (x < 0) return -x;
    else return x;
}

