/*
 * HydraulicActuator.cpp
 *
 *  Created on: Sep 20, 2013
 *      Author: victor
 */

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <hydraulic_actuator/HydraulicActuator.h>

HydraulicActuator::HydraulicActuator() {

    wn    = 0.0;
    E     = 0.0;
    In    = 0.0;
    Qn    = 0.0;
    dptn  = 0.0;
    Kv    = 0.0;
    Kva   = 0.0;
    Kvb   = 0.0;
    Plk   = 0.0;
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
    Qlk = 0.0;
    Pa  = 0.0;  Pb = 0.0;
    dPa = 0.0; dPb = 0.0;
    actuatorEffort = 0.0;
    actuatorPosition = 0.0;
    actuatorVelocity = 0.0;
    actuatorAcceleration = 0.0;
    lastActuatorVelocity = 0.0;
    valveCommand = 0.0;
    taskPeriod = 0.001;
    integrationSubSteps = 100;
    actuatorType = 0;
    intMethod = 0;
    spoolPos = 0.0;
    spoolVel = 0.0;
    pistonDiameter = 0.0;
    rodDiameter = 0.0;
    capacitanceA = 0.0; capacitanceB = 0.0;

    tareLeakageCoefficient = 0.0;
    tareFlowLeakageAt3000psi = 0.0;
    tareFlowLeakageAt1000psi = 0.0;
    tareFlowLeakage = 0.0;
    valveLeakage = 0.0;
    actuatorLeakage = 0.0;

    intFlowA = 0.0; intFlowB = 0.0;
    pistonFlowA = 0.0; pistonFlowB = 0.0;
    flowA = 0.0; flowB = 0.0;

    hydPowerDemand = 0.0;
    totalFlowDemand = 0.0;
    valveFlowDemand = 0.0;


    vd = 0.0;   X.setZero();
    dX.setZero();
    last_dX.setZero();

    X(2) = ps/2;
    X(3) = ps/2;
}


HydraulicActuator::~HydraulicActuator() {

}


void HydraulicActuator::setActuatorType(const int& actType) {

    actuatorType = actType;
}


void HydraulicActuator::setIntegrationMethod(const int& method){

    intMethod = method;
}


void HydraulicActuator::setVolumetricDisplacement(const double& volDisp){

    vd = volDisp;
}


double HydraulicActuator::getPressureA(){

    return Pa * 1E-5; //[bar]
}


double HydraulicActuator::getPressureB(){

    return Pb * 1E-5; //[bar]
}


double HydraulicActuator::getAreaBore(){ return Aa;}


double HydraulicActuator::getAreaRod(){ return Ab;}


double HydraulicActuator::getTotalFlowDemand(){

    return totalFlowDemand * 60000; //[l/min]
}


double HydraulicActuator::getValveFlowDemand(){

    return valveFlowDemand * 60000; //[l/min]
}


double HydraulicActuator::getFlowA(){

    return flowA * 60000; //[l/min]
}


double HydraulicActuator::getFlowB(){

    return flowB * 60000; //[l/min]
}


double HydraulicActuator::getIntFlowA(){

    return intFlowA * 60000; //[l/min]
}


double HydraulicActuator::getIntFlowB(){

    return intFlowB * 60000; //[l/min]
}


double HydraulicActuator::getPistonFlowA(){

    return pistonFlowA * 60000; //[l/min]
}


double HydraulicActuator::getPistonFlowB(){

    return pistonFlowB * 60000; //[l/min]
}

double HydraulicActuator::getCapacitanceA(){

    return capacitanceA;
}

double HydraulicActuator::getCapacitanceB(){

    return capacitanceB;
}

double HydraulicActuator::getValveLeakage(){

    return valveLeakage * 60000; //[l/min]
}


double HydraulicActuator::getActuatorLeakage(){

    return actuatorLeakage  * 60000; //[l/min]
}


double HydraulicActuator::getSpoolPosition(){

    return spoolPos / In * 100; //[%]
}


void HydraulicActuator::setTareFlowLeakageAt3000psi(const double& tare){

    tareFlowLeakageAt3000psi = tare;

    tareLeakageCoefficient = tareFlowLeakageAt3000psi / sqrt(3000.0/14.5*1E5);
}


void HydraulicActuator::setTareFlowLeakageAt1000psi(const double& tare){

    tareFlowLeakageAt1000psi = tare;

    tareLeakageCoefficient = tareFlowLeakageAt1000psi / sqrt(1000.0/14.5*1E5);
}


double HydraulicActuator::getHydraulicPowerDemand(){ return hydPowerDemand;}


void HydraulicActuator::computeHydCoefficients() {

    Kv = Qn/sqrt(dptn);  //Total flow rate coefficient ("valve gain")
    Kva = Kv * sqrt(2); //Partial flow rate coefficient of the valve
    Kvb = Kv * sqrt(2); //Partial flow rate coefficient of the valve
    KvlkA = Qlk / sqrt(2*Plk);  //Internal leakage flow coefficient
    KvlkB = Qlk / sqrt(2*Plk);  //Internal leakage flow coefficient
    Aa = 3.1415 * pow(pistonDiameter,2) / 4;  //bore area
    Ab = 3.1415 * (pow(pistonDiameter,2) - pow(rodDiameter,2)) / 4;  //rod area

    if(actuatorType){
        Aa = vd;
        Ab = vd;
    }

}



void HydraulicActuator::computeHydDynamics() {

    double valveLeakA, valveLeakB;

    valveCommand = sat(valveCommand,-In,In);
    valveCommand = deadZone(valveCommand, lw_dz, up_dz);


    for (int i = 1; i <= integrationSubSteps; i++){

        dX(0)= X(1);
        dX(1)= -wn*wn*X(0) - 2*E*wn*X(1) + wn*wn*valveCommand;

        pistonFlowA = Aa * actuatorVelocity;
        pistonFlowB = -Ab * actuatorVelocity;
        intFlowA = KvlkA * (sgn(ps-X(2)) * sqrt(abso(ps-X(2))) - sgn(X(2)-pt) * sqrt(abso(X(2)-pt)));
        intFlowB = KvlkB * (sgn(ps-X(3)) * sqrt(abso(ps-X(3))) - sgn(X(3)-pt) * sqrt(abso(X(3)-pt)));
        capacitanceA = (beta/(Va0 + Aa*actuatorPosition));
        capacitanceB = (beta/(Vb0 + Ab*(L - actuatorPosition)));

        if (X(0) >= up_dz)
        {
            flowA = Kva *(X(0)/In) * sgn(ps-X(2)) * sqrt(abso(ps-X(2)));
            dX(2) = capacitanceA * (flowA + intFlowA - pistonFlowA - leak*(X(2)-X(3)));

            flowB = -Kvb *(X(0)/In) * sgn(X(3)-pt) * sqrt(abso(X(3)-pt));
            dX(3) = capacitanceB * (flowB + intFlowB - pistonFlowB - leak*(X(3)-X(2)));
        }
        else {
            if (X(0) <= lw_dz)
            {
                flowA = Kva * (X(0)/In) * sgn(X(2)-pt) * sqrt(abso(X(2)-pt));
                dX(2) = capacitanceA * (flowA + intFlowA - pistonFlowA - leak*(X(2)-X(3)));

                flowB = -Kvb * (X(0)/In) * sgn(ps-X(3)) * sqrt(abso(ps-X(3)));
                dX(3) = capacitanceB * (flowB + intFlowB - pistonFlowB  - leak*(X(3)-X(2)));
            }
            else
            {
                dX(2) = 0;
                dX(3) = 0;
            }
        }


        //Compute integration
        //Euler method
        if(intMethod == 0){
            X = X + taskPeriod/integrationSubSteps * dX;
        }
        //Trapezoidal method
        else{
            X = X + taskPeriod/integrationSubSteps * 0.5 * (last_dX + dX);
            last_dX = dX;
        }

    }


    spoolPos = X(0);
    spoolVel = X(1);
    Pa = X(2);
    Pb = X(3);
    dPa = dX(2);
    dPb = dX(3);

    //Hydraulic force
    actuatorEffort = Aa*Pa - Ab*Pb;


    actuatorLeakage = leak * (Pa - Pb);

    valveLeakA = KvlkA * (sgn(ps-X(2)) * sqrt(abso(ps-X(2))) + sgn(X(2)-pt) * sqrt(abso(X(2)-pt)))/2;
    valveLeakB = KvlkB * (sgn(ps-X(3)) * sqrt(abso(ps-X(3))) + sgn(X(3)-pt) * sqrt(abso(X(3)-pt)))/2;
    tareFlowLeakage = tareLeakageCoefficient * sqrt(ps-pt);
    valveLeakage = valveLeakA + valveLeakB + tareFlowLeakage;


    if(spoolPos >= 0){

        totalFlowDemand = flowA + valveLeakage;
        valveFlowDemand = flowA;
    }
    else{

        totalFlowDemand = flowB + valveLeakage;
        valveFlowDemand = flowB;
    }

    hydPowerDemand = ps * totalFlowDemand;

}

/*
 * SATURATION of the valve
 */
double HydraulicActuator::sat(double val,double min, double max){

    if (val < min) return min;
    else if (val > max) return max;
    else return val;
}

/*
 * DEAD ZONE of the valve (DEAD ZONE)
 */
double HydraulicActuator::deadZone(double val, double min, double max){

    if ((val > min) && (val < max)) return 0;
    if (val >= max) return (val-max);
    if (val <= min) return (val-min);
}

/*
 * SIGNUM function
 */
int HydraulicActuator::sgn(double val) {

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
double HydraulicActuator::abso(double x){

    if (x < 0) return -x;
    else return x;
}

