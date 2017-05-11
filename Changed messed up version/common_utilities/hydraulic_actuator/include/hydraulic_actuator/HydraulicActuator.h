/*
 * HydraulicActuator.h
 *
 *  Created on: Sep 20, 2013
 *      Author: victor
 */

#ifndef HYDRAULICACTUATOR
#define HYDRAULICACTUATOR

#include <iostream>
#include <Eigen/Dense>

class HydraulicActuator {
public:
    HydraulicActuator();
    virtual ~HydraulicActuator();

    /*
     * Set actuator type: 0 for linear and 1 for rotary hydraulic actuator.
     */
    void setActuatorType(const int& actType);

    /*
     * Set actuator position.
     */
    void setActuatorPosition(const double& actPos);

    /*
     * Set actuator velocity.
     */
    void setActuatorVelocity(const double& actVel);

    /*
     * Compute hydraulic dynamics (valve and pressure dynamics).
     */
    void computeHydDynamics();

    /*
     * Get actuator's effort (force in case of a linear actuator and torque in
     * case of a rotary one.
     */
    void getActuatorEffort(const double& actVel);

    /*
     * Get pressure at chamber A [bar].
     */
    double getPressureA();

    /*
     * Get pressure at chamber B [bar].
     */
    double getPressureB();

    /*
     * This function overwrites the assigned Aa and Ab parameter values
     */
    void setVolumetricDisplacement(const double& volDisp);

    /*
     *
     */
    void computeHydCoefficients();

    /*
     *
     */
    double getAreaBore();

    /*
     *
     */
    double getAreaRod();

    /*
     * Set integration method. 0 for Euler and 1 for trapezoidal.
     * The default integration method is Euler.
     */
    void setIntegrationMethod(const int&);

    /*
     * Get flow required from the pump. Considers chamber flow + internal leakage
     */
    double getTotalFlowDemand();

    /*
     * Get flow demand excluding the valve leakages
     */
    double getValveFlowDemand();

    /*
     * Get flow into chamber A. Positive flow means flow entering the chamber.
     */
    double getFlowA();

    /*
     * Get flow into chamber B. Positive flow means flow entering the chamber.
     */
    double getFlowB();

    /*
     * Get flow into chamber A due to internal leakage [l/min].
     */
    double getIntFlowA();

    /*
     * Get flow into chamber B due to internal leakage [l/min].
     */
    double getIntFlowB();

    /*
     * Get flow into chamber A due to piston displacement [l/min].
     */
    double getPistonFlowA();

    /*
     * Get flow into chamber B due to piston displacement [l/min].
     */
    double getPistonFlowB();

    /*
     * Get valve internal leakage.
     */
    double getValveLeakage();

    /*
     * Get actuator internal leakage. Positive from chamber A to B.
     */
    double getActuatorLeakage();

    /*
     * Get hydraulic power required from the pump = ps * flowDemand
     */
    double getHydraulicPowerDemand();

    /*
     * Get spool position in % of total range
     */
    double getSpoolPosition();

    /*
     * Set tare flow leakage measured at 3000 PSI
     */
    void setTareFlowLeakageAt3000psi(const double&);

    /*
     * Set tare flow leakage measured at 1000 PSI
     */
    void setTareFlowLeakageAt1000psi(const double&);

    /*
     * Get capacitance at chamber A
     */
    double getCapacitanceA();

    /*
     * Get capacitance at chamber B
     */
    double getCapacitanceB();

private:
    double sat(double val,double min, double max);
    int sgn(double val);
    double deadZone(double val, double min, double max);
    double abso(double x);

public:
    double wn;  //Natural frequency of the valve
    double E;   //Damping coefficient of the valve
    double In;  //Nominal current of the valve
    double dptn;  //Total valve pressure drop
    double Qn;  //Nominal valve flow rate (flow rate at dptn pressure)
    double ps;  //Supply pressure
    double pt;  //Tank pressure
    double up_dz;  //Valve dead-zone (upper limit)
    double lw_dz;  //Valve dead-zone (lower limit)
    double beta;   //Bulk modulus
    double Va0;  //Dead volume at chamber A (in case of a linear actuator)
    double Vb0;  //Dead volume at chamber B (in case of a linear actuator)
    double pistonDiameter;  //[m]
    double rodDiameter;  //[m]
    double L;  //actuator position range ([m] or [rad] if a linear or rotary, respectively)
    double leak;  //Internal leakage of the actuator
    double Plk;  //Leakage pressure for leakage measurement.
    double Qlk;  //Leakage flow rate at "leakage pressure".
    double actuatorPosition;  //Actuator position ([m] or [rad] if linear or rotary, respectively)
    double actuatorVelocity;  //Actuator velocity ([m/s] or [rad/s] if linear or rotary, respectively)
    double actuatorAcceleration;  //Actuator position ([m] or [rad] if linear or rotary, respectively)
    double valveCommand;  //Command that goes to the valve (current [A])
    double taskPeriod;  //Task period
    double integrationSubSteps; //Number of integration sub-steps within one task period
    int actuatorType; //0 for linear and 1 for rotary actuator
    double tareFlowLeakageAt3000psi;
    double tareFlowLeakageAt1000psi;
    double tareFlowLeakage;


    /*
     * Actuator effort (or hydraulic force/torque).
     * Force if it is a linear actuator and torque if it is rotary one.
     */
    double actuatorEffort;
    double spoolPos, spoolVel;
    double Pa, Pb, dPa, dPb;

private:
    /*
     * State vector: X(1) = spool position, X(1) = spool velocity,
     * X(3) = pressure in chamber A, X(4) = pressure in chamber B.
     */
    Eigen::Matrix<double, 4,1> X;
    Eigen::Matrix<double, 4,1> dX; //time derivative of the state vector
    Eigen::Matrix<double, 4,1> last_dX; //memories used in the trapezoidal integration method
    double vd;  //volumetric displacement (in case of a rotary actuator)
    double Kv;  //Total flow rate coefficient ("valve gain")
    double Kva; //Partial flow rate coefficient of the valve
    double Kvb; //Partial flow rate coefficient of the valve
    double KvlkA;  //Internal leakage flow coefficient
    double KvlkB;  //Internal leakage flow coefficient
    double Aa;  //bore area (in case of a linear actuator)
    double Ab;  //rod area (in case of a linear actuator)
    int intMethod; //Integration method (0 for Euler and 1 for Trapezoidal)
    double totalFlowDemand; //Flow required from the pump. Considers chamber flow + internal leakage
    double flowA; //Flow into chamber A
    double flowB; //Flow into chamber B
    double valveLeakage; //Valve internal leakage
    double valveFlowDemand; //Flow demand excluding the valve leakages
    double actuatorLeakage; //Actuator internal leakage. Positive from chamber A to B.
    double hydPowerDemand; //Hydraulic power required from the pump = ps * flowDemand
    double tareLeakageCoefficient;
    double intFlowA, intFlowB; //Net flow that enters the chamber due to internal leakage
    double pistonFlowA, pistonFlowB; //Flow due to piston motion
    double capacitanceA, capacitanceB; //Chamber capacitance
    double lastActuatorVelocity;  //Last actuator velocity ([m/s] or [rad/s] if linear or rotary, respectively)

};






#endif /* HYDRAULICACTUATOR */
