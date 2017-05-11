/*
 * ForceController.h
 *
 *  Created on: Jun 6, 2016
 *      Author: victor
 */

#ifndef FORCECONTROLLER
#define FORCECONTROLLER

#include <iostream>
#include <Eigen/Dense>

class ForceController {
public:
	ForceController();
	virtual ~ForceController();

    /*
     * Set controller type:
     * - 0 for PID
     * - 1 for Feedback linearization + PID (FL+PID)
     * - 2 for Time-delay controller (TDC)
     */
    void setControllerType(const int& actType);

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
	void computeControlAction();

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
	 *
	 */
	double getAreaBore();

	/*
	 *
	 */
	double getAreaRod();

    /*
     * Get integral error
     */
    double getIntegralError();

    /*
     * Reset integral error (to assign 0)
     */
    void resetIntegralError();

    /*
     * Reset integral error (to assign a value)
     */
    void resetIntegralError(const double&);

    /*
     * Set saturation value for the integrated error
     */
    void setIntegralErrorSaturation(const double&);

    /*
     * Get force error
     */
    double getForceError();


private:
	double sat(double val,double min, double max);
	int sgn(double val);
	double deadZone(double val, double min, double max);
	double abso(double x);

public:

	double ps;  //Supply pressure
	double pt;  //Tank pressure
	double up_dz;  //Valve dead-zone (upper limit)
	double lw_dz;  //Valve dead-zone (lower limit)
	double beta;   //Bulk modulus
	double Va0;  //Dead volume at chamber A (in case of a linear actuator)
	double Vb0;  //Dead volume at chamber B (in case of a linear actuator)
	double L;  //actuator position range ([m] or [rad] if a linear or rotary, respectively)
	double leak;  //Internal leakage of the actuator
	double actuatorPosition;  //Actuator position ([m] or [rad] if linear or rotary, respectively)
	double actuatorVelocity;  //Actuator velocity ([m/s] or [rad/s] if linear or rotary, respectively)
	double actuatorAcceleration;  //Actuator position ([m] or [rad] if linear or rotary, respectively)
	double taskPeriod;  //Task period

	/*
	 * Actuator effort (or hydraulic force/torque).
	 * Force if it is a linear actuator and torque if it is rotary one.
	 */
	double actuatorEffort;
	double desiredActuatorEffort;
	double Pa, Pb, dPa, dPb;
	double controlAction;
	double Kp, Kd, Ki, Kvc, Kpc;
	double Aa;  //bore area (in case of a linear actuator)
	double Ab;  //rod area (in case of a linear actuator)

private:

    double vd;  //volumetric displacement (in case of a rotary actuator)
    double Kv;  //Total flow rate coefficient ("valve gain")
    double Kva; //Partial flow rate coefficient of the valve
    double Kvb; //Partial flow rate coefficient of the valve
    double KvlkA;  //Internal leakage flow coefficient
    double KvlkB;  //Internal leakage flow coefficient
    double flowA; //Flow into chamber A
    double flowB; //Flow into chamber B
    double actuatorLeakage; //Actuator internal leakage. Positive from chamber A to B.
    int controllerType; //0 for PID, 1 for FL+PID and 2 for TDC
    double forceErrorInt; //integrated force error
    double maxInt; //Maximum value for the integrated error
    double forceError; //Force tracking error

};






#endif /* ForceController */
