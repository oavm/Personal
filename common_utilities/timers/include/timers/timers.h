/*
 * timer.h
 *
 *  Created on: Mar 17, 2016
 *      Author: mfocchi
 */

#ifndef TIMERS_H_
#define TIMERS_H_

class Timers{
public:
	Timers(){}
	~Timers(){}
	inline void startTimer(double  startTime)
	{
		T0 = startTime;
		resetFlag = false;
	}
	inline void setDuration(double  dur)
	{
		duration = dur;
	}
	inline double getDuration()
	{
		return duration;
	}
	inline double getMissingTime(double time){
		double missing_time = 0;
		if (resetFlag){
			return duration;
		}
		if ( (time - T0)<=duration)
		{
			missing_time = duration - (time - T0);
		}
		return missing_time;
	}
	inline double getElapsedTime(double time)
	{
		return 	(time - T0);
	}
	inline bool isTimeElapsed(double time)
	{

		if ((time - T0) > duration){

			resetFlag = true;
			return true;
		}
		else
		{

			return false;
		}

	}
	inline bool changeDuration(double time, double NewDuration)
	{
		if (NewDuration > getElapsedTime(time)){
			duration = NewDuration;
			return true;
		} else
			return false;
	}
	inline bool resetTimer(){
		resetFlag = true;
	}

	bool resetFlag = true;
private:
	double T0 = 0.0;
	double duration = 0.0;

};
#endif /* TIMER_H_ */
