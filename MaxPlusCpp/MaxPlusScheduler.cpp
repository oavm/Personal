
#include <iostream>
#include <cmath>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/Eigen/Dense>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/MPA.h>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/MPScheduler.h>

class AngularFrequencyGenerator
{
  Eigen::ArrayXXd omega,eventsLog;
  double currentTime;
public:
  Eigen::ArrayXXd generateomega(double currentTime,Eigen::ArrayXXd eventsLog)
  {
    Eigen::ArrayXXd omega(1,4);
    for (int i = 0; i < 4; i++)
    {
      if (currentTime >= eventsLog(0,i) && currentTime < eventsLog(1,i + 4))
      {
        omega(i) = M_PI/(double)(eventsLog(1,i + 4) - eventsLog(0,i)); // Stance phase
      }
      else if (currentTime >= eventsLog(1,i + 4) && currentTime <= eventsLog(1,i))
      {
        omega(i) = M_PI/(double)(eventsLog(1,i) - eventsLog(1,i + 4)); // Swing phase
      }
      else
      {
        // omega(i) = M_PI/(double)(eventsLog(2,i + 4) - eventsLog(1,i));
      }
    }
    // for (int i = 0; i < 4; i++)
    // {
    //   if (currentTime >= eventsLog(0,i + 4) && currentTime <= eventsLog(0,i))
    //   {
    //     omega(i) = M_PI/(double)(eventsLog(0,i) - eventsLog(0,i + 4)); // Swing phase
    //   }
    //   else if (currentTime >= eventsLog(0,i) && currentTime < eventsLog(1,i + 4))
    //   {
    //     omega(i) = M_PI/(double)(eventsLog(1,i + 4) - eventsLog(0,i)); // Stance phase
    //   }
    //
    // }
    return omega;
  }
};

int main ()
{
  MaxPlusSchedule schedule; // Create schedule object
  AngularFrequencyGenerator omega;

  /* Gait parameters */
  int numberOfLegs,h; // Set number of legs of the platform
  double dutyFactor,stepFrequency,currentTime,compare,timeLeg; //
  Eigen::ArrayXXd omegaVector;
  Eigen::ArrayXXd xInitial(8,1),eventsLog,xNext;

  // Eigen::ArrayXXd gaitPattern(2,2);
  // Eigen::ArrayXXd timeDifference(1,2);
  // gaitPattern << 1,4,
  //                2,3;
  // dutyFactor = 0.25;
  // stepFrequency = 1.7;
  // timeLeg = (1/stepFrequency)*(1 - (2*(1 - dutyFactor)))*0.5;
	// timeDifference << timeLeg,timeLeg;
  // currentTime = 3;
  // numberOfLegs = 4;

  Eigen::ArrayXXd gaitPattern(4,1);
  Eigen::ArrayXXd timeDifference(1,4);
  gaitPattern << 1,
                 4,
                 2,
                 3;
  dutyFactor = 0.25;
  stepFrequency = 1.7;
  timeLeg = (1/stepFrequency)*(1 - (2*(1 - dutyFactor)))*0.25;
  timeDifference << timeLeg,timeLeg,timeLeg,timeLeg;
  currentTime = 3;
  numberOfLegs = 4;


  xInitial << 0,0,0,0,0,0,0,0;
  std::cout << "\nxInitial: \n" << xInitial;
  // xInitial << 3,3.75,4.5,5.25,2.4,3.15,3.9,4.65;

  schedule.set_gaitParameters(dutyFactor,stepFrequency,currentTime,
                              timeDifference,gaitPattern);

  std::cout << "\nTime leg: \n" << timeLeg;

  eventsLog = schedule.initiallist(xInitial);
  std::cout << "\nEvents log: \n" << eventsLog;

  eventsLog = schedule.updatefutureevents(eventsLog,currentTime);
  std::cout << "\nEvents log: \n" << eventsLog;

  eventsLog = schedule.initiallist(xInitial);
  std::cout << "\nEvents log: \n" << eventsLog;

  // omegaVector = omega.generateomega(currentTime,eventsLog);
  // std::cout << "\nAngular velocity: \n" << omegaVector;



  return 0;
}
