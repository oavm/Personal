
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
        omega(i) = M_PI/(double)(eventsLog(2,i + 4) - eventsLog(1,i));
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
  double dutyFactor,stepFrequency,currentTime,compare; //
  Eigen::ArrayXXd omegaVector;
  Eigen::ArrayXXd xInitial(8,1),eventsLog,xNext;
  Eigen::ArrayXXd gaitPattern(2,2);
  Eigen::ArrayXXd timeDifference(1,2);

  gaitPattern << 3,4,
                 1,2;
  timeDifference << 0.15,0.15;
  dutyFactor = 0.8;
  stepFrequency = 1/(double)3;
  currentTime = 6;
  numberOfLegs = 4;
  xInitial << 0,0,0,0,0,0,0,0;
  // xInitial << 3,3.75,4.5,5.25,2.4,3.15,3.9,4.65;

  schedule.set_gaitParameters(dutyFactor,stepFrequency,currentTime,
                              timeDifference,gaitPattern);

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
