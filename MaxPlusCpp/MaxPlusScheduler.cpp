
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
    }
    return omega;
  }
};

int main ()
{
  MaxPlusSchedule schedule; // Create schedule object
  AngularFrequencyGenerator omega;

  /* Gait parameters */
  int numberOfLegs; // Set number of legs of the platform
  double dutyFactor,stepFrequency,currentTime; //
  Eigen::ArrayXXd omegaVector;
  Eigen::ArrayXXd xInitial(8,1),eventsLog,xNext;
  Eigen::ArrayXXd gaitPattern(2,2);
  Eigen::ArrayXXd timeDifference(1,2);

  gaitPattern << 1,4,
                 2,3;
  timeDifference << 0.2,0.4;
  dutyFactor = 0.6;
  stepFrequency = 1/(double)3;
  currentTime = 4;
  numberOfLegs = 4;
  xInitial << 0,0,0,0,0,0,0,0;

  schedule.set_gaitParameters(numberOfLegs,dutyFactor,stepFrequency,currentTime,
                              timeDifference,gaitPattern);

  eventsLog = schedule.generatenextevent(xInitial);
  std::cout << "\nLog of events: \n" << eventsLog;

  omegaVector = omega.generateomega(currentTime,eventsLog);
  std::cout << "\nAngular velocities: \n" << omegaVector;

  return 0;
}
