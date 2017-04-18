
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
  int numberOfLegs,h; // Set number of legs of the platform
  double dutyFactor,stepFrequency,currentTime,compare; //
  Eigen::ArrayXXd omegaVector;
  Eigen::ArrayXXd xInitial(8,1),eventsLog,xNext;
  Eigen::ArrayXXd gaitPattern(2,2);
  Eigen::ArrayXXd timeDifference(1,2);

  gaitPattern << 1,4,
                 2,3;
  timeDifference << 0.2,0.4;
  dutyFactor = 0.6;
  stepFrequency = 1/(double)3;
  currentTime = 0;
  numberOfLegs = 4;
  xInitial << 2.9,4.3,4.2,2.8,1.7,3.1,3.1,1.7;

  schedule.set_gaitParameters(numberOfLegs,dutyFactor,stepFrequency,currentTime,
                              timeDifference,gaitPattern);

  eventsLog = schedule.generatenextevent(xInitial);

  std::cout << "\nEvents log: \n" << eventsLog;

  // omegaVector = omega.generateomega(currentTime,eventsLog);

  // for(int i = 0; i < 60; i++)
  // {
  //   currentTime = i*0.1;
  //   compare = eventsLog.row(1).maxCoeff() - currentTime;
  //   // std::cout << "\nLog of events: \n" <<  eventsLog.block<1,4>(1,4);
  //   if (compare <= 0)
  //   {
  //     xInitial = eventsLog.row(1).transpose();
  //     eventsLog = schedule.generatenextevent(xInitial);
  //   }
  //   omegaVector = omega.generateomega(currentTime,eventsLog);
  //   std::cout << "\nEvents log: \n" << eventsLog;
  //   std::cout << "\nAngular frequency: \n" << omegaVector;
  //   std::cout << "\nTime: \n" << currentTime;
  // }


  return 0;
}
