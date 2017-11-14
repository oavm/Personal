
#include <iostream>
#include <cmath>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/Eigen/Dense>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/MPA.h>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/MPScheduler.h>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/GaitTransition.h>

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
  MaxPlusSchedule schedule,schedule2; // Create schedule object
  AngularFrequencyGenerator omega;
  GaitTransition transition;

  /* Gait parameters */
  int numberOfLegs,h; // Set number of legs of the platform
  double dutyFactor,stepFrequency,currentTime,compare,timeLeg,Tf1; //
  Eigen::ArrayXXd omegaVector,transitionVector1,G,G1,A,A1,Pi;
  Eigen::ArrayXXd xInitial(8,1),eventsLog,xNext;


  Eigen::ArrayXXd gaitPattern2(2,2);
  Eigen::ArrayXXd timeDifference2(1,2);
  gaitPattern2 << 1,4,
                  2,3;
  dutyFactor = 0.65;
  stepFrequency = 1.05;
  timeLeg = (1/stepFrequency)*(1 - (2*(1 - dutyFactor)))*0.5;
  timeDifference2 << timeLeg,timeLeg;
  schedule2.set_gaitParameters(dutyFactor,stepFrequency,currentTime,
                              timeDifference2,gaitPattern2);

  Eigen::ArrayXXd gaitPattern(2,2);
  Eigen::ArrayXXd timeDifference(1,2);
  Eigen::ArrayXXd eigenVector(1,8);
  gaitPattern << 1,4,
                 2,3;
  dutyFactor = 0.6;
  stepFrequency = 2.95;
  timeLeg = (1/stepFrequency)*(1 - (2*(1 - dutyFactor)))*0.5;
  timeDifference << timeLeg,timeLeg;

  currentTime = 1.078;
  numberOfLegs = 4;


  xInitial << 0,0,0,0,0,0,0,0;
  std::cout << "\nxInitial: \n" << xInitial;
  // xInitial << 3,3.75,4.5,5.25,2.4,3.15,3.9,4.65;

  schedule.set_gaitParameters(dutyFactor,stepFrequency,currentTime,
                              timeDifference,gaitPattern);

  std::cout << "\nTime leg: \n" << timeLeg;

  eventsLog = schedule.initiallist(xInitial);
  std::cout << "\nEvents log: \n" << eventsLog;

  std::cout << "\nSize: \n" << eventsLog.cols();

  eigenVector = schedule2.maxpluseigenvector();
  std::cout << "\nEigenvector: \n" << eigenVector;

  transition.set_gaitTransition(schedule,schedule2);

  A = schedule.generatea();
  std::cout << "\nA: \n" << A;

  A1 = transition.generateatransition1();
  std::cout << "\nA1: \n" << A1;

  Tf1 = schedule.settf();
  std::cout << "\nTf1: \n" << Tf1;

  Pi = transition.computepi();
  std::cout << "\nPi: \n" << Pi;

  eventsLog = schedule.initiallist(xInitial);
  std::cout << "\nEvents log: \n" << eventsLog;

  eventsLog = transition.updatefutureeventsconstanstance(eventsLog,currentTime);
  std::cout << "\nEvents log: \n" << eventsLog;

  schedule = schedule2;

  currentTime = 1.79;

  eventsLog = schedule.updatefutureevents(eventsLog,currentTime);
  std::cout << "\nEvents log: \n" << eventsLog;



  return 0;
}
