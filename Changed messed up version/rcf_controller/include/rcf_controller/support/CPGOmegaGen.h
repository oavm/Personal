#ifndef _CPGOmegaGen_H_
#define _CPGOmegaGen_H_

class OmegaGenerator
{
// double currentTime;
  public:

/* Angular frequency generator */
    Eigen::ArrayXXd omegaGenerator(Eigen::ArrayXXd eventsLog,double maxPlusTime)
    {
      Eigen::ArrayXXd omegaVector(1,4);
      for (int i = 0; i < 4; i++)
      {
        if (maxPlusTime >= eventsLog(0,i) && maxPlusTime < eventsLog(1,i + 4))
        {
          omegaVector(i) = M_PI/(double)(eventsLog(1,i + 4) - eventsLog(0,i)); // Stance phase
        }
        else if (maxPlusTime >= eventsLog(1,i + 4) && maxPlusTime <= eventsLog(1,i))
        {
          omegaVector(i) = M_PI/(double)(eventsLog(1,i) - eventsLog(1,i + 4)); // Swing phase
        }
        else
        {
          omegaVector(i) = M_PI/(double)(eventsLog(2,i + 4) - eventsLog(1,i));
        }
      }
      return omegaVector;
    };

};

#endif /*_MPA_H_*/
