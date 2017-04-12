// classes example
#include <iostream>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/Eigen/Dense>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/MPA.h>

class MaxPlusSchedule
{
    double dutyFactor,stepFrequency,currentTime;
    int numberOfLegs;
    bool stanceLegs;
    Eigen::ArrayXXd timeDifference,gaitPattern;
  public:

    void set_gaitParameters (int,double,double,double,Eigen::ArrayXXd,
                             Eigen::ArrayXXd);

/* Set Tg */
  double settg()
  {
    double Tg;
    Tg = (1/(double)stepFrequency)*dutyFactor;
    return Tg;
  }

/* Set Tf */
  double settf()
  {
    double Tf;
    Tf = (1/(double)stepFrequency)*(1 - dutyFactor);
    return Tf;
  }

/* Generate P matrix */
    Eigen::ArrayXXd generatep()
    {
      MaxPlusAlgebra x;
      Eigen::ArrayXXd P;
      int length,p,q;
      P = x.maxplusnull(numberOfLegs);
      length = gaitPattern.rows();
      for (int i = 0; i < length - 1; i++)
      {
        for (int j = 0; j < gaitPattern.row(i+1).cols(); j++)
        {
          for (int k = 0; k < gaitPattern.row(i).cols(); k++)
          {
            p = gaitPattern(i+1,j) - 1;
            q = gaitPattern(i,k) - 1;
            P(p,q) = timeDifference(i);
          }
        }
      }
      return P;
    }

/* Generate Q matrix */
    Eigen::ArrayXXd generateq()
    {
      MaxPlusAlgebra x;
      Eigen::ArrayXXd Q;
      int length,p,q;
      Q = x.maxplusnull(numberOfLegs);
      length = gaitPattern.rows();
      for (int l = 0; l < gaitPattern.row(0).cols(); l++)
      {
        for (int m = 0; m < gaitPattern.row(length - 1).cols(); m++)
        {
          p = gaitPattern(0,l) - 1;
          q = gaitPattern(length - 1,m) - 1;
          Q(p,q) = timeDifference(length - 1);
        }
      }
      return Q;
    }

/* Generate G matrix */
    Eigen::ArrayXXd generateg()
    {
      MaxPlusAlgebra x;
      Eigen::ArrayXXd I,G,P;
      int length;
      double Tf;
      P = generatep();
      Tf = settf();
      length = P.cols();
      I = x.maxplusidentity(length);
      G = x.maxplusnull(2*length);
      for (int i = 0; i < length; i++)
      {
        for (int j = 0; j < length; j++)
        {
          I(i,j) = I(i,j) + Tf;
          G(i,length + j) = I(i,j);
          G(i + length,j) = P(i,j);
        }
      }
      return G;
    }

/* Generate H matrix */
    Eigen::ArrayXXd generateh()
    {
      MaxPlusAlgebra x;
      Eigen::ArrayXXd K,H,T,Q;
      int length;
      double Tg;
      Q = generateq();
      Tg = settg();
      length = Q.cols();
      K = x.maxplusidentity(length);
      H = x.maxplusnull(2*length);
      for (int i = 0; i < length; i++)
      {
        for (int j = 0; j < length; j++)
        {
          K(i,j) = K(i,j) + Tg;
        }
      }
      T = x.maxplusplus(K,Q);
      for (int i = 0; i < length; i++)
      {
        for (int j = 0; j < length; j++)
        {
          H(i + length,j) = T(i,j);
        }
      }
      return H;
    }

/* Generate A-star matrix */
    Eigen::ArrayXXd generateastar(Eigen::ArrayXXd A)
    {
      MaxPlusAlgebra x;
      Eigen::ArrayXXd As,As_;
      int length;
      length = A.cols();
      As = x.maxplusidentity(length);
      As_ = x.maxplusidentity(length);
      for (int i = 0; i < length -1; i++)
      {
        As = x.maxplusplus( x.maxplustimes(As,A),A);
      }
      As = x.maxplusplus(As,As_);
      return As;
    }

/* Generate A matrix */
    Eigen::ArrayXXd generatea()
    {
      MaxPlusAlgebra x;
      Eigen::ArrayXXd A,G,H;
      G = generateg();
      H = generateh();
      A = x.maxplustimes(generateastar(G),H);
      return A;
    }

/* Compute next list of events */
    Eigen::ArrayXXd generatenextevent(Eigen::ArrayXXd xInitial)
    {
      MaxPlusAlgebra x;
      Eigen::ArrayXXd xNext, A;
      A = generatea();
      xNext = x.maxplustimes(A,xInitial);
      return xNext;
    }
};

void MaxPlusSchedule::set_gaitParameters (int a,double b,double c,double d,
  Eigen::ArrayXXd e,Eigen::ArrayXXd f)
  {
    numberOfLegs = a;
    dutyFactor = b;
    stepFrequency = c;
    currentTime = d;
    timeDifference = e;
    gaitPattern = f;
  }


int main ()
{
  MaxPlusSchedule schedule;
  Eigen::ArrayXXd matrixTest;
  Eigen::ArrayXXd P,Q,G,H,xNext,eventsLog,A;
  Eigen::ArrayXXd xInitial(8,1);
  Eigen::ArrayXXd gaitPattern(2,2);
  Eigen::ArrayXXd timeDifference(1,2);
  int numberOfLegs;
  double dutyFactor,stepFrequency,currentTime;

  gaitPattern << 1,4,
                 2,3;
  timeDifference << 0.2,0.4;
  dutyFactor = 0.6;
  stepFrequency = 1/(double)3;
  currentTime = 10;
  numberOfLegs = 4;
  xInitial << 0,
              0,
              0,
              0,
              0,
              0,
              0,
              0;

  schedule.set_gaitParameters(numberOfLegs,dutyFactor,stepFrequency,currentTime,
                              timeDifference,gaitPattern);

  eventsLog = xInitial.transpose();
  for (int i = 1; i < 10; i++)
  {
    xNext = schedule.generatenextevent(xInitial);
    eventsLog.conservativeResize(i+1,8);
    eventsLog.row(i) = xNext.transpose();
    xInitial = xNext;
  }
  std::cout << "\nLog of events: \n" << eventsLog;
  return 0;
}
