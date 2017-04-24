#ifndef _MPScheduler_H_
#define _MPScheduler_H_

class MaxPlusSchedule
{
    double dutyFactor,stepFrequency,currentTime;
    bool stanceLegs;
    Eigen::ArrayXXd timeDifference,gaitPattern;
  public:

    // void set_gaitParameters (double,double,double,Eigen::ArrayXXd,
    //                          Eigen::ArrayXXd);

    void set_gaitParameters (double b,double c,double d,
     Eigen::ArrayXXd e,Eigen::ArrayXXd f)
     {
       dutyFactor = b;
       stepFrequency = c;
       currentTime = d;
       timeDifference = e;
       gaitPattern = f;
     };

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
      P = x.maxplusnull(4);
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
      Q = x.maxplusnull(4);
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
      Eigen::ArrayXXd eventsLog(2,8);
      eventsLog.row(0) = xInitial.transpose();
      A = generatea();
      xNext = x.maxplustimes(A,xInitial);
      eventsLog.row(1) = xNext.transpose();
      return eventsLog;
    }

/* From here, there are some experimental methods */
    Eigen::ArrayXXd incrementlist(Eigen::ArrayXXd xInitial,Eigen::ArrayXXd eventsLog)
    {
      MaxPlusAlgebra x;
      Eigen::ArrayXXd xNext, A;
      A = generatea();
      xNext = x.maxplustimes(A,xInitial);
      eventsLog.conservativeResize(eventsLog.rows() + 1,8);
      eventsLog.row(eventsLog.rows() - 1) << xNext.transpose();
      return eventsLog;
    }

    Eigen::ArrayXXd initiallist(Eigen::ArrayXXd xInitial)
    {
      MaxPlusAlgebra x;
      Eigen::ArrayXXd xNext, A;
      Eigen::ArrayXXd eventsLog(3,4*2);
      eventsLog.row(0) = xInitial.transpose();
      A = generatea();
      for (int i = 1; i < 3; i++)
      {
        xNext = x.maxplustimes(A,xInitial);
        eventsLog.row(i) = xNext.transpose();
        xInitial = xNext;
      }
      return eventsLog;
    }

    Eigen::ArrayXXd updatehappeningevents(double currentTime,
                                          Eigen::Matrix<bool, 4,1> legStance,
                                          Eigen::Matrix<bool, 4,1> oldStanceLegs,
                                          Eigen::ArrayXXd eventsLog)
    {
      for (int leg = 0; leg < 4; leg++)
      {
        if (legStance[leg] == true)
        {
          if (legStance[leg] != oldStanceLegs [leg])
          {
            eventsLog(1,leg+4) = currentTime;
          }
        }
        else
        {
          if (legStance[leg] != oldStanceLegs [leg])
          {
            eventsLog(1,leg) = currentTime;
          }
        }
      }
      return eventsLog;
    }

    Eigen::ArrayXXd computefutureevents(Eigen::ArrayXXd eventsLog)
    {
      MaxPlusAlgebra x;
      Eigen::ArrayXXd xNext, A, xInitial;
      A = generatea();
      xInitial = eventsLog.row(1).transpose();
      xNext = x.maxplustimes(A,xInitial);
      eventsLog.row(2) << xNext.transpose();
      return eventsLog;
    }

    Eigen::ArrayXXd updatefutureevents(Eigen::ArrayXXd eventsLog,
                                        double currentTime)
    {
      Eigen::ArrayXXd oldEvent,nextEvent;
      if (currentTime >= eventsLog.row(1).maxCoeff())
      {
        oldEvent = eventsLog.row(1);
        nextEvent = eventsLog.row(2);
        eventsLog.row(0) << oldEvent;
        eventsLog.row(1) << nextEvent;
        eventsLog = computefutureevents(eventsLog);
      }
      return eventsLog;
    }
};

// void MaxPlusSchedule::set_gaitParameters (double b,double c,double d,
//   Eigen::ArrayXXd e,Eigen::ArrayXXd f)
//   {
//     dutyFactor = b;
//     stepFrequency = c;
//     currentTime = d;
//     timeDifference = e;
//     gaitPattern = f;
//   };

  #endif /*_MPScheduler_H_*/
