#ifndef _GaitTransition_H_
#define _GaitTransition_H_

class GaitTransition
{
  MaxPlusSchedule gait1, gait2;

public:

  void set_gaitTransition (MaxPlusSchedule a,MaxPlusSchedule b)
   {
     gait1 = a;
     gait2 = b;
   };


   Eigen::ArrayXXd computepi()
   {
     Eigen::ArrayXXd eigenVectorG1,eigenVectorG2,extraStanceTime;
     Eigen::ArrayXXd diffVector(1,4);
     Eigen::ArrayXXd transitionVector1(1,4);
     double Tf1,Tf2;
     Tf1 = gait1.settf();
     Tf2 = gait2.settf();
     eigenVectorG1 = gait1.maxpluseigenvector();
     eigenVectorG2 = gait2.maxpluseigenvector();
     diffVector = (eigenVectorG2.block<1,4>(0,4) - eigenVectorG1.block<1,4>(0,0));
     extraStanceTime = diffVector - diffVector.minCoeff();
     return extraStanceTime;
   }

   Eigen::ArrayXXd computepivector1()
   {
     Eigen::ArrayXXd eigenVectorG1,eigenVectorG2,extraStanceTime;
     Eigen::ArrayXXd transitionVector1(1,4);
     double Tf1,Tf2;
     Tf1 = gait1.settf();
     Tf2 = gait2.settf();
     extraStanceTime = computepi();
     for (int i=0; i < extraStanceTime.cols(); i++)
     {
       transitionVector1(i) = Tf1 - extraStanceTime(i);
     }
     return transitionVector1;
   }

   Eigen::ArrayXXd computepivector2()
   {
     Eigen::ArrayXXd eigenVectorG1,eigenVectorG2,extraStanceTime;
     Eigen::ArrayXXd transitionVector2(1,4);
     double Tf1,Tf2;
     Tf1 = gait1.settf();
     Tf2 = gait2.settf();
     extraStanceTime = computepi();
     for (int i=0; i < extraStanceTime.cols(); i++)
     {
       transitionVector2(i) = Tf2 - extraStanceTime(i);
     }
     return transitionVector2;
   }

   Eigen::ArrayXXd computepivector12()
   {
     Eigen::ArrayXXd eigenVectorG1,eigenVectorG2,extraStanceTime;
     Eigen::ArrayXXd transitionVector12(1,4);
     double Tf1,Tf2,value1,value2;
     Tf1 = gait1.settf();
     Tf2 = gait2.settf();
     extraStanceTime = computepi();
     for (int i=0; i < extraStanceTime.cols(); i++)
     {
       value1 = std::min(extraStanceTime(i),Tf1);
       value2 = std::min(Tf1,Tf2);
       transitionVector12(i) = std::max(value1,value2);
     }
     return transitionVector12;
   }

   Eigen::ArrayXXd computepivector22()
   {
     Eigen::ArrayXXd eigenVectorG1,eigenVectorG2,extraStanceTime,transition12,vectorDiff;
     Eigen::ArrayXXd transitionVector22(1,4);
     double Tf1,Tf2,value1,value2;
     transition12 = computepivector12();
     Tf1 = gait1.settf();
     Tf2 = gait2.settf();
     extraStanceTime = computepi();
     vectorDiff = transition12 - extraStanceTime;
     value1 = vectorDiff.minCoeff();
     for (int i=0; i < extraStanceTime.cols(); i++)
     {
       transitionVector22(i) = Tf2 - (transition12(i) - extraStanceTime(i)) - value1;
     }
     return transitionVector22;
   }

   Eigen::ArrayXXd generatetransitiong1()
   {
     MaxPlusAlgebra x;
     Eigen::ArrayXXd I,G,P,transitionVector;
     int length;
     transitionVector = computepivector1();
     P = gait1.generatep();
     length = P.cols();
     I = x.maxplusidentity(length);
     G = x.maxplusnull(2*length);
     for (int i = 0; i < length; i++)
     {
       for (int j = 0; j < length; j++)
       {
         I(i,j) = I(i,j) + transitionVector(i);
         G(i,length + j) = I(i,j);
         G(i + length,j) = P(i,j);
       }
     }
     return G;
   }

   Eigen::ArrayXXd generatetransitiong2()
   {
     MaxPlusAlgebra x;
     Eigen::ArrayXXd I,G,P,transitionVector;
     int length;
     transitionVector = computepivector2();
     P = gait2.generatep();
     length = P.cols();
     I = x.maxplusidentity(length);
     G = x.maxplusnull(2*length);
     for (int i = 0; i < length; i++)
     {
       for (int j = 0; j < length; j++)
       {
         I(i,j) = I(i,j) + transitionVector(i);
         G(i,length + j) = I(i,j);
         G(i + length,j) = P(i,j);
       }
     }
     return G;
   }

   Eigen::ArrayXXd generateatransition1()
   {
     MaxPlusAlgebra x;
     Eigen::ArrayXXd A1,G,H;
     G = generatetransitiong1();
     H = gait1.generateh();
     A1 = x.maxplustimes(gait1.generateastar(G),H);
     return A1;
   }

   Eigen::ArrayXXd generateatransition2()
   {
     MaxPlusAlgebra x;
     Eigen::ArrayXXd A2,G,H;
     G = generatetransitiong2();
     H = gait2.generateh();
     A2 = x.maxplustimes(gait2.generateastar(G),H);
     return A2;
   }

   Eigen::ArrayXXd updatefutureeventsconstanstance(Eigen::ArrayXXd eventsLog,
                                       double currentTime)
   {
     Eigen::ArrayXXd oldEvent,nextEvent,futureEvent,A;
     MaxPlusAlgebra x;
     A = generateatransition1();
     if (currentTime >= eventsLog.row(1).maxCoeff())
     {
       oldEvent = eventsLog.row(1);
       // nextEvent = eventsLog.row(2);
       nextEvent = x.maxplustimes(A,oldEvent.transpose());
       // eventsLog.row(1) << nextEvent;
       eventsLog.row(0) << oldEvent;
       eventsLog.row(1) << nextEvent.transpose();
       futureEvent = x.maxplustimes(A,nextEvent);
       eventsLog.row(2) << futureEvent.transpose();
     }
     return eventsLog;
   }


};


  #endif /*_MPScheduler_H_*/
