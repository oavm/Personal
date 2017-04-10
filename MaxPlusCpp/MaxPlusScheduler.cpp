// classes example
#include <iostream>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/Eigen/Dense>
using namespace std;

class MaxPlusSchedule{
    double dutyFactor,stepFrequency,time;
    Eigen::MatrixXd timeDifference;
    int gaitPattern;
    bool stanceLegs;
  public:
    void set_values (double,double,bool,Eigen::MatrixXd);
    double area() {return dutyFactor*stepFrequency;}
    bool state() {return stanceLegs;}
    Eigen::MatrixXd vector_diplay() {return timeDifference;}
};

class MaxPlusAlgebra {
    Eigen::ArrayXXd a,b,c;
  public:
    void set_values (Eigen::ArrayXXd,Eigen::ArrayXXd,Eigen::ArrayXXd);
    Eigen::ArrayXXd maxplustimes() {

      Eigen::ArrayXXd x(a.rows(), b.cols());
      Eigen::ArrayXXd y(1,b.rows());


      Eigen::ArrayXXd P(2,2);
      Eigen::ArrayXXd Q(2,1);

      P << 2.0,3.0,
           4.0,5.0;
      Q << 1.0,2.0;


      for (int h = 0; h == P.rows(); h++){
        for (int j = 0; j == Q.cols(); j++){
          for (int k =0; k == Q.rows(); k++){
            y(1,k) = P(h,k) + Q(k,j);
          }
          x(h,j) = y.maxCoeff();
        }
      }
      return x;
    };
};

void MaxPlusSchedule::set_values (double x, double y, bool z,Eigen::MatrixXd wx) {
  dutyFactor = x;
  stepFrequency = y;
  stanceLegs = z;
  timeDifference = wx;
};

void MaxPlusAlgebra::set_values (Eigen::ArrayXXd p,Eigen::ArrayXXd q ,Eigen::ArrayXXd r){
  a = p;
  b = q;
  c = r;
};

int main () {
  MaxPlusSchedule rect;
  Eigen::MatrixXd v(1,4);
  v(0,0) = 1;
  v(0,1) = 2;
  v(0,2) = 3;
  v(0,3) = 4;
  rect.set_values (3.1,4.12,true,v);
  std::cout << "area: " << rect.area();
  std::cout << "\nstate: " << rect.state();
  std::cout << "\nvector:\n " << rect.vector_diplay();

  MaxPlusAlgebra maximo;
  Eigen::ArrayXXd A(2,2);
  Eigen::ArrayXXd B(2,1);
  Eigen::ArrayXXd C(2,2);
  A << 1.0,2.0,
       3.0,4.0;
  B << 5.0,6.0;
  C << 1.0,2.0,
       3.0,4.0;
  maximo.set_values (A,B,C);
  std::cout << "\nMax-plus multiplication: \n" << maximo.maxplustimes();
  std::cout << "\nCols: \n" << B.cols();
  std::cout << "\nCoefficient: \n" << B(1,0);

  Eigen::ArrayXXd x(A.rows(), B.cols());
  double y;
  for (int i = 0; i < A.rows(); i++){
    for (int j = 0; j < B.cols(); j++){
      for (int k = 0; k < B.rows(); k++){
        y = A(i,k);

      }
      //x(i,j) = y.maxCoeff();
      std::cout << "\n matrix coefficient: \n" << y;
    }
  }

  double P;
  P = A(0,1) + B(1,0);
  for (int count1 = 0; count1 < A.rows(); count1++) {

    std::cout << "\ncounter: \n" << B.cols();
  }



  return 0;
}
