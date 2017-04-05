// classes example
#include <iostream>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/Eigen/Dense>

class MaxPlusSchedule {
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

void MaxPlusSchedule::set_values (double x, double y, bool z,Eigen::MatrixXd wx) {
  dutyFactor = x;
  stepFrequency = y;
  stanceLegs = z;
  timeDifference = wx;
}

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
  return 0;
}
