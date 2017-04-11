// classes example
#include <iostream>
#include </home/octaviovillarreal/Git/Personal/MaxPlusCpp/Eigen/Dense>

class MaxPlusSchedule
{
    double dutyFactor,stepFrequency,time;
    Eigen::ArrayXXd timeDifference,gaitPattern;
    int numberOfLegs;
    bool stanceLegs;
  public:
    // Eigen::ArrayXXd generatepq(Eigen::ArrayXXd gaitPattern,
    //   Eigen::MatrixXd timeDifference,int numberOfLegs)
    // {
    //   MaxPlusAlgebra x;
    //   P = x.maxplusnull(numberOfLegs)
    //   Q = x.maxplusnull(numberOfLegs)
    //   length = gait.rows()
    //   for (int i = 0; i < length - 1; i++)
    //   {
    //     for (int j = 0; i < gait.rows)
    //   }
    // }
    void set_values (double,double,bool,Eigen::MatrixXd);
    double area() {return dutyFactor*stepFrequency;}
    bool state() {return stanceLegs;}
    Eigen::MatrixXd vector_diplay() {return timeDifference;}
};

class MaxPlusAlgebra
{
    Eigen::ArrayXXd a,b;
    int c;
  public:
    void set_values (Eigen::ArrayXXd a,Eigen::ArrayXXd b,int c);

/* Max-plus multiplication */
    Eigen::ArrayXXd maxplustimes(Eigen::ArrayXXd a,Eigen::ArrayXXd b)
    {
      Eigen::ArrayXXd x(a.rows(), b.cols());
      Eigen::ArrayXXd y(1,b.rows());
      for (int i = 0; i < a.rows(); i++)
      {
        for (int j = 0; j < b.cols(); j++)
        {
          for (int k =0; k < b.rows(); k++)
          {
            y(0,k) = a(i,k) + b(k,j);
          }
          x(i,j) = y.maxCoeff();
        }
      }
      return x;
    };

/* Max-plus addition*/
  Eigen::ArrayXXd maxplusplus(Eigen::ArrayXXd a,Eigen::ArrayXXd b)
  {
    Eigen::ArrayXXd x(a.rows(), b.cols());
    for (int i = 0; i < a.rows(); i++)
    {
      for (int j = 0; j < b.cols(); j++)
      {
        x(i,j) = std::max(a(i,j),b(i,j));
      }
    }
    return x;
  };

/* Max-plus null matrix*/
    Eigen::ArrayXXd maxplusnull(int c)
    {
      Eigen::ArrayXXd E(c,c);
      for (int i = 0; i < c; i++)
      {
        for (int j = 0; j < c; j++)
        {
          E(i,j) = INFINITY;
        }
      }
      return E;
    };

/* Max-plus identity matrix*/
      Eigen::ArrayXXd maxplusidentity(int c)
      {
        Eigen::ArrayXXd I(c,c);
        I = maxplusnull(c);
        for (int i = 0; i < c; i++)
        {
          I(i,i) = 0;
        }
        return I;
      };
};

int main ()
{
  MaxPlusAlgebra maximo;
  Eigen::ArrayXXd A(2,2);
  Eigen::ArrayXXd B(2,1);
  Eigen::ArrayXXd C(2,2);
  Eigen::ArrayXXd matrixTest;

  A << 3.0,4.0,
       5.0,6.0;
  B << 5.0,6.0;
  C << 1.0,2.0,
       3.0,4.0;

  std::cout << "\nMax-plus multiplication: \n" << maximo.maxplustimes(A,B);
  std::cout << "\nMax-plus addition: \n" << maximo.maxplusplus(A,C);
  std::cout << "\nNull: \n" << maximo.maxplusnull(2);
  std::cout << "\nIdentity: \n" << maximo.maxplusidentity(2);

  matrixTest = maximo.MaxPlusAlgebra::maxplustimes(A,C);
  std::cout << "\nTest: \n" << matrixTest;

  return 0;
}
