#ifndef _MPA_H_
#define _MPA_H_

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
          E(i,j) = -INFINITY;
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

#endif /*_MPA_H_*/
