#ifndef _tensorbuild_H_
#define _tensorbuild_H_

class TensorBuild
{
  Eigen::MatrixXd tensorMatrix;
  std::string indexMatrix;
  Eigen::MatrixXi tensorDimensions;

public:

  void set_tensorData (Eigen::MatrixXd a, std::string b, Eigen::MatrixXi c)
  {
    tensorMatrix = a;
    indexMatrix = b;
    tensorDimensions = c;
  }


  Eigen::Tensor<double, 4> createTensor()
  {

    int index_0,index_1,index_2,index_3;
    Eigen::MatrixXd index_vector(1,4);
    Eigen::Tensor<double, 4> tensor(tensorDimensions(0),tensorDimensions(1),tensorDimensions(2),tensorDimensions(3));


    std::ifstream indices;
    std::string line;

    int rows = 0;
    int cols = 0;
    int a = 0;

    indices.open(indexMatrix.c_str());

    while (std::getline(indices, line))
    {
      std::stringstream lineStream(line);
      std::string cell;
      while (std::getline(lineStream, cell, ','))
      {
        std::stringstream iss(cell);
        int number;
        std::vector<int> myNumbers;
        while ( iss >> number){

          index_vector(0,a) = number;
          ++a;
        }
        index_0 = index_vector(0,0);
        index_1 = index_vector(0,1);
        index_2 = index_vector(0,2);
        index_3 = index_vector(0,3);
        tensor(index_0,index_1,index_2,index_3) = tensorMatrix(rows,cols);
        ++cols;
        a = 0;
      }
      ++rows;
      cols = 0;
    }

    return tensor;
    };



};

  #endif /*_tensorbuild_H_*/
