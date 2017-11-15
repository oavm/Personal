
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include </home/octavio/Git/Personal/CNNCpp/Eigen/Dense>
#include </home/octavio/Git/Personal/CNNCpp/unsupported/Eigen/CXX11/Tensor>
#include </home/octavio/Git/Personal/CNNCpp/unsupported/Eigen/CXX11/Tensor>
#include <vector>
#include <set>
#include <string>
#include <istream>
#include <cstdlib>
#include <sstream>

#include </home/octavio/Git/Personal/CNNCpp/csvRead.h>
#include </home/octavio/Git/Personal/CNNCpp/tensorbuild.h>
#include </home/octavio/Git/Personal/CNNCpp/HyQNN.h>

using namespace std;

int main ()
{

  NeuralNetwork neuralNetwork;

  // Obtain image from csv file
  CsvReadFile image;
  Eigen::MatrixXd image_matrix;
  image.set_csv("image0.csv");
  image_matrix = image.createMatrix();
  Eigen::MatrixXi image_size(1,2);
  image_size << image_matrix.rows(),image_matrix.cols();
  // std::cout << "\n Image matrix: \n" << image_matrix;

  // Obtain matrix containing kernels from csv file
  CsvReadFile kernel0;
  Eigen::MatrixXd value_matrix;
  kernel0.set_csv("LF-SimpCNN-conv_part-layer_0-kernel.csv");
  value_matrix = kernel0.createMatrix();
  // std::cout << "\n Kernel matrix: \n" << value_matrix;

  // Obtain matrix containing kernels from csv file
  CsvReadFile kernel1;
  Eigen::MatrixXd value_matrix1;
  kernel1.set_csv("LF-SimpCNN-conv_part-layer_1-kernel.csv");
  value_matrix1 = kernel1.createMatrix();
  // std::cout << "\n Kernel matrix: \n" << value_matrix1;

  // Obtain vector of bias
  CsvReadFile b0;
  Eigen::MatrixXd bias0;
  b0.set_csv("LF-SimpCNN-conv_part-layer_0-bias.csv");
  bias0 = b0.createMatrix();
  // std::cout << "\nbias 0: \n" << bias0;

  // Obtain vector of bias
  CsvReadFile b1;
  Eigen::MatrixXd bias1;
  b1.set_csv("LF-SimpCNN-conv_part-layer_1-bias.csv");
  bias1 = b1.createMatrix();
  // std::cout << "\nbias 1: \n" << bias1;

  // Create kernel tensor
  TensorBuild kernel0Tensor;
  Eigen::MatrixXd kernel00;
  Eigen::MatrixXi size_tensor(1,4); // Define size of the Tensor
  size_tensor << 5,5,1,4;
  Eigen::Tensor<double, 4> K0(size_tensor(0),size_tensor(1),size_tensor(2),size_tensor(3)); // Define tensor for the kernel
  kernel0Tensor.set_tensorData(value_matrix,"SimpCNN-conv_part-layer_0-kernel_INDICES.csv",size_tensor);
  K0 = kernel0Tensor.createTensor();
  kernel00 = neuralNetwork.extract2DMatrix(0,0,K0);
  // std::cout << "\n Kernel 0 0: \n" << kernel00;

  // Create kernel tensor
  TensorBuild kernel1Tensor;
  Eigen::MatrixXd kernel10;
  Eigen::MatrixXi size_tensor1(1,4); // Define size of the Tensor
  size_tensor1 << 5,5,4,8;
  Eigen::Tensor<double, 4> K1(size_tensor1(0),size_tensor1(1),size_tensor1(2),size_tensor1(3)); // Define tensor for the kernel
  kernel1Tensor.set_tensorData(value_matrix1,"SimpCNN-conv_part-layer_1-kernel_INDICES.csv",size_tensor1);
  K1 = kernel1Tensor.createTensor();
  kernel10 = neuralNetwork.extract2DMatrix(2,2,K1);
  // std::cout << "\n Kernel 2 2: \n" << kernel10;

  // Perform convolution with all kernels
  // NeuralNetwork neuralNetwork;
  Eigen::Tensor<double, 4> tensorConvolution(size_tensor(0),size_tensor(1),size_tensor(2),size_tensor(3));
  Eigen::MatrixXd tensorConvolutionMatrix[size_tensor(2)][size_tensor(3)]; // Use tensor size definition to build kernel matrix
  Eigen::MatrixXd tensorConvolution_0(image_matrix.rows(),image_matrix.cols());
  Eigen::MatrixXd filtered_image_array[size_tensor(2)][size_tensor(3)];
  neuralNetwork.set_neuralNetwork(K0,K1,bias0,bias1,image_matrix);
  tensorConvolution = neuralNetwork.convolution(image_matrix,K0);
  tensorConvolution_0 = neuralNetwork.extract2DMatrix(0,0,tensorConvolution);
  // std::cout << "\n Filtered image \n" << tensorConvolution_0;

  // Activation function ReLu
  Eigen::MatrixXd poslin(image_size(0),image_size(1));
  Eigen::Tensor<double, 4> tensorReLu(size_tensor(0),size_tensor(1),size_tensor(2),size_tensor(3));
  tensorReLu = neuralNetwork.reLu();
  poslin = neuralNetwork.extract2DMatrix(0,0,tensorReLu);
  // std::cout << "\n Filtered image with activation \n" << poslin;

  // Apply Maxpool operation
  Eigen::MatrixXd maxPoolMatrix;
  Eigen::MatrixXd maxpool_array[size_tensor(2)][size_tensor(3)];
  Eigen::Tensor<double, 4> tensorMaxPool(size_tensor(0),size_tensor(1),size_tensor(2),size_tensor(3));
  tensorMaxPool = neuralNetwork.maxPool();
  maxPoolMatrix = neuralNetwork.extract2DMatrix(0,2,tensorMaxPool);
  // std::cout << "\n Matrix after maxpool \n" << maxPoolMatrix;

  // Test for single convolution
  Eigen::MatrixXd singleConvolution;
  Eigen::MatrixXd testKernel;
  testKernel = neuralNetwork.extract2DMatrix(0,0,K0);
  singleConvolution = neuralNetwork.singleConvolution(image_matrix,testKernel);
  // std::cout << "\n Single convolution \n" << singleConvolution;

  // Second layer
  Eigen::Tensor<double, 4> secondLayer;
  Eigen::MatrixXd secondLayerMatrix;
  secondLayer = neuralNetwork.convolutionInnerLayer();
  secondLayerMatrix = neuralNetwork.extract2DMatrix(0,0,secondLayer);
  std::cout << "\n Second layer \n" << secondLayerMatrix;


  return 0;
}
