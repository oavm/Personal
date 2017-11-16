
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include </home/octaviovillarreal/Git/Personal/CNNCpp/Eigen/Dense>
#include </home/octaviovillarreal/Git/Personal/CNNCpp/unsupported/Eigen/CXX11/Tensor>
#include </home/octaviovillarreal/Git/Personal/CNNCpp/unsupported/Eigen/CXX11/Tensor>
#include <vector>
#include <set>
#include <string>
#include <istream>
#include <cstdlib>
#include <sstream>

#include </home/octaviovillarreal/Git/Personal/CNNCpp/csvRead.h>
#include </home/octaviovillarreal/Git/Personal/CNNCpp/tensorbuild.h>
#include </home/octaviovillarreal/Git/Personal/CNNCpp/HyQNN.h>

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

  // Obtain matrix containing kernels from csv file
  CsvReadFile weight0;
  Eigen::MatrixXd weight_matrix;
  weight0.set_csv("LF-SimpCNN-ffnn_part-layer_0-W.csv");
  weight_matrix = weight0.createMatrix();
  // std::cout << "\n Weight 0: \n" << weight_matrix;

  // Obtain matrix containing kernels from csv file
  CsvReadFile weight1;
  Eigen::MatrixXd weight_matrix1;
  weight1.set_csv("LF-SimpCNN-ffnn_part-layer_1-W.csv");
  weight_matrix1 = weight1.createMatrix();
  // std::cout << "\n Weight 1: \n" << weight_matrix1;

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

  // Obtain vector of bias
  CsvReadFile b2;
  Eigen::MatrixXd bias2;
  b2.set_csv("LF-SimpCNN-ffnn_part-layer_0-b.csv");
  bias2 = b2.createMatrix();
  // std::cout << "\nbias 2: \n" << bias2;

  // Obtain vector of bias
  CsvReadFile b3;
  Eigen::MatrixXd bias3;
  b3.set_csv("LF-SimpCNN-ffnn_part-layer_1-b.csv");
  bias3 = b3.createMatrix();
  // std::cout << "\nbias 3: \n" << bias3;

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
  neuralNetwork.set_neuralNetwork(K0,K1,bias0,bias1,bias2,bias3,weight_matrix,weight_matrix1,image_matrix);
  tensorConvolution = neuralNetwork.convolution(image_matrix,K0);
  tensorConvolution_0 = neuralNetwork.extract2DMatrix(0,0,tensorConvolution);
  // std::cout << "\n Filtered image \n" << tensorConvolution_0;

  // Activation function ReLu
  Eigen::MatrixXd poslin(image_size(0),image_size(1));
  Eigen::Tensor<double, 4> tensorReLu(size_tensor(0),size_tensor(1),size_tensor(2),size_tensor(3));
  tensorReLu = neuralNetwork.reLu(tensorConvolution);
  poslin = neuralNetwork.extract2DMatrix(0,0,tensorReLu);
  // std::cout << "\n Filtered image with activation \n" << poslin;

  // Apply Maxpool operation
  Eigen::MatrixXd maxPoolMatrix;
  Eigen::MatrixXd maxpool_array[size_tensor(2)][size_tensor(3)];
  Eigen::Tensor<double, 4> tensorMaxPool(size_tensor(0),size_tensor(1),size_tensor(2),size_tensor(3));
  tensorMaxPool = neuralNetwork.maxPool(tensorReLu);
  maxPoolMatrix = neuralNetwork.extract2DMatrix(0,2,tensorMaxPool);
  // std::cout << "\n Matrix after maxpool \n" << maxPoolMatrix;

  // Test for single convolution
  Eigen::MatrixXd singleConvolution;
  Eigen::MatrixXd testKernel;
  testKernel = neuralNetwork.extract2DMatrix(0,0,K0);
  singleConvolution = neuralNetwork.singleConvolution(image_matrix,testKernel);
  // std::cout << "\n Single convolution \n" << singleConvolution;

  // Second layer
  // Convolution
  Eigen::Tensor<double, 4> secondLayer;
  Eigen::MatrixXd secondLayerMatrix;
  secondLayer = neuralNetwork.convolutionInnerLayer(tensorMaxPool);
  secondLayerMatrix = neuralNetwork.extract2DMatrix(0,7,secondLayer);
  // std::cout << "\n Second layer \n" << secondLayerMatrix;

  // ReLu
  Eigen::MatrixXd poslin2;
  Eigen::Tensor<double, 4> tensorReLu2(secondLayer.dimension(0),secondLayer.dimension(1),secondLayer.dimension(2),secondLayer.dimension(3));
  tensorReLu2 = neuralNetwork.reLu(secondLayer);
  poslin2 = neuralNetwork.extract2DMatrix(0,7,tensorReLu2);
  // std::cout << "\n Filtered image with activation \n" << poslin2;

  // maxPool
  Eigen::MatrixXd maxPoolMatrix2;
  Eigen::Tensor<double, 4> tensorMaxPool2;
  tensorMaxPool2 = neuralNetwork.maxPool(tensorReLu2);
  maxPoolMatrix2 = neuralNetwork.extract2DMatrix(0,0,tensorMaxPool2);
  // std::cout << "\n Matrix after maxpool \n" << maxPoolMatrix2;

  // Reshape
  Eigen::MatrixXd reshapeVector;
  reshapeVector = neuralNetwork.reshapeToVector(tensorMaxPool2);
  // std::cout << "\n Reshape vector \n" << reshapeVector;

  //Feedforward layer
  Eigen::MatrixXd::Index Feedforward;
  Feedforward = neuralNetwork.feedForward(reshapeVector);
  // std::cout << "\n Feedforward \n" << Feedforward;

  int prediction;
  prediction = neuralNetwork.predictOutput();
  std::cout << "\n Prediction: " << prediction;


  return 0;
}
