
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

  // Obtain vector of bias
  CsvReadFile b0;
  Eigen::MatrixXd bias0;
  b0.set_csv("LF-SimpCNN-conv_part-layer_0-bias.csv");
  bias0 = b0.createMatrix();
  // std::cout << "\nbias 0: \n" << bias0;

  // Create kernel tensor
  TensorBuild kernel0Tensor;
  Eigen::MatrixXi size_tensor(1,4); // Define size of the Tensor
  size_tensor << 5,5,1,4;
  Eigen::Tensor<double, 4> K0(size_tensor(0),size_tensor(1),size_tensor(2),size_tensor(3)); // Define tensor for the kernel
  Eigen::MatrixXd kernelTest;
  kernel0Tensor.set_tensorData(value_matrix,"SimpCNN-conv_part-layer_0-kernel_INDICES.csv",size_tensor);
  K0 = kernel0Tensor.createTensor();
  // std::cout << "\n Kernel 0 0: \n" << kernel0Tensor.createTensor();

  // Perform convolution with all kernels
  NeuralNetwork neuralNetwork;
  Eigen::Tensor<double, 4> tensorConvolution(size_tensor(0),size_tensor(1),size_tensor(2),size_tensor(3));
  Eigen::MatrixXd tensorConvolutionMatrix[size_tensor(2)][size_tensor(3)]; // Use tensor size definition to build kernel matrix
  Eigen::MatrixXd tensorConvolution_0(image_matrix.rows(),image_matrix.cols());
  Eigen::MatrixXd filtered_image_array[size_tensor(2)][size_tensor(3)];
  neuralNetwork.set_neuralNetwork(K0,K0,bias0,bias0,image_matrix);
  tensorConvolution = neuralNetwork.convolution();
  tensorConvolution_0 = neuralNetwork.extract2DMatrix(0,0,tensorConvolution);
  // std::cout << "\n Filtered image \n" << tensorConvolution_0;

  // Activation function ReLu
  Eigen::MatrixXd poslin(image_size(0),image_size(1));
  Eigen::MatrixXd poslin_array[size_tensor(2)][size_tensor(3)];
  double poslin_num;
  Eigen::Tensor<double, 4> tensorReLu(size_tensor(0),size_tensor(1),size_tensor(2),size_tensor(3));
  tensorReLu = neuralNetwork.reLu();
  poslin = neuralNetwork.extract2DMatrix(0,0,tensorReLu);
  // std::cout << "\n Filtered image with activation \n" << poslin;

  Eigen::MatrixXd test;

  test = Eigen::MatrixXd::Zero(0,15);
  std::cout << "\n Filtered image with activation \n" << test;

  // Apply Maxpool operation
  // Eigen::MatrixXd maxpool_extended(image_size(0) + image_size(0)%2,image_size(1) + image_size(1)%2);
  // Eigen::MatrixXd maxpool_padding_hor = Eigen::MatrixXd::Zero(1,16);
  // Eigen::MatrixXd maxpool_padding_ver = Eigen::MatrixXd::Zero(15,1);
  // Eigen::MatrixXd maxpool_array[size_tensor(2)][size_tensor(3)];
  // double maxpoolElement,element;
  //
  // maxpool_extended << image_matrix.transpose(),maxpool_padding_ver,
  //                     maxpool_padding_hor;
  //
  // for (int l = 0; l <= size_tensor(3) - 1; l++){
  //   for (int k = 0; k <= size_tensor(2) - 1; k++){
  //     for (int i = 0; i <= (image_size(0) - 1); i++){
  //       for (int j = 0; j <= (image_size(1) - 1); j++){
  //         element = maxpool_extended.block(i,j,size_tensor(0),size_tensor(1)).cwiseProduct(kernelMatrix[k][l]).sum();
  //         filtered_image(i,j) = element;
  //       }
  //     }
  //     filtered_image_array[k][l] = filtered_image;
  //   }
  // }


  return 0;
}
