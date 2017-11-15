#ifndef _HyQNN_H_
#define _HyQNN_H_

class NeuralNetwork
{
  Eigen::Tensor<double, 4> K0;
  Eigen::Tensor<double, 4> K1;
  Eigen::MatrixXd image,b0,b1;


public:
  void set_neuralNetwork (Eigen::Tensor<double, 4> a,Eigen::Tensor<double, 4> b, Eigen::MatrixXd c, Eigen::MatrixXd d, Eigen::MatrixXd e)
  {
    K0 = a;
    K1 = b;
    b0 = c;
    b1 = d;
    image = e;
  }

  Eigen::MatrixXd extract2DMatrix(int index_0, int index_1,Eigen::Tensor<double, 4> tensorMatrix)
  {
    int matrixHeight,matrixWidth,arrayHeight,arrayWidth;
    matrixHeight = tensorMatrix.dimension(0);
    matrixWidth = tensorMatrix.dimension(1);


    Eigen::MatrixXd matrixOut(matrixHeight,matrixWidth);

    for (int i = 0; i <= matrixHeight - 1; i++)
    {
      for (int j = 0; j <= matrixWidth - 1; j++)
      {
        matrixOut(i,j) = tensorMatrix(i,j,index_0,index_1);
      }
    }

    return matrixOut;

  }

  Eigen::MatrixXd singleConvolution(Eigen::MatrixXd image,Eigen::MatrixXd kernel)
  {
    int padHeight,padWidth,imageHeight,imageWidth,kernelHeight,kernelWidth,padTop,padLeft;
    double element;

    imageHeight = image.rows();
    imageWidth = image.cols();
    kernelHeight = kernel.rows();
    kernelWidth = kernel.cols();

    Eigen::MatrixXd singleConvolution(imageHeight,imageWidth);

    padHeight = ((imageHeight - 1) + kernelHeight - imageHeight);
    padWidth = ((imageWidth - 1) + kernelWidth - imageWidth);

    padTop = padHeight / 2;
    padLeft = padWidth / 2;

    Eigen::MatrixXd image_padding_hor = Eigen::MatrixXd::Zero(padTop,imageWidth + padWidth);
    Eigen::MatrixXd image_padding_ver = Eigen::MatrixXd::Zero(imageHeight,padLeft);
    Eigen::MatrixXd image_extended(imageHeight + padHeight,imageWidth + padWidth);

    image_extended << image_padding_hor,
                      image_padding_ver,image,image_padding_ver,
                      image_padding_hor;

    for (int i = 0; i <= imageHeight - 1; i++)
    {
      for (int j = 0; j <= imageWidth - 1; j++)
      {
        element = image_extended.block(i,j,kernelHeight,kernelWidth).cwiseProduct(kernel).sum();
        singleConvolution(i,j) = element;
      }
    }
    return singleConvolution;
  }

  Eigen::Tensor<double, 4> convolution(Eigen::MatrixXd image,Eigen::Tensor<double, 4> K0)
  {
    int padHeight,padWidth,imageHeight,imageWidth,kernelHeight,kernelWidth,padTop,padLeft;
    Eigen::MatrixXd K0_0(K0.dimension(0),K0.dimension(1));
    Eigen::MatrixXd kernelMatrix[K0.dimension(2)][K0.dimension(3)];

    for (int l = 0; l <= K0.dimension(3) - 1; l++)
    {
      for (int k = 0; k <= K0.dimension(2) - 1; k++)
      {
        for (int i = 0; i <= K0.dimension(0) - 1; i++)
        {
          for (int j = 0; j <= K0.dimension(1) - 1; j++)
          {
            K0_0(i,j) = K0(i,j,k,l);
          }
        }
        kernelMatrix[k][l] = K0_0;
      }
    }

    imageHeight = image.rows();
    imageWidth = image.cols();
    kernelHeight = K0.dimension(0);
    kernelWidth = K0.dimension(1);

    Eigen::Tensor<double, 4> filteredImage(imageHeight,imageWidth,K0.dimension(2),K0.dimension(3));

    padHeight = ((imageHeight - 1) + kernelHeight - imageHeight);
    padWidth = ((imageWidth - 1) + kernelWidth - imageWidth);

    padTop = padHeight / 2;
    padLeft = padWidth / 2;

    Eigen::MatrixXd image_padding_hor = Eigen::MatrixXd::Zero(padTop,imageWidth + padWidth);
    Eigen::MatrixXd image_padding_ver = Eigen::MatrixXd::Zero(imageHeight,padLeft);
    Eigen::MatrixXd image_extended(imageHeight + padHeight,imageWidth + padWidth);

    image_extended << image_padding_hor,
                      image_padding_ver,image.transpose(),image_padding_ver,
                      image_padding_hor;



    Eigen::MatrixXd bias0[1][K0.dimension(3)];
    Eigen::MatrixXd onesMatrix;

    onesMatrix = Eigen::MatrixXd::Ones(imageHeight,imageWidth);
    // Eigen::MatrixXd::Ones(imageHeight,imageWidth) onesMatrix;
    for (int i = 0; i <= b0.rows() - 1; i++)
    {
      bias0[0][i] = b0(i)*onesMatrix;
    }


    double element;
    Eigen::MatrixXd filtered_image(imageHeight,imageWidth);
    Eigen::MatrixXd filtered_image_array[kernelHeight][kernelWidth];

    // Convolution
    for (int l = 0; l <= K0.dimension(3) - 1; l++)
    {
      for (int k = 0; k <= K0.dimension(2) - 1; k++)
      {
        for (int i = 0; i <= imageHeight - 1; i++)
        {
          for (int j = 0; j <= imageWidth - 1; j++)
          {
            element = image_extended.block(i,j,K0.dimension(0),K0.dimension(1)).cwiseProduct(kernelMatrix[k][l]).sum() + bias0[k][l](i,j);
            filteredImage(i,j,k,l) = element;
          }
        }
      }
    }

    return filteredImage;
  }

  Eigen::Tensor<double, 4> reLu()
  {
    int kernelHeight,kernelWidth,imageHeight,imageWidth;
    imageHeight = image.rows();
    imageWidth = image.cols();

    Eigen::Tensor<double, 4> posLinTensor(imageHeight,imageWidth,K0.dimension(2),K0.dimension(3));
    Eigen::Tensor<double, 4> filteredIm(imageHeight,imageWidth,K0.dimension(2),K0.dimension(3));

    filteredIm = convolution(image,K0);

    double poslin_num;

    for (int l = 0; l <= K0.dimension(3) - 1; l++)
    {
      for (int k = 0; k <= K0.dimension(2) - 1; k++)
      {
        for (int i = 0; i <= imageHeight - 1; i++)
        {
          for (int j = 0; j <= imageWidth - 1; j++)
          {
            poslin_num = filteredIm(i,j,k,l);
            if (poslin_num > 0)
            {
              posLinTensor(i,j,k,l) = filteredIm(i,j,k,l);
            }
            else
            {
              posLinTensor(i,j,k,l) = 0;
            }
          }
        }
      }
    }
    return posLinTensor;
  }

  Eigen::Tensor<double, 4> maxPool()
  {
    int imageHeight,imageWidth;
    imageHeight = image.rows();
    imageWidth = image.cols();

    Eigen::Tensor<double, 4> maxPoolTensor((imageHeight + imageHeight%2)/2,(imageHeight + imageHeight%2)/2,K0.dimension(2),K0.dimension(3));
    Eigen::Tensor<double, 4> posLinIm(imageHeight,imageWidth,K0.dimension(2),K0.dimension(3));

    posLinIm = reLu();

    Eigen::MatrixXd maxPoolMatrix;
    Eigen::MatrixXd maxpool_extended(imageHeight + imageHeight%2,imageWidth + imageWidth%2);
    Eigen::MatrixXd maxpool_padding_hor = Eigen::MatrixXd::Zero(imageHeight%2,imageWidth + imageWidth%2);
    Eigen::MatrixXd maxpool_padding_ver = Eigen::MatrixXd::Zero(imageHeight,imageWidth%2);

    double maxPoolElement;

    for (int l = 0; l <= K0.dimension(3) - 1; l++)
    {
      for (int k = 0; k <= K0.dimension(2) - 1; k++)
      {
        maxPoolMatrix = extract2DMatrix(k,l,posLinIm);
        maxpool_extended << maxPoolMatrix,maxpool_padding_ver,
                            maxpool_padding_hor;
        for (int i = 0; i <= (imageHeight + imageHeight%2)/2 - 1; i++)
        {
          for (int j = 0; j <= (imageWidth + imageWidth%2)/2 - 1; j++)
          {
            maxPoolElement = maxpool_extended.block(2*i,2*j,2,2).maxCoeff();
            maxPoolTensor(i,j,k,l) = maxPoolElement;
          }
        }
      }
    }
    return maxPoolTensor;
  }

  Eigen::Tensor<double, 4> convolutionInnerLayer()
  {
    Eigen::MatrixXd maxPoolMatrix;
    Eigen::MatrixXd kernelMatrix;
    Eigen::MatrixXd singleConvolutions;
    Eigen::MatrixXd sumMatrix;
    Eigen::MatrixXd bias1[1][K1.dimension(3)];
    Eigen::MatrixXd onesMatrix;

    int imageHeight,imageWidth;
    double element;
    imageHeight = image.rows();
    imageWidth = image.cols();

    onesMatrix = Eigen::MatrixXd::Ones(imageHeight,imageWidth);
    for (int i = 0; i <= b1.rows() - 1; i++)
    {
      bias1[0][i] = b1(i)*onesMatrix;
    }

    Eigen::Tensor<double, 4> maxPoolTensor((imageHeight + imageHeight%2)/2,(imageHeight + imageHeight%2)/2,K0.dimension(2),K0.dimension(3));
    Eigen::Tensor<double, 4> convolutionTensor((imageHeight + imageHeight%2)/2,(imageHeight + imageHeight%2)/2,K0.dimension(2),K1.dimension(3));
    Eigen::MatrixXd convolutionArray[K0.dimension(2)][K1.dimension(2)];
    Eigen::MatrixXd convolutionArray2[K0.dimension(2)][K1.dimension(3)];



    maxPoolTensor = maxPool();

    for (int j = 0; j <= K1.dimension(3) - 1; j++)
    {
      for (int i = 0; i<= K1.dimension(2) - 1; i++)
      {
        maxPoolMatrix = extract2DMatrix(0,i,maxPoolTensor);
        kernelMatrix = extract2DMatrix(i,j,K1);
        singleConvolutions = singleConvolution(maxPoolMatrix,kernelMatrix);
        convolutionArray[0][i] = singleConvolutions;
      }
      sumMatrix = convolutionArray[0][0] + convolutionArray[0][1] + convolutionArray[0][2] + convolutionArray[0][3];
      convolutionArray2[0][j] = sumMatrix;
    }

    // std::cout << "\nConvolution array\n" << maxPoolTensor.dimension(1);

    for (int l = 0; l <= K1.dimension(3) - 1; l++)
    {
      for (int k = 0; k <= K0.dimension(2) - 1; k++)
      {
        for (int i = 0; i <= maxPoolTensor.dimension(0) - 1; i++)
        {
          for (int j = 0; j<= maxPoolTensor.dimension(1) - 1; j++)
          {
            // element = convolutionArray2[k][l](i,j);
            convolutionTensor(i,j,k,l) = convolutionArray2[k][l](i,j) + bias1[k][l](i,j);
          }
        }
      }
    }

    // std::cout << "\nConvolution array\n" << element;

    return convolutionTensor;

  }


};
  #endif /*_HyQNN_H_*/
