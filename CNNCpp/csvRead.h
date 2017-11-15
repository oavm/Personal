#ifndef _csvRead_H_
#define _csvRead_H_

class CsvReadFile
{
  std::string filename;

public:

  void set_csv (std::string a)
  {
    filename = a;
  }


  Eigen::MatrixXd createMatrix()
  {

      std::ifstream file;
      std::ifstream file2;
      std::string line;

      int rows = 0;
      int cols = 0;

      file.open(filename.c_str());
      file2.open(filename.c_str());

      while (std::getline(file, line))
      {
        cols = 0;
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ','))
        {
          std::istringstream os(cell);
          double d;
          os >> d;
            ++cols;
        }
        ++rows;
      }

      Eigen::MatrixXd matrix(rows,cols);

      rows = 0;
      cols = 0;

      while (std::getline(file2, line))
      {
        cols = 0;
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ','))
        {
          std::istringstream os(cell);
          double d;
          os >> d;
            matrix(rows,cols) = d;
            ++cols;
        }
        ++rows;
      }
      return matrix;
  };



};

  #endif /*_csvRead_H_*/
