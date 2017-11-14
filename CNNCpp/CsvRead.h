#ifndef _csvRead_H_
#define _csvRead_H_

class CsvRead
{
  string filename;

public:

  void set_csv (string a)
  {
    filename = a;
  }


  Eigen::MatrixXd createMatrix()
  {

      std::ifstream file;

      std::string line;

      int rows = 0;
      int cols = 0;

      file.open(filename);

      while (std::getline(file, line))
      {
        cols = 0;
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ','))
        {
          istringstream os(cell);
          double d;
          os >> d;
            ++cols;
        }
        ++rows;
      }

      Eigen::MatrixXd matrix(rows,cols);

      while (std::getline(file, line))
      {
        cols = 0;
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ','))
        {
          istringstream os(cell);
          double d;
          os >> d;
            matrix(rows,cols) = d;
            ++cols;
        }
        ++rows;
      }

      return matrix;


  }



}
