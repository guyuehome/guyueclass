#include <iostream>
#include <Eigen/Dense>
using Eigen::Matrix2d;
int main()
{
  Matrix2d m;
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;

  Matrix2d m2 = m*m;
  std::cout << m2 << std::endl;
}
