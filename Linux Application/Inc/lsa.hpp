#ifndef LSA_HPP
#define LSA_HPP

#include <fstream>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

class LSAHandle
{
    public:
    LSAHandle(void);
    Eigen::Matrix<float, 2, Eigen::Dynamic> A;
    float      X[2];
    private:

};




#endif
