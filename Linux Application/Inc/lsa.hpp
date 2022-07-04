#ifndef LSA_HPP
#define LSA_HPP

#include <fstream>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include "main.hpp"
#include "common.hpp"
#include <sstream>

#define GRAVITY_ACCELERATION 9.781f

class LSAHandle
{
    public:
    LSAHandle(float l1_input);
    void InitAandB(void);
    void KneeJointMovementLSA(std::string filename);
    void HipJointMovementLSA(std::string filename);
    Eigen::Matrix<float, Eigen::Dynamic, 2>     A {{0,0},{0,0}};
    Eigen::Vector2f                             X {0,0};
    Eigen::VectorXf                             b {0};
    float                                       L1Length;
    FloatUInt8                                  output_a1;
    FloatUInt8                                  output_m1;
    FloatUInt8                                  output_a2;
    FloatUInt8                                  output_m2;
    private:

};





#endif
