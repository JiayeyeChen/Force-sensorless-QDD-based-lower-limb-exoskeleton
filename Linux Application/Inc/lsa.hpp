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
    uint8_t lsaUSBTxBuf[19];
    void UpdateLSAResultTxBuf(void);
    
    Eigen::Matrix<float, Eigen::Dynamic, 2>     A_knee {{0,0},{0,0}};
    Eigen::Matrix<float, Eigen::Dynamic, 3>     A_hip {{0,0},{0,0}, {0,0}};
    Eigen::Vector2f                             X_knee {0,0};
    Eigen::Vector3f                             X_hip {0,0};
    Eigen::VectorXf                             b {0};
    float                                       L1Length;
    FloatUInt8                                  output_J2;
    FloatUInt8                                  output_X2;
    FloatUInt8                                  output_J1;
    FloatUInt8                                  output_X1;
    FloatUInt8                                  output_m2;
    private:

};





#endif
