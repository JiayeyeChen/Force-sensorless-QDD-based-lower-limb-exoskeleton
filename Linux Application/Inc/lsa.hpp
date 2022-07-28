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
    uint8_t ifLSACalculationFinished;
    void UpdateLSAResultTxBuf(void);
    
    Eigen::Matrix<float, Eigen::Dynamic, 2>     A_knee;
    Eigen::Matrix<float, Eigen::Dynamic, 2>     A_hip;
    Eigen::Vector2f                             X_knee {0,0};
    Eigen::Vector3f                             X_hip {0,0,0};
    Eigen::VectorXf                             b_knee {0};
    Eigen::VectorXf                             b_hip {0};
    float                                       L1Length;
    FloatUInt8                                  output_J2;
    FloatUInt8                                  output_X2;
    FloatUInt8                                  output_J1;
    FloatUInt8                                  output_X1;
    private:

};





#endif
