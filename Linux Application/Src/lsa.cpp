#include "lsa.hpp"

LSAHandle::LSAHandle(float l1_input):ifLSACalculationFinished(0)
{
    L1Length = l1_input;
}

void LSAHandle::InitAandB(void)
{
    A_knee.conservativeResize(1, 2);
    A_knee << 0, 0;
    A_hip.conservativeResize(1, 3);
    A_hip << 0, 0, 0;
    b_hip.conservativeResize(1, 1);
    b_knee.conservativeResize(1, 1);
    b_hip << 0;
    b_knee << 0;
}

void LSAHandle::KneeJointMovementLSA(std::string filename)
{
    std::ifstream kneeFile;
    kneeFile.open(filename.data());
    uint64_t dataIndex = 0;
    uint64_t matrixRowIndex = 0;
    while(kneeFile.good())
    {
        std::string line;
        getline(kneeFile, line);
//        std::cout<<"New line: " << line<<std::endl;
//        std::cout<<"Line length: "<<line.length()<<std::endl;
        if (!line.length())
        {
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> inverse;
            inverse = A_knee.transpose()*A_knee;
            inverse = inverse.inverse();
            X_knee = inverse * A_knee.transpose() * b_knee;
            output_J2.f = X_knee[0];
            output_X2.f = X_knee[1];
            std::cout<<"Knee LSA finished. Size of data rows: "<< matrixRowIndex << std::endl;
            std::cout<<"J2 = " << output_J2.f <<std::endl;
            std::cout<<"X2 = " << output_X2.f<<std::endl;
            kneeFile.close();
            break;
        }
        if(dataIndex)
        {
            float theta0, theta1, theta1Vel, theta1Acc, \
                  theta2, theta2Vel, theta2Acc, \
                  torque1, torque2;
            
            std::string tempStr;
            std::stringstream curDataSlotStringStream(line);
            getline(curDataSlotStringStream, tempStr, ',');//Index
            getline(curDataSlotStringStream, tempStr, ',');//Time(ms)
            getline(curDataSlotStringStream, tempStr, ',');//Theda0
            theta0 = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta1 = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta1Vel = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta1Acc = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta2 = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta2Vel = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta2Acc = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            torque1 = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            torque2 = atof(tempStr.c_str());

            A_knee.conservativeResize(matrixRowIndex + 1, 2);
            b_knee.conservativeResize(matrixRowIndex + 1);

            A_knee(matrixRowIndex, 0) = theta1Acc + theta2Acc;
            A_knee(matrixRowIndex, 1) = L1Length * theta1Acc * cos(theta2) + L1Length * powf32(theta1Vel, 2.0f) * sin(theta2) -GRAVITY_ACCELERATION * sin(theta0 + theta1 + theta2);
            b_knee(matrixRowIndex) = torque2;
            matrixRowIndex++;
        }
        dataIndex++;
    }
}

void LSAHandle::HipJointMovementLSA(std::string filename)
{
    std::ifstream hipFile;
    hipFile.open(filename.data());
    uint64_t dataIndex = 0;
    uint64_t matrixRowIndex = 0;
    while(hipFile.good())
    {
        std::string line;
        getline(hipFile, line);
//        std::cout<<"New line: " << line<<std::endl;
//        std::cout<<"Line length: "<<line.length()<<std::endl;
        if (!line.length())
        {
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> inverse;
            inverse = A_hip.transpose()*A_hip;
            inverse = inverse.inverse();
            X_hip = inverse * A_hip.transpose() * b_hip;
            output_J1.f = X_hip[0];
            output_X1.f = X_hip[1];
            output_m2.f = X_hip[2];
            std::cout<<"Hip LSA finished. Size of data rows: "<< matrixRowIndex << std::endl;
            std::cout<<"J1 = " << output_J1.f <<std::endl;
            std::cout<<"X1 = " << output_X1.f <<std::endl;
            std::cout<<"m2 = " << output_m2.f <<std::endl;
            hipFile.close();
            break;
        }
        if(dataIndex)
        {
            float theta0, theta1, theta1Vel, theta1Acc, \
                  theta2, theta2Vel, theta2Acc, \
                  torque1, torque2;
            
            std::string tempStr;
            std::stringstream curDataSlotStringStream(line);
            getline(curDataSlotStringStream, tempStr, ',');//Index
            getline(curDataSlotStringStream, tempStr, ',');//Time(ms)
            getline(curDataSlotStringStream, tempStr, ',');//Theda0
            theta0 = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta1 = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta1Vel = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta1Acc = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta2 = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta2Vel = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            theta2Acc = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            torque1 = atof(tempStr.c_str());
            getline(curDataSlotStringStream, tempStr, ',');
            torque2 = atof(tempStr.c_str());

            A_hip.conservativeResize(matrixRowIndex + 1, 3);
            b_hip.conservativeResize(matrixRowIndex + 1);

            A_hip(matrixRowIndex, 0) = theta1Acc;
            A_hip(matrixRowIndex, 1) = -GRAVITY_ACCELERATION * sin(theta0 + theta1);
            A_hip(matrixRowIndex, 2) = powf32(L1Length, 2.0f) * theta1Acc - GRAVITY_ACCELERATION * L1Length * sin(theta0 + theta1);
            b_hip(matrixRowIndex) = torque1 + output_X2.f * (L1Length * theta2Vel * (2.0f * theta1Vel + theta2Vel) * sin(theta2) \
                                                         - L1Length * theta2Acc * cos(theta2) \
                                                         - 2.0f * L1Length * theta1Acc * cos(theta2) + GRAVITY_ACCELERATION * sin(theta0 + theta1 + theta2)) \
                                                         - output_J2.f * (theta1Acc + theta2Acc);
            matrixRowIndex++;
        }
        dataIndex++;
    }
}

void LSAHandle::UpdateLSAResultTxBuf(void)
{
      uint8_t ptr = 0;
      lsaUSBTxBuf[ptr++] = 'L';
      lsaUSBTxBuf[ptr++] = 'S';
      lsaUSBTxBuf[ptr++] = 'A';
      lsaUSBTxBuf[ptr++] = output_J1.b8[0];
      lsaUSBTxBuf[ptr++] = output_J1.b8[1];
      lsaUSBTxBuf[ptr++] = output_J1.b8[2];
      lsaUSBTxBuf[ptr++] = output_J1.b8[3];
      lsaUSBTxBuf[ptr++] = output_X1.b8[0];
      lsaUSBTxBuf[ptr++] = output_X1.b8[1];
      lsaUSBTxBuf[ptr++] = output_X1.b8[2];
      lsaUSBTxBuf[ptr++] = output_X1.b8[3];
      lsaUSBTxBuf[ptr++] = output_J2.b8[0];
      lsaUSBTxBuf[ptr++] = output_J2.b8[1];
      lsaUSBTxBuf[ptr++] = output_J2.b8[2];
      lsaUSBTxBuf[ptr++] = output_J2.b8[3];
      lsaUSBTxBuf[ptr++] = output_X2.b8[0];
      lsaUSBTxBuf[ptr++] = output_X2.b8[1];
      lsaUSBTxBuf[ptr++] = output_X2.b8[2];
      lsaUSBTxBuf[ptr++] = output_X2.b8[3];
      lsaUSBTxBuf[ptr++] = output_m2.b8[0];
      lsaUSBTxBuf[ptr++] = output_m2.b8[1];
      lsaUSBTxBuf[ptr++] = output_m2.b8[2];
      lsaUSBTxBuf[ptr++] = output_m2.b8[3];
}
