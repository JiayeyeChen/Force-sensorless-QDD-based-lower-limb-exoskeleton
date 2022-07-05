#include "lsa.hpp"

LSAHandle::LSAHandle(float l1_input)
{
    L1Length = l1_input;
}

void LSAHandle::InitAandB(void)
{
    A.conservativeResize(1, 2);
    A << 0, 0;
    b.conservativeResize(1, 1);
    b << 0;
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
            inverse = A.transpose()*A;
            inverse = inverse.inverse();
            X = inverse * A.transpose() * b;
            output_a2.f = X[0]/X[1];
            output_m2.f = X[1]/output_a2.f;
            std::cout<<"Knee LSA finished. Size of data rows: "<< matrixRowIndex << std::endl;
            std::cout<<"a2 = " << output_a2.f << " m"<<std::endl;
            std::cout<<"m2 = " << output_m2.f<< " kg" <<std::endl;
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

            A.conservativeResize(matrixRowIndex + 1, 2);
            b.conservativeResize(matrixRowIndex + 1);

            A(matrixRowIndex, 0) = theta1Acc + theta2Acc;
            A(matrixRowIndex, 1) = L1Length * theta1Acc * cos(theta2) + L1Length * powf32(theta1Vel, 2) * sin(theta2) -GRAVITY_ACCELERATION * sin(theta0 + theta1 + theta2);
            b(matrixRowIndex) = torque2;
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
            inverse = A.transpose()*A;
            inverse = inverse.inverse();
            X = inverse * A.transpose() * b;
            output_a1.f = X[0]/X[1];
            output_m1.f = X[1]/output_a1.f;
            std::cout<<"Hip LSA finished. Size of data rows: "<< matrixRowIndex << std::endl;
            std::cout<<"a1 = " << output_a1.f << " m"<<std::endl;
            std::cout<<"m1 = " << output_m1.f<< " kg" <<std::endl;
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

            A.conservativeResize(matrixRowIndex + 1, 2);
            b.conservativeResize(matrixRowIndex + 1);

            A(matrixRowIndex, 0) = theta1Acc;
            A(matrixRowIndex, 1) = -GRAVITY_ACCELERATION * sin(theta0 + theta1);
            b(matrixRowIndex) = torque1 + GRAVITY_ACCELERATION * output_m2.f * output_a2.f * sin(theta0+theta1+theta2) + \
                                GRAVITY_ACCELERATION * output_m2.f * L1Length * sin(theta0+theta1) + theta2Vel*(2*theta1Vel+theta2Vel)*output_m2.f*L1Length*output_a2.f*sin(theta2) - \
                                output_m2.f*output_a2.f*L1Length*theta2Acc*cos(theta2) - output_m2.f*powf32(output_a2.f,2)*theta2Acc - 2*output_m2.f*L1Length*output_a2.f*theta1Acc*cos(theta2) - \
                                output_m2.f*powf32(output_a2.f, 2)*theta1Acc - output_m2.f*powf32(L1Length, 2)*theta1Acc;
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
      lsaUSBTxBuf[ptr++] = output_a1.b8[0];
      lsaUSBTxBuf[ptr++] = output_a1.b8[1];
      lsaUSBTxBuf[ptr++] = output_a1.b8[2];
      lsaUSBTxBuf[ptr++] = output_a1.b8[3];
      lsaUSBTxBuf[ptr++] = output_m1.b8[0];
      lsaUSBTxBuf[ptr++] = output_m1.b8[1];
      lsaUSBTxBuf[ptr++] = output_m1.b8[2];
      lsaUSBTxBuf[ptr++] = output_m1.b8[3];
      lsaUSBTxBuf[ptr++] = output_a2.b8[0];
      lsaUSBTxBuf[ptr++] = output_a2.b8[1];
      lsaUSBTxBuf[ptr++] = output_a2.b8[2];
      lsaUSBTxBuf[ptr++] = output_a2.b8[3];
      lsaUSBTxBuf[ptr++] = output_m2.b8[0];
      lsaUSBTxBuf[ptr++] = output_m2.b8[1];
      lsaUSBTxBuf[ptr++] = output_m2.b8[2];
      lsaUSBTxBuf[ptr++] = output_m2.b8[3];
}
