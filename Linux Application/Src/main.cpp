#include <main.hpp>

bool ifExit = false;

void USB_RxCargo(USBCommunicationHandle* husbcom)
{
    while(1)
    {
        husbcom->ReceiveCargo();
    }
}

void Keyboard(void)
{
    std::cin.get();
    ifExit = true;
}
//argv[1]: USB device address. argv[2]: Datalog filename. argv[3]: L1 length in m
int main(int argc, char** argv)
{
    LSAHandle hLSA(atof(argv[3]));
    USBCommunicationHandle hUSBCom(argv[1], 921600);
    ExoskeletonHandle hExoskeleton(10);
    std::thread Thread_USB_RxCargo(USB_RxCargo, &hUSBCom);
    std::thread Thread_KeyboardInput(Keyboard);
    while(1)
    {
        hUSBCom.DataLogManager();

        if(hUSBCom.ifNewMsgIsThisString("System ID start request"))
        {
            hUSBCom.SendText("SystemID start noted");
            hExoskeleton.curMainTask = EXOSKELETON_MAIN_TASK_SYSTEM_ID;
            hExoskeleton.curSubTask = EXOSKELETON_SUB_TASK_SYSTEMID_LOG_KNEE_JOINT_DATA;
            hUSBCom.StartDataLogPassive("SysID Knee.csv");
            std::cout<<"System ID started! Logging Knee Joint Movement Data"<<std::endl;
        }

        switch (hExoskeleton.curMainTask)
        {
        case EXOSKELETON_MAIN_TASK_FREE:
        {

        }
            break;
        case EXOSKELETON_MAIN_TASK_SYSTEM_ID:
        {
            switch (hExoskeleton.curSubTask)
            {
            case EXOSKELETON_SUB_TASK_SYSTEMID_LOG_KNEE_JOINT_DATA:
            {
                if(hUSBCom.ifNewMsgIsThisString("Datalog start"))
                {
                    hUSBCom.StartDataLogPassive("SysID Hip.csv");
                    std::cout<<"Start Logging Hip Joint Movement Data"<<std::endl;
                    hExoskeleton.curSubTask = EXOSKELETON_SUB_TASK_SYSTEMID_LOG_HIP_JOINT_DATA;
                }
            }
                break;
            case EXOSKELETON_SUB_TASK_SYSTEMID_LOG_HIP_JOINT_DATA:
            {
                if(hUSBCom.ifNewMsgIsThisString("Receiving results from PC..."))
                {
                    hUSBCom.fileStream.close();
                    hExoskeleton.curSubTask = EXOSKELETON_SUB_TASK_SYSTEMID_LEAST_SQUARE_APPROXIMATION;
                }
            }
                break;
            case EXOSKELETON_SUB_TASK_SYSTEMID_LEAST_SQUARE_APPROXIMATION:
            {
                hLSA.KneeJointMovementLSA("SysID Knee.csv");
                hLSA.HipJointMovementLSA("SysID Hip.csv");
                hLSA.UpdateLSAResultTxBuf();
                hUSBCom.TransmitCargo(hLSA.lsaUSBTxBuf, 19);
                if(hUSBCom.ifNewMsgIsThisString("System ID end"))
                {
                    hUSBCom.SendText("Roger that");
                    hExoskeleton.curSubTask = EXOSKELETON_SUB_TASK_NONE;
                    hExoskeleton.curMainTask = EXOSKELETON_MAIN_TASK_FREE;
                }
                
                //std::cout<<"Lets do LSA now!!"<<std::endl;
            }
                break;
            default:
                break;
            }
        }
            break;
        case EXOSKELETON_MAIN_TASK_ACTIVE_CONTROL:

            break;
        default:
            break;
        }

        if(ifExit)
            break;
        else if(hUSBCom.ifNewMsgIsThisString("STOP"))
            break;
    }

    hUSBCom.fileStream.close();
    return 0;
}
