#include <main.hpp>

bool ifExit = false;

void USB_RxCargo(USBCommunicationHandle* husbcom)
{
    while(1)
    {
        husbcom->ReceiveCargo();
    }
}

void USB_RxCargoProcessing(USBCommunicationHandle* husbcom, ExoskeletonHandle* hexoskeleton, char* filename)
{
    std::cin.get();
    ifExit = true;
    // while(1)
    // {
        
    //     // if (husbcom->ifNewMessage)
    //     // {
    //     //     std::string msg((const char*)husbcom->rxMessageCfrm, husbcom->rxMessageLen);
    //     //     if (!msg.compare("Datalog start"))
    //     //         husbcom->StartDataLogPassive(filename);
    //     //     else if (!msg.compare("System ID start request"))
    //     //     {
    //     //         husbcom->SendText("SystemID start noted");
    //     //         hexoskeleton->curMainTask = EXOSKELETON_MAIN_TASK_SYSTEM_ID;
    //     //         hexoskeleton->curSubTask = EXOSKELETON_SUB_TASK_SYSTEMID_LOG_KNEE_JOINT_DATA;
    //     //         husbcom->StartDataLogPassive("SysID Knee.csv");
    //     //         std::cout<<"System ID started! Logging Knee Joint Movement Data"<<std::endl;
    //     //     }


    //     //     husbcom->DataLogManager();
    //     //     husbcom->ifNewMessage = 0;
    //     // }
    // }

}

int main(int argc, char** argv)
{
    USBCommunicationHandle hUSBCom(argv[1], 921600);
    ExoskeletonHandle hExoskeleton(10);
    std::thread Thread_USB_RxCargo(USB_RxCargo, &hUSBCom);
    std::thread THread_USB_RxCargoProcessing(USB_RxCargoProcessing, &hUSBCom, &hExoskeleton, argv[2]);

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
                if(hUSBCom.ifNewMsgIsThisString("Pls calculate results"))
                {
                    hUSBCom.fileStream.close();
                    hExoskeleton.curSubTask = EXOSKELETON_SUB_TASK_SYSTEMID_LEAST_SQUARE_APPROXIMATION;
                }
            }
                break;
            case EXOSKELETON_SUB_TASK_SYSTEMID_LEAST_SQUARE_APPROXIMATION:
            {
                std::cout<<"Lets do LSA now!!"<<std::endl;
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
