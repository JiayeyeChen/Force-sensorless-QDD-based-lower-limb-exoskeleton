#include <main.hpp>

void USB_RxCargo(USBCommunicationHandle* husbcom)
{
    while(1)
    {
        husbcom->ReceiveCargo();
    }
}

void USB_RxCargoProcessing(USBCommunicationHandle* husbcom, ExoskeletonHandle* hexoskeleton, char* filename)
{
    while(1)
    {
        // if (husbcom->ifNewMessage)
        // {
        //     std::string msg((const char*)husbcom->rxMessageCfrm, husbcom->rxMessageLen);
        //     if (!msg.compare("Datalog start"))
        //         husbcom->StartDataLogPassive(filename);
        //     else if (!msg.compare("System ID start request"))
        //     {
        //         husbcom->SendText("SystemID start noted");
        //         hexoskeleton->curMainTask = EXOSKELETON_MAIN_TASK_SYSTEM_ID;
        //         hexoskeleton->curSubTask = EXOSKELETON_SUB_TASK_SYSTEMID_LOG_KNEE_JOINT_DATA;
        //         husbcom->StartDataLogPassive("SysID Knee.csv");
        //         std::cout<<"System ID started! Logging Knee Joint Movement Data"<<std::endl;
        //     }


        //     husbcom->DataLogManager();
        //     husbcom->ifNewMessage = 0;
        // }
    }

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
        switch (hExoskeleton.curMainTask)
        {
        case EXOSKELETON_MAIN_TASK_FREE:
        {
            if(hUSBCom.ifNewMsgIsThisString("System ID start request"))
            {
                hUSBCom.SendText("SystemID start noted");
                hExoskeleton.curMainTask = EXOSKELETON_MAIN_TASK_SYSTEM_ID;
                hExoskeleton.curSubTask = EXOSKELETON_SUB_TASK_SYSTEMID_LOG_KNEE_JOINT_DATA;
                hUSBCom.StartDataLogPassive("SysID Knee.csv");
                std::cout<<"System ID started! Logging Knee Joint Movement Data"<<std::endl;
            }
        }
            break;
        case EXOSKELETON_MAIN_TASK_SYSTEM_ID:
        {
            switch (hExoskeleton.curSubTask)
            {
            case EXOSKELETON_SUB_TASK_SYSTEMID_LOG_KNEE_JOINT_DATA:
            {

            }
                break;
            case EXOSKELETON_SUB_TASK_SYSTEMID_LOG_HIP_JOINT_DATA:
            {

            }
                break;
            case EXOSKELETON_SUB_TASK_SYSTEMID_LEAST_SQUARE_APPROXIMATION:
            {

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
    }
}
