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
        if (husbcom->ifNewMessage)
        {
            std::string msg((const char*)husbcom->rxMessageCfrm, husbcom->rxMessageLen);
            if (!msg.compare("Datalog start"))
                husbcom->StartDataLogPassive(filename);
            else if (!msg.compare("System ID start request"))
            {
                husbcom->SendText("SystemID start noted");
                hexoskeleton->curMainTask = EXOSKELETON_MAIN_TASK_SYSTEM_ID;
                std::cout<<"System ID started!"<<std::endl;
            }


            husbcom->DataLogManager();
            husbcom->ifNewMessage = 0;
        }
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
