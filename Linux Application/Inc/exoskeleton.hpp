#ifndef EXOSKELETON_H
#define EXOSKELETON_H

#include "stdint.h"

enum Exoskeleton_Main_Task
{
    EXOSKELETON_MAIN_TASK_FREE,
    EXOSKELETON_MAIN_TASK_SYSTEM_ID,
    EXOSKELETON_MAIN_TASK_ACTIVE_CONTROL
};

enum Exoskeleton_Sub_Task
{
    EXOSKELETON_SUB_TASK_NONE,
    EXOSKELETON_SUB_TASK_SYSTEMID_LOG_KNEE_JOINT_DATA,
    EXOSKELETON_SUB_TASK_SYSTEMID_LOG_HIP_JOINT_DATA,
    EXOSKELETON_SUB_TASK_SYSTEMID_LEAST_SQUARE_APPROXIMATION
};

class ExoskeletonHandle
{
private:

public:
    uint8_t val;
    
    enum Exoskeleton_Main_Task curMainTask;
    enum Exoskeleton_Sub_Task curSubTask;
    ExoskeletonHandle(uint8_t test_input);
    ~ExoskeletonHandle();
};





#endif
