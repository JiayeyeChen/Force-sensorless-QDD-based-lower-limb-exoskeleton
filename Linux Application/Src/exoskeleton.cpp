#include "exoskeleton.hpp"

ExoskeletonHandle::ExoskeletonHandle(uint8_t test_input): curMainTask(EXOSKELETON_MAIN_TASK_FREE), curSubTask(EXOSKELETON_SUB_TASK_NONE)
{
    val = test_input;
}

ExoskeletonHandle::~ExoskeletonHandle()
{
}
