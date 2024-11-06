#ifndef MY_MAIN_H
#define MY_MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"


#ifdef __cplusplus
extern "C" {
#endif
        extern osThreadId_t mainTaskHandle;
        void StartMainTask(void *argument);
#ifdef __cplusplus
}
#endif

#endif