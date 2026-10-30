#ifndef PAGE_FIRE_H
#define PAGE_FIRE_H

#include "tjc_hmi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    PAGE_FIRE_ACTION_NONE = 0,
    PAGE_FIRE_ACTION_RETURN_HOME
} PageFire_Action_t;

void PageFire_Init(void);
void PageFire_Enter(void);
void PageFire_Leave(void);
void PageFire_Task(void);

/* 由dartboard_hmi.c将收到的屏幕事件转交到这里 */
PageFire_Action_t PageFire_HandleEvent(TJC_HMI *hmi,
                                      const TJC_Event *event);

/* 停止页面控制的所有执行机构 */
void PageFire_StopAll(void);

#ifdef __cplusplus
}
#endif

#endif


