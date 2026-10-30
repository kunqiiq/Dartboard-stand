/**
 * @file scheduler.h
 * @brief 裸机主循环任务调度器接口
 *
 * 用户在 main.c 中完成：
 *   Scheduler_Init();
 *   while (1) { Scheduler_Run(); }
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

void Scheduler_Init(void);
void Scheduler_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
