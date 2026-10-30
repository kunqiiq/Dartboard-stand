#ifndef DARTBOARD_HMI_H
#define DARTBOARD_HMI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 初始化USART1串口屏：
 *
 * - 初始化TJC协议解析器；
 * - 注册USART1接收和错误回调；
 * - 启动ReceiveToIdle DMA；
 * - 默认切换到page5启动页（页面ID=0）。
 */
void DartboardHMI_Init(void);

/*
 * 串口屏周期任务。
 *
 * 建议由scheduler每1 ms调用。
 */
void DartboardHMI_Task(void);

/*
 * 获取当前页面ID：
 *
 * 0 = page5 上电启动页
 * 1 = page0 首页
 * 2 = page1 发射页
 * 3 = page2 记录页
 * 4 = page3 调平测试页
 * 5 = keybdA 在线输入键盘页
 * 6 = page4 最近四条记录页
 *
 * page0预留b_auto（ID=4），松开时启动自动调平。
 */
uint8_t DartboardHMI_GetCurrentPage(void);

#ifdef __cplusplus
}
#endif

#endif /* DARTBOARD_HMI_H */

