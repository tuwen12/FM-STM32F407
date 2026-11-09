#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

/* 初始化电机引脚及脉冲定时器。 */
void Motor_Init(void);
/* 设置运行方向：1 正转，-1 反转，0 停止。 */
void Motor_SetRun(int8_t dir);
/* 每次主循环调用，扫描双键并进行20次循环消抖。 */
void Motor_KeyScan(void);

#endif
