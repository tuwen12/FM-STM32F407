#include "bsp_motor.h"
#include "main.h"
#include <stdio.h>

/* ================= 主轴电机：TSDA-C11A 脉冲/方向控制 =================
 * 接线：共阳极接法——CN3 的 PUL+ / DIR+ / ENA+ 接 +5V，PUL- / DIR- / ENA-
 *       分别接 PD12 / PB1 / PD3（经 3.3V→5V 电平转换，高电平须达到约 5V）。
 *       GPIO 高 → 光耦截止、驱动器正负端压差低；GPIO 低 → 光耦导通、压差高。
 *       - PUL-：空闲保持高（无脉冲），运行时 10kHz 方波，低电平 30µs 为导通段
 *       - DIR-：GPIO 高 = 压差低 = 正转，GPIO 低 = 压差高 = 反转
 *       - ENA-：GPIO 高 = 压差低 = 使能，GPIO 低 = 压差高 = 失能
 * 若实测方向或使能相反，把对应的 LEVEL 宏改成相反电平即可。
 * 按键：KEY_1(PA0) 正转、KEY_2(PA1) 反转，两键都不按（或同时按下）停止。
 * 速度：10kHz ÷ 10000 脉冲/圈 = 1 圈/秒 = 60RPM（电子齿轮 1:1 时）
 ******************************************************************************/
#define MOTOR_PULSE_HZ           10000UL
#define MOTOR_TIM_CLK_HZ         84000000UL                   // TIM4 挂 APB1，定时器时钟 = 42MHz × 2
#define MOTOR_TIM_PERIOD         (MOTOR_TIM_CLK_HZ / MOTOR_PULSE_HZ - 1U)
#define MOTOR_CCR_IDLE           (MOTOR_TIM_PERIOD + 1U)      // CCR > ARR：PWM 恒高（PUL- 高、光耦截止，无脉冲）
#define MOTOR_CCR_RUN            ((MOTOR_TIM_PERIOD + 1U) * 7U / 10U) // 70% 占空比（GPIO 高 70µs、低 30µs，低电平段光耦导通）

#define MOTOR_PUL_GPIO_Port      GPIOD
#define MOTOR_PUL_Pin            GPIO_PIN_12                  // TIM4_CH1，AF2
#define MOTOR_DIR_GPIO_Port      GPIOB
#define MOTOR_DIR_Pin            GPIO_PIN_1
#define MOTOR_ENA_GPIO_Port      GPIOD
#define MOTOR_ENA_Pin            GPIO_PIN_3

#define MOTOR_DIR_FORWARD_LEVEL  GPIO_PIN_SET                  // DIR- 高 = 压差低 = 正转(CCW)
#define MOTOR_DIR_REVERSE_LEVEL  GPIO_PIN_RESET                // DIR- 低 = 压差高 = 反转(CW)
#define MOTOR_ENA_ENABLE_LEVEL   GPIO_PIN_SET                  // ENA- 高 = 压差低 = 使能
#define MOTOR_ENA_DISABLE_LEVEL  GPIO_PIN_RESET                // ENA- 低 = 压差高 = 失能(自由)

/* ================= 主轴电机：脉冲位置控制 ================= */
void Motor_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    // DIR：PB1 推挽输出，初始正转（GPIO 高 → 驱动器压差低）
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_DIR_FORWARD_LEVEL);
    gpio.Pin   = MOTOR_DIR_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_DIR_GPIO_Port, &gpio);

    // ENA：PD3 推挽输出，初始失能（GPIO 低 → 驱动器压差高）
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, MOTOR_ENA_DISABLE_LEVEL);
    gpio.Pin   = MOTOR_ENA_Pin;
    HAL_GPIO_Init(MOTOR_ENA_GPIO_Port, &gpio);

    // PUL：PD12 复用为 TIM4_CH1（接 PUL-）
    gpio.Pin       = MOTOR_PUL_Pin;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(MOTOR_PUL_GPIO_Port, &gpio);

    // TIM4 通道1：PWM 模式1，空闲时 GPIO 恒高（驱动器侧压差低，无脉冲）
    TIM4->CR1   = 0;
    TIM4->PSC   = 0;
    TIM4->ARR   = MOTOR_TIM_PERIOD;
    TIM4->CCR1  = MOTOR_CCR_IDLE;
    TIM4->CCMR1 = TIM_CCMR1_OC1PE | (0x6UL << TIM_CCMR1_OC1M_Pos);  // PWM1 + 预装载
    TIM4->CCER  = TIM_CCER_CC1E;                                    // 高电平有效
    TIM4->EGR   = TIM_EGR_UG;                                       // 立即装入
    TIM4->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;                       // 启动计数
}

void Motor_SetRun(int8_t dir)                 // dir: 1-正转, -1-反转, 0-停止
{
    static int8_t lastDir = 0;

    if (dir == 0)
    {
        TIM4->CCR1 = MOTOR_CCR_IDLE;                                // 先停脉冲
        HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, MOTOR_ENA_DISABLE_LEVEL);
        lastDir = 0;
        printf("MOTOR: STOP\r");
        return;
    }

    if (dir == lastDir) return;                                    // 方向未变化，无需重复操作

    TIM4->CCR1 = MOTOR_CCR_IDLE;                                   // 换向前先停脉冲
    HAL_Delay(2);                                                  // 等当前 PWM 周期走完
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin,
                      dir > 0 ? MOTOR_DIR_FORWARD_LEVEL : MOTOR_DIR_REVERSE_LEVEL);
    HAL_Delay(1);                                                  // 方向建立时间（手册要求 ≥5µs）

    if (lastDir == 0)                                              // 从停止状态启动，需要先使能
    {
        HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, MOTOR_ENA_ENABLE_LEVEL);
        HAL_Delay(20);                                             // 等驱动器使能完成
    }

    TIM4->CCR1 = MOTOR_CCR_RUN;                                    // 输出脉冲
    lastDir = dir;
    if (dir > 0) printf("MOTOR: RUN FWD\r");
    else         printf("MOTOR: RUN REV\r");
}

void Motor_KeyScan(void)
{
    static int8_t   motorDir    = 0;                          // 当前运行方向：1-正转, -1-反转, 0-停止
    static uint16_t motorKeyCnt = 0;
    uint8_t keyFwd = (HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == GPIO_PIN_SET) ? 1 : 0;  // KEY_1 按下为高
    uint8_t keyRev = (HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin) == GPIO_PIN_RESET) ? 1 : 0; // KEY_2 按下为低
    int8_t  motorWant = 0;

    if (keyFwd && !keyRev)      motorWant = 1;
    else if (keyRev && !keyFwd) motorWant = -1;
    else                        motorWant = 0;                // 都不按或两键同时按下：停止

    if (motorWant != motorDir)
    {
        if (++motorKeyCnt >= 20)
        {
            motorKeyCnt = 0;
            motorDir = motorWant;
            Motor_SetRun(motorDir);
        }
    }
    else
    {
        motorKeyCnt = 0;
    }
}
