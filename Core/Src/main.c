/* USER CODE BEGIN Header */
/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【淘    宝】  魔女开发板 1      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【实验名称】  2.8寸_显示综合 + UART2_IMU接入
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库
 **
 ** 【 CubeMX 】  1- 无需使用CubeMX配置引脚
 **
 ** 【移植说明】  1- 复制本工程的LCD_ILI9341和W25Q128文件夹，到目标工程文件夹中;
 **               2- 添加头文件路径： Keil > Option > C/C++ > Include Paths, 添加上述两个目标文件夹的目录路径;
 **               3- 添加C文件到工程: Keil > 左侧工程管理器中双击目标文件夹 > 选择 bsp_LCD_ILI9341.c和bsp_W25Q128.c;
 **               4- 添加文件引用:    #include bsp_LCD_ILI9341.h，即哪个文件中需要使用其函数功能，就在其代码文件顶部添加引用;
 **
 ** 【代码使用】  1- 初始化：直接调用h中函数即可完成初始化。
 **               2- 文本的输出，已编写4种示范函数，可按需自行编写、修改;
 **               3- 画线画圆等图形基本函数，已编写示范函数，可在此基础上，按需编写更完善的图形函数，如显示图片、绘制按钮等.
 **
 **
 **
 *******************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

#include "bsp_W25Q128.h"
#include "bsp_LCD_ILI9341.h"
#include "bsp_XPT2046.h"
#include "bsp_UART.h"
#include "bsp_motor.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ================= IMU: DFRobot SEN0692 (Modbus-RTU over UART2) ==============
 * addr 0x4A, 115200-8-N-1 (migrated from 9600 at boot, reg 0x0003); fn 0x04 read 3 words
 * accel reg 0x0006, scale +/-2g  -> 16384 LSB/g  (module current range)
 * gyro  reg 0x0009, scale +/-250dps -> 65.536 LSB/dps
 * resp frame: 4A 04 06 D0 D1 D2 D3 D4 D5 CRC_L CRC_H  (11 bytes, big-endian int16)
 ******************************************************************************/
#define IMU_ADDR            0x4A
#define IMU_REG_ACC         0x0006
#define IMU_REG_GYR         0x0009
#define IMU_ACC_LSB_PER_G   16384.0f
#define IMU_GYR_LSB_PER_DPS 65.536f

static float   imuAcc[3] = {0};                    // g
static float   imuGyr[3] = {0};                    // dps
static uint8_t imuAccOK = 0, imuGyrOK = 0;
static uint16_t imuErrCnt = 0;
static uint8_t  imuBaud115 = 1;                 // 1: link at 115200, 0: fell back to 9600
static float   imuRoll = 0, imuPitch = 0;         // deg, complementary filter
static uint8_t imuAttInit = 0;                     // 0: seed from accelerometer on next update

static uint16_t IMU_CRC16(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              { crc >>= 1; }
        }
    }
    return crc;                                    // wire order: low byte first
}

/* One-shot module baud migration: write reg 0x0003 with an out-of-table value.
 * Per DFRobot wiki, values outside the 8-entry table fall back to 115200 (persisted).
 * Called while MCU runs at 9600; if the module is already at 115200 the frame is
 * garbage to it and the wait simply times out - harmless and idempotent. */
static void IMU_SetBaud115200(void)
{
    uint8_t f[8];
    f[0] = IMU_ADDR;
    f[1] = 0x06;                                   // write single holding register
    f[2] = 0x00;
    f[3] = 0x03;                                   // reg 0x0003: baud rate config
    f[4] = 0x00;
    f[5] = 0xFF;                                   // out-of-table -> module uses 115200
    uint16_t crc = IMU_CRC16(f, 6);
    f[6] = (uint8_t)(crc & 0xFF);
    f[7] = (uint8_t)(crc >> 8);
    UART2_ClearRx();
    UART2_SendData(f, 8);
    HAL_Delay(300);                                // module saves config to flash
    UART2_ClearRx();                               // drop any 9600-speed ack bytes
}

static void IMU_SendQuery(uint16_t reg)
{
    uint8_t f[8];
    f[0] = IMU_ADDR;
    f[1] = 0x04;                                   // read input registers
    f[2] = (uint8_t)(reg >> 8);
    f[3] = (uint8_t)(reg & 0xFF);
    f[4] = 0x00;
    f[5] = 0x03;                                   // 3 words (X/Y/Z)
    uint16_t crc = IMU_CRC16(f, 6);
    f[6] = (uint8_t)(crc & 0xFF);
    f[7] = (uint8_t)(crc >> 8);
    UART2_ClearRx();
    UART2_SendData(f, 8);
}

static uint8_t IMU_Parse(float out[3], float lsb)
{
    uint8_t *p = UART2_GetRxData();
    if (UART2_GetRxNum() != 11)        return 0;
    if (p[0] != IMU_ADDR)              return 0;
    if (p[1] != 0x04 || p[2] != 0x06)  return 0;
    uint16_t crc = IMU_CRC16(p, 9);
    if (p[9] != (uint8_t)(crc & 0xFF) || p[10] != (uint8_t)(crc >> 8)) return 0;
    for (uint8_t k = 0; k < 3; k++)
    {
        int16_t raw = (int16_t)((p[3 + 2 * k] << 8) | p[4 + 2 * k]);
        out[k] = (float)raw / lsb;
    }
    return 1;
}

/* fixed-width text "TAG:-123.456UNIT", integer math only (no %f, MicroLIB safe) */
/* Probe: read VID (input reg 0x0000, expect 0x3343 per wiki). */
static uint8_t IMU_Probe(uint8_t tries)
{
    uint8_t f[8];
    f[0] = IMU_ADDR;
    f[1] = 0x04;
    f[2] = 0x00;
    f[3] = 0x00;                               // input reg 0x0000: VID
    f[4] = 0x00;
    f[5] = 0x01;                               // 1 word
    uint16_t crc = IMU_CRC16(f, 6);
    f[6] = (uint8_t)(crc & 0xFF);
    f[7] = (uint8_t)(crc >> 8);
    for (uint8_t t = 0; t < tries; t++)
    {
        UART2_ClearRx();
        UART2_SendData(f, 8);
        for (uint32_t w = 0; w < 30; w++)       // 30ms window per try
        {
            HAL_Delay(1);
            if (UART2_GetRxNum())
            {
                uint8_t *p = UART2_GetRxData();
                uint8_t n = UART2_GetRxNum();
                uint16_t c = IMU_CRC16(p, 5);
                uint8_t ok = (n == 7) && (p[0] == IMU_ADDR) && (p[1] == 0x04) && (p[2] == 0x02)
                          && (p[3] == 0x33) && (p[4] == 0x43)
                          && (p[5] == (uint8_t)c) && (p[6] == (uint8_t)(c >> 8));
                UART2_ClearRx();
                if (ok) return 1;
            }
        }
    }
    UART2_ClearRx();
    return 0;
}

static void IMU_Format(char *s, const char *tag, float val, uint8_t dec3)
{
    int32_t m = (int32_t)(val * 1000.0f + (val >= 0 ? 0.5f : -0.5f));
    uint8_t neg = (m < 0) ? 1 : 0;
    if (neg) m = -m;
    if (dec3)
        sprintf(s, "%s:%c%ld.%03ld", tag, neg ? '-' : ' ', (long)(m / 1000), (long)(m % 1000));
    else
        sprintf(s, "%s:%c%ld.%01ld", tag, neg ? '-' : ' ', (long)(m / 1000), (long)((m % 1000) / 100));
}

static void IMU_Display(void)
{
    char s[24];
    if (imuAccOK)
    {
        IMU_Format(s, "AX", imuAcc[0], 1); LCD_String(10,  40, s, 16, GREEN, BLACK);
        IMU_Format(s, "AY", imuAcc[1], 1); LCD_String(10,  62, s, 16, GREEN, BLACK);
        IMU_Format(s, "AZ", imuAcc[2], 1); LCD_String(10,  84, s, 16, GREEN, BLACK);
    }
    if (imuGyrOK)
    {
        IMU_Format(s, "GX", imuGyr[0], 0); LCD_String(10, 116, s, 16, CYAN, BLACK);
        IMU_Format(s, "GY", imuGyr[1], 0); LCD_String(10, 138, s, 16, CYAN, BLACK);
        IMU_Format(s, "GZ", imuGyr[2], 0); LCD_String(10, 160, s, 16, CYAN, BLACK);
    }
    if (imuAttInit)
    {
        IMU_Format(s, "ROL", imuRoll,  0); LCD_String(10, 184, s, 16, YELLOW, BLACK);
        IMU_Format(s, "PIT", imuPitch, 0); LCD_String(10, 206, s, 16, YELLOW, BLACK);
    }
    if (imuErrCnt > 15)
        LCD_String(10, 232, "IMU: OFFLINE  ", 16, RED, BLACK);
    else if (imuBaud115)
        LCD_String(10, 232, "IMU: OK 115200", 16, YELLOW, BLACK);
    else
        LCD_String(10, 232, "IMU: OK 9600  ", 16, YELLOW, BLACK);
}

/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

    Motor_Init();                                                              // 先初始化电机引脚，避免上电期间 PUL-/DIR-/ENA- 悬空
    UART1_Init(115200);
    UART2_Init(9600);                                                          // SEN0692 factory default
    IMU_SetBaud115200();                                                       // one-shot: move module to 115200 (idempotent)
    UART2_Init(115200);                                                        // UART2: IMU SEN0692 (PA2-TX, PA3-RX, Modbus-RTU)
    if (!IMU_Probe(3))                          // no valid reply at 115200
    {
        UART2_Init(9600);                       // migration failed: stay at 9600
        imuBaud115 = 0;
        IMU_Probe(2);
    }

    printf("\r\r\r=========== STM32F407 板卡上电初始化完成 ===========\r");             // 注意，printf函数，必须重定向后才能使用，否则程序会卡死; 本示例已重写fputc, 在main.c的底部附近
    printf("系统运行时钟：           %d MHz\r", SystemCoreClock / 1000000);       // 输出系统运行时钟的频率，以检查是否配置正常

    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);              // LED控制引脚置高电平，红亮灭

    W25Q128_Init();                                                               // W25Q128 初始化

    LCD_Init();                                                                   // LCD ILI9341 初始化
    LCD_SetDir(0);                                                                // 设置显示方向：0-竖屏、1-横屏

    // 顶部标题
    LCD_String(5, 5, "STM32F407", 16, GREEN, BLACK);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

        /** 1ms延时 **/
        HAL_Delay(1);                                                      // 1ms节拍；下方按键消抖、IMU超时和刷新周期均以此为基准

        /** 触摸检查 **/
        //XPT2046_TouchHandler();                                            // 触摸屏状态检测：1:检测是否有按下，2:如果有按下，就处理好按下的坐标

        /** 检查UART1是否收到数据 (串口助手) **/
        if (UART1_GetRxNum())                                              // 判断UART1是否获取到新一帧数据：函数返回的接收字节数，大于0，即为收到新一帧数据
        {
            uint8_t *rxData = UART1_GetRxData();                           // 获取数据的地址
            uint16_t rxNum = UART1_GetRxNum();                             // 获取数据的字节数
            printf("\r<<<<< UART1 接收到一帧数据 \r");                     // 提示UART1接收到数据
            printf("字节数：%d \r", rxNum);                                // 显示字节数
            printf("ASCII : %s\r", (char *)rxData);                        // 显示数据，以ASCII方式显示，即以字符串的方式显示
            printf("16进制: ");                                            // 显示数据，以16进制方式，显示每一个字节的值
            for (uint16_t i = 0; i < rxNum; i++)                           // 逐个字节输出
                printf("0x%02X ", rxData[i]);                              // 以16进制显示
            printf("\r\r");                                                // 显示换行
            UART1_ClearRx();                                               // 重要：清0接收标志，即清0接收到的字节数; 每次处理完成数据，就要调用这个函数清0

            // UART1_SendString("Hello World！\r");                        // 示范 UART1_SendString( )的使用; 可接受像pritnf的格式化参数
            // UART1_SendData((uint8_t*)"Hello World!\r", 13);             // 示范 UART1_SendData( )的使用, 可用于发送字符串、数组、结构体数据等
        }

        /** 主轴电机：KEY_1(PA0) 正转、KEY_2(PA1) 反转，都不按停止（20ms 消抖） **/
        Motor_KeyScan();

        /** 每500ms闪烁一次蓝色LED **/
        /** IMU: SEN0692 Modbus-RTU poll state machine (UART2 @115200) **/
        static uint8_t  imuPhase = 0;              // 0-idle 1-waitAcc 2-waitGyr
        static uint16_t imuTickMs = 0, imuWaitMs = 0;
        imuTickMs++;
        if (imuPhase == 0)
        {
            if (imuTickMs >= 30)                   // poll cycle ~30ms (~33Hz)
            {
                imuTickMs = 0;
                IMU_SendQuery(IMU_REG_ACC);
                imuPhase = 1;
                imuWaitMs = 0;
            }
        }
        else if (++imuWaitMs > 60)                 // 60ms response timeout
        {
            UART2_ClearRx();
            imuErrCnt++;
            imuPhase = 0;
        }
        else if (UART2_GetRxNum())
        {
            if (imuPhase == 1)
            {
                imuAccOK = IMU_Parse(imuAcc, IMU_ACC_LSB_PER_G);
                IMU_SendQuery(IMU_REG_GYR);
                imuPhase = 2;
                imuWaitMs = 0;
            }
            else
            {
                imuGyrOK = IMU_Parse(imuGyr, IMU_GYR_LSB_PER_DPS);
                imuPhase = 0;
                imuTickMs = 0;
                if (imuAccOK || imuGyrOK) imuErrCnt = 0; else imuErrCnt++;
                /* attitude: complementary filter, acc angle + gyro integration */
                {
                    float aR = atan2(imuAcc[1], imuAcc[2]) * 57.2958f;
                    float aP = atan2(-imuAcc[0], sqrt(imuAcc[1] * imuAcc[1] + imuAcc[2] * imuAcc[2])) * 57.2958f;
                    if (!imuAttInit) { imuRoll = aR; imuPitch = aP; imuAttInit = 1; }
                    imuRoll  = 0.98f * (imuRoll  + imuGyr[0] * 0.03f) + 0.02f * aR;
                    imuPitch = 0.98f * (imuPitch + imuGyr[1] * 0.03f) + 0.02f * aP;
                }
            }
        }

        /** IMU display: refresh at 10Hz regardless of link state **/
        static uint16_t imuDispMs = 0;
        if (++imuDispMs >= 100)
        {
            imuDispMs = 0;
            IMU_Display();
        }

        static uint16_t msLED = 0;
        if (++msLED == 500)                                                // 每500ms执行一次
        {
            msLED = 0;                                                     // 计数清0
            HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);          // 规律地闪烁蓝色LED，方便外部观察系统是否正常运行
        }

    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */



/******************************************************************************
 * 函  数： HAL_GPIO_EXTI_Callback
 * 功  能： EXTI中断回调函数
 * 参  数： uint16_t  GPIO_Pin    引脚编号，参数范围：GPIO_Pin_0 ~ GPIO_Pin_15
 * 返回值： 无
 * 备  注： CubeMX生成的代码在思维和风格上，与使用标准库编写时有很大的区别：
 *          1：这个是回调函数，不是中断服务函数。完整的调用：EXTIx_IRQHandler() > HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_x) > HAL_GPIO_EXTI_Callback(GPIO_PIN_x)
 *          2：必须使用这个函数名称，因为它在CubeMX生成时，已被写好了各种函数调用、函数弱定义(在stm32xx_hal_gpio.c的底部); 不建议在原弱定义中增添代码，而是重写本函数
 *          3：中断条件触发后，本函数被自动调用，无需进行人工调用; 也无需进行中断标志的清理，因为HAL_GPIO_EXTI_IRQHandler()函数中已有操作;
 *          4：生成的所有EXTI中断服务函数，都统一调用这个函数，以引脚编号作参数
 *          5：判断参数传进来的引脚编号，即可知道是哪个外部中断线产生的中断信号
******************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_1_Pin)                                  // 按键1 产生的中断信号：KEY_1、PA0、闲时下拉、按下时高电平、上升沿触发
    {
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);     // 反转红色LED灯
        printf("KEY_1 按下\r");                                 // 通过串口助手输出提示
    }

    if (GPIO_Pin == KEY_2_Pin)                                  // 按键2 产生的中断信号：KEY_2、PA1、闲时上拉、按下时低电平、下降沿触发
    {
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);     // 反转红色LED灯
        printf("KEY_2 按下\r");                                 // 通过串口助手输出提示
    }

    if (GPIO_Pin == KEY_3_Pin)                                  // 按键3 产生的中断信号：KEY_3、PA4、闲时上拉、按下低高电平、下降沿触发
    {
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);     // 反转红色LED灯
        printf("KEY_3 按下\r");                                 // 通过串口助手输出提示
    }
}




/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and linerxNumber,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
