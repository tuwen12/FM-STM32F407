#ifndef __BSP_PRESSURE_H
#define __BSP_PRESSURE_H

#include <stdint.h>

/******************************************************************************
 * 4-20mA 压力变送器业务层（RS485 / Modbus-RTU）
 *
 * 功能：
 *   1. 解析 RS485 收到的 Modbus-RTU 应答帧（功能码 0x03 / 0x04）；
 *   2. 取出 4-20mA 采集模块返回的原始值；
 *   3. 按 4mA = 0MPa、20mA = 1MPa 线性换算成 0~1MPa 的压力值；
 *   4. 带欠量程 / 超量程判断与限幅。
 *
 * 说明：
 *   - 本模块只做“协议解析 + 数值换算”，不初始化串口、不控制 RS485 方向脚；
 *   - 串口仍使用工程现成的 UART3（PB10/PB11，RS485 跳线）：
 *         uint8_t *frame = UART3_GetRxData();
 *         uint16_t len   = UART3_GetRxNum();
 *         if (Pressure_ParseModbusFrame(frame, len, &cfg, &pv)) { ... }
 *         UART3_ClearRx();
 *   - 默认原始值量程按“0.001mA”格式：4mA=4000，20mA=20000；
 *     若模块输出的是 12 位 ADC 原始码（0~4095），请把
 *     raw_at_4mA / raw_at_20mA 改成 0 / 4095；
 *   - 若模块直接返回 mA 浮点值或 ASCII，则自行解析后调用
 *     Pressure_FromCurrent(mA)。
 ******************************************************************************/

#define PRESSURE_MPA_AT_4MA      0.0f
#define PRESSURE_MPA_AT_20MA     1.0f

typedef enum
{
    PRESSURE_OK = 0,             // 4~20mA 正常范围
    PRESSURE_UNDER_RANGE,        // 低于 4mA（断线/欠量程）
    PRESSURE_OVER_RANGE,         // 高于 20mA（超量程）
    PRESSURE_CONFIG_ERR          // 配置非法（量程端点相同等）
} PressureStatus;

typedef struct
{
    uint16_t raw_at_4mA;         // 4mA 对应的原始值
    uint16_t raw_at_20mA;        // 20mA 对应的原始值
    float    mpa_at_4mA;         // 4mA 对应的压力（默认 0MPa）
    float    mpa_at_20mA;        // 20mA 对应的压力（默认 1MPa）
    uint8_t  reg_index;          // 应答帧中第几个 16 位寄存器（从 0 开始）
} PressureConfig;

typedef struct
{
    uint16_t       raw;          // 原始值
    float          mA;           // 换算出的电流（mA）
    float          mpa;           // 换算出的压力（MPa）
    PressureStatus status;
} PressureValue;

/* 填充默认配置：4000/20000 原始值，0~1MPa，取第 0 个寄存器 */
void Pressure_GetDefaultConfig(PressureConfig *cfg);

/* 由电流值（mA）直接换算压力（4mA=0MPa，20mA=1MPa） */
PressureValue Pressure_FromCurrent(float mA);

/* 由 RS485 原始值换算压力（按 cfg 的端点线性映射） */
PressureValue Pressure_FromRaw(uint16_t raw, const PressureConfig *cfg);

/* 解析 Modbus-RTU 应答帧：
 *   [从机地址][功能码 0x03/0x04][字节数][数据...][CRC_L][CRC_H]
 * 返回 1=解析成功，0=帧格式/CRC/寄存器索引错误 */
uint8_t Pressure_ParseModbusFrame(const uint8_t *frame, uint16_t len,
                                  const PressureConfig *cfg, PressureValue *out);

/* 生成 Modbus-RTU 读寄存器请求（8 字节）：
 *   [从机][功能码][寄存器高][寄存器低][数量高][数量低][CRC_L][CRC_H]
 * 返回固定 8 */
uint16_t Pressure_BuildReadRequest(uint8_t slave, uint8_t func,
                                   uint16_t reg, uint16_t count,
                                   uint8_t out[8]);

#endif
