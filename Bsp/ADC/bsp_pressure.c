#include "bsp_pressure.h"

/******************************************************************************
 * 本地函数
 ******************************************************************************/
static uint16_t Pressure_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x0001U)
            {
                crc >>= 1;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static float Pressure_Clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/******************************************************************************
 * 默认配置
 ******************************************************************************/
void Pressure_GetDefaultConfig(PressureConfig *cfg)
{
    if (cfg == 0) return;

    cfg->raw_at_4mA  = 4000U;                    // 0.001mA 格式
    cfg->raw_at_20mA = 20000U;
    cfg->mpa_at_4mA  = PRESSURE_MPA_AT_4MA;
    cfg->mpa_at_20mA = PRESSURE_MPA_AT_20MA;
    cfg->reg_index   = 0U;
}

/******************************************************************************
 * 由电流值换算
 ******************************************************************************/
PressureValue Pressure_FromCurrent(float mA)
{
    PressureValue out;

    out.raw = 0U;
    out.mA  = mA;

    if (mA < 4.0f)
    {
        out.mpa    = PRESSURE_MPA_AT_4MA;
        out.status = PRESSURE_UNDER_RANGE;
    }
    else if (mA > 20.0f)
    {
        out.mpa    = PRESSURE_MPA_AT_20MA;
        out.status = PRESSURE_OVER_RANGE;
    }
    else
    {
        out.mpa    = PRESSURE_MPA_AT_4MA
                   + (mA - 4.0f) * (PRESSURE_MPA_AT_20MA - PRESSURE_MPA_AT_4MA) / 16.0f;
        out.status = PRESSURE_OK;
    }

    return out;
}

/******************************************************************************
 * 由 RS485 原始值换算
 ******************************************************************************/
PressureValue Pressure_FromRaw(uint16_t raw, const PressureConfig *cfg)
{
    PressureValue out;
    float ratio;

    out.raw = raw;
    out.mA  = 4.0f;
    out.mpa = PRESSURE_MPA_AT_4MA;

    if ((cfg == 0) || (cfg->raw_at_20mA <= cfg->raw_at_4mA))
    {
        out.status = PRESSURE_CONFIG_ERR;
        return out;
    }

    if (raw < cfg->raw_at_4mA)
    {
        /* 低于 4mA：压力按 0MPa 处理，电流保留实际值便于诊断 */
        out.mA     = 4.0f - (float)(cfg->raw_at_4mA - raw)
                   * 16.0f / (float)(cfg->raw_at_20mA - cfg->raw_at_4mA);
        out.mpa    = cfg->mpa_at_4mA;
        out.status = PRESSURE_UNDER_RANGE;
        return out;
    }

    ratio = (float)(raw - cfg->raw_at_4mA) / (float)(cfg->raw_at_20mA - cfg->raw_at_4mA);

    out.mA  = 4.0f + 16.0f * ratio;
    out.mpa = cfg->mpa_at_4mA + (cfg->mpa_at_20mA - cfg->mpa_at_4mA) * ratio;

    if (raw > cfg->raw_at_20mA)
    {
        out.mpa    = cfg->mpa_at_20mA;
        out.status = PRESSURE_OVER_RANGE;
        return out;
    }

    out.mpa    = Pressure_Clamp(out.mpa, cfg->mpa_at_4mA, cfg->mpa_at_20mA);
    out.status = PRESSURE_OK;
    return out;
}

/******************************************************************************
 * 解析 Modbus-RTU 应答帧
 ******************************************************************************/
uint8_t Pressure_ParseModbusFrame(const uint8_t *frame, uint16_t len,
                                  const PressureConfig *cfg, PressureValue *out)
{
    uint16_t crc;
    uint16_t raw;
    uint8_t  bytes;
    uint16_t index;

    if ((frame == 0) || (cfg == 0) || (out == 0)) return 0U;
    if (len < 5U) return 0U;

    if ((frame[1] != 0x03U) && (frame[1] != 0x04U)) return 0U;  // 读保持/输入寄存器

    bytes = frame[2];
    if ((bytes < 2U) || (len != (uint16_t)(5U + bytes))) return 0U;

    crc = Pressure_CRC16(frame, (uint16_t)(len - 2U));
    if ((frame[len - 2U] != (uint8_t)(crc & 0xFFU)) ||
        (frame[len - 1U] != (uint8_t)(crc >> 8)))
    {
        return 0U;
    }

    index = cfg->reg_index;
    if (index >= (uint16_t)(bytes / 2U)) return 0U;

    raw = ((uint16_t)frame[3U + 2U * index] << 8) |
           (uint16_t)frame[4U + 2U * index];

    *out = Pressure_FromRaw(raw, cfg);
    return 1U;
}

/******************************************************************************
 * 生成 Modbus-RTU 读寄存器请求
 ******************************************************************************/
uint16_t Pressure_BuildReadRequest(uint8_t slave, uint8_t func,
                                   uint16_t reg, uint16_t count,
                                   uint8_t out[8])
{
    uint16_t crc;

    if (out == 0) return 0U;

    out[0] = slave;
    out[1] = func;
    out[2] = (uint8_t)(reg >> 8);
    out[3] = (uint8_t)(reg & 0xFFU);
    out[4] = (uint8_t)(count >> 8);
    out[5] = (uint8_t)(count & 0xFFU);

    crc = Pressure_CRC16(out, 6U);
    out[6] = (uint8_t)(crc & 0xFFU);
    out[7] = (uint8_t)(crc >> 8);

    return 8U;
}
