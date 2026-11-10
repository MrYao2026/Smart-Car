/**
 * @file    bsp_ina219.c
 * @brief   INA219 总线电压检测驱动，使用 PB6/PB7 软件模拟 I2C。
 */

#include "bsp_ina219.h"
#include <math.h>

/* INA219 默认 7 位 I2C 地址；发送地址字节时需左移一位。 */
#define INA219_I2C_ADDRESS          0x40U
/* INA219 配置寄存器地址，用于设置量程和连续采样模式。 */
#define INA219_REG_CONFIG           0x00U
/* INA219 总线电压寄存器地址。 */
#define INA219_REG_BUS_VOLTAGE      0x02U
/* 总线电压寄存器每个有效计数代表 4 mV。 */
#define INA219_BUS_VOLTAGE_LSB_V    0.004f
/* INA219 与 OLED 共用 PB6 时钟线、PB7 数据线。 */
#define INA219_SCL_PIN              GPIO_PIN_6
#define INA219_SDA_PIN              GPIO_PIN_7
/* 12 位总线采样、32 V 量程、320 mV 分流量程、连续测量模式。 */
#define INA219_CONFIG_VALUE         0x399FU

/**
 * @brief 软件 I2C 半周期延时，与 OLED 驱动采用相同的短延时方式。
 */
static void INA219_I2C_Delay(void)
{
    for (volatile uint8_t delay_count = 0U; delay_count < 4U; delay_count++)
    {
        __NOP();
    }
}

/**
 * @brief 在 PB6/PB7 上产生 I2C 起始条件。
 */
static void INA219_I2C_Start(void)
{
    /* 先释放总线，再在 SCL 为高时拉低 SDA，形成起始条件。 */
    HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN | INA219_SDA_PIN, GPIO_PIN_SET);
    INA219_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, INA219_SDA_PIN, GPIO_PIN_RESET);
    INA219_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN, GPIO_PIN_RESET);
    INA219_I2C_Delay();
}

/**
 * @brief 在 PB6/PB7 上产生 I2C 停止条件并释放总线。
 */
static void INA219_I2C_Stop(void)
{
    /* SCL 为低时拉低 SDA，随后释放 SCL 和 SDA，结束当前传输。 */
    HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN | INA219_SDA_PIN, GPIO_PIN_RESET);
    INA219_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN, GPIO_PIN_SET);
    INA219_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, INA219_SDA_PIN, GPIO_PIN_SET);
    INA219_I2C_Delay();
}

/**
 * @brief 发送一个字节，并在第九个时钟周期读取从机 ACK。
 * @param data 需要发送的数据。
 * @return 从机应答时返回 1，否则返回 0。
 */
static uint8_t INA219_I2C_WriteByte(uint8_t data)
{
    /* I2C 按最高有效位优先传输数据。 */
    for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++)
    {
        /* 开漏输出写 1 表示释放 SDA，写 0 表示将 SDA 拉低。 */
        HAL_GPIO_WritePin(GPIOB, INA219_SDA_PIN,
                          ((data & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        INA219_I2C_Delay();
        HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN, GPIO_PIN_SET);
        INA219_I2C_Delay();
        HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN, GPIO_PIN_RESET);
        data <<= 1U;
    }

    /* 释放 SDA，由 INA219 在第九个时钟周期输出应答位。 */
    HAL_GPIO_WritePin(GPIOB, INA219_SDA_PIN, GPIO_PIN_SET);
    INA219_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN, GPIO_PIN_SET);
    INA219_I2C_Delay();
    const uint8_t acknowledged =
        (HAL_GPIO_ReadPin(GPIOB, INA219_SDA_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN, GPIO_PIN_RESET);
    INA219_I2C_Delay();

    return acknowledged;
}

/**
 * @brief 从 INA219 接收一个字节，并发送 ACK 或 NACK。
 * @param acknowledge 非零时发送 ACK，零时发送 NACK 结束读取。
 * @return 从 INA219 读取到的数据字节。
 */
static uint8_t INA219_I2C_ReadByte(uint8_t acknowledge)
{
    uint8_t data = 0U;

    /* 释放 SDA，让 INA219 驱动数据线；数据仍按最高位优先接收。 */
    HAL_GPIO_WritePin(GPIOB, INA219_SDA_PIN, GPIO_PIN_SET);
    for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++)
    {
        data <<= 1U;
        INA219_I2C_Delay();
        HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN, GPIO_PIN_SET);
        INA219_I2C_Delay();
        if (HAL_GPIO_ReadPin(GPIOB, INA219_SDA_PIN) == GPIO_PIN_SET)
        {
            data |= 1U;
        }
        HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN, GPIO_PIN_RESET);
        INA219_I2C_Delay();
    }

    /* 读取最后一位后，主机发送应答；最后一个字节发送 NACK 表示读取结束。 */
    HAL_GPIO_WritePin(GPIOB, INA219_SDA_PIN,
                      (acknowledge != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    INA219_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN, GPIO_PIN_SET);
    INA219_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, INA219_SDA_PIN, GPIO_PIN_SET);
    INA219_I2C_Delay();

    return data;
}

/**
 * @brief 向 INA219 的指定寄存器写入一个 16 位数值。
 * @param reg 寄存器地址。
 * @param value 要写入的 16 位数值，高字节先发送。
 * @return 所有字节均收到 ACK 时返回 HAL_OK，否则返回 HAL_ERROR。
 */
static HAL_StatusTypeDef INA219_WriteRegister(uint8_t reg, uint16_t value)
{
    INA219_I2C_Start();
    uint8_t acknowledged =
        INA219_I2C_WriteByte((uint8_t)(INA219_I2C_ADDRESS << 1U));
    if (acknowledged != 0U)
    {
        acknowledged &= INA219_I2C_WriteByte(reg);
    }
    if (acknowledged != 0U)
    {
        acknowledged &= INA219_I2C_WriteByte((uint8_t)(value >> 8U));
    }
    if (acknowledged != 0U)
    {
        acknowledged &= INA219_I2C_WriteByte((uint8_t)value);
    }
    INA219_I2C_Stop();

    return (acknowledged != 0U) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief 从 INA219 的指定寄存器读取一个 16 位数值。
 * @param reg 寄存器地址。
 * @param value 用于接收读取值的地址。
 * @return 传输成功返回 HAL_OK；参数无效或任一写地址未应答返回 HAL_ERROR。
 */
static HAL_StatusTypeDef INA219_ReadRegister(uint8_t reg, uint16_t *value)
{
    if (value == NULL)
    {
        return HAL_ERROR;
    }

    /* 先通过写地址选择待读寄存器，随后使用重复起始切换为读方向。 */
    INA219_I2C_Start();
    if (INA219_I2C_WriteByte((uint8_t)(INA219_I2C_ADDRESS << 1U)) == 0U)
    {
        INA219_I2C_Stop();
        return HAL_ERROR;
    }
    if (INA219_I2C_WriteByte(reg) == 0U)
    {
        INA219_I2C_Stop();
        return HAL_ERROR;
    }

    INA219_I2C_Start();
    const uint8_t read_address =
        (uint8_t)((INA219_I2C_ADDRESS << 1U) | 0x01U);
    if (INA219_I2C_WriteByte(read_address) == 0U)
    {
        INA219_I2C_Stop();
        return HAL_ERROR;
    }

    const uint8_t high_byte = INA219_I2C_ReadByte(1U);
    const uint8_t low_byte = INA219_I2C_ReadByte(0U);
    INA219_I2C_Stop();
    *value = (uint16_t)(((uint16_t)high_byte << 8U) | low_byte);

    return HAL_OK;
}

/**
 * @brief 初始化 INA219 的软件 I2C 引脚并启动连续电压测量。
 * @return 配置寄存器写入成功返回 HAL_OK，通信失败返回 HAL_ERROR。
 */
HAL_StatusTypeDef INA219_Init(void)
{
    /* 使用 GPIO 开漏输出与 OLED 共用 PB6/PB7；外部上拉维持总线空闲高电平。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = INA219_SCL_PIN | INA219_SDA_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    HAL_GPIO_WritePin(GPIOB, INA219_SCL_PIN | INA219_SDA_PIN, GPIO_PIN_SET);

    /* 将量程设置为 32 V，并启用分流与总线电压的连续采样模式。 */
    return INA219_WriteRegister(INA219_REG_CONFIG, INA219_CONFIG_VALUE);
}

/**
 * @brief 读取总线电压寄存器并换算为伏特。
 * @return 总线电压；I2C 通信失败时返回 NAN，供调用者显式处理错误。
 */
float INA219_ReadBusVoltage(void)
{
    uint16_t register_value = 0U;

    if (INA219_ReadRegister(INA219_REG_BUS_VOLTAGE, &register_value) != HAL_OK)
    {
        return NAN;
    }

    /* 位 [15:3] 是以 4 mV 为单位的总线电压值，低三位为状态位。 */
    const uint16_t bus_voltage_counts = (uint16_t)(register_value >> 3U);
    return (float)bus_voltage_counts * INA219_BUS_VOLTAGE_LSB_V;
}
