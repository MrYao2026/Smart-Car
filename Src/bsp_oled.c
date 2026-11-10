/**
 * @file    bsp_oled.c
 * @brief   SSD1306 OLED 软件 I2C 驱动，仅提供所需的 ASCII 字符显示。
 */

#include "bsp_oled.h"
#include <string.h>

/* SSD1306 常用 7 位地址为 0x3C；软件 I2C 发送时需左移一位，写地址为 0x78。 */
#define OLED_I2C_WRITE_ADDRESS 0x78U
/* I2C 的 SDA、SCL 分别连接到工程预留的 PB7、PB6 引脚。 */
#define OLED_SDA_PIN GPIO_PIN_7
#define OLED_SCL_PIN GPIO_PIN_6
/* 128 像素宽屏最多可容纳 21 个 6 像素宽的字符。 */
#define OLED_MAX_LINE_LENGTH 21U

/**
 * @brief 提供主循环使用的 ASCII 字符 5x7 字模。
 * @param character 要查询的字符。
 * @return 字模列数据；不支持的字符返回空白字模。
 */
static const uint8_t *OLED_GetGlyph(char character)
{
    /* 修改说明：补充 Timeout 文本所需的字母字模，确保 OLED 超时状态可读。 */
    static const uint8_t blank[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t digit_0[5] = {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU};
    static const uint8_t digit_1[5] = {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U};
    static const uint8_t digit_2[5] = {0x42U, 0x61U, 0x51U, 0x49U, 0x46U};
    static const uint8_t digit_3[5] = {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U};
    static const uint8_t digit_4[5] = {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U};
    static const uint8_t digit_5[5] = {0x27U, 0x45U, 0x45U, 0x45U, 0x39U};
    static const uint8_t digit_6[5] = {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U};
    static const uint8_t digit_7[5] = {0x01U, 0x71U, 0x09U, 0x05U, 0x03U};
    static const uint8_t digit_8[5] = {0x36U, 0x49U, 0x49U, 0x49U, 0x36U};
    static const uint8_t digit_9[5] = {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU};
    static const uint8_t uppercase_a[5] = {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU};
    static const uint8_t uppercase_b[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U};
    static const uint8_t uppercase_c[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
    static const uint8_t uppercase_d[5] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
    static const uint8_t uppercase_e[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
    static const uint8_t uppercase_f[5] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U};
    static const uint8_t uppercase_g[5] = {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU};
    static const uint8_t uppercase_h[5] = {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU};
    static const uint8_t uppercase_i[5] = {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U};
    static const uint8_t uppercase_j[5] = {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U};
    static const uint8_t uppercase_k[5] = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U};
    static const uint8_t uppercase_l[5] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
    static const uint8_t uppercase_m[5] = {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU};
    static const uint8_t uppercase_n[5] = {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU};
    static const uint8_t uppercase_o[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
    static const uint8_t uppercase_p[5] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U};
    static const uint8_t uppercase_q[5] = {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU};
    static const uint8_t uppercase_r[5] = {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U};
    static const uint8_t uppercase_s[5] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U};
    static const uint8_t uppercase_t[5] = {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U};
    static const uint8_t uppercase_u[5] = {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU};
    static const uint8_t uppercase_v[5] = {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU};
    static const uint8_t uppercase_w[5] = {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU};
    static const uint8_t uppercase_x[5] = {0x63U, 0x14U, 0x08U, 0x14U, 0x63U};
    static const uint8_t uppercase_y[5] = {0x07U, 0x08U, 0x70U, 0x08U, 0x07U};
    static const uint8_t uppercase_z[5] = {0x61U, 0x51U, 0x49U, 0x45U, 0x43U};
    static const uint8_t lowercase_a[5] = {0x20U, 0x54U, 0x54U, 0x54U, 0x78U};
    static const uint8_t lowercase_b[5] = {0x7FU, 0x48U, 0x44U, 0x44U, 0x38U};
    static const uint8_t lowercase_c[5] = {0x38U, 0x44U, 0x44U, 0x44U, 0x20U};
    static const uint8_t lowercase_d[5] = {0x40U, 0x44U, 0x44U, 0x48U, 0x7FU};
    static const uint8_t lowercase_e[5] = {0x38U, 0x54U, 0x54U, 0x54U, 0x18U};
    static const uint8_t lowercase_f[5] = {0x08U, 0x7EU, 0x09U, 0x01U, 0x02U};
    static const uint8_t lowercase_g[5] = {0x08U, 0x14U, 0x54U, 0x54U, 0x3CU};
    static const uint8_t lowercase_h[5] = {0x7FU, 0x08U, 0x04U, 0x04U, 0x78U};
    static const uint8_t lowercase_i[5] = {0x00U, 0x44U, 0x7DU, 0x40U, 0x00U};
    static const uint8_t lowercase_j[5] = {0x20U, 0x40U, 0x44U, 0x3DU, 0x00U};
    static const uint8_t lowercase_k[5] = {0x7FU, 0x10U, 0x28U, 0x44U, 0x00U};
    static const uint8_t lowercase_l[5] = {0x00U, 0x41U, 0x7FU, 0x40U, 0x00U};
    static const uint8_t lowercase_m[5] = {0x7CU, 0x04U, 0x18U, 0x04U, 0x78U};
    static const uint8_t lowercase_n[5] = {0x7CU, 0x08U, 0x04U, 0x04U, 0x78U};
    static const uint8_t lowercase_o[5] = {0x38U, 0x44U, 0x44U, 0x44U, 0x38U};
    static const uint8_t lowercase_p[5] = {0x7CU, 0x14U, 0x14U, 0x14U, 0x08U};
    static const uint8_t lowercase_q[5] = {0x08U, 0x14U, 0x14U, 0x18U, 0x7FU};
    static const uint8_t lowercase_r[5] = {0x7CU, 0x08U, 0x04U, 0x04U, 0x08U};
    static const uint8_t lowercase_s[5] = {0x48U, 0x54U, 0x54U, 0x54U, 0x20U};
    static const uint8_t lowercase_t[5] = {0x04U, 0x3FU, 0x44U, 0x40U, 0x20U};
    static const uint8_t lowercase_u[5] = {0x3CU, 0x40U, 0x40U, 0x20U, 0x7CU};
    static const uint8_t lowercase_v[5] = {0x1CU, 0x20U, 0x40U, 0x20U, 0x1CU};
    static const uint8_t lowercase_w[5] = {0x3CU, 0x40U, 0x30U, 0x40U, 0x3CU};
    static const uint8_t lowercase_x[5] = {0x44U, 0x28U, 0x10U, 0x28U, 0x44U};
    static const uint8_t lowercase_y[5] = {0x0CU, 0x50U, 0x50U, 0x50U, 0x3CU};
    static const uint8_t lowercase_z[5] = {0x44U, 0x64U, 0x54U, 0x4CU, 0x44U};
    static const uint8_t colon[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
    static const uint8_t period[5] = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
    static const uint8_t exclamation[5] = {0x00U, 0x00U, 0x5FU, 0x00U, 0x00U};
    static const uint8_t left_parenthesis[5] = {0x00U, 0x1CU, 0x22U, 0x41U, 0x00U};
    static const uint8_t right_parenthesis[5] = {0x00U, 0x41U, 0x22U, 0x1CU, 0x00U};
    static const uint8_t caret[5] = {0x08U, 0x04U, 0x02U, 0x04U, 0x08U};
    static const uint8_t underscore[5] = {0x40U, 0x40U, 0x40U, 0x40U, 0x40U};

    /* 数字字符连续排列，通过下标快速选择对应的数字字模。 */
    static const uint8_t *const digits[10] = {
        digit_0, digit_1, digit_2, digit_3, digit_4,
        digit_5, digit_6, digit_7, digit_8, digit_9
    };

    /* 数字字符按连续 ASCII 编码排列，直接返回对应的数字字模。 */
    if ((character >= '0') && (character <= '9'))
    {
        return digits[(uint8_t)(character - '0')];
    }

    /* 返回当前距离字符串所需的字母、标点符号字模。 */
    switch (character)
    {
        case 'A': return uppercase_a;
        case 'B': return uppercase_b;
        case 'C': return uppercase_c;
        case 'D': return uppercase_d;
        case 'E': return uppercase_e;
        case 'F': return uppercase_f;
        case 'G': return uppercase_g;
        case 'H': return uppercase_h;
        case 'I': return uppercase_i;
        case 'J': return uppercase_j;
        case 'K': return uppercase_k;
        case 'L': return uppercase_l;
        case 'M': return uppercase_m;
        case 'N': return uppercase_n;
        case 'O': return uppercase_o;
        case 'P': return uppercase_p;
        case 'Q': return uppercase_q;
        case 'R': return uppercase_r;
        case 'S': return uppercase_s;
        case 'T': return uppercase_t;
        case 'U': return uppercase_u;
        case 'V': return uppercase_v;
        case 'W': return uppercase_w;
        case 'X': return uppercase_x;
        case 'Y': return uppercase_y;
        case 'Z': return uppercase_z;
        case 'a': return lowercase_a;
        case 'b': return lowercase_b;
        case 'c': return lowercase_c;
        case 'd': return lowercase_d;
        case 'e': return lowercase_e;
        case 'f': return lowercase_f;
        case 'g': return lowercase_g;
        case 'h': return lowercase_h;
        case 'i': return lowercase_i;
        case 'j': return lowercase_j;
        case 'k': return lowercase_k;
        case 'l': return lowercase_l;
        case 'm': return lowercase_m;
        case 'n': return lowercase_n;
        case 'o': return lowercase_o;
        case 'p': return lowercase_p;
        case 'q': return lowercase_q;
        case 'r': return lowercase_r;
        case 's': return lowercase_s;
        case 't': return lowercase_t;
        case 'u': return lowercase_u;
        case 'v': return lowercase_v;
        case 'w': return lowercase_w;
        case 'x': return lowercase_x;
        case 'y': return lowercase_y;
        case 'z': return lowercase_z;
        case ':': return colon;
        case '.': return period;
        case '!': return exclamation;
        case '(': return left_parenthesis;
        case ')': return right_parenthesis;
        case '^': return caret;
        case '_': return underscore;
        case ' ': return blank;
        default:  return blank;
    }
}

/**
 * @brief 提供 I2C 软件时序所需的短暂建立时间。
 */
static void OLED_I2C_Delay(void)
{
    /* 通过短暂的 volatile 计数循环生成 GPIO 电平切换间隔。 */
    for (volatile uint8_t delay_count = 0U; delay_count < 8U; delay_count++)
    {
    }
}

/**
 * @brief 产生 I2C 起始条件。
 */
static void OLED_I2C_Start(void)
{
    /* 先释放两条总线，再在 SCL 为高时拉低 SDA，产生起始条件。 */
    HAL_GPIO_WritePin(GPIOB, OLED_SDA_PIN | OLED_SCL_PIN, GPIO_PIN_SET);
    OLED_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, OLED_SDA_PIN, GPIO_PIN_RESET);
    OLED_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, OLED_SCL_PIN, GPIO_PIN_RESET);
    OLED_I2C_Delay();
}

/**
 * @brief 产生 I2C 停止条件。
 */
static void OLED_I2C_Stop(void)
{
    /* SCL 保持低时先拉低 SDA，再释放 SCL 并释放 SDA，结束本次传输。 */
    HAL_GPIO_WritePin(GPIOB, OLED_SCL_PIN | OLED_SDA_PIN, GPIO_PIN_RESET);
    OLED_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, OLED_SCL_PIN, GPIO_PIN_SET);
    OLED_I2C_Delay();
    HAL_GPIO_WritePin(GPIOB, OLED_SDA_PIN, GPIO_PIN_SET);
    OLED_I2C_Delay();
}

/**
 * @brief 通过 PB7 发送一个字节并读取 SSD1306 的应答位。
 * @param data 待发送的字节。
 * @return 收到从机应答返回 1，否则返回 0。
 */
static uint8_t OLED_I2C_WriteByte(uint8_t data)
{
    /* 按最高位优先的 I2C 规则依次输出 8 个数据位。 */
    for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++)
    {
        /* 当前数据位为 1 时释放 SDA，为 0 时主动将 SDA 拉低。 */
        HAL_GPIO_WritePin(GPIOB, OLED_SDA_PIN,
                          ((data & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        OLED_I2C_Delay();
        /* 拉高 SCL，通知 OLED 采样当前数据位。 */
        HAL_GPIO_WritePin(GPIOB, OLED_SCL_PIN, GPIO_PIN_SET);
        OLED_I2C_Delay();
        /* 拉低 SCL，为下一位数据建立时钟低电平。 */
        HAL_GPIO_WritePin(GPIOB, OLED_SCL_PIN, GPIO_PIN_RESET);
        /* 左移数据，使下一个待发送位移动到最高位。 */
        data <<= 1U;
    }

    /* 释放 SDA 供 OLED 在第九个时钟周期输出应答位。 */
    HAL_GPIO_WritePin(GPIOB, OLED_SDA_PIN, GPIO_PIN_SET);
    OLED_I2C_Delay();
    /* 拉高 SCL，使 OLED 可以输出 ACK 或 NACK。 */
    HAL_GPIO_WritePin(GPIOB, OLED_SCL_PIN, GPIO_PIN_SET);
    OLED_I2C_Delay();
    /* SDA 被 OLED 拉低表示 ACK；保持 SCL 高时读取该应答电平。 */
    const uint8_t acknowledged =
        (HAL_GPIO_ReadPin(GPIOB, OLED_SDA_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    /* 拉低 SCL 结束应答位时钟，并为后续传输做好准备。 */
    HAL_GPIO_WritePin(GPIOB, OLED_SCL_PIN, GPIO_PIN_RESET);
    OLED_I2C_Delay();

    /* 将应答状态交给调用者检查，避免忽略 OLED 无应答故障。 */
    return acknowledged;
}

/**
 * @brief 发送一条 SSD1306 命令。
 * @param command 要写入 SSD1306 的控制命令。
 * @return 从机正确应答返回 HAL_OK，否则返回 HAL_ERROR。
 */
static HAL_StatusTypeDef OLED_WriteCommand(uint8_t command)
{
    /* 每笔命令传输均由起始条件、从机地址、命令控制字节和命令组成。 */
    OLED_I2C_Start();
    const uint8_t address_ack = OLED_I2C_WriteByte(OLED_I2C_WRITE_ADDRESS);
    const uint8_t control_ack = OLED_I2C_WriteByte(0x00U);
    const uint8_t command_ack = OLED_I2C_WriteByte(command);
    /* 无论命令应答是否成功，都发送停止条件释放 I2C 总线。 */
    OLED_I2C_Stop();

    /* 任一传输阶段未收到应答都报告错误，便于上层进入错误处理。 */
    return ((address_ack != 0U) && (control_ack != 0U) &&
            (command_ack != 0U)) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief 设置 SSD1306 的页地址和列地址。
 * @param page 要设置的显示页，范围为 0 至 7。
 * @return 地址设置成功返回 HAL_OK，否则返回 HAL_ERROR。
 */
static HAL_StatusTypeDef OLED_SetPosition(uint8_t page)
{
    /* 页地址由 0xB0 加页号构成，列地址先从第 0 列开始。 */
    if (OLED_WriteCommand((uint8_t)(0xB0U | page)) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (OLED_WriteCommand(0x00U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (OLED_WriteCommand(0x10U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 成功完成页地址与高低列地址设置后返回成功状态。 */
    return HAL_OK;
}

/**
 * @brief 初始化 PB6/PB7 软件 I2C 及 SSD1306 显示屏。
 * @return 初始化成功返回 HAL_OK；I2C 无应答时返回 HAL_ERROR。
 */
HAL_StatusTypeDef OLED_Init(void)
{
    /* 打开 GPIOB 时钟，使 PB6 和 PB7 可以作为软件 I2C 引脚使用。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 设置 PB6/PB7 为开漏输出，并启用内部上拉作为外部上拉的辅助。 */
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = OLED_SDA_PIN | OLED_SCL_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    /* 开漏总线空闲时应释放 SDA 和 SCL，保持两条总线为高电平。 */
    HAL_GPIO_WritePin(GPIOB, OLED_SDA_PIN | OLED_SCL_PIN, GPIO_PIN_SET);

    /* 依次发送 SSD1306 初始化命令，配置 128x64 屏幕及电荷泵。 */
    static const uint8_t initialization_commands[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0x8DU, 0x14U, 0x20U, 0x02U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
        0x81U, 0x7FU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U, 0xAFU
    };

    /* 逐条发送初始化命令；任何命令无应答都立即返回错误。 */
    for (uint8_t command_index = 0U;
         command_index < (uint8_t)sizeof(initialization_commands);
         command_index++)
    {
        if (OLED_WriteCommand(initialization_commands[command_index]) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    /* 初始化完成后清空八个显示页，避免上电随机显存内容残留。 */
    for (uint8_t page = 0U; page < 8U; page++)
    {
        if (OLED_SetPosition(page) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* 每页包含 128 列，逐列写入 0 清除显存像素。 */
        OLED_I2C_Start();
        uint8_t acknowledged = OLED_I2C_WriteByte(OLED_I2C_WRITE_ADDRESS);
        acknowledged &= OLED_I2C_WriteByte(0x40U);
        for (uint8_t column = 0U; column < 128U; column++)
        {
            acknowledged &= OLED_I2C_WriteByte(0x00U);
        }
        OLED_I2C_Stop();

        /* 页清除期间任何字节无应答均报告初始化失败。 */
        if (acknowledged == 0U)
        {
            return HAL_ERROR;
        }
    }

    /* 所有初始化与清屏操作成功完成。 */
    return HAL_OK;
}

/**
 * @brief 在指定页显示 ASCII 字符串。
 * @param page OLED 页编号，范围为 0 至 7；页 0 是屏幕第一行。
 * @param text 以空字符结尾的 ASCII 字符串。
 * @return 显示成功返回 HAL_OK；参数无效或 I2C 无应答时返回 HAL_ERROR。
 */
HAL_StatusTypeDef OLED_ShowString(uint8_t page, const char *text)
{
    /* 检查页号和字符串指针，防止无效参数造成越界或非法访问。 */
    if ((page >= 8U) || (text == NULL))
    {
        return HAL_ERROR;
    }

    /* 仅支持一行可完整显示的字符串，避免字符写入超出屏幕范围。 */
    const size_t text_length = strlen(text);
    if (text_length > OLED_MAX_LINE_LENGTH)
    {
        return HAL_ERROR;
    }

    /* 将光标定位在指定页的最左侧列。 */
    if (OLED_SetPosition(page) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 开始一次连续数据传输，以减少逐字符设置地址造成的总线开销。 */
    OLED_I2C_Start();
    uint8_t acknowledged = OLED_I2C_WriteByte(OLED_I2C_WRITE_ADDRESS);
    acknowledged &= OLED_I2C_WriteByte(0x40U);

    /* 逐字符发送 5 列字模，并在每个字符后补一列空白作为间距。 */
    for (size_t text_index = 0U; text_index < text_length; text_index++)
    {
        const uint8_t *glyph = OLED_GetGlyph(text[text_index]);
        for (uint8_t column = 0U; column < 5U; column++)
        {
            acknowledged &= OLED_I2C_WriteByte(glyph[column]);
        }
        acknowledged &= OLED_I2C_WriteByte(0x00U);
    }

    /* 补齐剩余字符列为空白，清除上一条更长消息留下的尾部像素。 */
    for (size_t blank_index = text_length; blank_index < OLED_MAX_LINE_LENGTH;
         blank_index++)
    {
        for (uint8_t column = 0U; column < 6U; column++)
        {
            acknowledged &= OLED_I2C_WriteByte(0x00U);
        }
    }

    /* 字符数据发送结束后释放总线。 */
    OLED_I2C_Stop();

    /* 返回整行传输的应答状态，避免静默掩盖地址或连线问题。 */
    return (acknowledged != 0U) ? HAL_OK : HAL_ERROR;
}
