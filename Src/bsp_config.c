/**
 * @file    bsp_config.c
 * @brief   STM32F103C8T6 运行参数的内部 Flash 保存与校验。
 */

#include "bsp_config.h"
#include "main.h"

/* STM32F103C8T6 为 64 KB Flash，末尾 1 KB 页专用于参数保存。 */
#define CONFIG_FLASH_ADDRESS 0x0800FC00UL
#define CONFIG_MAGIC 0x5343U
#define CONFIG_CHECK_XOR 0xA55AU
#define CONFIG_DEFAULT_MAX_SPEED 100U
#define CONFIG_DEFAULT_AVOID_DISTANCE 15U

typedef struct
{
    uint16_t magic;
    uint16_t max_speed;
    uint16_t avoid_distance;
    uint16_t checksum;
} ConfigFlashRecord;

/* 记录布局必须是完整的四个半字，供 F1 Flash 半字写入使用。 */
_Static_assert(sizeof(ConfigFlashRecord) == 8U,
               "ConfigFlashRecord must occupy four halfwords");

/**
 * @brief 计算配置记录校验值。
 */
static uint16_t Config_Checksum(uint16_t max_speed, uint16_t avoid_distance)
{
    return (uint16_t)(CONFIG_MAGIC ^ max_speed ^ avoid_distance ^
                      CONFIG_CHECK_XOR);
}

/**
 * @brief 检查参数是否位于支持范围。
 */
static uint8_t Config_IsValid(const Config_t *config)
{
    return (uint8_t)((config != NULL) &&
                     (config->max_speed >= 1U) &&
                     (config->max_speed <= 100U) &&
                     (config->avoid_distance >= 1U) &&
                     (config->avoid_distance <= 400U));
}

/**
 * @brief 从 Flash 读取参数；未写入或数据损坏时采用安全默认值。
 */
Config_t Config_Load(void)
{
    const volatile ConfigFlashRecord *const flash_record =
        (const volatile ConfigFlashRecord *)CONFIG_FLASH_ADDRESS;
    Config_t config = {
        CONFIG_DEFAULT_MAX_SPEED,
        CONFIG_DEFAULT_AVOID_DISTANCE
    };

    if ((flash_record->magic == CONFIG_MAGIC) &&
        (flash_record->checksum ==
         Config_Checksum(flash_record->max_speed,
                         flash_record->avoid_distance)))
    {
        const Config_t stored_config = {
            flash_record->max_speed,
            flash_record->avoid_distance
        };
        if (Config_IsValid(&stored_config) != 0U)
        {
            config = stored_config;
        }
    }

    return config;
}

/**
 * @brief 擦除专用页、写入记录并回读验证。
 */
int Config_Save(const Config_t *config)
{
    if (Config_IsValid(config) == 0U)
    {
        return (int)HAL_ERROR;
    }

    const ConfigFlashRecord record = {
        CONFIG_MAGIC,
        config->max_speed,
        config->avoid_distance,
        Config_Checksum(config->max_speed, config->avoid_distance)
    };
    const uint16_t record_words[4] = {
        record.magic,
        record.max_speed,
        record.avoid_distance,
        record.checksum
    };
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    HAL_StatusTypeDef status;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return (int)HAL_ERROR;
    }

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = CONFIG_FLASH_ADDRESS;
    erase.NbPages = 1U;
    status = HAL_FLASHEx_Erase(&erase, &page_error);

    for (uint32_t word_index = 0U;
         (status == HAL_OK) && (word_index < 4U);
         word_index++)
    {
        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_HALFWORD,
            CONFIG_FLASH_ADDRESS + (word_index * sizeof(uint16_t)),
            record_words[word_index]);
    }

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        status = HAL_ERROR;
    }

    if (status != HAL_OK)
    {
        return (int)HAL_ERROR;
    }

    /* 回读并完整核验，避免将掉电或编程错误报告为成功保存。 */
    const Config_t verified = Config_Load();
    return ((verified.max_speed == config->max_speed) &&
            (verified.avoid_distance == config->avoid_distance))
               ? (int)HAL_OK
               : (int)HAL_ERROR;
}
