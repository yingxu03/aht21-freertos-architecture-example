/*******************************************************************************
 * @file    bsp_aht21_driver.c
 * @author  yingxu031
 * @version V1.0.0
 * @date    2025-11-xx
 * @brief   AHT21温湿度传感器驱动实现文件
 * 
 * @note    此驱动采用面向接口编程设计，支持裸机与RTOS环境
 *          通过依赖注入方式解耦硬件依赖，提高代码可移植性
 *          
 ******************************************************************************/

#include "bsp_aht21_driver.h"
#include <string.h>

/****************************** 内部宏定义 ************************************/

/* AHT21传感器地址 */
#define AHT21_I2C_ADDRESS         0x38

/* 命令定义 */
#define AHT21_CMD_INIT            0xBE    /* 初始化命令 */
#define AHT21_CMD_MEASURE         0xAC    /* 测量命令 */
#define AHT21_CMD_SOFT_RESET      0xBA    /* 软复位命令 */
#define AHT21_CMD_CALIBRATE       0xE1    /* 校准命令 */

/* 状态位定义 */
#define AHT21_STATUS_BUSY         (1 << 7)    /* 忙标志位 */
#define AHT21_STATUS_CALIBRATED   (1 << 3)    /* 校准标志位 */

/* 延时时间定义（单位：ms） */
#define AHT21_DELAY_INIT          10      /* 初始化延时 */
#define AHT21_DELAY_MEASURE       75      /* 测量延时 */
#define AHT21_DELAY_RESET         20      /* 复位延时 */

/* 校验常量 */
#define AHT21_ID_EXPECTED         0x15    /* 预期ID值 */

/****************************** 内部静态函数声明 ******************************/

static int8_t aht21_check_status(bsp_aht21_driver_t *aht21_instance);
static int8_t aht21_send_command(bsp_aht21_driver_t *aht21_instance, 
                                 uint8_t cmd, uint8_t data1, uint8_t data2);
static int8_t aht21_wait_for_ready(bsp_aht21_driver_t *aht21_instance, 
                                   uint32_t timeout_ms);
static int8_t aht21_read_raw_data(bsp_aht21_driver_t *aht21_instance, 
                                  uint8_t *raw_data);
static float aht21_convert_temperature(uint8_t *raw_data);
static float aht21_convert_humidity(uint8_t *raw_data);

/****************************** 构造函数实现 **********************************/

/**
 * @brief  AHT21驱动实例构造函数
 */
int8_t aht21_inst(
    bsp_aht21_driver_t *aht21_instance,
    lic_driver_interface_t *lic_instance,
    timebase_interface_t *timebase,
#if OS_SUPPORT_ENABLED
    yield_interface_t *rtos_yield
#else
    void *rtos_yield
#endif
)
{
    /* 参数检查 */
    if (aht21_instance == NULL || lic_instance == NULL || timebase == NULL) {
        return AHT21_ERR_PARAM;
    }
    
    /* 检查I2C接口函数指针 */
    if (lic_instance->pfinit == NULL ||
        lic_instance->pfdeinit == NULL ||
        lic_instance->pfwrite_reg == NULL ||
        lic_instance->pfread_reg == NULL) {
        return AHT21_ERR_PARAM;
    }
    
    /* 检查时基接口函数指针 */
    if (timebase->pfget_tick_count == NULL) {
        return AHT21_ERR_PARAM;
    }
    
    /* 绑定外部依赖接口 */
    aht21_instance->pricdriver_interface = lic_instance;
    aht21_instance->ptimebase_interface = timebase;
    
#if OS_SUPPORT_ENABLED
    if (rtos_yield != NULL && rtos_yield->rtos_yield != NULL) {
        aht21_instance->rtos_yield_interface = rtos_yield;
    } else {
        aht21_instance->rtos_yield_interface = NULL;
    }
#endif
    
    /* 绑定内部函数指针 */
    aht21_instance->pfinst = aht21_inst;
    aht21_instance->pfinit = aht21_init;
    aht21_instance->pfdeinit = aht21_deinit;
    aht21_instance->pfread_id = aht21_read_id;
    aht21_instance->pfread_temperature = aht21_read_temperature;
    aht21_instance->pfread_humidity = aht21_read_humidity;
    aht21_instance->pfhibernating = aht21_hibernating;
    aht21_instance->pfwakeup = aht21_wakeup;
    
    return AHT21_OK;
}

/****************************** HAL功能函数实现 *******************************/

/**
 * @brief  初始化AHT21传感器
 */
int8_t aht21_init(bsp_aht21_driver_t *aht21_instance)
{
    int8_t ret;
    uint8_t status;
    
    /* 参数检查 */
    if (aht21_instance == NULL || 
        aht21_instance->pricdriver_interface == NULL ||
        aht21_instance->ptimebase_interface == NULL) {
        return AHT21_ERR_PARAM;
    }
    
    /* 初始化I2C接口 */
    ret = aht21_instance->pricdriver_interface->pfinit();
    if (ret != 0) {
        return AHT21_ERR_I2C;
    }
    
    /* 发送软复位命令 */
    ret = aht21_send_command(aht21_instance, AHT21_CMD_SOFT_RESET, 0x00, 0x00);
    if (ret != AHT21_OK) {
        return ret;
    }
    
    /* 等待复位完成 */
    if (aht21_instance->ptimebase_interface->pfget_tick_count) {
        uint32_t start_time = aht21_instance->ptimebase_interface->pfget_tick_count();
        while ((aht21_instance->ptimebase_interface->pfget_tick_count() - start_time) < AHT21_DELAY_RESET) {
#if OS_SUPPORT_ENABLED
            if (aht21_instance->rtos_yield_interface != NULL &&
                aht21_instance->rtos_yield_interface->rtos_yield != NULL) {
                aht21_instance->rtos_yield_interface->rtos_yield();
            }
#endif
        }
    }
    
    /* 发送初始化命令 */
    ret = aht21_send_command(aht21_instance, AHT21_CMD_INIT, 0x08, 0x00);
    if (ret != AHT21_OK) {
        return ret;
    }
    
    /* 等待初始化完成 */
    if (aht21_instance->ptimebase_interface->pfget_tick_count) {
        uint32_t start_time = aht21_instance->ptimebase_interface->pfget_tick_count();
        while ((aht21_instance->ptimebase_interface->pfget_tick_count() - start_time) < AHT21_DELAY_INIT) {
#if OS_SUPPORT_ENABLED
            if (aht21_instance->rtos_yield_interface != NULL &&
                aht21_instance->rtos_yield_interface->rtos_yield != NULL) {
                aht21_instance->rtos_yield_interface->rtos_yield();
            }
#endif
        }
    }
    
    /* 检查传感器状态 */
    ret = aht21_check_status(aht21_instance);
    if (ret != AHT21_OK) {
        return ret;
    }
    
    return AHT21_OK;
}

/**
 * @brief  去初始化AHT21传感器
 */
int8_t aht21_deinit(bsp_aht21_driver_t *aht21_instance)
{
    /* 参数检查 */
    if (aht21_instance == NULL || 
        aht21_instance->pricdriver_interface == NULL) {
        return AHT21_ERR_PARAM;
    }
    
    /* 使传感器进入休眠模式 */
    int8_t ret = aht21_hibernating(aht21_instance);
    if (ret != AHT21_OK) {
        return ret;
    }
    
    /* 去初始化I2C接口 */
    ret = aht21_instance->pricdriver_interface->pfdeinit();
    if (ret != 0) {
        return AHT21_ERR_I2C;
    }
    
    return AHT21_OK;
}

/**
 * @brief  读取AHT21传感器ID
 */
int8_t aht21_read_id(bsp_aht21_driver_t *aht21_instance)
{
    int8_t ret;
    uint8_t reg_addr = 0x71;  /* ID寄存器地址 */
    uint8_t id_buffer[3] = {0};
    
    /* 参数检查 */
    if (aht21_instance == NULL || 
        aht21_instance->pricdriver_interface == NULL) {
        return AHT21_ERR_PARAM;
    }
    
    /* 读取ID寄存器 */
    ret = aht21_instance->pricdriver_interface->pfread_reg(AHT21_I2C_ADDRESS, 
                                                          id_buffer, 
                                                          sizeof(id_buffer));
    if (ret != 0) {
        return AHT21_ERR_I2C;
    }
    
    /* 验证ID */
    if (id_buffer[0] == AHT21_ID_EXPECTED) {
        return (int8_t)id_buffer[0];
    }
    
    return AHT21_ERR_CHECKSUM;
}

/**
 * @brief  读取温度值
 */
int8_t aht21_read_temperature(bsp_aht21_driver_t *aht21_instance, float *temp)
{
    int8_t ret;
    uint8_t raw_data[6] = {0};
    
    /* 参数检查 */
    if (aht21_instance == NULL || temp == NULL) {
        return AHT21_ERR_PARAM;
    }
    
    /* 发送测量命令 */
    ret = aht21_send_command(aht21_instance, AHT21_CMD_MEASURE, 0x33, 0x00);
    if (ret != AHT21_OK) {
        return ret;
    }
    
    /* 等待测量完成 */
    ret = aht21_wait_for_ready(aht21_instance, AHT21_DELAY_MEASURE);
    if (ret != AHT21_OK) {
        return ret;
    }
    
    /* 读取原始数据 */
    ret = aht21_read_raw_data(aht21_instance, raw_data);
    if (ret != AHT21_OK) {
        return ret;
    }
    
    /* 转换温度值 */
    *temp = aht21_convert_temperature(raw_data);
    
    return AHT21_OK;
}

/**
 * @brief  读取湿度值
 */
int8_t aht21_read_humidity(bsp_aht21_driver_t *aht21_instance, float *humidity)
{
    int8_t ret;
    uint8_t raw_data[6] = {0};
    
    /* 参数检查 */
    if (aht21_instance == NULL || humidity == NULL) {
        return AHT21_ERR_PARAM;
    }
    
    /* 发送测量命令 */
    ret = aht21_send_command(aht21_instance, AHT21_CMD_MEASURE, 0x33, 0x00);
    if (ret != AHT21_OK) {
        return ret;
    }
    
    /* 等待测量完成 */
    ret = aht21_wait_for_ready(aht21_instance, AHT21_DELAY_MEASURE);
    if (ret != AHT21_OK) {
        return ret;
    }
    
    /* 读取原始数据 */
    ret = aht21_read_raw_data(aht21_instance, raw_data);
    if (ret != AHT21_OK) {
        return ret;
    }
    
    /* 转换湿度值 */
    *humidity = aht21_convert_humidity(raw_data);
    
    return AHT21_OK;
}

/**
 * @brief  使AHT21传感器进入休眠模式
 */
int8_t aht21_hibernating(bsp_aht21_driver_t *aht21_instance)
{
    /* AHT21没有专门的休眠命令，通过停止测量来实现节能 */
    /* 可以在此处添加降低I2C频率或关闭电源等操作 */
    return AHT21_OK;
}

/**
 * @brief  唤醒AHT21传感器
 */
int8_t aht21_wakeup(bsp_aht21_driver_t *aht21_instance)
{
    /* AHT21从休眠唤醒后需要重新初始化 */
    return aht21_init(aht21_instance);
}

/****************************** 内部静态函数实现 ******************************/

/**
 * @brief  检查传感器状态
 */
static int8_t aht21_check_status(bsp_aht21_driver_t *aht21_instance)
{
    uint8_t status_reg[3] = {0};
    int8_t ret;
    
    /* 读取状态寄存器 */
    ret = aht21_instance->pricdriver_interface->pfread_reg(AHT21_I2C_ADDRESS, 
                                                          status_reg, 
                                                          sizeof(status_reg));
    if (ret != 0) {
        return AHT21_ERR_I2C;
    }
    
    /* 检查忙标志 */
    if (status_reg[0] & AHT21_STATUS_BUSY) {
        return AHT21_ERR_BUSY;
    }
    
    /* 检查校准标志 */
    if (!(status_reg[0] & AHT21_STATUS_CALIBRATED)) {
        /* 传感器未校准，需要进行校准 */
        ret = aht21_send_command(aht21_instance, AHT21_CMD_CALIBRATE, 0x08, 0x00);
        if (ret != AHT21_OK) {
            return ret;
        }
        
        /* 等待校准完成 */
        if (aht21_instance->ptimebase_interface->pfget_tick_count) {
            uint32_t start_time = aht21_instance->ptimebase_interface->pfget_tick_count();
            while ((aht21_instance->ptimebase_interface->pfget_tick_count() - start_time) < AHT21_DELAY_INIT) {
#if OS_SUPPORT_ENABLED
                if (aht21_instance->rtos_yield_interface != NULL &&
                    aht21_instance->rtos_yield_interface->rtos_yield != NULL) {
                    aht21_instance->rtos_yield_interface->rtos_yield();
                }
#endif
            }
        }
    }
    
    return AHT21_OK;
}

/**
 * @brief  发送命令到传感器
 */
static int8_t aht21_send_command(bsp_aht21_driver_t *aht21_instance, 
                                uint8_t cmd, uint8_t data1, uint8_t data2)
{
    uint8_t cmd_buffer[3] = {cmd, data1, data2};
    int8_t ret;
    
    ret = aht21_instance->pricdriver_interface->pfwrite_reg(AHT21_I2C_ADDRESS, 
                                                           cmd_buffer, 
                                                           sizeof(cmd_buffer));
    if (ret != 0) {
        return AHT21_ERR_I2C;
    }
    
    return AHT21_OK;
}

/**
 * @brief  等待传感器准备就绪
 */
static int8_t aht21_wait_for_ready(bsp_aht21_driver_t *aht21_instance, 
                                  uint32_t timeout_ms)
{
    uint8_t status;
    uint32_t start_time = 0;
    uint32_t current_time = 0;
    
    /* 获取起始时间 */
    if (aht21_instance->ptimebase_interface->pfget_tick_count) {
        start_time = aht21_instance->ptimebase_interface->pfget_tick_count();
    }
    
    /* 轮询状态寄存器，等待测量完成 */
    do {
        /* 读取状态寄存器 */
        int8_t ret = aht21_instance->pricdriver_interface->pfread_reg(AHT21_I2C_ADDRESS, 
                                                                     &status, 
                                                                     1);
        if (ret != 0) {
            return AHT21_ERR_I2C;
        }
        
        /* 检查忙标志 */
        if (!(status & AHT21_STATUS_BUSY)) {
            return AHT21_OK;
        }
        
        /* 让出CPU */
#if OS_SUPPORT_ENABLED
        if (aht21_instance->rtos_yield_interface != NULL &&
            aht21_instance->rtos_yield_interface->rtos_yield != NULL) {
            aht21_instance->rtos_yield_interface->rtos_yield();
        }
#endif
        
        /* 检查超时 */
        if (aht21_instance->ptimebase_interface->pfget_tick_count) {
            current_time = aht21_instance->ptimebase_interface->pfget_tick_count();
            if ((current_time - start_time) >= timeout_ms) {
                return AHT21_ERR_TIMEOUT;
            }
        }
        
    } while (1);
}

/**
 * @brief  读取原始数据
 */
static int8_t aht21_read_raw_data(bsp_aht21_driver_t *aht21_instance, 
                                 uint8_t *raw_data)
{
    int8_t ret;
    
    /* 读取6字节原始数据（状态+湿度+温度+校验和） */
    ret = aht21_instance->pricdriver_interface->pfread_reg(AHT21_I2C_ADDRESS, 
                                                          raw_data, 
                                                          6);
    if (ret != 0) {
        return AHT21_ERR_I2C;
    }
    
    /* 检查忙标志 */
    if (raw_data[0] & AHT21_STATUS_BUSY) {
        return AHT21_ERR_BUSY;
    }
    
    /* 简单校验和检查（实际AHT21没有校验和，这里可以添加CRC校验） */
    /* 注意：AHT21实际没有校验字节，这里保持兼容性 */
    
    return AHT21_OK;
}

/**
 * @brief  转换温度值
 */
static float aht21_convert_temperature(uint8_t *raw_data)
{
    uint32_t temp_raw = 0;
    float temperature;
    
    /* 提取温度原始数据（第3-5字节，20位） */
    temp_raw = ((uint32_t)(raw_data[3] & 0x0F) << 16) |
               ((uint32_t)raw_data[4] << 8) |
               raw_data[5];
    
    /* 转换为摄氏度 */
    temperature = (float)temp_raw * 200.0f / 1048576.0f - 50.0f;
    
    return temperature;
}

/**
 * @brief  转换湿度值
 */
static float aht21_convert_humidity(uint8_t *raw_data)
{
    uint32_t humi_raw = 0;
    float humidity;
    
    /* 提取湿度原始数据（第1-3字节，20位） */
    humi_raw = ((uint32_t)raw_data[1] << 12) |
               ((uint32_t)raw_data[2] << 4) |
               ((raw_data[3] & 0xF0) >> 4);
    
    /* 转换为百分比 */
    humidity = (float)humi_raw * 100.0f / 1048576.0f;
    
    /* 限制范围 0-100% */
    if (humidity > 100.0f) {
        humidity = 100.0f;
    } else if (humidity < 0.0f) {
        humidity = 0.0f;
    }
    
    return humidity;
}
