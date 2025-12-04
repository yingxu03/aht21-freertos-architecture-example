#include "my_i2c_driver.h"
#include "bsp_aht21_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* 全局I2C互斥锁（保护多任务访问） */
static SemaphoreHandle_t i2c_mutex = NULL;

/* I2C驱动接口实现 */
static lic_driver_interface_t my_i2c_interface = {
    .pfinit = my_i2c_init,
    .pfdeinit = my_i2c_deinit,
    .pfwrite_reg = my_i2c_write_reg,
    .pfread_reg = my_i2c_read_reg
};

/* I2C初始化 */
int8_t my_i2c_init(void)
{
    /* 初始化硬件I2C */
    // HAL_I2C_Init(&hi2c1);
    
    /* 创建互斥锁 */
    if (i2c_mutex == NULL) {
        i2c_mutex = xSemaphoreCreateMutex();
        if (i2c_mutex == NULL) {
            return -1;
        }
    }
    
    return 0;
}

/* I2C去初始化 */
int8_t my_i2c_deinit(void)
{
    /* 删除互斥锁 */
    if (i2c_mutex != NULL) {
        vSemaphoreDelete(i2c_mutex);
        i2c_mutex = NULL;
    }
    
    /* 去初始化硬件I2C */
    // HAL_I2C_DeInit(&hi2c1);
    
    return 0;
}

/* I2C写寄存器（带超时保护） */
int8_t my_i2c_write_reg(uint8_t addr, uint8_t *pdata, uint8_t size)
{
    TickType_t timeout_ticks = pdMS_TO_TICKS(100); /* 100ms超时 */
    
    /* 获取互斥锁 */
    if (xSemaphoreTake(i2c_mutex, timeout_ticks) != pdTRUE) {
        return -1; /* 超时 */
    }
    
    /* 执行I2C写操作 */
    // HAL_I2C_Master_Transmit(&hi2c1, addr << 1, pdata, size, timeout_ticks);
    
    BaseType_t ret = 0; /* 模拟返回值 */
    
    /* 释放互斥锁 */
    xSemaphoreGive(i2c_mutex);
    
    return ret;
}

/* I2C读寄存器（带超时保护） */
int8_t my_i2c_read_reg(uint8_t addr, uint8_t *pdata, uint8_t size)
{
    TickType_t timeout_ticks = pdMS_TO_TICKS(100); /* 100ms超时 */
    
    /* 获取互斥锁 */
    if (xSemaphoreTake(i2c_mutex, timeout_ticks) != pdTRUE) {
        return -1; /* 超时 */
    }
    
    /* 执行I2C读操作 */
    // HAL_I2C_Master_Receive(&hi2c1, addr << 1, pdata, size, timeout_ticks);
    
    BaseType_t ret = 0; /* 模拟返回值 */
    
    /* 释放互斥锁 */
    xSemaphoreGive(i2c_mutex);
    
    return ret;
}

/* 获取I2C接口实例 */
lic_driver_interface_t* get_i2c_interface(void)
{
    return &my_i2c_interface;
}
