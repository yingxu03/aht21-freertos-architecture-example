/*******************************************************************************
 * @file    bsp_aht21_driver.h
 * @author  yingxu031
 * @version V1.0.0
 * @date    2025-11-xx
 * @brief   AHT21温湿度传感器驱动头文件
 * 
 * @note    此驱动采用面向接口编程设计，支持裸机与RTOS环境
 *          通过依赖注入方式解耦硬件依赖，提高代码可移植性
 *          
 * 使用说明：
 * 1. 需要在包含此文件前定义 OS_SUPPORTING 宏以启用RTOS支持
 * 2. 使用 aht21_inst() 构造函数初始化驱动实例
 * 3. 调用 aht21_init() 初始化传感器
 * 4. 调用 read_temperature/read_humidity 读取数据
 * 
 ******************************************************************************/

#ifndef BSP_AHT21_DRIVER_H
#define BSP_AHT21_DRIVER_H

/****************************** 包含文件 **************************************/
#include <stdio.h>
#include <stdint.h>

#include "bsp_aht21_reg.h"

/****************************** 宏定义 ****************************************/
/* 操作系统支持标志 */
#ifdef OS_SUPPORTING
    #define OS_SUPPORT_ENABLED 1
#else
    #define OS_SUPPORT_ENABLED 0
#endif

/****************************** 类型定义 **************************************/

#ifdef OS_SUPPORTING
/* RTOS 让出CPU接口 */
typedef struct
{
    void (*rtos_yield) (void);            /* 当前线程让出CPU */
} yield_interface_t;
#endif // OS_SUPPORTING

/****************************** 接口结构体定义 ********************************/

/*----------------------------------------------------------------------------
 * I2C 驱动接口结构体
 *---------------------------------------------------------------------------*/
typedef struct
{
    int8_t (*pfinit) (void);                         /* I2C初始化 */
    int8_t (*pfdeinit) (void);                       /* I2C去初始化 */
    int8_t (*pfwrite_reg) (uint8_t addr, 
                           uint8_t *pdata, 
                           uint8_t size);            /* 写寄存器 */
    int8_t (*pfread_reg) (uint8_t addr, 
                          uint8_t *pdata, 
                          uint8_t size);             /* 读寄存器 */
} lic_driver_interface_t;

/*----------------------------------------------------------------------------
 * 时基接口结构体
 *---------------------------------------------------------------------------*/
typedef struct
{
    int32_t (*pfget_tick_count) (void);              /* 获取系统时间戳 */
} timebase_interface_t;

/****************************** AHT21驱动主结构体 *****************************/

/*----------------------------------------------------------------------------
 * AHT21 驱动结构体
 * 说明：此结构体包含所有外部依赖接口和功能函数指针
 *---------------------------------------------------------------------------*/
typedef struct bsp_aht21_driver_t
{
    /*------------------------- 外部依赖接口 --------------------------------*/
    lic_driver_interface_t *pricdriver_interface;    /* I2C驱动接口指针 */
    timebase_interface_t *ptimebase_interface;       /* 时基接口指针 */
    
#if OS_SUPPORT_ENABLED
    yield_interface_t *rtos_yield_interface;         /* RTOS让出CPU接口指针 */
#endif

    /*------------------------- 构造函数 ------------------------------------*/
    int8_t (*pfinst)(
        void *aht21_instance,                        /* AHT21实例指针 */
        lic_driver_interface_t *lic_instance,        /* I2C驱动实例 */
        timebase_interface_t *timebase,              /* 时基实例 */
#if OS_SUPPORT_ENABLED
        yield_interface_t *rtos_yield                /* RTOS让出CPU函数 */
#else
        void *rtos_yield                             /* 裸机模式下为NULL */
#endif
    );

    /*------------------------- HAL功能接口 ---------------------------------*/
    int8_t (*pfinit)(void *aht21_instance);          /* #1 传感器初始化 */
    int8_t (*pfdeinit)(void *aht21_instance);        /* #2 传感器去初始化 */
    int8_t (*pfread_id)(void *aht21_instance);       /* #3 读取传感器ID */
    int8_t (*pfread_temperature)(void *aht21_instance, 
                                 float *temp);       /* #4 读取温度值 */
    int8_t (*pfread_humidity)(void *aht21_instance, 
                              float *humidity);      /* #5 读取湿度值 */
    int8_t (*pfhibernating)(void *aht21_instance);   /* #6 传感器休眠 */
    int8_t (*pfwakeup)(void *aht21_instance);        /* #7 传感器唤醒 */
    
} bsp_aht21_driver_t;

/****************************** 函数声明 **************************************/

/*============================================================================
 * 构造函数组
 *===========================================================================*/

/**
 * @brief  AHT21驱动实例构造函数
 * @param  aht21_instance  AHT21驱动实例指针
 * @param  lic_instance    I2C驱动接口实例
 * @param  timebase        时基接口实例
 * @param  rtos_yield      RTOS让出CPU函数（裸机为NULL）
 * @return 执行状态：0-成功，<0-失败
 * 
 * @note   此函数将外部接口绑定到AHT21驱动实例，实现依赖注入
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
);

/*============================================================================
 * HAL功能函数组
 *===========================================================================*/

/**
 * @brief  初始化AHT21传感器
 * @param  aht21_instance  AHT21驱动实例指针
 * @return 执行状态：0-成功，<0-失败
 */
int8_t aht21_init(bsp_aht21_driver_t *aht21_instance);

/**
 * @brief  去初始化AHT21传感器
 * @param  aht21_instance  AHT21驱动实例指针
 * @return 执行状态：0-成功，<0-失败
 */
int8_t aht21_deinit(bsp_aht21_driver_t *aht21_instance);

/**
 * @brief  读取AHT21传感器ID
 * @param  aht21_instance  AHT21驱动实例指针
 * @return 传感器ID（成功时），<0-失败
 */
int8_t aht21_read_id(bsp_aht21_driver_t *aht21_instance);

/**
 * @brief  读取温度值
 * @param  aht21_instance  AHT21驱动实例指针
 * @param  temp            [输出]温度值（摄氏度）
 * @return 执行状态：0-成功，<0-失败
 */
int8_t aht21_read_temperature(bsp_aht21_driver_t *aht21_instance, 
                              float *temp);

/**
 * @brief  读取湿度值
 * @param  aht21_instance  AHT21驱动实例指针
 * @param  humidity        [输出]湿度值（百分比）
 * @return 执行状态：0-成功，<0-失败
 */
int8_t aht21_read_humidity(bsp_aht21_driver_t *aht21_instance, 
                           float *humidity);

/**
 * @brief  使AHT21传感器进入休眠模式
 * @param  aht21_instance  AHT21驱动实例指针
 * @return 执行状态：0-成功，<0-失败
 */
int8_t aht21_hibernating(bsp_aht21_driver_t *aht21_instance);

/**
 * @brief  唤醒AHT21传感器
 * @param  aht21_instance  AHT21驱动实例指针
 * @return 执行状态：0-成功，<0-失败
 */
int8_t aht21_wakeup(bsp_aht21_driver_t *aht21_instance);

/******************************************************************************
 * 错误码定义
 ******************************************************************************/
#define AHT21_OK                  0     /* 操作成功 */
#define AHT21_ERR_INIT           -1     /* 初始化失败 */
#define AHT21_ERR_PARAM          -2     /* 参数错误 */
#define AHT21_ERR_I2C            -3     /* I2C通信错误 */
#define AHT21_ERR_TIMEOUT        -4     /* 操作超时 */
#define AHT21_ERR_CHECKSUM       -5     /* 校验和错误 */
#define AHT21_ERR_BUSY           -6     /* 传感器忙 */
#define AHT21_ERR_NOT_INIT       -7     /* 传感器未初始化 */

#endif /* BSP_AHT21_DRIVER_H */

/****************************** 文件结束 **************************************/