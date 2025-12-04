/**
 * @file bsp_aht21_handler.h
 * @brief AHT21传感器处理器层头文件
 * @details 
 * 实现BSP层与操作系统、应用层的完全解耦。
 * 采用接口化设计，便于移植和测试。
 * 
 * @version 1.0
 * @author yingxu031
 * @date 2025-11-xx
 */

#ifndef BSP_AHT21_HANDLER_H
#define BSP_AHT21_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 包含头文件 ============================ */
#include "ec_bsp_aht21_driver.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // 修正：原为stding.h

/* ============================ 前向声明 ============================ */
// 接口类型前向声明，避免包含具体头文件
typedef struct iic_driver_interface iic_driver_interface_t;
typedef struct system_timebase_interface system_timebase_interface_t;
typedef struct bsp_aht21 bsp_aht21_t;

// 温湿度事件结构体前向声明（需要在类型定义前声明）
typedef struct temp_humi_event temp_humi_event_t;

/* ============================ 宏定义 ============================ */
#define AHT21_HANDLER_SUCCESS     0   /**< 操作成功 */
#define AHT21_HANDLER_ERROR      -1   /**< 操作失败 */
#define AHT21_HANDLER_BUSY       -2   /**< 设备繁忙 */
#define AHT21_HANDLER_TIMEOUT    -3   /**< 操作超时 */
#define AHT21_HANDLER_INVALID    -4   /**< 无效参数 */

/* ============================ 数据类型定义 ============================ */

/**
 * @brief 温湿度数据类型枚举
 * @note 用于指定需要读取的数据类型
 */
typedef enum {
    TEMP_HUMI_EVENT_TYPE_TEMP = 0,    /**< 仅读取温度 */
    TEMP_HUMI_EVENT_TYPE_HUMI,        /**< 仅读取湿度 */
    TEMP_HUMI_EVENT_TYPE_BOTH,        /**< 同时读取温湿度 */
} temp_humi_type_t;

/**
 * @brief 温湿度请求事件结构体
 * @details 用于异步请求温湿度数据
 */
struct temp_humi_event {
    float *temp;                      /**< 温度数据存储指针 */
    float *humi;                      /**< 湿度数据存储指针 */
    uint32_t *lifetime;               /**< 数据生命周期(可选) */
    uint32_t *timestamp;              /**< 时间戳(可选) */
    temp_humi_type_t type_of_data;    /**< 请求的数据类型 */
    void (*callback)(float *, float *); /**< 数据就绪回调函数 */
};

/**
 * @brief AHT21处理器结构体
 * @details 负责协调OS、BSP和Core层，实现完全解耦
 * 这个结构体包含AHT21传感器实例以及与操作系统交互所需的资源
 */
typedef struct bsp_aht21_handler_t {
    /* ========== Core层接口 ========== */
    iic_driver_interface_t *iic_driver_interface_table;     /**< I2C驱动接口 */
    system_timebase_interface_t *timebase;  /**< 系统时基接口 */
    
    /* ========== RTOS层接口 ========== */
    temp_humi_event_t *temp_humi_event_instance;     /**< 温湿度请求事件（修正拼写）*/
    void *rtos_yield;                       /**< 操作系统任务切换函数指针 */
    
    /* ========== BSP层接口 ========== */
    bsp_aht21_t *aht21_instance;            /**< AHT21传感器实例 */
    
    /* ========== 处理器自有函数接口 ========== */
    /** 构造/解构函数 */
    int8_t (*pf_inst)(
        struct bsp_aht21_handler_t *handler,
        bsp_aht21_t *bsp_aht21_instance,
        iic_driver_interface_t *iic_instance,
        system_timebase_interface_t *timebase,
        void *rtos_yield
    );
    
    int8_t (*pf_deinst)(struct bsp_aht21_handler_t *handler); /**< 解构函数 */
    
    /** 资源管理函数 */
    int8_t (*pf_init)(struct bsp_aht21_handler_t *handler);     /**< 初始化处理器资源 */
    int8_t (*pf_deinit)(struct bsp_aht21_handler_t *handler);   /**< 逆初始化处理器资源 */
    
    /** 数据操作函数 */
    int8_t (*pf_get_temp_humi_data)(struct bsp_aht21_handler_t *handler, 
                                   temp_humi_event_t *event); /**< 获取温湿度数据 */
    
    /** BSP驱动实例化函数 */
    int8_t (*pf_inst_bsp_driver)(struct bsp_aht21_handler_t *handler); /**< 实例化BSP驱动 */
    
    /* ========== 内部状态 ========== */
    bool is_initialized;         /**< 处理器初始化状态 */
    bool measurement_in_progress; /**< 测量进行中标志 */
    
} bsp_aht21_handler_t;

/* ============================ 外部API函数声明 ============================ */

/**
 * @brief 构造AHT21处理器实例
 * @details 创建并配置处理器实例，建立与各层的连接
 * 
 * @param handler[out]           处理器实例指针
 * @param bsp_aht21_instance[in] BSP层AHT21实例
 * @param iic_instance[in]       I2C驱动接口实例
 * @param timebase[in]           系统时基接口实例
 * @param rtos_yield[in]         操作系统任务切换函数
 * @return int8_t 0-成功，其他-错误码
 */
int8_t aht21_handler_inst(
    bsp_aht21_handler_t *handler,
    bsp_aht21_t *bsp_aht21_instance,
    iic_driver_interface_t *iic_instance,
    system_timebase_interface_t *timebase,
    void *rtos_yield
);

/**
 * @brief 解构AHT21处理器实例
 * @details 释放处理器实例占用的资源
 * 
 * @param handler[in] 处理器实例
 * @return int8_t 0-成功，其他-错误码
 */
int8_t aht21_handler_deinst(bsp_aht21_handler_t *handler);

/**
 * @brief 初始化AHT21处理器资源
 * @details 初始化硬件接口、RTOS资源等
 * 
 * @param handler[in] 处理器实例
 * @return int8_t 0-成功，其他-错误码
 */
int8_t aht21_handler_init(bsp_aht21_handler_t *handler);

/**
 * @brief 逆初始化AHT21处理器资源
 * @details 释放所有分配的资源
 * 
 * @param handler[in] 处理器实例
 * @return int8_t 0-成功，其他-错误码
 */
int8_t aht21_handler_deinit(bsp_aht21_handler_t *handler);

/**
 * @brief 获取温湿度数据（阻塞/非阻塞）
 * @details 根据配置执行阻塞或非阻塞读取
 * 
 * @param handler[in] 处理器实例
 * @param event[in]   请求事件（包含回调函数则为非阻塞）
 * @return int8_t 0-成功，其他-错误码
 */
int8_t aht21_handler_get_temp_humi_data(
    bsp_aht21_handler_t *handler,
    temp_humi_event_t *event
);

/**
 * @brief 温湿度处理器线程函数
 * @details 由操作系统调用，处理异步请求
 * 
 * @param arg[in] 线程参数（通常为处理器实例）
 */
void temp_humi_handler_thread(void *arg);

/**
 * @brief 发送温湿度事件
 * @details 向处理器发送数据请求事件
 * 
 * @param event[in] 温湿度事件
 * @return int8_t 0-成功，其他-错误码
 */
int8_t temp_humi_event_handler_send(temp_humi_event_t *event);

/**
 * @brief 实例化BSP驱动
 * @details 内部函数，用于构造BSP层驱动实例
 * 
 * @param handler[in] 处理器实例
 * @return int8_t 0-成功，其他-错误码
 */
int8_t inst_bsp_driver_instance(bsp_aht21_handler_t *handler);

/* ============================ 内联辅助函数 ============================ */

/**
 * @brief 检查处理器是否已初始化
 * @param handler[in] 处理器实例
 * @return bool true-已初始化，false-未初始化
 */
static inline bool aht21_handler_is_initialized(bsp_aht21_handler_t *handler) {
    return (handler != NULL) && handler->is_initialized;
}

/**
 * @brief 检查测量是否在进行中
 * @param handler[in] 处理器实例
 * @return bool true-进行中，false-空闲
 */
static inline bool aht21_handler_is_busy(bsp_aht21_handler_t *handler) {
    return (handler != NULL) && handler->measurement_in_progress;
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_AHT21_HANDLER_H */