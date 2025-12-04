#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/* 包含驱动头文件 */
#include "bsp_aht21_driver.h"
#include "bsp_aht21_handler.h"

/* 包含接口头文件 */
#include "my_i2c_driver.h"
#include "freertos_timebase.h"
#include "freertos_yield.h"

/* 全局变量 */
static bsp_aht21_driver_t aht21_driver;
static bsp_aht21_handler_t aht21_handler;
static bsp_aht21_t aht21_instance; /* 假设的BSP实例 */

/* 事件组 */
static EventGroupHandle_t aht21_events = NULL;
#define AHT21_MEASURE_COMPLETE_BIT   (1 << 0)
#define AHT21_DATA_READY_BIT         (1 << 1)

/* 消息队列 */
static QueueHandle_t temp_humi_queue = NULL;

/* 互斥锁 */
static SemaphoreHandle_t aht21_mutex = NULL;

/* 温湿度数据结构 */
typedef struct {
    float temperature;
    float humidity;
    uint32_t timestamp;
} temp_humi_data_t;

/* 温湿度事件结构 */
static temp_humi_event_t temp_humi_event = {
    .temp = NULL,
    .humi = NULL,
    .lifetime = NULL,
    .timestamp = NULL,
    .type_of_data = TEMP_HUMI_EVENT_TYPE_BOTH,
    .callback = NULL
};

/******************************************************************************
 * 回调函数实现
 ******************************************************************************/

/* 温湿度数据就绪回调 */
static void temp_humi_data_ready_callback(float *temp, float *humi)
{
    static temp_humi_data_t data;
    
    if (temp != NULL && humi != NULL) {
        /* 填充数据 */
        data.temperature = *temp;
        data.humidity = *humi;
        data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        /* 发送到队列 */
        if (temp_humi_queue != NULL) {
            xQueueSend(temp_humi_queue, &data, portMAX_DELAY);
        }
        
        /* 设置事件标志 */
        if (aht21_events != NULL) {
            xEventGroupSetBits(aht21_events, AHT21_DATA_READY_BIT);
        }
    }
}

/******************************************************************************
 * FreeRTOS任务函数
 ******************************************************************************/

/* AHT21传感器任务 */
static void aht21_sensor_task(void *pvParameters)
{
    int8_t ret;
    
    printf("[AHT21] Sensor task started.\n");
    
    /* 等待系统就绪 */
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    /* 初始化AHT21驱动 */
    printf("[AHT21] Initializing driver...\n");
    
    /* 构造驱动实例 */
    ret = aht21_inst(
        &aht21_driver,
        get_i2c_interface(),           /* I2C驱动接口 */
        get_freertos_timebase(),       /* 时基接口 */
        get_freertos_yield_interface() /* RTOS让出CPU接口 */
    );
    
    if (ret != AHT21_OK) {
        printf("[AHT21] Driver instance creation failed: %d\n", ret);
        vTaskDelete(NULL);
        return;
    }
    
    /* 初始化传感器 */
    ret = aht21_init(&aht21_driver);
    if (ret != AHT21_OK) {
        printf("[AHT21] Sensor initialization failed: %d\n", ret);
        vTaskDelete(NULL);
        return;
    }
    
    printf("[AHT21] Sensor initialized successfully.\n");
    
    /* 读取传感器ID */
    int8_t sensor_id = aht21_read_id(&aht21_driver);
    if (sensor_id >= 0) {
        printf("[AHT21] Sensor ID: 0x%02X\n", sensor_id);
    } else {
        printf("[AHT21] Failed to read sensor ID: %d\n", sensor_id);
    }
    
    /* 主循环 */
    while (1) {
        float temperature = 0.0f;
        float humidity = 0.0f;
        
        /* 获取互斥锁 */
        if (xSemaphoreTake(aht21_mutex, portMAX_DELAY) == pdTRUE) {
            
            /* 读取温度 */
            ret = aht21_read_temperature(&aht21_driver, &temperature);
            if (ret == AHT21_OK) {
                printf("[AHT21] Temperature: %.2f°C\n", temperature);
            } else {
                printf("[AHT21] Failed to read temperature: %d\n", ret);
            }
            
            /* 读取湿度 */
            ret = aht21_read_humidity(&aht21_driver, &humidity);
            if (ret == AHT21_OK) {
                printf("[AHT21] Humidity: %.2f%%\n", humidity);
            } else {
                printf("[AHT21] Failed to read humidity: %d\n", ret);
            }
            
            /* 释放互斥锁 */
            xSemaphoreGive(aht21_mutex);
        }
        
        /* 等待10秒后再次测量 */
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/* 温湿度处理器任务 */
static void temp_humi_handler_task(void *pvParameters)
{
    printf("[Handler] Temperature/Humidity handler task started.\n");
    
    /* 构造处理器实例 */
    int8_t ret = aht21_handler_inst(
        &aht21_handler,
        &aht21_instance,               /* BSP实例 */
        get_i2c_interface(),           /* I2C接口 */
        get_freertos_timebase(),       /* 时基接口 */
        (void*)taskYIELD               /* RTOS让出CPU函数 */
    );
    
    if (ret != AHT21_HANDLER_SUCCESS) {
        printf("[Handler] Handler instance creation failed: %d\n", ret);
        vTaskDelete(NULL);
        return;
    }
    
    /* 初始化处理器 */
    ret = aht21_handler_init(&aht21_handler);
    if (ret != AHT21_HANDLER_SUCCESS) {
        printf("[Handler] Handler initialization failed: %d\n", ret);
        vTaskDelete(NULL);
        return;
    }
    
    printf("[Handler] Handler initialized successfully.\n");
    
    /* 设置回调函数 */
    temp_humi_event.callback = temp_humi_data_ready_callback;
    
    /* 主循环 */
    while (1) {
        /* 创建局部变量存储数据 */
        float temp_data = 0.0f;
        float humi_data = 0.0f;
        
        /* 配置事件 */
        temp_humi_event.temp = &temp_data;
        temp_humi_event.humi = &humi_data;
        temp_humi_event.type_of_data = TEMP_HUMI_EVENT_TYPE_BOTH;
        
        /* 获取温湿度数据（非阻塞，使用回调） */
        ret = aht21_handler_get_temp_humi_data(&aht21_handler, &temp_humi_event);
        
        if (ret == AHT21_HANDLER_SUCCESS) {
            printf("[Handler] Measurement started, waiting for callback...\n");
            
            /* 等待数据就绪事件 */
            EventBits_t bits = xEventGroupWaitBits(
                aht21_events,
                AHT21_DATA_READY_BIT,
                pdTRUE,            /* 清除事件位 */
                pdFALSE,           /* 等待所有位 */
                pdMS_TO_TICKS(5000) /* 5秒超时 */
            );
            
            if (bits & AHT21_DATA_READY_BIT) {
                printf("[Handler] Data received via callback\n");
            } else {
                printf("[Handler] Timeout waiting for data\n");
            }
        } else {
            printf("[Handler] Failed to start measurement: %d\n", ret);
        }
        
        /* 等待5秒后再次测量 */
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* 温湿度数据处理任务 */
static void temp_humi_data_task(void *pvParameters)
{
    temp_humi_data_t data;
    
    printf("[Data] Data processing task started.\n");
    
    while (1) {
        /* 从队列接收数据 */
        if (xQueueReceive(temp_humi_queue, &data, portMAX_DELAY) == pdTRUE) {
            /* 处理数据 */
            printf("[Data] Processing data:\n");
            printf("  Temperature: %.2f°C\n", data.temperature);
            printf("  Humidity: %.2f%%\n", data.humidity);
            printf("  Timestamp: %lu ms\n", data.timestamp);
            
            /* 这里可以添加数据存储、告警判断等逻辑 */
            
            /* 温度告警检查 */
            if (data.temperature > 30.0f) {
                printf("[Data] WARNING: High temperature detected!\n");
            }
            
            if (data.humidity > 80.0f) {
                printf("[Data] WARNING: High humidity detected!\n");
            }
            
            /* 数据有效性检查 */
            if (data.temperature < -40.0f || data.temperature > 85.0f ||
                data.humidity < 0.0f || data.humidity > 100.0f) {
                printf("[Data] ERROR: Invalid sensor data!\n");
            }
        }
    }
}

/******************************************************************************
 * 初始化函数
 ******************************************************************************/

void aht21_example_init(void)
{
    BaseType_t ret;
    
    printf("AHT21 FreeRTOS Example Initializing...\n");
    
    /* 创建事件组 */
    aht21_events = xEventGroupCreate();
    if (aht21_events == NULL) {
        printf("ERROR: Failed to create event group!\n");
        return;
    }
    
    /* 创建消息队列 */
    temp_humi_queue = xQueueCreate(TEMP_HUMI_QUEUE_SIZE, sizeof(temp_humi_data_t));
    if (temp_humi_queue == NULL) {
        printf("ERROR: Failed to create message queue!\n");
        return;
    }
    
    /* 创建互斥锁 */
    aht21_mutex = xSemaphoreCreateMutex();
    if (aht21_mutex == NULL) {
        printf("ERROR: Failed to create mutex!\n");
        return;
    }
    
    /* 创建AHT21传感器任务 */
    ret = xTaskCreate(
        aht21_sensor_task,
        "AHT21_Sensor",
        AHT21_TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY_NORMAL,
        NULL
    );
    
    if (ret != pdPASS) {
        printf("ERROR: Failed to create AHT21 sensor task!\n");
    }
    
    /* 创建温湿度处理器任务 */
    ret = xTaskCreate(
        temp_humi_handler_task,
        "TempHumi_Handler",
        TEMP_HUMI_TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY_NORMAL,
        NULL
    );
    
    if (ret != pdPASS) {
        printf("ERROR: Failed to create temperature/humidity handler task!\n");
    }
    
    /* 创建数据处理任务 */
    ret = xTaskCreate(
        temp_humi_data_task,
        "TempHumi_Data",
        TEMP_HUMI_TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY_LOW,
        NULL
    );
    
    if (ret != pdPASS) {
        printf("ERROR: Failed to create data processing task!\n");
    }
    
    printf("AHT21 FreeRTOS Example Initialization Complete.\n");
    printf("Tasks are now running...\n");
    printf("==============================================\n");
}

/******************************************************************************
 * 清理函数
 ******************************************************************************/

void aht21_example_cleanup(void)
{
    printf("Cleaning up AHT21 example...\n");
    
    /* 删除所有任务 */
    // 在FreeRTOS中，通常通过删除任务句柄来删除任务
    // 这里假设任务句柄已保存
    
    /* 删除事件组 */
    if (aht21_events != NULL) {
        vEventGroupDelete(aht21_events);
        aht21_events = NULL;
    }
    
    /* 删除队列 */
    if (temp_humi_queue != NULL) {
        vQueueDelete(temp_humi_queue);
        temp_humi_queue = NULL;
    }
    
    /* 删除互斥锁 */
    if (aht21_mutex != NULL) {
        vSemaphoreDelete(aht21_mutex);
        aht21_mutex = NULL;
    }
    
    /* 去初始化AHT21驱动 */
    aht21_deinit(&aht21_driver);
    
    /* 解构处理器 */
    aht21_handler_deinst(&aht21_handler);
    
    printf("AHT21 example cleanup complete.\n");
}