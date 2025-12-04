#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* FreeRTOS任务配置 */
#define TASK_PRIORITY_HIGH        3
#define TASK_PRIORITY_NORMAL      2
#define TASK_PRIORITY_LOW         1

/* 堆栈大小配置 */
#define AHT21_TASK_STACK_SIZE     512
#define TEMP_HUMI_TASK_STACK_SIZE 512

/* 队列配置 */
#define TEMP_HUMI_QUEUE_SIZE      5

#endif /* FREERTOS_CONFIG_H */