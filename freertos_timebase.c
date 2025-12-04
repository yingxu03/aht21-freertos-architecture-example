#include "freertos_timebase.h"
#include "FreeRTOS.h"
#include "task.h"

/* FreeRTOS时基接口 */
static timebase_interface_t freertos_timebase = {
    .pfget_tick_count = freertos_get_tick_count
};

/* 获取系统时间戳（毫秒） */
int32_t freertos_get_tick_count(void)
{
    return (int32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* 获取时基接口实例 */
timebase_interface_t* get_freertos_timebase(void)
{
    return &freertos_timebase;
}