#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "esp_err.h"
#include "driver/i2c_master.h"
// Inicia a task de sensores
esp_err_t sensor_task_start(i2c_master_bus_handle_t bus_handle);

#endif // SENSOR_TASK_H