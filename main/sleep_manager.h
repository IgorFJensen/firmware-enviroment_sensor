#ifndef SLEEP_MANAGER_H_
#define SLEEP_MANAGER_H_

#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t sleep_manager_start(i2c_master_bus_handle_t bus_handle);

#endif // SLEEP_MANAGER_H_
