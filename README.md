# PCA9685 ESP-IDF Component

A simple ESP-IDF driver for the **PCA9685 16-channel 12-bit PWM controller**.  
Supports freqency and duty cycle, no support for channel offset

---

## Features
- Configure PWM frequency (24–1526 Hz)  
- Set duty cycle on any channel (0–15, 12-bit resolution)  
- Bulk update of all channels using bitmask  
- Soft restart and sleep mode
- Inverted / normal output modes  
- Output mode selection (open-drain or totem-pole)  
- Subaddress and all-call addressing  
- Read/write **MODE1** and **MODE2** registers into handle struct  

---
## API
### GENERAL
```c
esp_err_t pca9685_init(pca9685_handle_t *handle, i2c_master_bus_handle_t *i2c_bus, uint8_t address);
esp_err_t pca9685_restart(pca9685_handle_t *dev_handle);
esp_err_t pca9685_sleep(pca9685_handle_t *dev_handle, bool sleep);
esp_err_t pca9685_set_pwm_frequency(pca9685_handle_t *dev_handle, uint16_t freq);
esp_err_t pca9685_set_pwm_value(pca9685_handle_t *dev_handle, uint8_t channel, uint16_t value);
esp_err_t pca9685_update_pwm_values(pca9685_handle_t *dev_handle, uint16_t bitmask);
```
---
### EXTRA
```c
esp_err_t pca9685_set_subaddress(pca9685_handle_t *dev_handle, uint8_t num, uint8_t address_val, bool en);
esp_err_t pca9685_get_prescaler_and_freq(pca9685_handle_t *dev_handle);
esp_err_t pca9685_set_output_inverted(pca9685_handle_t *dev_handle, bool inverted);
esp_err_t pca9685_set_output_open_drain(pca9685_handle_t *dev_handle, bool od);
esp_err_t pca9685_get_prescaler_and_freq(pca9685_handle_t *dev_handle);
esp_err_t pca9685_read_modes_reg(pca9685_handle_t *dev_handle);
esp_err_t pca9685_write_modes_reg(pca9685_handle_t *dev_handle, uint8_t reg);
```
## EXAMPLE 
conncect 5V servo to *channel 0*
```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "pca9685.h"

static const char *TAG = "PCA9685_TEST";

#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define PCA9685_ADDR 0x40

void app_main(void)
{
    pca9685_handle_t pca;

    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
    ESP_ERROR_CHECK(pca9685_init(&pca, &i2c_bus, PCA9685_ADDR));
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&pca, 50));

    while (1) {
        pca9685_sleep(&pca, false);
        ESP_ERROR_CHECK(pca9685_set_pwm_value(&pca, 0, 205)); //servo 0deg
        ESP_LOGI(TAG, "channel 0, 0 deg");
        vTaskDelay(pdMS_TO_TICKS(1000));  
        ESP_LOGI(TAG, "channel 0, full deg");
        ESP_ERROR_CHECK(pca9685_set_pwm_value(&pca, 0, 410)); //servo full deg
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "all channels fade");
        for (uint8_t i = 0; i < PCA9685_CHANNEL_ALL; i++){
            pca.channel_pwm_value[i] = (i* 255);
        }
        pca9685_update_pwm_values(&pca, 0);    //set different values on each channel
        vTaskDelay(pdMS_TO_TICKS(3000));   

        pca.channel_pwm_value[0] = 305;
        pca9685_update_pwm_values(&pca, (int16_t)(0x1)); //set mid pos (servo) on channel 0 
          ESP_LOGI(TAG, "channel 0 mid pos");  
        vTaskDelay(pdMS_TO_TICKS(2000));
        pca9685_sleep(&pca, true);    //turn off out
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

```

