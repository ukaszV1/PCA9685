#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>

#define PCA9685_I2C_DEFAULT_FREQUENCY   400000 
#define PCA9685_MAX_PWM_VALUE           4095
#define PCA9685_TIMEOUT_MS              100
#define PCA9685_CHANNEL_ALL             16

typedef enum {
    PCA9685_CHANNEL_0 = 0,
    PCA9685_CHANNEL_1,
    PCA9685_CHANNEL_2,
    PCA9685_CHANNEL_3,
    PCA9685_CHANNEL_4,
    PCA9685_CHANNEL_5,
    PCA9685_CHANNEL_6,
    PCA9685_CHANNEL_7,
    PCA9685_CHANNEL_8,
    PCA9685_CHANNEL_9,
    PCA9685_CHANNEL_10,
    PCA9685_CHANNEL_11,
    PCA9685_CHANNEL_12,
    PCA9685_CHANNEL_13,
    PCA9685_CHANNEL_14,
    PCA9685_CHANNEL_15
} pca9685_channel_t;

typedef struct {
    i2c_device_config_t      i2c_config;
    i2c_master_bus_handle_t  i2c_bus_handle;
    i2c_master_dev_handle_t  i2c_dev_handle;
    
    uint16_t freq;
    uint8_t prescale;
    uint16_t channel_pwm_value[PCA9685_CHANNEL_ALL];
    
    uint8_t sub_value[3];
    uint8_t allcalladr;

    union {
        uint8_t reg_val;
        struct {
            uint8_t ALLCALL : 1;   // bit 0
            uint8_t SUB3    : 1;   
            uint8_t SUB2    : 1;   
            uint8_t SUB1    : 1;   
            uint8_t SLEEP   : 1;   
            uint8_t AI      : 1;   
            uint8_t EXTCLK  : 1;   
            uint8_t RESTART : 1;   // bit 7
        };
    } mode1;

    union {
        uint8_t reg_val;
        struct {
            uint8_t OUTNE   : 2;   // bits 0-1
            uint8_t OUTDRV  : 1;   
            uint8_t OCH     : 1;   
            uint8_t INVRT   : 1;   
            uint8_t RESERVED: 3;   // bits 5-7
        };
    } mode2;

} pca9685_handle_t;



/**
* @brief Init pca9685 device
* @param handle handle to be initialized
* @param i2c_bus bus handle to attatch device to
* @param address default 0x20
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_init(pca9685_handle_t *handle, i2c_master_bus_handle_t *i2c_bus, uint8_t address);

/**
* @brief Set subaddress value and state
* @param dev_handle handle of device
* @param num address num (0-2)
* @param address_val to set
* @param en shall be enabled to work(1)
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_set_subaddress(pca9685_handle_t *dev_handle, uint8_t num, uint8_t address_val, bool en);

/**
* @brief soft restart
* @param dev_handle handle of device
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_restart(pca9685_handle_t *dev_handle);


/**
* @brief set sleep
* @param dev_handle handle of device
* @param sleep true if want sleep
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_sleep(pca9685_handle_t *dev_handle, bool sleep);


/**
* @brief set outputs logic inverted
* @param dev_handle handle of device
* @param inverted false for normal behaviour
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_set_output_inverted(pca9685_handle_t *dev_handle, bool inverted);

/**
* @brief set outputs behaviour 
* @param dev_handle handle of device
* @param od true for open drain false for totempole
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_set_output_open_drain(pca9685_handle_t *dev_handle, bool od);

/**
* @brief set global PWM frequency 
* @param dev_handle handle of device
* @param freq frequency (24-1526)Hz
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_set_pwm_frequency(pca9685_handle_t *dev_handle, uint16_t freq);


/**
* @brief fetch prescaler and calculate frequency
* @param dev_handle handle of device
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_get_prescaler_and_freq(pca9685_handle_t *dev_handle);

/**
* @brief fetch prescaler and calculate frequency
* @param dev_handle handle of device
* @param channel  pca9685_channel_t (0-15)
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_set_pwm_value(pca9685_handle_t *dev_handle,
                                uint8_t channel,
                                uint16_t value);

/**
* @brief Loads pwm values selected by bitmask from current handle table values
* @param dev_handle handle of device
* @param bitmask (0 or 0xFFFF) for all else update selected by bimask only
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_update_pwm_values(pca9685_handle_t *dev_handle, uint16_t bitmask);


/**
* @brief Fetch register values, store in hadle struct
* @param dev_handle handle of device
* @return esp_err_t ESP_OK on success
*/
esp_err_t pca9685_fetch_modes_reg(pca9685_handle_t *dev_handle);
