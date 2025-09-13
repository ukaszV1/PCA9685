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


