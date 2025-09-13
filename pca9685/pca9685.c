#include "pca9685.h"
#include <string.h>
#include <math.h>
#include <driver/i2c_master.h>
#include "freertos/task.h"

static const char *TAG = "pca9685";

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define RETURN_ON_ERROR(x) do {        \
    esp_err_t __err_rc = (x);          \
    if (__err_rc != ESP_OK) return __err_rc; \
} while (0)
#define REG_LED_N(x)  (REG_LEDX + (x) * 4)

#define PCA9685_INTERNAL_FREQ 25000000UL
#define WAKEUP_DELAY_US 500

#define PCA9685_ALLCALLADR_DEFAULT 0x70

#define PCA9685_SB1_DEFAULT  0xE2
#define PCA9685_SB2_DEFAULT  0xE4
#define PCA9685_SB3_DEFAULT  0xE8

#define REG_MODE1      0x00
#define REG_MODE2      0x01
#define REG_SUBADR1    0x02
#define REG_ALLCALLADR 0x05
#define REG_LEDX       0x06
#define REG_ALL_LED    0xFA
#define REG_PRE_SCALE  0xFE

#define MODE1_RESTART_BIT   (1 << 7)
#define MODE1_SLEEP_BIT (1 << 4)
#define MODE1_SUB_CNT 3

#define MODE2_INVRT_BIT   (1 << 4)
#define MODE2_OUTDRV_BIT  (1 << 2)

#define LED_FULL_ON_OFF (1 << 4)//makes out always on 

#define MIN_PRESCALER 0x03
#define MAX_PRESCALER 0xFF
#define MAX_SUBADDR   2


static inline esp_err_t update_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t r;
    //read current register values
    //only one register at time
    RETURN_ON_ERROR(i2c_master_transmit_receive(dev_handle, &reg, 1, &r, 1, PCA9685_TIMEOUT_MS));
    //cler and set
    r = (r & ~mask) | val;
    //pack and transmit
    RETURN_ON_ERROR(i2c_master_transmit(dev_handle, (uint8_t[]){reg, r}, 2, PCA9685_TIMEOUT_MS));
    return ESP_OK;
}

esp_err_t pca9685_init(pca9685_handle_t *handle, i2c_master_bus_handle_t *i2c_bus, uint8_t address)
{
    CHECK_ARG(handle && i2c_bus && address);
    //set fields of handle to 0
    memset(handle, 0, sizeof(pca9685_handle_t));

    //create config of i2c
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = PCA9685_I2C_DEFAULT_FREQUENCY
    };
    //add device to bus
    RETURN_ON_ERROR(i2c_master_bus_add_device(*i2c_bus, &dev_cfg, &handle->i2c_dev_handle));

    //setup handle 
    handle->sub_value[0] = PCA9685_SB1_DEFAULT;
    handle->sub_value[1] = PCA9685_SB2_DEFAULT;
    handle->sub_value[2] = PCA9685_SB3_DEFAULT;
    handle->allcalladr = PCA9685_ALLCALLADR_DEFAULT;
    handle->i2c_bus_handle = *i2c_bus;
    return ESP_OK;
}


esp_err_t pca9685_set_subaddress(pca9685_handle_t *dev_handle, uint8_t num, uint8_t address_val, bool en)
{
    CHECK_ARG(dev_handle);
    //check num of address
    if (num > MAX_SUBADDR) {return ESP_ERR_INVALID_ARG;}

    //select correct address 
    uint8_t reg = REG_SUBADR1 + num;
    //prepare value
    uint8_t data = address_val << 1;

    RETURN_ON_ERROR(i2c_master_transmit(dev_handle->i2c_dev_handle, (uint8_t[]){reg, data}, 2, PCA9685_TIMEOUT_MS));
    uint8_t mask = 1 << (MODE1_SUB_CNT - num);
    //update register to set enable bit
    RETURN_ON_ERROR(update_reg(dev_handle->i2c_dev_handle, REG_MODE1, mask, en ? mask : 0));
    //update handle
    dev_handle->mode1.reg_val = (dev_handle->mode1.reg_val & ~(1 << (MODE1_SUB_CNT - num))) | (en << (MODE1_SUB_CNT - num));
    dev_handle->sub_value[num] = address_val;

    return ESP_OK;
}

esp_err_t pca9685_restart(pca9685_handle_t *dev_handle)
{
    CHECK_ARG(dev_handle);

    uint8_t mode;
    RETURN_ON_ERROR(i2c_master_transmit_receive(dev_handle->i2c_dev_handle, (uint8_t[]){ REG_MODE1 }, 1, &mode, 1, PCA9685_TIMEOUT_MS));

    if (mode & MODE1_RESTART_BIT)
    {

        uint8_t data[2] = {REG_MODE1 ,mode & ~MODE1_SLEEP_BIT};
        RETURN_ON_ERROR(i2c_master_transmit(dev_handle->i2c_dev_handle, data, 2, PCA9685_TIMEOUT_MS));
        //delay after wakeup
        esp_rom_delay_us(WAKEUP_DELAY_US);
    }

    uint8_t new_mode[2] ={REG_MODE1, (mode & ~MODE1_SLEEP_BIT) | MODE1_RESTART_BIT};
    RETURN_ON_ERROR(i2c_master_transmit(dev_handle->i2c_dev_handle, new_mode, 2 ,PCA9685_TIMEOUT_MS));

    dev_handle->mode1.SLEEP = false;
    return ESP_OK;
}


esp_err_t pca9685_fetch_modes_reg(pca9685_handle_t *dev_handle)
{
    CHECK_ARG(dev_handle);
    //read two modes registers and store data in handle
    RETURN_ON_ERROR(i2c_master_transmit_receive(dev_handle->i2c_dev_handle,
         (uint8_t[]){ REG_MODE1 }, 1, &dev_handle->mode1.reg_val, 1, PCA9685_TIMEOUT_MS));
    RETURN_ON_ERROR(i2c_master_transmit_receive(dev_handle->i2c_dev_handle,
         (uint8_t[]){ REG_MODE2 }, 1, &dev_handle->mode2.reg_val, 1, PCA9685_TIMEOUT_MS));

    return ESP_OK;
}

esp_err_t pca9685_sleep(pca9685_handle_t *dev_handle, bool sleep)
{
    CHECK_ARG(dev_handle);
    uint8_t val = sleep ? MODE1_SLEEP_BIT : 0;
    //set to sleep by updateing registers
    ESP_ERROR_CHECK(update_reg(dev_handle->i2c_dev_handle, REG_MODE1, (uint8_t)MODE1_SLEEP_BIT, val));
    return ESP_OK;
}

esp_err_t pca9685_set_output_inverted(pca9685_handle_t *dev_handle, bool inverted)
{
    CHECK_ARG(dev_handle);
    uint8_t val = inverted ? MODE2_INVRT_BIT : 0;
    //update registers 
    RETURN_ON_ERROR(update_reg(dev_handle->i2c_dev_handle, REG_MODE2, MODE2_INVRT_BIT, val));
    return ESP_OK;
}

esp_err_t pca9685_set_output_open_drain(pca9685_handle_t *dev_handle, bool od)
{
    CHECK_ARG(dev_handle);
    uint8_t val = od ? 0 : MODE2_OUTDRV_BIT; // open-drain = 0, totem-pole = 1
    RETURN_ON_ERROR(update_reg(dev_handle->i2c_dev_handle, REG_MODE2, MODE2_OUTDRV_BIT, val));
    return ESP_OK;
}

esp_err_t pca9685_set_prescaler(pca9685_handle_t *dev_handle, uint8_t prescaler_val)
{
    CHECK_ARG(dev_handle && prescaler_val >= MIN_PRESCALER);

    uint8_t mode;

    RETURN_ON_ERROR(i2c_master_transmit_receive(dev_handle->i2c_dev_handle, (uint8_t[]){ REG_MODE1 }, 1, &mode, 1, PCA9685_TIMEOUT_MS));

    RETURN_ON_ERROR(update_reg(dev_handle->i2c_dev_handle, REG_MODE1, MODE1_SLEEP_BIT, MODE1_SLEEP_BIT));

    RETURN_ON_ERROR(i2c_master_transmit(dev_handle->i2c_dev_handle, (uint8_t[]){ REG_PRE_SCALE, prescaler_val }, 2, PCA9685_TIMEOUT_MS));

    RETURN_ON_ERROR(update_reg(dev_handle->i2c_dev_handle, REG_MODE1, MODE1_SLEEP_BIT, 0));

    dev_handle->prescale = prescaler_val;
    dev_handle->mode1.reg_val = (mode & ~MODE1_SLEEP_BIT);

    return ESP_OK;
}

esp_err_t pca9685_get_prescaler_and_freq(pca9685_handle_t *dev_handle)
{
    CHECK_ARG(dev_handle);
    RETURN_ON_ERROR(i2c_master_transmit_receive(dev_handle->i2c_dev_handle, (uint8_t[]){ REG_PRE_SCALE }, 1, &dev_handle->prescale, 1, PCA9685_TIMEOUT_MS));
    //calculate freq form prescaller
    dev_handle->freq = (uint16_t)(PCA9685_INTERNAL_FREQ / ((uint32_t)PCA9685_MAX_PWM_VALUE * (dev_handle->prescale + 1)));
    return ESP_OK;
}

esp_err_t pca9685_set_pwm_frequency(pca9685_handle_t *dev_handle, uint16_t freq)
{
    CHECK_ARG(dev_handle && freq != 0);
    //calculate prescaler from freq
    uint8_t prescaler = (uint8_t)(round((float)PCA9685_INTERNAL_FREQ / (PCA9685_MAX_PWM_VALUE * freq)) - 1);
    //check if fits in limits
    CHECK_ARG(prescaler >=  MIN_PRESCALER);

    RETURN_ON_ERROR(pca9685_set_prescaler(dev_handle, prescaler));

    dev_handle->freq = freq;
    dev_handle->prescale = prescaler;
    return ESP_OK;
}

esp_err_t pca9685_set_pwm_value(pca9685_handle_t *dev_handle, uint8_t channel, uint16_t val)
{
    CHECK_ARG(dev_handle && channel <= PCA9685_CHANNEL_ALL && val <= PCA9685_MAX_PWM_VALUE);
    //all led register allow to set one global value
    uint8_t reg = (channel == PCA9685_CHANNEL_ALL) ? REG_ALL_LED : REG_LED_N(channel);

    bool full_on  = (val == PCA9685_MAX_PWM_VALUE);
    bool full_off = (val == 0);
    uint16_t raw = full_on ? 0x0FFF : val;
    //goes like: ON - (out hight at time) off - (out low at time)
    uint8_t buf[5];
    buf[0] = reg;                                    // Register address
    buf[1] = 0;                                      // ON_L
    buf[2] = full_on ? LED_FULL_ON_OFF : 0;          // ON_H
    buf[3] = raw & 0xFF;                             // OFF_L
    buf[4] = full_off ? (LED_FULL_ON_OFF | (raw >> 8)) : (raw >> 8); // OFF_H

    dev_handle->channel_pwm_value[channel] = val;
    return i2c_master_transmit(dev_handle->i2c_dev_handle, buf, sizeof(buf), PCA9685_TIMEOUT_MS);
}


esp_err_t pca9685_update_pwm_values(pca9685_handle_t *dev_handle, uint16_t bitmask)
{
    CHECK_ARG(dev_handle);

    if (bitmask == 0 || bitmask == 0xFFFF)
    {
        uint8_t buf[1 + PCA9685_CHANNEL_ALL * 4];
        buf[0] = REG_LED_N(0); // start at LED0

        for (uint8_t ch = 0; ch < PCA9685_CHANNEL_ALL; ch++)
        {
            uint16_t val = dev_handle->channel_pwm_value[ch];
            bool full_on  = (val == PCA9685_MAX_PWM_VALUE);
            bool full_off = (val == 0);
            uint16_t raw  = full_on ? 4095 : val;

            buf[1 + ch * 4 + 0] = 0;                       // ON_L
            buf[1 + ch * 4 + 1] = full_on ? LED_FULL_ON_OFF : 0; // ON_H
            buf[1 + ch * 4 + 2] = raw & 0xFF;              // OFF_L
            buf[1 + ch * 4 + 3] = full_off ? (LED_FULL_ON_OFF | (raw >> 8)) : (raw >> 8); // OFF_H
        }

        return i2c_master_transmit(dev_handle->i2c_dev_handle, buf, sizeof(buf), PCA9685_TIMEOUT_MS);
    }

    for (uint8_t ch = 0; ch < PCA9685_CHANNEL_ALL; ch++)
    {
        if (!(bitmask & (1 << ch))) continue;

        uint16_t val = dev_handle->channel_pwm_value[ch];
        bool full_on  = (val == PCA9685_MAX_PWM_VALUE);
        bool full_off = (val == 0);
        uint16_t raw  = full_on ? 4095 : val;

        uint8_t buf[5];
        buf[0] = REG_LED_N(ch);              // LED register for this channel
        buf[1] = 0;                          // ON_L
        buf[2] = full_on ? LED_FULL_ON_OFF : 0; // ON_H
        buf[3] = raw & 0xFF;                 // OFF_L
        buf[4] = full_off ? (LED_FULL_ON_OFF | (raw >> 8)) : (raw >> 8); // OFF_H

        RETURN_ON_ERROR(i2c_master_transmit(dev_handle->i2c_dev_handle, buf, sizeof(buf), PCA9685_TIMEOUT_MS));
    }

    return ESP_OK;
}
