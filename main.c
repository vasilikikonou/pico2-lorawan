#include <stdio.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "tusb.h"
#include "config.h"

#define DHT20_I2C_ADDR 0x38
#define BMP280_ADDR 0x76 
#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

#define RAIN_SENSOR_PIN 27  // ADC1 = GPIO27
#define RAIN_ADC_CHANNEL 1

#define LDR_SENSOR_PIN 28   // ADC2 = GPIO28
#define LDR_ADC_CHANNEL 2

// BMP280 hardware registers
#define REG_CONFIG 0xF5
#define REG_CTRL_MEAS 0xF4
#define REG_RESET 0xE0

#define REG_TEMP_XLSB 0xFC
#define REG_TEMP_LSB 0xFB
#define REG_TEMP_MSB 0xFA

#define REG_PRESSURE_XLSB 0xF9
#define REG_PRESSURE_LSB 0xF8
#define REG_PRESSURE_MSB 0xF7

// BMP280 calibration registers
#define REG_DIG_T1_LSB 0x88
#define REG_DIG_T1_MSB 0x89
#define REG_DIG_T2_LSB 0x8A
#define REG_DIG_T2_MSB 0x8B
#define REG_DIG_T3_LSB 0x8C
#define REG_DIG_T3_MSB 0x8D
#define REG_DIG_P1_LSB 0x8E
#define REG_DIG_P1_MSB 0x8F
#define REG_DIG_P2_LSB 0x90
#define REG_DIG_P2_MSB 0x91
#define REG_DIG_P3_LSB 0x92
#define REG_DIG_P3_MSB 0x93
#define REG_DIG_P4_LSB 0x94
#define REG_DIG_P4_MSB 0x95
#define REG_DIG_P5_LSB 0x96
#define REG_DIG_P5_MSB 0x97
#define REG_DIG_P6_LSB 0x98
#define REG_DIG_P6_MSB 0x99
#define REG_DIG_P7_LSB 0x9A
#define REG_DIG_P7_MSB 0x9B
#define REG_DIG_P8_LSB 0x9C
#define REG_DIG_P8_MSB 0x9D
#define REG_DIG_P9_LSB 0x9E
#define REG_DIG_P9_MSB 0x9F

#define NUM_CALIB_PARAMS 24

// BMP280 calibration structure
struct bmp280_calib_param {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
};

struct bmp280_calib_param bmp280_params;

// pin configuration for SX12xx radio module
const struct lorawan_sx12xx_settings sx12xx_settings = {
    .spi = {
        .inst = spi1,
        .mosi = 11,
        .miso = 12,
        .sck  = 10,
        .nss  = 3
    },
    .reset = 15,
    .busy = 2,
    .dio1  = 20
};

// OTAA settings
const struct lorawan_otaa_settings otaa_settings = {
    .device_eui   = LORAWAN_DEVICE_EUI,
    .app_eui      = LORAWAN_APP_EUI,
    .app_key      = LORAWAN_APP_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
};

// I2C Initialization
void i2c_sensors_init(void) {
    i2c_init(I2C_PORT, 100 * 1000); // 100kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    sleep_ms(100);
    printf("I2C initialized on GP%d (SDA) and GP%d (SCL)\n", I2C_SDA_PIN, I2C_SCL_PIN);
}

// I2C Scanner
void i2c_scan(void) {
    printf("\nI2C Bus Scan:\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        int ret;
        uint8_t rxdata;
        ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Found devices: @ = detected\n\n");
}

// DHT20 Functions
bool dht20_read(float *temperature, float *humidity) {
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t data[7];
    
    int ret = i2c_write_blocking(I2C_PORT, DHT20_I2C_ADDR, cmd, 3, false);
    if (ret < 0) return false;
    
    sleep_ms(80);
    
    ret = i2c_read_blocking(I2C_PORT, DHT20_I2C_ADDR, data, 7, false);
    if (ret < 0) return false;
    
    if (data[0] & 0x80) return false;
    
    uint32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((data[3] & 0xF0) >> 4);
    uint32_t raw_temp = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    
    *humidity = (float)raw_humidity / 1048576.0 * 100.0;
    *temperature = (float)raw_temp / 1048576.0 * 200.0 - 50.0;
    return true;
}

// BMP280 Functions 
void bmp280_init(void) {
    uint8_t buf[2];

    const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;

    buf[0] = REG_CONFIG;
    buf[1] = reg_config_val;
    i2c_write_blocking(I2C_PORT, BMP280_ADDR, buf, 2, false);

    const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);
    buf[0] = REG_CTRL_MEAS;
    buf[1] = reg_ctrl_meas_val;
    i2c_write_blocking(I2C_PORT, BMP280_ADDR, buf, 2, false);
}

void bmp280_read_raw(int32_t* temp, int32_t* pressure) {
    uint8_t buf[6];
    uint8_t reg = REG_PRESSURE_MSB;
    i2c_write_blocking(I2C_PORT, BMP280_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, BMP280_ADDR, buf, 6, false);

    *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
}

void bmp280_reset(void) {
    uint8_t buf[2] = { REG_RESET, 0xB6 };
    i2c_write_blocking(I2C_PORT, BMP280_ADDR, buf, 2, false);
}

int32_t bmp280_convert(int32_t temp, struct bmp280_calib_param* params) {
    int32_t var1, var2;
    var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) * ((int32_t)params->dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) * ((temp >> 4) - ((int32_t)params->dig_t1))) >> 12) * ((int32_t)params->dig_t3)) >> 14;
    return var1 + var2;
}

int32_t bmp280_convert_temp(int32_t temp, struct bmp280_calib_param* params) {
    int32_t t_fine = bmp280_convert(temp, params);
    return (t_fine * 5 + 128) >> 8;
}

int32_t bmp280_convert_pressure(int32_t pressure, int32_t temp, struct bmp280_calib_param* params) {
    int32_t t_fine = bmp280_convert(temp, params);

    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)params->dig_p6);
    var2 += ((var1 * ((int32_t)params->dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)params->dig_p4) << 16);
    var1 = (((params->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)params->dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)params->dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;
    }
    converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)params->dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)params->dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + params->dig_p7) >> 4));
    return converted;
}

void bmp280_get_calib_params(struct bmp280_calib_param* params) {
    uint8_t buf[NUM_CALIB_PARAMS] = { 0 };
    uint8_t reg = REG_DIG_T1_LSB;
    i2c_write_blocking(I2C_PORT, BMP280_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, BMP280_ADDR, buf, NUM_CALIB_PARAMS, false);

    params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];
}

bool bmp280_check_and_init(void) {
    printf("Checking BMP280 at address 0x%02X...\n", BMP280_ADDR);
    
    uint8_t chip_id_reg = 0xD0;
    uint8_t chip_id;
    
    if (i2c_write_blocking(I2C_PORT, BMP280_ADDR, &chip_id_reg, 1, true) < 0) {
        printf("BMP280: Write failed (device not responding)\n");
        return false;
    }
    
    if (i2c_read_blocking(I2C_PORT, BMP280_ADDR, &chip_id, 1, false) < 0) {
        printf("BMP280: Read failed\n");
        return false;
    }
    
    printf("BMP280: Chip ID = 0x%02X ", chip_id);
    if (chip_id == 0x58) {
        printf("(BMP280 detected)\n");
    } else if (chip_id == 0x60) {
        printf("(BME280 detected - should work)\n");
    } else {
        printf("(WRONG CHIP ID!)\n");
        printf("Try changing BMP280_ADDR to 0x77\n");
        return false;
    }
    
    // Reset the device
    printf("BMP280: Resetting...\n");
    bmp280_reset();
    sleep_ms(100);
    
    // Configure BMP280
    printf("BMP280: Configuring...\n");
    bmp280_init();
    
    // Get calibration parameters
    printf("BMP280: Reading calibration...\n");
    bmp280_get_calib_params(&bmp280_params);
    
    printf("BMP280: Initialized successfully!\n");
    return true;
}

bool bmp280_read_values(float *pressure, float *temperature) {
    int32_t raw_temperature;
    int32_t raw_pressure;
    
    bmp280_read_raw(&raw_temperature, &raw_pressure);
    
    // Check for invalid data
    if (raw_pressure == 0x80000 || raw_temperature == 0x80000) {
        return false;
    }
    
    int32_t temp_int = bmp280_convert_temp(raw_temperature, &bmp280_params);
    int32_t pressure_int = bmp280_convert_pressure(raw_pressure, raw_temperature, &bmp280_params);
    
    *temperature = temp_int / 100.0f;
    *pressure = pressure_int / 1000.0f;  // Convert Pa to kPa
    
    return true;
}

// Rain Sensor Functions
void rain_sensor_init(void) {
    adc_init();
    adc_gpio_init(RAIN_SENSOR_PIN);
}

uint16_t read_rain_sensor(void) {
    adc_select_input(RAIN_ADC_CHANNEL);
    return adc_read();
}

const char* get_rain_status(uint16_t adc_value) {
    uint16_t scaled_value = adc_value >> 2;
    
    if (scaled_value < 101) {
        return "DRY";
    } else if (scaled_value > 499) {
        return "WET";
    } else {
        return "MOIST";
    }
}

// LDR (Light Sensor) Functions
void ldr_sensor_init(void) {
    adc_gpio_init(LDR_SENSOR_PIN);
}

uint16_t read_ldr_sensor(void) {
    adc_select_input(LDR_ADC_CHANNEL);
    return adc_read();
}

#define ADC_BRIGHT_THRESHOLD 101
#define ADC_LIGHT_THRESHOLD 59
#define ADC_DIM_THRESHOLD 19


uint8_t get_light_percentage(uint16_t adc_value) {
    
    // // If we are at or above "BRIGHT", it's 100%
    if (adc_value >= ADC_BRIGHT_THRESHOLD) {
        return 100;
    }
    // If we are at or below "DIM", it's 0%
    if (adc_value <= ADC_DIM_THRESHOLD) {
        return 0;
    }
  
    uint32_t percent = ((uint32_t)adc_value - ADC_DIM_THRESHOLD) * 100 / (ADC_BRIGHT_THRESHOLD - ADC_DIM_THRESHOLD);
    return (uint8_t)percent;
}

// const char* get_light_level(uint16_t adc_value) {
    // uint8_t percent = get_light_percentage(adc_value);
    
const char* get_light_level(uint16_t adc_value) {
    
    if (adc_value > ADC_BRIGHT_THRESHOLD) {
        return "BRIGHT";
    } else if (adc_value > ADC_LIGHT_THRESHOLD) {
        return "LIGHT";
    } else if (adc_value > ADC_DIM_THRESHOLD) {
        return "DIM";
    } else {
        return "DARK";
    }

}

// MAIN
int main(void) {  
    stdio_init_all();

    printf("Pico LoRaWAN Weather Station\n");

    // Initialize LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Initialize I2C
    printf("Initializing I2C...\n");
    i2c_sensors_init();
    
    // Scan I2C bus
    i2c_scan();
    
    sleep_ms(200);
    
    // Test DHT20
    printf("Testing DHT20...\n");
    float test_temp, test_humid;
    if (dht20_read(&test_temp, &test_humid)) {
        printf("DHT20: OK - %.2f°C, %.2f%%\n\n", test_temp, test_humid);
    } else {
        printf("DHT20: FAILED\n\n");
    }
    
    // Initialize BMP280
    bool bmp280_available = bmp280_check_and_init();
    if (!bmp280_available) {
        printf("\n!!! BMP280 FAILED TO INITIALIZE !!!\n");
        printf("Weather station will continue without pressure sensor\n\n");
    } else {
        // Test read
        float test_press, test_bmp_temp;
        sleep_ms(250);
        if (bmp280_read_values(&test_press, &test_bmp_temp)) {
            printf("BMP280: Test read OK - %.2f hPa, %.2f°C\n\n", test_press * 10, test_bmp_temp);
        } else {
            printf("BMP280: Test read FAILED\n\n");
            bmp280_available = false;
        }
    }
    
    // Initialize rain sensor
    rain_sensor_init();
    printf("Rain sensor: OK (GP27/ADC1)\n");
    
    // Initialize LDR sensor
    ldr_sensor_init();
    printf("LDR sensor: OK (GP28/ADC2)\n\n");

    // Initialize LoRaWAN
    printf("Initializing LoRaWAN... ");
    if (lorawan_init_otaa(&sx12xx_settings, LORAWAN_REGION, &otaa_settings) < 0) {
        printf("failed!\n");
        while (1) tight_loop_contents();
    }
    printf("success!\n");

    // Join network
    printf("Joining LoRaWAN network...");
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process_timeout_ms(1000);
        printf(".");
    }
    printf(" joined!\n\n");

    // Timing
    absolute_time_t last_send_time = get_absolute_time();
    const uint32_t send_interval_ms = 65*1000; // 65 seconds

    // Main loop
    while (1) {
        lorawan_process();

        if (lorawan_is_joined() && 
            absolute_time_diff_us(last_send_time, get_absolute_time()) >= (send_interval_ms * 1000)) {
            
            last_send_time = get_absolute_time();
            
            printf("\n=== Reading Sensors ===\n");
            
            float temperature = 0.0;
            float humidity = 0.0;
            float pressure = 0.0;
            float bmp_temp = 0.0;
            uint16_t rain_adc = 0;
            uint16_t ldr_adc = 0;
            
            // Read DHT20
            if (dht20_read(&temperature, &humidity)) {
                printf("DHT20: %.2f°C, %.2f%%\n", temperature, humidity);
            } else {
                printf("DHT20: FAILED\n");
                temperature = -999.0;
                humidity = -999.0;
            }
            
            // Read BMP280
            if (bmp280_available && bmp280_read_values(&pressure, &bmp_temp)) {
                printf("BMP280: %.2f hPa (%.2f°C)\n", pressure * 10, bmp_temp);
                pressure = pressure * 10; // Convert kPa to hPa
            } else {
                if (bmp280_available) {
                    printf("BMP280: READ FAILED\n");
                }
                pressure = 0.0;
            }
            
            // Read rain sensor
            rain_adc = read_rain_sensor();
            const char* rain_status = get_rain_status(rain_adc);
            printf("Rain: %d (%s)\n", rain_adc, rain_status);
            
            // Read LDR sensor
            ldr_adc = read_ldr_sensor();
            uint8_t light_percent = get_light_percentage(ldr_adc);
            const char* light_level = get_light_level(ldr_adc);
            printf("Light: %d ADC (%d%% - %s)\n", ldr_adc, light_percent, light_level);
            
            // Prepare payload (11 bytes total)
            // Format: [temp(2), humid(2), pressure(2), rain_adc(2), rain_status(1), ldr_adc(2)]
            uint8_t payload[11];
            
            int16_t temp_x100 = (int16_t)(temperature * 100);
            payload[0] = (temp_x100 >> 8) & 0xFF;
            payload[1] = temp_x100 & 0xFF;
            
            uint16_t humid_x100 = (uint16_t)(humidity * 100);
            payload[2] = (humid_x100 >> 8) & 0xFF;
            payload[3] = humid_x100 & 0xFF;
            
            uint16_t pressure_x10 = (uint16_t)(pressure * 10);
            payload[4] = (pressure_x10 >> 8) & 0xFF;
            payload[5] = pressure_x10 & 0xFF;
            
            payload[6] = (rain_adc >> 8) & 0xFF;
            payload[7] = rain_adc & 0xFF;
            
            if (rain_status[0] == 'D') payload[8] = 0;
            else if (rain_status[0] == 'M') payload[8] = 1;
            else payload[8] = 2;
            
            // LDR data (2 bytes)
            payload[9] = (ldr_adc >> 8) & 0xFF;
            payload[10] = ldr_adc & 0xFF;
            
            printf("\nSending to TTN...");
            if (lorawan_send_unconfirmed(payload, sizeof(payload), 2) < 0) {
                printf(" FAILED\n");
            } else {
                printf(" OK\n");
            }
            
            printf("=== Next in 5 min ===\n\n");
        }
        
        sleep_ms(100);
    }

    return 0;
}
