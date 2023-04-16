#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "string.h"

#define TAG "CJMCU_8118"

#define portTICK_RATE_MS portTICK_PERIOD_MS

#define CJMCU_8118_I2C_SDA_IO GPIO_NUM_26
#define CJMCU_8118_I2C_SCL_IO GPIO_NUM_27
#define CJMCU_8118_I2C_FREQ_HZ 100000

#define MAX_I2C_RXTX_BUF   15        
#define I2C_MASTER_TIMEOUT_MS   1000 
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

typedef struct
{
    uint8_t i2c_port;   // номер порта i2c  (0/1)
    uint8_t i2c_devadr; // адрес утройства
} i2c_dev_t;


esp_err_t i2c_master_init(uint8_t port, uint8_t sda_pin, uint8_t scl_pin, uint32_t clk)
{
    int i2c_master_port = (int)port;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (int)sda_pin;
    conf.sda_pullup_en = 1; // GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (int)scl_pin;
    conf.scl_pullup_en = 1; //GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clk;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_read(i2c_dev_t dev, uint8_t reg, void *data, uint8_t len)
{
    if (len > MAX_I2C_RXTX_BUF) // слишком длинная строка данных
    {
        return ESP_FAIL;
    }
    return i2c_master_write_read_device(dev.i2c_port, dev.i2c_devadr, &reg, 1, (uint8_t *)data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

esp_err_t i2c_write(i2c_dev_t dev, uint8_t reg, void *data, uint8_t len)
{
    uint8_t buf[MAX_I2C_RXTX_BUF+1] = {0};
    if (len > MAX_I2C_RXTX_BUF) // слишком длинная строка данных
    {
        return ESP_FAIL;
    }
    buf[0] = reg;
    memcpy(&buf[1], (uint8_t *)data, len);
    return i2c_master_write_to_device(dev.i2c_port, dev.i2c_devadr, buf, len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}
esp_err_t i2c_read_only(i2c_dev_t dev, void *data, uint8_t len)
{
    if (len > MAX_I2C_RXTX_BUF) // слишком длинная строка данных
    {
        return ESP_FAIL;
    }
        return i2c_master_read_from_device(dev.i2c_port, dev.i2c_devadr, (uint8_t *)data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

#define CCS811_I2C_ADDRESS 0x5A // default

/* CCS811 */
/* CCS811 register addresses */
#define CCS811_REG_STATUS 0x00
#define CCS811_REG_MEAS_MODE 0x01
#define CCS811_REG_ALG_RESULT_DATA 0x02
#define CCS811_REG_RAW_DATA 0x03
#define CCS811_REG_ENV_DATA 0x05
#define CCS811_REG_NTC 0x06
#define CCS811_REG_THRESHOLDS 0x10
#define CCS811_REG_BASELINE 0x11

#define CCS811_REG_HW_ID 0x20
#define CCS811_REG_HW_VER 0x21
#define CCS811_REG_FW_BOOT_VER 0x23
#define CCS811_REG_FW_APP_VER 0x24

#define CCS811_REG_ERROR_ID 0xe0

#define CCS811_REG_APP_ERASE 0xf1
#define CCS811_REG_APP_DATA 0xf2
#define CCS811_REG_APP_VERIFY 0xf3
#define CCS811_REG_APP_START 0xf4
#define CCS811_REG_SW_RESET 0xff

// status register bits
#define CCS811_STATUS_ERROR 0x01     // error, details in CCS811_REG_ERROR
#define CCS811_STATUS_DATA_RDY 0x08  // new data sample in ALG_RESULT_DATA
#define CCS811_STATUS_APP_VALID 0x10 // valid application firmware loaded
#define CCS811_STATUS_FW_MODE 0x80   // firmware is in application mode

// error register bits
#define CCS811_ERR_WRITE_REG_INV 0x01  // invalid register address on write
#define CCS811_ERR_READ_REG_INV 0x02   // invalid register address on read
#define CCS811_ERR_MEASMODE_INV 0x04   // invalid requested measurement mode
#define CCS811_ERR_MAX_RESISTANCE 0x08 // maximum sensor resistance exceeded
#define CCS811_ERR_HEATER_FAULT 0x10   // heater current not in range
#define CCS811_ERR_HEATER_SUPPLY 0x20  // heater voltage not applied correctly
// Meashure Mode
#define CCS811_MODE_IDLE 0x00  // Idle, low current mode
#define CCS811_MODE_1S 0x10    // Constant Power mode, IAQ values every 1 s
#define CCS811_MODE_10S 0x20   // Pulse Heating mode, IAQ values every 10 s
#define CCS811_MODE_60S 0x30   // Low Power Pulse Heating, IAQ values every 60 s
#define CCS811_MODE_250MS 0x40 // Constant Power mode, RAW data every 250 ms

#define CCS811_ALG_DATA_ECO2_HB 0
#define CCS811_ALG_DATA_ECO2_LB 1
#define CCS811_ALG_DATA_TVOC_HB 2
#define CCS811_ALG_DATA_TVOC_LB 3
#define CCS811_ALG_DATA_STATUS 4
#define CCS811_ALG_DATA_ERROR_ID 5
#define CCS811_ALG_DATA_RAW_HB 6
#define CCS811_ALG_DATA_RAW_LB 7

/*HDC1080*/
// dev addr
#define HDC1080_ADDR 0x40
// reg addr
#define HDC1080_TEMPERATURE 0x00
#define HDC1080_HUMIDITY 0x01
#define HDC1080_CONFIGURATION 0x02
// manufacturer id
#define HDC1080_MANUFACTURER_ID 0xFE
#define HDC1080_DEVICE_ID 0xFF
#define HDC1080_SERIAL_ID_FIRST 0xFB
#define HDC1080_SERIAL_ID_MID 0xFC
#define HDC1080_SERIAL_ID_LAST 0xFD

#define HDC1080_INIT 0x10 // 14 bit  temp+hum meashured

/**
 * forward declaration of functions for internal use only
 */

esp_err_t ccs811_check_error_status(i2c_dev_t dev);
esp_err_t ccs811_is_available(i2c_dev_t dev);

esp_err_t ccs811_init_sensor(i2c_dev_t dev)
{

    // check whether sensor is available including the check of the hardware
    // id and the error state
    esp_err_t ret = ccs811_is_available(dev);
    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 Sensor is not available.");
        return ret;
    }

    const static uint8_t sw_reset[4] = {0x11, 0xe5, 0x72, 0x8a};

    // doing a software reset first
    ret = i2c_write(dev, CCS811_REG_SW_RESET, (uint8_t *)sw_reset, 4);
    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 Could not reset the sensor.");
        return ret;
    }

    uint8_t status;

    // wait 100 ms after the reset
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // get the status to check whether sensor is in bootloader mode
    ret = i2c_read(dev, CCS811_REG_STATUS, &status, 1);
    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 Could not read status register %02x.", CCS811_REG_STATUS);
        return ret;
    }

    // if sensor is in bootloader mode (FW_MODE == 0), it has to switch
    // to the application mode first
    if (!(status & CCS811_STATUS_FW_MODE))
    {
        // check whether valid application firmware is loaded
        if (!(status & CCS811_STATUS_APP_VALID))
        {
            ESP_LOGE(TAG, "CCS811 Sensor is in boot mode, but has no valid application.");
            return ESP_FAIL;
        }

        // swtich to application mode
        status = 0;
        ret = i2c_write(dev, CCS811_REG_APP_START, &status, 0);
        if (ret)
        {
            ESP_LOGE(TAG, "CCS811 Could not start application");
            return ret;
        }

        // wait 100 ms after starting the app
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // get the status to check whether sensor switched to application mode
        ret = i2c_read(dev, CCS811_REG_STATUS, &status, 1);
        if ((ret != 0) || !(status & CCS811_STATUS_FW_MODE))
        {
            ESP_LOGE(TAG, "CCS811 Could not start application Status bit %x ", status);
            return ret;
        }
    }

    // try to set default measurement mode to *ccs811_mode_1s*
    // read  meashure mode
    ret = i2c_read(dev, CCS811_REG_MEAS_MODE, &status, 1);
    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 Could not read mode");
        return ret;
    }

    status &= 0xff;
    status |= CCS811_MODE_1S;

    // write back measurement mode register
    i2c_write(dev, CCS811_REG_MEAS_MODE, &status, 1);
    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 Could not set measurement mode.");
        return ret;
    }

    ret = i2c_read(dev, CCS811_REG_MEAS_MODE, &status, 1);
    if (ret != 0 || (status & 0xF0) != CCS811_MODE_1S)
    {
        ESP_LOGE(TAG, "CCS811 Could not set measurement mode %x ", status);
        return ret;
    }
    return ret;
}

esp_err_t ccs811_get_results(i2c_dev_t dev, uint16_t *iaq_tvoc, uint16_t *iaq_eco2, uint8_t *raw_i, uint16_t *raw_v)
{

    uint8_t data[8];

    // read IAQ sensor values and RAW sensor data including status and error id
    esp_err_t ret = i2c_read(dev, CCS811_REG_ALG_RESULT_DATA, data, 8);
    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 Could not read sensor data.");
        return ret;
    }

    // check for errors
    if (data[CCS811_ALG_DATA_STATUS] & CCS811_STATUS_ERROR)
    {
        return ccs811_check_error_status(dev);
    }

    // check whether new data are ready, if not, latest values are read from sensor
    // and error_code is set
    if (!(data[CCS811_ALG_DATA_STATUS] & CCS811_STATUS_DATA_RDY))
    {
        ESP_LOGE(TAG, "CCS811 No new data.");
    }

    // if *iaq* is not NULL return IAQ sensor values
    if (iaq_tvoc)
        *iaq_tvoc = data[CCS811_ALG_DATA_TVOC_HB] << 8 | data[CCS811_ALG_DATA_TVOC_LB];
    if (iaq_eco2)
        *iaq_eco2 = data[CCS811_ALG_DATA_ECO2_HB] << 8 | data[CCS811_ALG_DATA_ECO2_LB];

    // if *raw* is not NULL return RAW sensor data
    if (raw_i)
        *raw_i = data[CCS811_ALG_DATA_RAW_HB] >> 2;
    if (raw_v)
        *raw_v = (data[CCS811_ALG_DATA_RAW_HB] & 0x03) << 8 | data[CCS811_ALG_DATA_RAW_LB];

    return ESP_OK;
}
esp_err_t ccs811_set_environmental_data(i2c_dev_t dev, float temperature, float humidity)
{

    uint16_t temp = (temperature + 25) * 512; // -25 °C maps to 0
    uint16_t hum = humidity * 512;

    // fill environmental data
    uint8_t data[4] = {temp >> 8, temp & 0xff, hum >> 8, hum & 0xff};

    // send environmental data to the sensor
    esp_err_t ret = i2c_write(dev, CCS811_REG_ENV_DATA, data, 4);
    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 Could not write environmental data to sensor.");
        return ret;
    }

    return ret;
}

esp_err_t ccs811_check_error_status(i2c_dev_t dev)
{
    uint8_t status;
    uint8_t err_reg;
    esp_err_t ret;

    // check status register
    ret = i2c_read(dev, CCS811_REG_STATUS, &status, 1);

    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 ERR READ Satatus reg \n");
        return ret;
    }
    if (!status & CCS811_STATUS_ERROR)
        // everything is fine
        return ESP_OK;

    // Check the error register
    ret = i2c_read(dev, CCS811_REG_STATUS, &err_reg, 1);
    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 ERR READ err reg \n");
        return ret;
    }

    if (err_reg & CCS811_ERR_WRITE_REG_INV)
    {
        ESP_LOGE(TAG, "CCS811 Received an invalid register for write.");
        return err_reg;
    }

    if (err_reg & CCS811_ERR_READ_REG_INV)
    {
        ESP_LOGE(TAG, "CCS811 Received an invalid register for read.");
        return err_reg;
    }

    if (err_reg & CCS811_ERR_MEASMODE_INV)
    {
        ESP_LOGE(TAG, "CCS811 Received an invalid measurement mode request.");
        return err_reg;
    }

    if (err_reg & CCS811_ERR_MAX_RESISTANCE)
    {
        ESP_LOGE(TAG, "CCS811 Sensor resistance measurement has reached or exceeded the maximum range.");
        return err_reg;
    }

    if (err_reg & CCS811_ERR_HEATER_FAULT)
    {
        ESP_LOGE(TAG, "CCS811 Heater current not in range.");
        return err_reg;
    }

    if (err_reg & CCS811_ERR_HEATER_SUPPLY)
    {
        ESP_LOGE(TAG, "CCS811 Heater voltage is not being applied correctly.");
        return err_reg;
    }

    return ESP_FAIL;
}

esp_err_t ccs811_is_available(i2c_dev_t dev)
{
    uint8_t reg_data[5];

    // check hardware id (register 0x20) and hardware version (register 0x21)
    esp_err_t ret = i2c_read(dev, CCS811_REG_HW_ID, reg_data, 5);
    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 ERR READ ID \n");
        return ret;
    }

    if (reg_data[0] != 0x81)
    {
        ESP_LOGE(TAG, "CCS811 Wrong hardware ID %02x, should be 0x81\n", reg_data[0]);
        return ESP_FAIL;
    }
    ESP_LOGE(TAG, "CCS811 hardware version:      %02x\n", reg_data[1]);
    ESP_LOGE(TAG, "CCS811 firmware boot version: %02x\n", reg_data[3]);
    ESP_LOGE(TAG, "CCS811 firmware app version:  %02x\n", reg_data[4]);

    return ret;
}

esp_err_t hdc1080_init(i2c_dev_t dev)
{
    uint16_t data = HDC1080_INIT;
    uint16_t snid = 0;
    esp_err_t ret;

    ret = i2c_write(dev, HDC1080_CONFIGURATION, (uint8_t *)&data, 2);
    if (ret)
    {
        ESP_LOGE(TAG, "HDC1080 ERR write config\n");
    }

    vTaskDelay(100);

    ret = i2c_read(dev, 0xFE, (uint8_t *)&snid, 2);
    if (ret)
    {
        ESP_LOGE(TAG, "HDC1080 ERR read ID\n");
    }
    else
        ESP_LOGE(TAG, "HDC1080 read ID %x\n", snid);

    return ret;
}

esp_err_t hdc1080_read_data(i2c_dev_t dev, float *temp, float *hum)
{
    struct
    {
        uint16_t temp;
        uint16_t hum;
    } raw_temp_hum;

    uint32_t raw = 0;

    esp_err_t ret;
    //trigger start
    ret = i2c_write(dev, HDC1080_TEMPERATURE, (uint8_t *)&raw, 0);
    if (ret)
    {
        ESP_LOGE(TAG, "HDC1080 ERR write start meashure\n");
        return ret;
    }
    vTaskDelay(20 / portTICK_PERIOD_MS); // min delay 14 mc

    // read raw  data
    ret = i2c_read_only(dev, (uint8_t *)&raw, 4);
    if (ret)
    {
        ESP_LOGE(TAG, "HDC1080 ERR read data temp & hum\n");
        return ret;
    }
    // данные с датчика в обратной последовательнисти байт (msb<>lsb)
    raw_temp_hum.temp = ((raw & 0xFF) << 8) + ((raw & 0xFF00) >> 8);
    raw_temp_hum.hum = ((raw & 0xFF0000) >> 8) + ((raw & 0xFF000000) >> 24);
    *temp = (float)raw_temp_hum.temp * 165 / 65536 - 40;
    *hum = (float)raw_temp_hum.hum * 100 / 65536;
    return ESP_OK;
}

void AirReadTask(void *p)
{
    i2c_dev_t dev_ccs811;
    dev_ccs811.i2c_port = I2C_NUM_1;
    dev_ccs811.i2c_devadr = CCS811_I2C_ADDRESS;

    i2c_dev_t dev_hdc1080;
    dev_hdc1080.i2c_port = I2C_NUM_1;
    dev_hdc1080.i2c_devadr = HDC1080_ADDR;

    //инициализировать 12c канал 1 
    esp_err_t ret = i2c_master_init(I2C_NUM_1, CJMCU_8118_I2C_SDA_IO, CJMCU_8118_I2C_SCL_IO, CJMCU_8118_I2C_FREQ_HZ);
    if (ret)
    {
        ESP_LOGE(TAG, "I2C INIT ERR %d\n", ret);
    }

    
    ret = ccs811_init_sensor(dev_ccs811);
    if (ret)
    {
        ESP_LOGE(TAG, "CCS811 INIT ERR %d\n", ret);
    }

    ret = hdc1080_init(dev_hdc1080);
    if (ret)
    {
        ESP_LOGE(TAG, "HDC1080 INIT ERR %d\n", ret);
    }

    uint16_t tvoc, eco2, raw_v;
    uint8_t raw_i;
    float temp, hum;
    while (1)
    {
        // read TVOC & ECO2
        ret = ccs811_get_results(dev_ccs811, &tvoc, &eco2, &raw_i, &raw_v);
        if (ret)
        {
            ESP_LOGE(TAG, "CCS811 ERR READ DATA %d\n", ret);
        }
        else
        {
            ESP_LOGI(TAG, "CCS811 DATA tvoc %d, eco2 %d", tvoc, eco2);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
        //read Temp & Hum
        ret = hdc1080_read_data(dev_hdc1080, &temp, &hum);
        if (ret)
        {
            ESP_LOGE(TAG, "HDC1080 ERR READ DATA %d\n", ret);
        }
        else
        {
            ESP_LOGI(TAG, "HDC1080 DATA temp %f hum %f", temp, hum);

            ret = ccs811_set_environmental_data(dev_ccs811,temp,hum);
            if (ret)
            {
                ESP_LOGE(TAG, "CCS811 ERR WRITE TEMP & HUM  %d\n", ret);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void test_air()
{
xTaskCreate(AirReadTask, "air read", 2048*4, NULL, 1, NULL);
}