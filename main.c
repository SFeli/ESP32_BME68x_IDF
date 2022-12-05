#include "stdio.h"
#include "stdlib.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "bme68x.h"
#include "bme68x_defs.h"

// Parameters from sdkconfig 		-> please run menuconfig
//#define I2C_MASTER_NUM              0         /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_SCL_IO  		CONFIG_I2C_MASTER_SCL		/*!< e.g. 23 GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO		CONFIG_I2C_MASTER_SDA		/*!< e.g. 18 GPIO number used for I2C master data  */
#define I2C_MASTER_NUM			CONFIG_I2C_MASTER_NUM		/*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ      400000       /*!< e.g.400000   I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define RTOS_DELAY_1SEC          ( 1 * 1000 / portTICK_PERIOD_MS)

#define BME680_CHIP_ID              0xD0        /*!< Primary address of the BME68x Sensor */
#define BOSCH_RESET_VALUE           0xE0		// E0

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
// #define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE

int8_t rslt;
i2c_cmd_handle_t 			cmd;			// I2C handle
struct bme68x_dev  			bme68x;			// Sensor as Object
struct bme68x_conf 			conf;			// Configuration
struct bme68x_heatr_conf 	heatr_conf;		// Heater configuration
struct bme68x_data 			data;			// Date
uint8_t 					n_fields;		// neues Feld für 68x
static const char *TAG = "ESP32-BME68x-IDF  ";

/**
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Pointer to the data buffer to store the read data.
 * @param[in]     length   : Length of the reg_data array to read
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
static BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    char* TAGR = "ESP32-BME68x-Read ";
    uint8_t addr = BME68X_I2C_ADDR_HIGH;				// e.g. 0x76 or 0x77

    if (len == 0) return true;

    cmd = i2c_cmd_link_create();

    if (reg_addr)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( addr << 1 ) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg_addr, true);
        if (!reg_data)
            i2c_master_stop(cmd);
    }
    if (reg_data)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( addr << 1 ) | I2C_MASTER_READ, true);
        if (len > 1) i2c_master_read(cmd, reg_data, len-1, 0);
        i2c_master_read_byte(cmd, reg_data + len-1, 1);
        i2c_master_stop(cmd);
    }
    esp_err_t err = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return err;

    /*    uint32_t z_i;
        for (z_i = 0; z_i < len; ++z_i) {
        	ESP_LOGD(TAGR, "z_i[%ld] : , %x" , z_i, reg_data[z_i]);
        } */

	if (len == 1) {
		ESP_LOGD(TAGR, "Reg: %0x Data: %0x len: %ld" , reg_addr, reg_data[0], len);
	}
	if (len == 2) {
		ESP_LOGD(TAGR, "Reg: %0x Data: %0x %0x len: %ld" , reg_addr, reg_data[0], reg_data[1], len);
	}
	if (len == 3) {
		ESP_LOGD(TAGR, "Reg: %0x Data: %0x %0x %0x len: %ld" , reg_addr, reg_data[0], reg_data[1], reg_data[2], len);
	}
	if (len == 4) {
		ESP_LOGD(TAGR, "Reg: %0x Data: %0x %0x %0x %0x len: %ld" , reg_addr, reg_data[0], reg_data[1], reg_data[2], reg_data[3], len);
	}
	if (len >= 5) {
		ESP_LOGD(TAGR, "Reg: %0x Data: %0x %0x %0x %0x %0x len: %ld" , reg_addr, reg_data[0], reg_data[1], reg_data[2], reg_data[3], reg_data[4], len);
	}

	switch (reg_addr)
	{
	case 0x74:
		ESP_LOGD(TAGR, "Register to read  %x -> Mode-Selection %x", reg_addr, (uint8_t)*reg_data);
		break;
	case 0x1D:
		ESP_LOGD(TAGR, "Register to read  %x -> Messstatus %x", reg_addr, (uint8_t)*reg_data);
		break;
	case 0xd0:
		ESP_LOGD(TAGR, "Register to read  %x -> Chip-ID: %x", reg_addr, (uint8_t)*reg_data);
		break;
	case 0x50:
		ESP_LOGD(TAGR, "Register to read  %x -> 0th Current DAC address", reg_addr);
		break;
	case 0x5a:
		ESP_LOGD(TAGR, "Register to read  %x -> 0th Res heat address", reg_addr);
		break;
	case 0x64:
		ESP_LOGD(TAGR, "Register to read  %x -> 0th Gas wait address", reg_addr);
		break;
	case 0x71:
		ESP_LOGD(TAGR, "Register to read  %x -> CTRL_GAS_1 address", reg_addr);
		break;
	case 0x8a:
		ESP_LOGD(TAGR, "Register to read  %x -> Coefficents No1", reg_addr);
		break;
	case 0xe1:
		ESP_LOGD(TAGR, "Register to read  %x -> Coefficents No2", reg_addr);
		break;
	case 0x00:
		ESP_LOGD(TAGR, "Register to read  %x -> Coefficents No3 res_heat_val", reg_addr);
		break;
	case 0xf0:
		ESP_LOGD(TAGR, "Register to read  %x -> Variant-ID %x", reg_addr, (uint8_t)*reg_data);
		break;
	case 0xf4:
		ESP_LOGD(TAGR, "Register to read  %x -> controls", reg_addr);
		break;
	case 0xf7:
		ESP_LOGD(TAGR, "Register to read  %x -> burst-read of values", reg_addr);
		break;
	default:
		ESP_LOGD(TAGR, "Register to read  %x -> not known", reg_addr);
		break;
	}
}

/**
 *  @brief Function for writing the sensor's registers through I2C bus.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] len      : No of bytes to write.
 *  @param[in] intf_ptr : Pointer Interface pointer
 *  @return Status of execution
 *  @retval BMP68x_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP68x_INTF_RET_SUCCESS -> Failure.
 */
static BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t  len, void *intf_ptr)
{
    char* TAGW = "ESP32-BMP68x-Write";
    uint8_t addr = BME68X_I2C_ADDR_HIGH;


    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    if (reg_addr) {
        i2c_master_write_byte(cmd, reg_addr, true);
    }
    if (reg_data) {
        i2c_master_write(cmd, reg_data, len, true);
    }
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return 0;

    //uint32_t z_i;
    //for (z_i = 0; z_i < len; ++z_i) {
    //	ESP_LOGD(TAGW, "z_i[%ld] : , %x" , z_i, reg_data[z_i]);
    //}
    if (len == 1) {
    	ESP_LOGD(TAGW, "Reg: %0x Data: %0x Len: %ld" , reg_addr, reg_data[0], len);
    }
    if (len >= 2) {
    	ESP_LOGD(TAGW, "Reg: %0x Data: %0x %0x Len: %ld" , reg_addr, reg_data[0], reg_data[1], len);
    }
    switch (reg_addr)
    {
    case 0x70:
        ESP_LOGD(TAGW, "Register to write %x %i -> Heater", reg_addr, (uint8_t)*reg_data);
        break;
    case 0x71:
        ESP_LOGD(TAGW, "Register to write %x %i -> Config", reg_addr, (uint8_t)*reg_data);
        break;
    case 0x74:
        ESP_LOGD(TAGW, "Register to write %x %i -> Mode Selection (0 sleep / 1 forced)", reg_addr, (uint8_t)*reg_data);
        ESP_LOGD(TAGW, "1 bit %x", (reg_data[0] >> 1) & 0x1);
        ESP_LOGD(TAGW, "0 bit %x", (reg_data[0] >> 0) & 0x1);
        break;
    case 0xe0:
        ESP_LOGD(TAGW, "Register to write %X -> RESET", reg_addr);
        //ESP_LOGD(TAGW, "%x %x %x %x", ((reg_data[0] >> 3) & 0x1), ((reg_data[0] >> 2) & 0x1), ((reg_data[0] >> 1) & 0x1), ((reg_data[0] >> 0) & 0x1));
        //ESP_LOGD(TAGW, "%x %x %x %x", ((reg_data[0] >> 7) & 0x1), ((reg_data[0] >> 6) & 0x1), ((reg_data[0] >> 5) & 0x1), ((reg_data[0] >> 4) & 0x1));
        break;
    case 0xf4:
        ESP_LOGD(TAGW, "Register to write %x -> controls", reg_addr);
        break;
    default:
        ESP_LOGD(TAGW, "Register to write %x -> not known", reg_addr);
        break;
    }
}

/*!
 * Delay function 
 */
void bme68x_delay_us(uint32_t period_ms, void *intf_ptr)
{
    vTaskDelay(period_ms / portTICK_PERIOD_MS / 100);
    //printf("Function delay 1\n");
    //vTaskDelay(1000);
    //printf("Function delay 2\n");
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_cmd_handle_t cmd;
	ESP_LOGI(TAG, "I2C_MASTER_PORT  :  %d", I2C_NUM_0);
	ESP_LOGI(TAG, "I2C_MASTER_MODE  :  %d (1..master or 0..slave)", I2C_MODE_MASTER);
	ESP_LOGI(TAG, "I2C_MASTER_SDA_IO:  %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "I2C_MASTER_SCL_IO:  %d", I2C_MASTER_SCL_IO);
   	ESP_LOGI(TAG, "I2C_MASTER_FREQ_HZ  %d", I2C_MASTER_FREQ_HZ);

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    rslt = i2c_param_config(I2C_NUM_0, &i2c_conf);
    if (rslt != ESP_OK)
        ESP_LOGE(TAG, "Error in i2c-configuration RC=%d", rslt);
    rslt = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (rslt != ESP_OK)
        ESP_LOGE(TAG, "Error in i2c-driver RC=%d", rslt);
    else
        ESP_LOGI(TAG, "I2C - initialisation RC=%d", rslt);
//*************// Verify if I2C slave is working properly
  // Test  1
/*    int ret;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x76 << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGI(TAG,"Test 1 (76) Returncode %i", ret);*/

 // Test 2
 /*   cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x77 << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGI(TAG,"Test 2 (77) Returncode %i", ret); */

//	Test Ende
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME68X_I2C_ADDR_HIGH << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    rslt = i2c_master_cmd_begin(I2C_NUM_0, cmd, RTOS_DELAY_1SEC);
    if (rslt != ESP_OK) {
        ESP_LOGE(TAG, "I2C slave NOT working or wrong I2C slave address - RC=%d", rslt);
    }
    i2c_cmd_link_delete(cmd);
    return rslt;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello from ESP32!");
    //esp_log_level_set("*", ESP_LOG_INFO);

    
    if (i2c_master_init() == ESP_OK)
    {
        // Link I2C - Library to BMP2 - calls
        static uint8_t dev_addr;
        //int8_t rslt = BMP2_OK;
        dev_addr     = BME68X_I2C_ADDR_HIGH;

/*!
 * @brief Bus communication function pointer which should be mapped to the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
//typedef BME68X_INTF_RET_TYPE (*bme68x_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

        bme68x.read  = bme68x_i2c_read;
        bme68x.write = bme68x_i2c_write;
        bme68x.intf  = BME68X_I2C_INTF;        /* Holds the I2C device addr */
        bme68x.intf_ptr = &dev_addr;
        ESP_LOGD(TAG, "Print &dev_addr   %x", dev_addr);
        ESP_LOGD(TAG, "Print bme68x.inf_ptr %x", *(int *)bme68x.intf_ptr);
        //bme68x.delay_us = delay_ms;         /* Configure delay in microseconds */
        bme68x.delay_us = bme68x_delay_us;
        bme68x.amb_temp = 25;

        rslt = bme68x_init(&bme68x);
        ESP_LOGI(TAG, "BME68x initialization RC=%d - Variant-ID (0..low / 1 .. high) %ld", rslt, bme68x.variant_id);

        /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
        uint8_t reg_data[2];
        rslt = bme68x_i2c_read(BME680_CHIP_ID, reg_data, 1, &dev_addr);
        ESP_LOGI(TAG, "WHO_AM_I = %X", reg_data[0]);

        //rslt = bme68x_selftest_check(&bme68x);
        //ESP_LOGI(TAG, "BME68x_selftest_check RC=%d", rslt);


    /* Always read the current settings before writing, especially when all the configuration is not modified */
        rslt = bme68x_get_conf(&conf, &bme68x);
        ESP_LOGI(TAG, "BME68X get config RC=%d", rslt);

        /* Configuring the over-sampling mode, filter coefficient and output data rate */
        conf.filter = BME68X_FILTER_OFF;

        /* Setting the output data rate */
        conf.odr = BME68X_ODR_NONE;

        /* Over-sampling mode is set as high resolution i.e., os_pres = 8x and os_temp = 1x */
        //conf.os_mode = BME68x_OS_MODE_HIGH_RESOLUTION;
        conf.os_hum  = BME68X_OS_16X;
        conf.os_pres = BME68X_OS_1X;
        conf.os_temp = BME68X_OS_2X;

        rslt = bme68x_set_conf(&conf, &bme68x);
        ESP_LOGI(TAG, "BME68x set config RC=%d", rslt);

     /* Heater temperature in degree Celsius */
        uint16_t temp_prof[10] = { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 };
     /* Multiplier to the shared heater duration */
        uint16_t mul_prof[10] = { 5, 2, 10, 30, 5, 5, 5, 5, 5, 5 };
        heatr_conf.heatr_temp_prof = temp_prof;
        heatr_conf.heatr_dur_prof  = mul_prof;
     /* Shared heating duration in milliseconds */
        //heatr_conf.shared_heatr_dur = 140 - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme68x) / 1000);
        heatr_conf.profile_len = 10;

        heatr_conf.enable     = BME68X_ENABLE;
        //heatr_conf.heatr_temp = BME68X_HIGH_TEMP;
        //heatr_conf.heatr_dur  = 100; 				// 	BME68X_HEATR_DUR1;
        rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme68x);
        ESP_LOGI(TAG, "BMP68x heater config RC=%d", rslt);

        while (1)
        {
            rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme68x);
            ESP_LOGD(TAG, "BMP68x heater config RC=%d", rslt);

            //rslt = bme68x_set_conf(&conf, &bme68x);
            //ESP_LOGI(TAG, "BME68x set config RC=%d", rslt);

			/* Set forced power mode 				---> must be in the while - loop !!!   */
			rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme68x);
			ESP_LOGD(TAG, "BME68x set power  RC=%d", rslt);

	        /* Calculate delay period in microseconds */
			uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme68x) + (heatr_conf.heatr_dur * 1000);
	        bme68x.delay_us(del_period, bme68x.intf_ptr);
			ESP_LOGD(TAG, "BME68x del_period = %ld", del_period);

            rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme68x);

#ifdef BME68X_USE_FPU
      ESP_LOGI(TAG, "Data-> Temp: %.2f C // Humi: %.2f (%%)// Pres: %.1f hPa // IDAC: %d  // Gas-Residence %.2f Ohm", data.temperature, data.humidity, data.pressure/100, data.idac, data.gas_resistance);
#else
      ESP_LOGI(TAG, "Data-> Temp: %.1f C // Humi: %.2f (%%) // Pres: %.2lu hPa // IDAC: %d  // Gas-Residence %lu Ohm", (data.temperature/100.0), (data.humidity/1000.0), (long unsigned int)(data.pressure)/100, data.idac, data.gas_resistance);

#endif
        }
    }
}
