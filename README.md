# ESP32_BME68x_IDF
ESP32 with the Library BME68x for IDF
This example shows how to use the BOSCH API 'BME68x' with a BME680 - Sensor.

How to user

Use the Espressif-IDE based on the Eclipse CDT using ESP-IDF framework.
For Windows just Download 

Configure the project
Open file "sdkconfig" and set your specific parameters
- I2C_Master_SCT 
- I2C_Master_SDA

Build and flash

Note the CO2-Equivalent and IAQ are not calculated because this is done in Library BSEC / BESC2 which is not part of this example. It shows the Resistance 
