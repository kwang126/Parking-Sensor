#ifndef __Temp_Sensor_H__
#define __Temp_Sensor_H__

#define SENSOR_I2C_ADDRESS 0x70
#define CMD_MEASURE 0x7CA2      // Measure command for temp & humidity

class Temp_Sensor {

public:
    void init();    // Install I2C driver
    float read_temp_c();    // Return temperature in celcius
    float read_temp_f();    // Return temperature in fahrenheit
    float read_humidity();  // Return percentage humidity

};

#endif
