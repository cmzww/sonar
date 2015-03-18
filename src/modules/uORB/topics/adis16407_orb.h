#ifndef ADIS16407_ORB_H
#define	ADIS16407_ORB_H

#include <stdint.h>
#include "../uORB.h"

/**
 * report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct adis_report {

//SUPPLY_OUT  0x02  Power supply
float power_supply;	//binary format ,2.418mv/LSB ,0V=0X000

//XGYRO_OUT  0x04  Gyroscope, x-axis
//YGYRO_OUT  0x06  Gyroscope, y-axis
//ZGYRO_OUT  0x08  Gyroscope, z-axis
float x_gyro;					//twos complement format,0.05°/sec per LSB, when SENS_AVG[15:8] = 0x04
float y_gyro;					//twos complement format,0.05°/sec per LSB, when SENS_AVG[15:8] = 0x04
float z_gyro;					//twos complement format,0.05°/sec per LSB, when SENS_AVG[15:8] = 0x04

//XACCL_OUT  0x0A  Accelerometer, x-axis
//YACCL_OUT  0x0C  Accelerometer, y-axis
//ZACCL_OUT  0x0E  Accelerometer, z-axis
float x_accl;					//twos complement format,3.333 mg per LSB
float y_accl;					//twos complement format,3.333 mg per LSB
float z_accl;					//twos complement format,3.333 mg per LSB

//XMAGN_OUT  0x10  Magnetometer, x-axis
//YMAGN_OUT  0x12  Magnetometer, y-axis
//ZMAGN_OUT  0x14  Magnetometer, z-axis
float x_mag;						// twos complement format, 0.5 mgauss per LSB
float y_mag;						// twos complement format, 0.5 mgauss per LSB
float z_mag;						// twos complement format, 0.5 mgauss per LSB

//BARO_OUT  0x16  Barometer/pressure, higher,this is sufficient for most of applications
//BARO_OUTL  0x18  Barometer/pressure, lower
float baro_out;			//binary data format,0.08  mbar per LSB, 0x0000 = 0 mbar
float baro_outl;			//binary data format,0.0003125 mbar/LSB, 0x0000 = 0 mbar

//TEMP_OUT 1 0x1A  Internal temperature
float temp;						//twos complement,0.136°C/LSB,注意，这里是25°C = 0x000

//AUX_ADC
float aux_adc;				//binary format ,0.8059mv/LSB,0V=0X000

//ERROR FLAG
bool z_accl_err;
bool y_accl_err;
bool x_accl_err;

bool z_gyro_err;
bool y_gyro_err;
bool x_gyro_err;

bool flash_updated;
bool spi_comm;
bool overranged;
bool self_test_failed;
bool flash_test_failed;
bool baro_test_failed;
};

/* register this as object request broker structure */
ORB_DECLARE(adis_report);

#endif
