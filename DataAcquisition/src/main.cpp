#include <Arduino.h>
#include "mpu9250.h"

/* Mpu9250 object, SPI bus, CS on pin 10 */
bfs::Mpu9250 imu(&SPI, 10);

void setup()
{
	/* Serial to display data */
	Serial.begin(115200);
	while(!Serial) {}
	/* Start the SPI bus */
	SPI.begin();
	/* Initialize and configure IMU */
	if(!imu.Begin())
	{
		Serial.println("Error initializing communication with IMU");
		while(1) {}
	}
	/* Set the sample rate divider */
	if(!imu.ConfigSrd(19))
	{
		Serial.println("Error configured SRD");
		while(1) {}
	}
}

void loop()
{
	/* Check if data read */
	if(imu.Read())
	{
		Serial.print(imu.new_imu_data());
		Serial.print("\t");
		Serial.print(imu.new_mag_data());
		Serial.print("\t");
		Serial.print(imu.accel_x_mps2());
		Serial.print("\t");
		Serial.print(imu.accel_y_mps2());
		Serial.print("\t");
		Serial.print(imu.accel_z_mps2());
		Serial.print("\t");
		Serial.print(imu.gyro_x_radps());
		Serial.print("\t");
		Serial.print(imu.gyro_y_radps());
		Serial.print("\t");
		Serial.print(imu.gyro_z_radps());
		Serial.print("\t");
		Serial.print(imu.mag_x_ut());
		Serial.print("\t");
		Serial.print(imu.mag_y_ut());
		Serial.print("\t");
		Serial.print(imu.mag_z_ut());
		Serial.print("\t");
		Serial.print(imu.die_temp_c());
		Serial.print("\n");
	}
}
