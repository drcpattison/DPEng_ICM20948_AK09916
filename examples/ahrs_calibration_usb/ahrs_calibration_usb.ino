#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DPEng_ICM20948_AK09916.h>

DPEng_ICM20948 dpEng = DPEng_ICM20948(0x948A, 0x948B, 0x948C);

// This sketch can be used to output raw sensor data in a format that
// can be understoof by MotionCal from PJRC. Download the application
// from http://www.pjrc.com/store/prop_shield.html and make note of the
// magentic offsets after rotating the sensors sufficiently.
//
// You should end up with 3 offsets for X/Y/Z, which are displayed
// in the top-right corner of the application.

void setup()
{
  Serial.begin(115200);

  // Wait for the Serial Monitor to open (comment out to run without Serial Monitor)
  // while(!Serial);

  Serial.println(F("DPEng 9 DOF AHRS Calibration Example")); Serial.println("");

  // Initialize the sensors.
  if(!dpEng.begin(ICM20948_ACCELRANGE_4G, GYRO_RANGE_250DPS, ICM20948_ACCELLOWPASS_50_4_HZ))
  {
    /* There was a problem detecting the sensor ... check your connections */
    Serial.println("Ooops, no sensor detected ... Check your wiring!");
    while(1);
  }

}
  
void loop(void)
{
	sensors_event_t accel_event;
	sensors_event_t gyro_event;
	sensors_event_t mag_event; // Need to read raw data, which is stored at the same time

    // Get new data samples
	dpEng.getEvent(&accel_event, &gyro_event, &mag_event);

	// Print the sensor data
	Serial.print("Raw:");
  
    Serial.print(dpEng.accel_raw.x);
    Serial.print(',');
    Serial.print(dpEng.accel_raw.y);
    Serial.print(',');
    Serial.print(dpEng.accel_raw.z);
    Serial.print(',');

    Serial.print(dpEng.gyro_raw.x);
    Serial.print(',');
    Serial.print(dpEng.gyro_raw.y);
    Serial.print(',');
    Serial.print(dpEng.gyro_raw.z);
    Serial.print(',');

    Serial.print(dpEng.mag_raw.x);
    Serial.print(',');
    Serial.print(dpEng.mag_raw.y);
    Serial.print(',');
    Serial.print(dpEng.mag_raw.z);
    Serial.println();

	delay(50);
}
