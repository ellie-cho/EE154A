#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); // IMU class with I2C interface
File logFile; // file on the sd card to write the data to

/* Initialize IMU */
bool setupIMU() {
  // Try to initialise and warn if we couldn't detect the chip
  while (!lsm.begin()) {
    Serial.println("Looking for LSM9DS1");
  }
  Serial.println("Found LSM9DS1");

  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G, lsm.LSM9DS1_ACCELDATARATE_119HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G, lsm.LSM9DS1_ACCELDATARATE_476HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G, lsm.LSM9DS1_ACCELDATARATE_952HZ);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);

  return true;
}

/* Initialize SD card */
bool setupSD() {
  if (!SD.begin(8)) {
    return false;
  }

  logFile = SD.open("kriv8", FILE_WRITE);
  if (!logFile) {
    Serial.println("Failed to open the file!");
    return false;
  }
  return true;
}


void setup() {
  Serial.begin(115200);
  Serial.println("SD Card / IMU demo");
  
  if (!setupIMU()) {
    Serial.println("IMU initialization failed!");
    return;
  }

  if (!setupSD()) {
    Serial.println("SD card initialization failed!");
    return;
  }

  Serial.println("Setup successful, starting to write");
}

void loop() {
  lsm.read();

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  logFile.print(millis()); logFile.print(",");
  logFile.print(a.acceleration.x); logFile.print(",");
  logFile.print(a.acceleration.y); logFile.print(",");
  logFile.print(a.acceleration.z); logFile.print(",");
  logFile.print(g.gyro.x); logFile.print(",");
  logFile.print(g.gyro.y); logFile.print(",");
  logFile.print(g.gyro.z); logFile.print(",");
  logFile.print(m.magnetic.x); logFile.print(",");
  logFile.print(m.magnetic.y); logFile.print(",");
  logFile.print(m.magnetic.z); logFile.print("\n");

  logFile.flush();
}
