#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <BME280I2C.h>
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); // IMU class with I2C interface
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
File logFile; // file on the sd card to write the data to
String inputTemp;

// Temperature sensor:
int temp_Pin = A1;
int current_Pin = A0;
int res = 10000; // value of the resistor used for the current sensor
double tempReading, currentReading;



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
  Wire.begin();
  if (!setupIMU()) {
    Serial.println("IMU initialization failed!");
    return;
  }

  if (!setupSD()) {
    Serial.println("SD card initialization failed!");
    return;
  }
  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }
  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
  Serial.println("Setup successful, starting to write");
}

void loop() {
  lsm.read();
  // this is the voltage value corresponding to the temperature
  // which should be calculated using this link: 
  // https://www.vishay.com/en/thermistors/curve-computation-list/

  tempReading = analogRead(temp_Pin) * 1023 / 3.3; 

  // using voltage divider concept to measure battery current
  currentReading = 3.3 / (2.3 * (analogRead(current_Pin) * 1023 / 3.3));
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
  logFile.print(m.magnetic.z); logFile.print(",");
  logFile.print(tempReading); logFile.print(",");
  // TODO: Gave up on writing code for BME...
  /*
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  Stream* client = &logFile;

  float temp(NAN), hum(NAN), pres(NAN);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  client->print(temp); logFile.print(",");
  client->print(hum); logFile.print(",");
  client->print(pres); logFile.print("\n");
  */

  logFile.flush();
}

//////////////////////////////////////////////////////////////////
void printBME280Data
(
   Stream* client
)
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print(temp);
   client->print(", ");
   client->print(hum);
   client->print(", ");
   client->print(pres);
   client->print("\n");

   delay(1000);
}