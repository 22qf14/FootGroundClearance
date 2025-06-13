
// Wearable Foot Ground Clearance measurement device code
// Safia Adair - May 2025

//This code uses 2 sensors on the same I2C bus by using the XSHUT pin of one of the sensors

//This is the I2C communication library, sensor library and SD card library 
#include <Wire.h>
#include <SPI.h>
#include <VL53L1X_ULD.h>
#include <SD.h>

// Define SD card chip select pin, pin 10 for Arduino Nano Every
#define SD_CS 10 

#define SENSOR_1_XSHUT 3
#define SENSOR_2_I2C_ADDRESS 0x55

// Create sensor objects
VL53L1X_ULD sensor_1;
VL53L1X_ULD sensor_2;

// File object for SD logging
File logFile;

// For IMU
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
float angle_pitch = 0, angle_roll = 0;
float alpha = 0.98;
unsigned long lastIMUTime = 0;

void setup() {
  delay(500);
  Serial.begin(115200);  // Start Serial communication to display sensor readings, between brackets is the baud rate (= bit per second)
  Wire.begin();          // Initialize I2C communication
  Wire.setClock(400000); // use 400 kHz I2C
  lastIMUTime = millis();

  // Initialize MPU6050 (GY-521)
  Wire.beginTransmission(0x68); // MPU6050 I2C address
  Wire.write(0x6B);             // Power management register
  Wire.write(0);                // Wake up sensor
  Wire.endTransmission(true);

  //Initialize SD card module
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    while (1);  // Stop further execution
  }
  Serial.println("SD Card initialized.");
  delay(500);//This delay is important, otherways file will not be made in next step.

  // Open the log file on SD card
  //Tips: keep file name short and make sure "Wire.setClock()" is in line, there is a delay before this step, and you use a .txt file and not .csv.
  logFile = SD.open("LOG.TXT", FILE_WRITE);
  if (logFile) {
      logFile.println("Timestamp (ms), Distance sensor 1(mm), Distance sensor 2(mm), Pitch (deg), Roll (deg)");  // Add column headers
      logFile.close();
      Serial.println("Log file created successfully.");
  } else {
      Serial.println("Failed to create log file!");
      while (1);  // Stop further execution
  }

  //INITIALISING THE SENSORS
  // Turn off sensor 1 by pulling the XSHUT pin LOW
  pinMode(SENSOR_1_XSHUT, OUTPUT);
  digitalWrite(SENSOR_1_XSHUT, LOW);

  // Initialize sensor 2
  VL53L1_Error status = sensor_2.Begin(SENSOR_2_I2C_ADDRESS);
  if (status != VL53L1_ERROR_NONE) {
    // If the sensor could not be initialized print out the error code. -7 is timeout
    Serial.println("Could not initialize sensor 2, error code: " + String(status));
    while (1) {}
  }
  Serial.println("Sensor 2 initialized");

  // Set the I2C address of sensor 2 to a different address as the default. This is necessary to have two sensors, they cant be on same adres. 
  sensor_2.SetI2CAddress(SENSOR_2_I2C_ADDRESS);

  // Turn on sensor 1 by pulling the XSHUT pin HIGH
  digitalWrite(SENSOR_1_XSHUT, HIGH);

  // Initialize sensor 1
  status = sensor_1.Begin();
  if (status != VL53L1_ERROR_NONE) {
    // If the sensor could not be initialized print out the error code. -7 is timeout
    Serial.println("Could not initialize sensor 1, error code: " + String(status));
    while (1) {}
  }
  Serial.println("Sensor 1 initialized");

  // Set distance mode using correct values (1 = Short ~1.3 meter , 2 = Long ~ 4 meter)
  sensor_1.SetDistanceMode(1);  
  sensor_2.SetDistanceMode(1);
  Serial.println("Distance mode set.");

  // Set timing budget (available values: 15, 20, 33, 50, 100, 200, 500 ms)
  sensor_1.SetTimingBudgetInMs(33);
  sensor_2.SetTimingBudgetInMs(33);
  Serial.println("Timing budget set in milli seconds.");

  // Start continuous ranging
  sensor_1.StartRanging();
  sensor_2.StartRanging();
  Serial.println("Setup complete.");
}

void loop() {
  // Checking if data is available. This can also be done through the hardware interrupt
  uint8_t dataReady_sensor_1 = false, dataReady_sensor_2 = false;
  while(!dataReady_sensor_1 || !dataReady_sensor_2) {
    sensor_1.CheckForDataReady(&dataReady_sensor_1);
    sensor_2.CheckForDataReady(&dataReady_sensor_2);
    delay(5);
  }
  uint32_t timestamp = millis();  // Get current time

  uint16_t distance1, distance2;
  
  // Read distance, print to serial monitor
  sensor_1.GetDistanceInMm(&distance1); // Update sensor 1 distance
  sensor_2.GetDistanceInMm(&distance2); // Update sensor 1 distance
  readIMUAndComputeAngles(); // Update pitch and roll
  Serial.print("Time: ");
  Serial.print(timestamp);
  Serial.print(" ms, Distance sensor 1: ");
  Serial.print(distance1);
  Serial.print(" mm, Distance Sensor 2: ");
  Serial.print(distance2);
  Serial.println(" mm.");
  Serial.print(" Pitch: ");
  Serial.print(angle_pitch);
  Serial.print("°, Roll: ");
  Serial.println(angle_roll);

  // Write to SD card
  logFile = SD.open("LOG.TXT", FILE_WRITE);
  if (logFile) {
    logFile.print(timestamp);
   logFile.print(",");
    logFile.print(distance1);
    logFile.print(",");
    logFile.print(distance2);
    logFile.print(",");
    logFile.print(angle_pitch);
    logFile.print(",");
    logFile.println(angle_roll);
    logFile.close();
  } else {
      Serial.println("Error writing to SD card!");
  }

  // After reading the results reset the interrupt to be able to take another measurement
  sensor_1.ClearInterrupt();
  sensor_2.ClearInterrupt();

  delay(45);  // 50ms delay for 20Hz sampling (note the 5 ms delay earlier)
}

void readIMUAndComputeAngles() {
  // Get time delta
  unsigned long now = millis();
  float dt = (now - lastIMUTime) / 1000.0;
  lastIMUTime = now;

  // Read accelerometer and gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  if (Wire.available() < 14) {
    Serial.println("IMU read error!");
    return;
  }

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); 
  Wire.read(); // Skip temp
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  // Convert to proper units
  float ax = accX / 16384.0;
  float ay = accY / 16384.0;
  float az = accZ / 16384.0;
  float gX = gyroX / 131.0;
  float gY = gyroY / 131.0;

  // Calculate pitch and roll from accelerometer
  float pitch_acc = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float roll_acc  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  // Apply complementary filter
  angle_pitch = alpha * (angle_pitch + gX * dt) + (1 - alpha) * pitch_acc;
  angle_roll  = alpha * (angle_roll + gY * dt) + (1 - alpha) * roll_acc;
}
