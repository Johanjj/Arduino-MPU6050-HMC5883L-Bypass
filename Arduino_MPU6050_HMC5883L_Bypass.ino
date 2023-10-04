/******************************************************************
// My learning Sources
MPU6050 Datasheet https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
https://youtube.com/@carbonaeronautics?si=PKxU4YJIEQuEx2X3
https://forum.arduino.cc/t/extracting-a-magnetic-heading-from-gy-86/185955/4
https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
https://electropeak.com/learn/interfacing-gy-87-10dof-imu-mpu6050-hmc5883l-bmp085-module-with-arduino/
https://github.com/sleemanj/HMC5883L_Simple
******************************************************************/

#include <Wire.h>
#include <HMC5883L_Simple.h>

HMC5883L_Simple Compass;

const int mpuAddr = 0x68;  // MPU6050 register address

// Define the gyroscope and accelerometer variables
float rateRoll, ratePitch, rateYaw;
float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;
int rateCalibrationNumber;
float accX, accY, accZ;
float angleRoll, anglePitch;

// Define the MPU6050 temperature variable
float mpuTemp;

// Define the HMC5883L magnetometer sensor variable
float heading;

// Timer for loop
uint32_t loopTimer;
long loopTimerInterval = 10000;

// Set accelerometer measurement offset
float accXOffset = 0;
float accYOffset = 0;
float accZOffset = 0;

// Set gyroscope measurement offset
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

void accelgyro_signals() {
  // Start I2C communication with the accelerometer
  Wire.beginTransmission(mpuAddr);
  // Switch on low-pass filter
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure accelerometer output
  Wire.beginTransmission(mpuAddr);
  // Set the sensitivity scale factor range (+-8g)
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Access registers accelerometer measurements from the sensor
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(mpuAddr, 6);  // Request register for accelerometer measurements

  // Read accelerometer measurements to physical values
  int16_t accXLSB = Wire.read() << 8 | Wire.read();
  int16_t accYLSB = Wire.read() << 8 | Wire.read();
  int16_t accZLSB = Wire.read() << 8 | Wire.read();

  // Convert the measurement units to g (+-8g -> 4096 LSB/g)
  accX = (float)accXLSB / 4096 + accXOffset;
  accY = (float)accYLSB / 4096 + accYOffset;
  accZ = (float)accZLSB / 4096 + accZOffset;

  // Start I2C communication with the gyroscope
  Wire.beginTransmission(mpuAddr);
  // Set the sensitivity scale factor range (+-500 °/s)
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  // Access registers accelerometer measurements from the sensor
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(mpuAddr, 6);  // Request register for gyroscope measurements

  // Read gyroscope measurements to physical values
  int16_t gyroXLSB = Wire.read() << 8 | Wire.read();
  int16_t gyroYLSB = Wire.read() << 8 | Wire.read();
  int16_t gyroZLSB = Wire.read() << 8 | Wire.read();

  // Convert the measurement units to °/s (+-500 °/s -> 65.5 LSB/°/s)
  rateRoll = (float)gyroXLSB / 65.5 + gyroXOffset;
  ratePitch = (float)gyroYLSB / 65.5 + gyroYOffset;
  rateYaw = (float)gyroZLSB / 65.5 + gyroZOffset;

  // Calculate the absolute angles
  angleRoll = atan(accY / sqrt(accX * accX + accZ * accZ)) * 1 / (3.142 / 180);
  anglePitch = -atan(accX / sqrt(accY * accY + accZ * accZ)) * 1 / (3.142 / 180);

  // Configure and access registers MPU6050 temperature measurements from the sensor
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x41);
  Wire.endTransmission();

  Wire.requestFrom(mpuAddr, 2);  // Request register for MPU6050 temperature measurements

  // Read MPU6050 temperature measurements to physical values
  int16_t mpuTempLSB = Wire.read() << 8 | Wire.read();

  // Convert the measurement units to temperature in degrees °C (formula from the datasheet)
  mpuTemp = (float)mpuTempLSB / 340.0 + 36.53;

  // Start I2C communication with the MPU6050
  Wire.beginTransmission(mpuAddr);
  // Set the bypass mode to use auxiliary SDA SCL (XDA XCL) for gateway to HMC5883L sensor (magnetometer)
  Wire.write(0x37);
  Wire.write(0x2);
  Wire.endTransmission();
}
void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Set I2C clock speed to 400kHz for MPU6050
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // Start MPU6050 sensor in power mode
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Perform the calibration measurements
  for (rateCalibrationNumber = 0; rateCalibrationNumber < 2000; rateCalibrationNumber++) {
    accelgyro_signals();
    rateCalibrationRoll += rateRoll;
    rateCalibrationPitch += ratePitch;
    rateCalibrationYaw += rateYaw;
    delay(1);
  }

  // Calculate the calibration values
  rateCalibrationRoll /= 2000;
  rateCalibrationPitch /= 2000;
  rateCalibrationYaw /= 2000;

  // Initialize hmc5883l
  Compass.SetDeclination(23, 35, 'E');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);

  loopTimer = micros();  // Initialize loop timer in micros
}
void loop() {
  accelgyro_signals();

  // Calibrated rotation rate/ angular velocity values
  rateRoll -= rateCalibrationRoll;
  ratePitch -= rateCalibrationPitch;
  rateYaw -= rateCalibrationYaw;

  // Get Acceleration offset value from here
  // there must be an offset if g != 0 when accX static or g != +-1.0 when accX fully roll
  // there must be an offset if g != 0 when accY static or g != +-1.0 when accY fully pitch
  // there must be an offset if g != 0 when accZ fully roll/pitch or g != +-1.0 when accZ static
  Serial.print("Acceleration X [g]: ");
  Serial.print(accX);
  Serial.print("\t");
  Serial.print("Acceleration Y [g]: ");
  Serial.print(accY);
  Serial.print("\t");
  Serial.print("Acceleration Z [g]: ");
  Serial.println(accZ);

  // Get Gyroscope offset value from here
  // there must be an offset if °/s != 0 when rateRoll static
  // there must be an offset if °/s != 0 when ratePitch static
  // there must be an offset if °/s != 0 when rateYaw static
  Serial.print("Rate Roll [°/s]= ");
  Serial.print(rateRoll);
  Serial.print("\t");
  Serial.print("Rate Pitch [°/s]= ");
  Serial.print(ratePitch);
  Serial.print("\t");
  Serial.print("Rate Yaw [°/s]= ");
  Serial.println(rateYaw);

  Serial.print("Roll Angle [°]: ");
  Serial.print(angleRoll);  // Roll angle value
  Serial.print("\t");
  Serial.print("Pitch Angle [°]: ");
  Serial.println(anglePitch);  // Pitch angle value

  Serial.print("MPU6050 Temperature [°C]: ");
  Serial.println(mpuTemp);

  heading = Compass.GetHeadingDegrees();
  Serial.print("Heading: ");
  Serial.println(heading);

  Serial.println();

  while (micros() - loopTimer < loopTimerInterval)
    ;
  loopTimer = micros();

  // delay(1000);
}