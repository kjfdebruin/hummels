#include <Wire.h>
#include "SensorFusion.h"
SF fusion;

unsigned char compass_i2c = 0x1e;
unsigned char gyro_i2c = 0x21;
unsigned char accelerometer_i2c = 0x53;

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;
int status;

float ax_calib, ay_calib, az_calib;
float gx_calib, gy_calib, gz_calib;
float mx_calib, my_calib, mz_calib;

char str[512];

//#define EULER_DATA
//#define RAW_DATA
//#define PROCESSING
#define SERIAL_PLOTTER


void setup() {
  Serial.begin(9600);          // start serial communication at 9600bps
  Wire.begin();

  // Set up Compass
  Wire.beginTransmission(compass_i2c);
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();
  delay(100);

  // Set up Gyro
  Wire.beginTransmission(gyro_i2c);
  Wire.write(byte(0x03)); // Cycle time register
  Wire.write(byte(0x01)); // Convert time
  Wire.endTransmission();
  delay(100);

  // Set up Accelerometer
  Wire.beginTransmission(accelerometer_i2c);
  Wire.write(byte(0x2d)); // Power_Control register
  Wire.write(byte(0x08)); // Link and measure mode
  Wire.endTransmission();
  delay(100);

  // Accel offset calibration - Z-axis
  Wire.beginTransmission(accelerometer_i2c);
  Wire.write(0x20); // Z-axis offset register
  Wire.write(13);
  Wire.endTransmission();
  delay(100);

  calibrate();
}

float convertGyroADCStepsToDegreesPerSecond (int steps) {
  // Range: 0.35 to 2.35 Volts. i.e. zero at 1.35V (2213 steps)
  // Sensitivity: 1 step = 1.22 deg/s

  return (steps - 2213) * 1.22;
}

float convertAccelerometerADCStepsToMetersPerSecondSquared (int steps) {
  // Range: +-2g
  // Sensitivity: 1 step = 3.9 mg => 0.0039 g per step
  // Then times g by 9.80665 to get m/s/s
  // Above combined, times by 0.0383072265625

  return steps * 0.0383072;
}

float convertCompassADCStepsToMilliGaussPerSecond (int steps) {
  return (steps - 2047) * 2.6875;
}

float convertTemperatureADCStepsToDegreesCelsius (int steps) {
  //0.1521 degC per step

  return steps * 0.1521;
}

float convertMilliGaussToMicroTesla (float mG) {
  // 1 mG = 0.1 uT
  return mG / 0.1;
}

float convertDegreesToRadians (float deg) {
  // pi/180 = 0.0174533

  return deg * 0.0174533;
}

void readCompass(int &x, int &y, int &z)
{
  Wire.beginTransmission(compass_i2c);
  Wire.write(byte(0x03));
  Wire.endTransmission();

  Wire.requestFrom(compass_i2c, 6);

  if (6 <= Wire.available()) {
    int d1 = Wire.read(); //X MSB
    int d2 = Wire.read(); //X LSB
    int d3 = Wire.read(); //Y MSB
    int d4 = Wire.read(); //Y LSB
    int d5 = Wire.read(); //Z MSB
    int d6 = Wire.read(); //Z LSB

    x = ((d1 & 0x80) != 0) ? (((~0) >> 16) << 16) | ((d1 << 8) + d2) : (d1 << 8) + d2;
    y = ((d3 & 0x80) != 0) ? (((~0) >> 16) << 16) | ((d3 << 8) + d4) : (d3 << 8) + d4;
    z = ((d5 & 0x80) != 0) ? (((~0) >> 16) << 16) | ((d5 << 8) + d6) : (d5 << 8) + d6;
  }
}

void readAccel(int &x, int &y, int &z)
{
  Wire.beginTransmission(accelerometer_i2c);
  Wire.write(byte(0x32));
  Wire.endTransmission();

  Wire.requestFrom(accelerometer_i2c, 6);

  if (6 <= Wire.available()) {
    int d1 = Wire.read(); //X MSB
    int d2 = Wire.read(); //X LSB
    int d3 = Wire.read(); //Y MSB
    int d4 = Wire.read(); //Y LSB
    int d5 = Wire.read(); //Z MSB
    int d6 = Wire.read(); //Z LSB

    x = ( d1 | d2 << 8); // X-axis value
    y = ( d3 | d4 << 8); // Y-axis value
    z = ( d5 | d6 << 8); // Z-axis value
  }
}

int readGyroChannel (int index)
{
  int high, low, d1, d2;

  high = (0xf0 & (0x01 << index)) >> 4;//CH5 ~ CH8
  low = (0x0f & (0x01 << index)) << 4;//CH1 ~ CH4

  Wire.beginTransmission(gyro_i2c);
  Wire.write(byte(0x02));
  Wire.write(byte(high));
  Wire.write(byte(low + 0x0c));
  Wire.endTransmission();
  delay(20);

  Wire.beginTransmission(gyro_i2c);
  Wire.write(byte(0x00));
  Wire.endTransmission();

  Wire.requestFrom(gyro_i2c, 2);
  if (2 <= Wire.available()) {
    d1 = Wire.read();
    d2 = Wire.read();
    return (d1 & 0x0f) * 256 + d2;
  }

  return 0;
}

void readGyro(int &x, int &x45, int &y, int &y45, int &z, int &z45, int &idgTemp, int &iszTemp)
{
  x = readGyroChannel(0);
  x45 = readGyroChannel(1);
  y = readGyroChannel(2);
  y45 = readGyroChannel(3);
  z = readGyroChannel(4);
  z45 = readGyroChannel(5);
  idgTemp = readGyroChannel(6);
  iszTemp = readGyroChannel(7);
}

void calibrate() {
  calibrateAccelerometer();
  calibrateGyro();
  calibrateCompass();
}

void calibrateAccelerometer() {
  delay(500);

  int accelX, accelY, accelZ;
  float ax1, ay1, az1, ax2, ay2, az2;

  // Take first reading
  readAccel(accelX, accelY, accelZ);
  ax1 = convertAccelerometerADCStepsToMetersPerSecondSquared(accelX);
  ay1 = convertAccelerometerADCStepsToMetersPerSecondSquared(accelY);
  az1 = convertAccelerometerADCStepsToMetersPerSecondSquared(accelZ);

  // Pause and take second reading
  delay(500);
  readAccel(accelX, accelY, accelZ);
  ax2 = convertAccelerometerADCStepsToMetersPerSecondSquared(accelX);
  ay2 = convertAccelerometerADCStepsToMetersPerSecondSquared(accelY);
  az2 = convertAccelerometerADCStepsToMetersPerSecondSquared(accelZ);

  // Average the difference between measurements and expected acceleration
  ax_calib = (-1 * ax1 + -1 * ax2) / 2;
  ay_calib = (-1 * ay1 + -1 * ay2) / 2;
  az_calib = ((-9.80665 - az1) + (-9.80665 - az2)) / 2; // z acceleration is 9.80665 m/s^2
}

void calibrateGyro() {
  delay(500);

  int gyroX, gyroX45, gyroY, gyroY45, gyroZ, gyroZ45, gyroIgTemp, gyroIszTemp;
  float gx1, gy1, gz1, gx2, gy2, gz2;

  // Take first reading
  readGyro(gyroX, gyroX45, gyroY, gyroY45, gyroZ, gyroZ45, gyroIgTemp, gyroIszTemp);
  gx1 = convertDegreesToRadians(convertGyroADCStepsToDegreesPerSecond(gyroX));
  gy1 = convertDegreesToRadians(convertGyroADCStepsToDegreesPerSecond(gyroY));
  gz1 = convertDegreesToRadians(convertGyroADCStepsToDegreesPerSecond(gyroZ));

  // Pause and take second reading
  delay(500);
  readGyro(gyroX, gyroX45, gyroY, gyroY45, gyroZ, gyroZ45, gyroIgTemp, gyroIszTemp);
  gx2 = convertDegreesToRadians(convertGyroADCStepsToDegreesPerSecond(gyroX));
  gy2 = convertDegreesToRadians(convertGyroADCStepsToDegreesPerSecond(gyroY));
  gz2 = convertDegreesToRadians(convertGyroADCStepsToDegreesPerSecond(gyroZ));

  // Average the difference between measurements and expected acceleration
  gx_calib = (-1 * gx1 + -1 * gx2) / 2;
  gy_calib = (-1 * gy1 + -1 * gy2) / 2;
  gz_calib = (-1 * gz1 + -1 * gz2) / 2;
}

void calibrateCompass() {
  mx_calib = 0;
  my_calib = 0;
  mz_calib = 0;
}

void loop() {
  int compassX, compassY, compassZ;
  readCompass(compassX, compassY, compassZ);

  int gyroX, gyroX45, gyroY, gyroY45, gyroZ, gyroZ45, gyroIgTemp, gyroIszTemp;
  readGyro(gyroX, gyroX45, gyroY, gyroY45, gyroZ, gyroZ45, gyroIgTemp, gyroIszTemp);

  int accelX, accelY, accelZ;
  readAccel(accelX, accelY, accelZ);

  ax = convertAccelerometerADCStepsToMetersPerSecondSquared(accelX) + ax_calib;
  ay = convertAccelerometerADCStepsToMetersPerSecondSquared(accelY) + ay_calib;
  az = convertAccelerometerADCStepsToMetersPerSecondSquared(accelZ) + az_calib;

  gx = convertDegreesToRadians(convertGyroADCStepsToDegreesPerSecond(gyroX)) + gx_calib;
  gy = convertDegreesToRadians(convertGyroADCStepsToDegreesPerSecond(gyroY) + gy_calib);
  gz = convertDegreesToRadians(convertGyroADCStepsToDegreesPerSecond(gyroZ)) + gz_calib;

  mx = convertMilliGaussToMicroTesla(convertCompassADCStepsToMilliGaussPerSecond(compassX)) + mx_calib;
  my = convertMilliGaussToMicroTesla(convertCompassADCStepsToMilliGaussPerSecond(compassY)) + my_calib;
  mz = convertMilliGaussToMicroTesla(convertCompassADCStepsToMilliGaussPerSecond(compassZ)) + mz_calib;

  temp = convertTemperatureADCStepsToDegreesCelsius(gyroIgTemp);

#ifdef RAW_DATA
  //  Serial.println("From last Update:" + String(deltat));
  //  Serial.println("GYRO:x" + String(gx) + "y:" + String(gy)+ "z:" + String(gz));
  //  Serial.println("ACC:x" + String(ax) + "y:" + String(ay)+ "z:" + String(az));
  //  Serial.println("MAG:x" + String(mx) + "y:" + String(my)+ "z:" + String(mz));
  //  Serial.println("TEMP:" + String(temp));
  //  Serial.println(String(gx) + ", " + String(gy)+ ", " + String(gz) + ", " + String(ax) + ", " + String(ay) + ", " + String(az) + ", " + String(mx) + ", " + String(my) + ", " + String(mz));
//  Serial.println(String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," + String(mx) + "," + String(my) + "," + String(mz));
  Serial.println(String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz));
  delay(10);

#endif

  deltat = fusion.deltatUpdate();
  //  fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick

  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();

#ifdef EULER_DATA
  Serial.println("Pitch:" + String(pitch) + "Roll:" + String(roll) + "Yaw:" + String(yaw));
#endif

#ifdef PROCESSING
  roll = fusion.getRollRadians();
  pitch = fusion.getPitchRadians();
  yaw = fusion.getYawRadians();
  Serial.println(String(pitch) + "," + String(roll) + "," + String(yaw));
#endif

#ifdef SERIAL_PLOTTER
  Serial.println(String(pitch) + ", " + String(roll) + ", " + String(yaw));
#endif
}
