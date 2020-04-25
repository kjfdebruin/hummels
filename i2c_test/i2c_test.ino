#include <Wire.h>

unsigned char compass_i2c = 0x1e;
unsigned char gyro_i2c = 0x21;
unsigned char accelerometer_i2c = 0x53;

void setup() {
  Serial.begin(9600);          // start serial communication at 9600bps
  Wire.begin();

  // Set up Compass
  Wire.beginTransmission(compass_i2c);
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();
  delay(100);
  
  //Set up Gyro
  Wire.beginTransmission(gyro_i2c);
  Wire.write(byte(0x03));
  Wire.write(byte(0x01));
  Wire.endTransmission();
  delay(100);

  //Set up Accelerometer
  Wire.beginTransmission(accelerometer_i2c);
  Wire.write(byte(0x2d)); // Power_Control register
  Wire.write(byte(0x28)); // Link and measure mode
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(accelerometer_i2c);
  Wire.write(byte(0x31)); // Data_Format register
  Wire.write(byte(0x08)); // Full_Resolution
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(accelerometer_i2c);
  Wire.write(byte(0x38)); // FIFO_Control_Format register
  Wire.write(byte(0x00)); // Bypass mode
  Wire.endTransmission();
  delay(100);
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
    
    x = ((d1 & 0x80) != 0) ? (((~0)>>16)<<16) | ((d1<<8)+d2): (d1<<8)+d2;
    y = ((d3 & 0x80) != 0) ? (((~0)>>16)<<16) | ((d3<<8)+d4): (d3<<8)+d4;
    z = ((d5 & 0x80) != 0) ? (((~0)>>16)<<16) | ((d5<<8)+d6): (d5<<8)+d6;
  }
}

void readAccelerometer(int &x, int &y, int &z)
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
    
    x = ((d1 & 0x80) != 0) ? (((~0)>>16)<<16) | ((d1<<8)+d2): (d1<<8)+d2;
    y = ((d3 & 0x80) != 0) ? (((~0)>>16)<<16) | ((d3<<8)+d4): (d3<<8)+d4;
    z = ((d5 & 0x80) != 0) ? (((~0)>>16)<<16) | ((d5<<8)+d6): (d5<<8)+d6;
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
    return (d1 & 0x0f)*256+d2;
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

void loop() {
  int compassX, compassY, compassZ;
  readCompass(compassX, compassY, compassZ);
//  Serial.println(String(compassX) + ", " + String(compassY) + ", " + String(compassZ));

  int gyroX, gyroX45, gyroY, gyroY45, gyroZ, gyroZ45, gyroIgTemp, gyroIszTemp;
  readGyro(gyroX, gyroX45, gyroY, gyroY45, gyroZ, gyroZ45, gyroIgTemp, gyroIszTemp);
//  Serial.println(String(gyroX) + ", " + String(gyroX45) + ", " + String(gyroY)+ ", " + String(gyroY45)+ ", " + String(gyroZ)+ ", " + String(gyroZ45)+ ", " + String(gyroIgTemp)+ ", " + String(gyroIszTemp));

  int accelX, accelY, accelZ;
  readAccelerometer(accelX, accelY, accelZ);
  Serial.println(String(accelX) + ", " + String(accelY) + ", " + String(accelZ));
  delay(500);
}
