/*
   MPU 6050 data collection and transmission for JAMZ drone delivery
   by Connor Harper, Jason Clapiz, Karsten Lowe, Leo Tan
   based on sofware originally written by Dejan https://howtomechatronics.com
*/
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;//accelerometer reading
float GyroX, GyroY, GyroZ;//gyroscope reading
float accAngleX, accAngleY, gyroAngleX, gyroAngleY;//Accelerometer and gyrocsope angle
float roll, pitch, yaw;//drone's position data variables
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;//calculated error during MPU6050 self calibration
float elapsedTime, currentTime, previousTime;//tracks time based on when the code was initialized
int c = 0;//count calibration readings

int moderateValue = 690, violentValue = 820; //threshold range value
void setup() {
  Serial.begin(19200);//baud rate for the serial monitor
  Wire.begin(); // Initialize comunication
  Wire.beginTransmission(MPU); // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B); // Talk to the register 6B
  Wire.write(0x00); // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); //end the transmission
  calculate_IMU_error();//runs the error calculation function before the main code executes
}
void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);//starts communiucation with the MPU 6050
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);//prevents the communication from ending
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 8192.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 8192.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 8192.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //calculates the acceleration in the X axis while taking into account the error found during self calibration
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;//calculates the acceleration in the Y axis while taking into account the error found during self calibration
  // === Read gyroscope data === //
  previousTime = currentTime; // Previous time is stored before the actual time read
  currentTime = millis(); // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);// Start communication with MPU6050 // MPU=0x68
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);//prevents the communication from ending
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values, the error will self correct based on the calibration when the system was initialized
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  // Modifies the stored gyroscope angles based on the previous recorded angles
  gyroAngleX = gyroAngleX + GyroX;
  gyroAngleY = gyroAngleY + GyroY;
  yaw = yaw + GyroZ;
  // Complementary filter - combine acceleromter and gyro angle values
  gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
  gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;
  //Renames effective variables for ease of use during angle filtering
  roll = gyroAngleX;
  pitch = gyroAngleY;

  //checks read values against set values that constitute violent or moderate shaking
  //if one of the conditions is met, the code skips over the rest in an effort to optimise run time. 
  if (roll > violentValue || roll < -violentValue || pitch > violentValue || pitch < -violentValue) {
    Serial.print("Violent_Shaking, ");
  } 
  else if (roll > moderateValue && roll < violentValue){ 
    Serial.print("Moderate_Shaking, ");
  }
  else if(roll < -moderateValue && roll > -violentValue){ 
      Serial.print("Moderate_Shaking, ");
  }
  else if(pitch > moderateValue && pitch < violentValue){
      Serial.print("Moderate_Shaking, ");
  }
  else if(pitch < -moderateValue && pitch > -violentValue) {
      Serial.print("Moderate_Shaking, ");
  }
  else {
    Serial.print("Stable, ");
  }
  printValues(roll, pitch, yaw);
}
void calculate_IMU_error() {
  // This funtion is called in the setup section to calculate the accelerometer and gyro data error
  while (c < 200) {// Read Accelerometer values 200 times
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 8192.0;//divide by 8192 to get the proper axis value based on data sheet
    AccY = (Wire.read() << 8 | Wire.read()) / 8192.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 8192.0;
    // Sum all readings using trig to calculate the accelerational error
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}

//print the calculated values into the serial monitor
void printValues(float roll, float pitch, float yaw) {
  Serial.print(roll);
  Serial.print(",");
  Serial.println(pitch);
  //Serial.print(",");
 // Serial.println(yaw);
}