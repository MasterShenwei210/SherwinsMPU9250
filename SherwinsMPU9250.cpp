#include "SherwinsMPU9250.h"


/**
Input the declination angle if magnetometer is used.
Depends on location. You can find your declination
angle at: http://www.ngdc.noaa.gov/geomag-web/

PARAMETERS:
    1) desired accelerometer scale. Enter 2, 4, 8, or 16.
    stand for +/- 2g, 4g, 8g, 16g
    (lower ranges are more accurate)
    2) desired gyroscope scale. Enter 250, 500, 1000, or 2000.
    stand for +/- 250deg/sec, 500deg/sec, 1000deg/sec, 2000deg/sec
    (lower ranges are more accurate)
**/
SherwinsMPU9250::SherwinsMPU9250(int accelScale, int gyroScale, float decl){
    if(decl != 0){
        declination = decl * PI/180;
    }

    switch(accelScale){
        case 2:
            accelConfigByte = 0b00000000;
            accelConvert = 16384;
            break;

        case 4:
            accelConfigByte = 0b00001000;
            accelConvert = 8192;
            break;

        case 8:
            accelConfigByte = 0b00010000;
            accelConvert = 4096;
            break;

        case 16:
            accelConfigByte = 0b00011000;
            accelConvert = 2048;
            break;
    }

    switch(gyroScale){
        case 250:
            gyroConfigByte = 0b00000000;
            gyroConvert = 131;
            break;

        case 500:
            accelConfigByte = 0b00001000;
            accelConvert = 8192;
            break;

        case 1000:
            accelConfigByte = 0b00010000;
            accelConvert = 4096;
            break;

        case 2000:
            accelConfigByte = 0b00011000;
            accelConvert = 2048;
            break;
    }
}

void SherwinsMPU9250::writeByte(uint16_t address, uint8_t data){
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

void SherwinsMPU9250::writeMagByte(uint16_t address, uint8_t data){
    Wire.beginTransmission(MAG_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}
void SherwinsMPU9250::readByte(uint16_t address, uint8_t *dataOut){
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.requestFrom(MPU9250_ADDRESS, 1);

    if(Wire.available() > 0){
        while(Wire.available() > 0){
            buffer[0] = Wire.read();
        }
    }
    Wire.endTransmission();

    *dataOut = buffer[0];
}

void SherwinsMPU9250::readMagByte(uint16_t address, uint8_t *dataOut){
    Wire.beginTransmission(MAG_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.beginTransmission(MAG_ADDRESS);
    Wire.requestFrom(MAG_ADDRESS, 1);

    if(Wire.available() > 0){
        while(Wire.available() > 0){
            buffer[0] = Wire.read();
        }
    }
    Wire.endTransmission();
    *dataOut = buffer[0];
}

void SherwinsMPU9250::initialize_MPU9250(){
    readByte(ACCEL_CONFIG_REG, &buffer[0]);
    buffer[0] |= accelConfigByte;
    writeByte(ACCEL_CONFIG_REG, buffer[0]);

    readByte(GYRO_CONFIG_REG, &buffer[0]);
    buffer[0] |= gyroConfigByte;
    writeByte(GYRO_CONFIG_REG, buffer[0]);

    writeByte(POWER_MGN1_REG, 0b00000001);
}

void SherwinsMPU9250::initializeDt(){
   mil = millis();
   lastMil = (int) mil;
}

/**
Sets up the MPU9250 registers, takes around 5 sec
to calibrate minimum and maximum magnetometer
values for soft and hard iron correction, initializes
roll and pitch with accelerometer and yaw with magnetometer,
and sets up for dt to find gyro angles.

IMPORTANT: need to rotate and move around the sensor
while it's calibrating the min and max magnetometer
values.
**/
void SherwinsMPU9250::initializeEverything(){
    initialize_MPU9250();
    magnitometerCalibration();
    initializeAngles();
    initializeDt();
}

/**
Sets up MPU9250 registers, initializes roll and pitch with
accelerometer, and sets up for dt to find gyro angles.

NOTE:
    Use only if magnetometer is not going to be used
**/
void SherwinsMPU9250::initializeWithoutMagnetometer(){
    initialize_MPU9250();
    initializeAngles();
    initializeDt();
}

void SherwinsMPU9250::readRaw9axis(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz){
    readByte(ACCEL_XOUT_H, &buffer[0]);
    readByte(ACCEL_XOUT_L, &buffer[1]);
    readByte(ACCEL_YOUT_H, &buffer[2]);
    readByte(ACCEL_YOUT_L, &buffer[3]);
    readByte(ACCEL_ZOUT_H, &buffer[4]);
    readByte(ACCEL_ZOUT_L, &buffer[5]);

    readByte(GYRO_XOUT_H, &buffer[6]);
    readByte(GYRO_XOUT_L, &buffer[7]);
    readByte(GYRO_YOUT_H, &buffer[8]);
    readByte(GYRO_YOUT_L, &buffer[9]);
    readByte(GYRO_ZOUT_H, &buffer[10]);
    readByte(GYRO_ZOUT_L, &buffer[11]);

    writeByte(INT_PIN_CFG, 0b00000010);
    delay(10);
    writeMagByte(MAG_ENABLE, 0b00000001);
    delay(10);

    readMagByte(MAG_XOUT_H, &buffer[12]);
    readMagByte(MAG_XOUT_L, &buffer[13]);
    readMagByte(MAG_YOUT_H, &buffer[14]);
    readMagByte(MAG_YOUT_L, &buffer[15]);
    readMagByte(MAG_ZOUT_H, &buffer[16]);
    readMagByte(MAG_ZOUT_L, &buffer[17]);

    *ax = (buffer[0] << 8) | buffer[1];
    *ay = (buffer[2] << 8) | buffer[3];
    *az = (buffer[4] << 8) | buffer[5];
    *gx = (buffer[6] << 8) | buffer[7];
    *gy = (buffer[8] << 8) | buffer[9];
    *gz = (buffer[10] << 8) | buffer[11];
    *mx = (buffer[12] << 8) | buffer[13];
    *my = (buffer[14] << 8) | buffer[15];
    *mz = (buffer[16] << 8) | buffer[17];

}

/**
Gets raw sensor values and converts LSB to the
right units.

Accelerometer data: g's
Gyroscope data: degrees/sec
Magnetometer data: microTeslas

PARAMETERS:
    Addresses of six floats in respective order
    to store data in.
**/
void SherwinsMPU9250::getBurstRead(float *Ax, float *Ay, float *Az, float *Gx, float *Gy, float *Gz, float *Mx, float *My, float *Mz){
  readRaw9axis(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  axn = (float)ax;
  ayn = (float)ay;
  azn = (float)az;

  gxn = (float)gx;
  gyn = (float)gy;
  gzn = (float)gz;

  mxn = (float)mx;
  myn = (float)my;
  mzn = (float)mz;


  *Ax = axn / accelConvert;
  *Ay = ayn / accelConvert;
  *Az = azn / accelConvert;

  *Gx = gxn / gyroConvert;
  *Gy = gyn / gyroConvert;
  *Gz = gzn / gyroConvert;

  *Mx = mxn * magConvert;
  *My = myn * magConvert;
  *Mz = mzn * magConvert;
}

/**
Initializes angles using accelerometer for gyroscope
calculation.
**/
void SherwinsMPU9250::initializeAngles(){
    getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);

    zAngle = atan2(accel_x,accel_y)*180/PI;
    yAngle = atan2(accel_z,accel_x)*180/PI;
    xAngle = atan2(accel_y,accel_z)*180/PI;

 }

/**
Returns the number of milliseconds since
the last dt was recorded.
**/
float SherwinsMPU9250::getDt(){
  mil = millis();
  nowMil = (int) mil;
  milDifference = nowMil - lastMil;
  lastMil = nowMil;
  dt = ((float) milDifference) * 1/1000;
  return dt;
}

/**
Handles all magnetometer calibrations.
**/
void SherwinsMPU9250::magnitometerCalibration(){
    calibrateMinMax();

    xmagCorrection = (xmaxMag + xminMag)/2;
    ymagCorrection = (ymaxMag + yminMag)/2;
    zmagCorrection = (zmaxMag + zminMag)/2;

    findScales();
    Yyaw = Calculate_heading(mag_x,mag_y);
}

/**
Calibrates minimum and maximum magnetometer
values to fix soft and hard iron errors.

IMPORTANT: need to rotate and move around the sensor
while it's calibrating the min and max magnetometer
values.
**/
void SherwinsMPU9250::calibrateMinMax(){
    Serial.println("calibrating...");
    for(i = 0; i<300;i++){
    getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);
    xmaxMag = max(xmaxMag,mag_x);
    ymaxMag = max(ymaxMag,mag_y);
    zmaxMag = max(zmaxMag,mag_z);

    xminMag = min(xminMag,mag_x);
    yminMag = min(yminMag,mag_y);
    zminMag = min(zminMag,mag_z);
    delay(1);
  }
}

/**
Calculates the x,y,z scale factors to fix
soft iron error for magnetometer.
**/
void SherwinsMPU9250::findScales(){
    fixedMaxx = xmaxMag - xmagCorrection;
    fixedMaxy = ymaxMag - ymagCorrection;
    fixedMaxz = zmaxMag - zmagCorrection;

    fixedMinx = xminMag - xmagCorrection;
    fixedMiny = yminMag - ymagCorrection;
    fixedMinz = zminMag - zmagCorrection;

    avgsX = (fixedMaxx + abs(fixedMinx))/2;
    avgsY = (fixedMaxy + abs(fixedMiny))/2;
    avgsZ = (fixedMaxz + abs(fixedMinz))/2;

    avgRad = (avgsX + avgsY + avgsZ )/3;

    Xscale = avgRad/avgsX;
    Yscale = avgRad/avgsY;
    Zscale = avgRad/avgsZ;
}

float SherwinsMPU9250::hardIron_xMag(float xMag){
  xMag -= xmagCorrection ;
  return xMag;
}

float SherwinsMPU9250::hardIron_yMag(float yMag){
  yMag -= ymagCorrection ;
  return yMag;
}

float SherwinsMPU9250::hardIron_zMag(float xMag){
 zMag -= zmagCorrection;
  return zMag;
}

float SherwinsMPU9250::softIron_xMag(float xMag){
  xMag *= Xscale;
  return xMag;
}

float SherwinsMPU9250::softIron_yMag(float yMag){
  yMag *= Yscale;
  return yMag;
}

float SherwinsMPU9250::softIron_zMag(float zMag){
  zMag *= Zscale;
  return zMag;
}

float SherwinsMPU9250::xMagCorrection(float xMag){
  xMag = softIron_xMag(hardIron_xMag(xMag));
  return xMag;
}

float SherwinsMPU9250::yMagCorrection(float yMag){
  yMag = softIron_yMag(hardIron_yMag(yMag));
  return yMag;
}

float SherwinsMPU9250::zMagCorrection(float zMag){
  zMag = softIron_zMag(hardIron_zMag(zMag));
  return zMag;
}

/**
The following three functions calculate
pitch and roll using only the accelerometer.

NOTE: Cannot calculate yaw since gravity measurement won't
change as the sensor rotates.
**/

/**
PARAMETERS:
    1) X axis acceleration
    2) y axis acceleration
**/
float SherwinsMPU9250::Calculate_z_aAngle(float xAcc, float yAcc){
  Aangle = atan2(xAcc,yAcc)*180/PI;
  return Aangle;
}

/**
PARAMETERS:
    1) X axis acceleration
    2) Z axis acceleration
**/
float SherwinsMPU9250::Calculate_y_aAngle(float xAcc, float zAcc){
  Aangle = atan2(xAcc,zAcc)*180/PI;
  return Aangle;
}

/**
PARAMETERS:
    1) Y axis acceleration
    2) Z axis acceleration
**/
float SherwinsMPU9250::Calculate_x_aAngle(float yAcc, float zAcc){
  Aangle = atan2(yAcc,zAcc)*180/PI;
  return Aangle;
}

/**
The following three functions return
pitch and roll using only the accelerometer.

NOTE: Cannot find yaw since gravity measurement won't
change as the sensor rotates.
**/
float SherwinsMPU9250::find_z_aAngle(){
  getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);
  return Calculate_z_aAngle(accel_x, accel_y);
}

float SherwinsMPU9250::find_y_aAngle(){
  getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);
  return Calculate_y_aAngle(accel_x, accel_z);
}

float SherwinsMPU9250::find_x_aAngle(){
  getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);
  return Calculate_x_aAngle(accel_y, accel_z);
}

/**
The following three functions calculate
the angle using only the gyroscope.

NOTE: The angles will drift due to
noise.
**/

/**
PARAMETERS:
    1) angular velocity around Z axis.
**/
float SherwinsMPU9250::Calculate_z_gAngle(float zGyr){
    addedGangle = zGyr * getDt();
    zAngle += addedGangle;

 return zAngle;
}

/**
PARAMETERS:
    1) angular velocity around Y axis.
**/
float SherwinsMPU9250::Calculate_y_gAngle(float yGyr){
    addedGangle = yGyr * getDt();
    yAngle += addedGangle;

 return yAngle;
}

/**
PARAMETERS:
    1) angular velocity around X axis.
**/
float SherwinsMPU9250::Calculate_x_gAngle(float xGyr){
    addedGangle = xGyr * getDt();
    xAngle += addedGangle;

    return xAngle;
}

/**
The following three functions return
the angle using only the gyroscope.

NOTE: The angles will drift due to
noise.
**/
float SherwinsMPU9250::find_z_gAngle(){
    getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);

    return Calculate_z_gAngle(gyro_z);
}

float SherwinsMPU9250::find_y_gAngle(){
    getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);

    return Calculate_y_gAngle(gyro_y);
}

float SherwinsMPU9250::find_x_gAngle(){
    getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);

    return Calculate_x_gAngle(gyro_x);
}
/**
The following three functions calculate
pitch and roll using the gyroscope and
accelerometer.

Utilizes a low pass filter for the best pitch
and roll calculation.

NOTE: Cannot find yaw since gravity measurement
won't change as the sensor rotates.
**/

/**
PARAMETERS:
    1) acceleration on X axis
    2) acceleration on Y axis
    3) angular velocity around Z axis
**/
float SherwinsMPU9250::Calculate_filtered_zAngle(float xAcc, float yAcc, float zGyr){
  Aangle = atan2(yAcc,xAcc)*180/PI;

  addedGangle = zGyr * getDt();
  zAngle = .02 * (addedGangle + zAngle) + .98 * Aangle;

  return zAngle;
}

/**
PARAMETERS:
    1) acceleration on X axis
    2) angular velocity around Y axis
    3) acceleration on Z axis
**/
float SherwinsMPU9250::Calculate_filtered_yAngle(float xAcc, float yGyr, float zAcc){
  Aangle = atan2(zAcc,xAcc)*180/PI;

  addedGangle = zGyr * getDt();
  yAngle = .02 * (addedGangle + yAngle) + .98 * Aangle;

  return yAngle;
}

/**
PARAMETERS:
    1) angular velocity around X axis
    2) acceleration on Y axis
    3) acceleration on Z axis
**/
float SherwinsMPU9250::Calculate_filtered_xAngle(float xGyr, float yAcc, float zAcc){
  Aangle = atan2(yAcc,zAcc)*180/PI;

  addedGangle = xGyr * getDt();
  xAngle = .02 * (addedGangle + xAngle) + .98 * Aangle;

  return xAngle;
}

/**
The following three functions return
pitch and roll using the gyroscope and
accelerometer.

Utilizes a low pass filter for the best
roll calculation.

NOTE: Cannot find yaw since gravity measurement
won't change as the sensor rotates.
**/
float SherwinsMPU9250::findFiltered_zAngle(){
  getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);
  zAngle = Calculate_filtered_zAngle(accel_x, accel_y, gyro_z);

  return zAngle;
}

float SherwinsMPU9250::findFiltered_yAngle(){
  getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);
  yAngle = Calculate_filtered_yAngle(accel_x, gyro_y, accel_z);

  return yAngle;
}

float SherwinsMPU9250::findFiltered_xAngle(){
  getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);
  xAngle = Calculate_filtered_xAngle(gyro_x, accel_y, accel_z);

  return xAngle;
}

/**
Calculates the heading around Z axis using
magnetometer data.

PARAMETERS:
    1) magnetic field on X axis
    2) magnetic field on Y axis
**/
float SherwinsMPU9250::Calculate_heading(float xMag, float yMag){
   xMag = xMagCorrection(xMag);
   yMag = yMagCorrection(yMag);

   heading =atan2(yMag,xMag);
   heading += declination;

   if(heading <0) {
    heading += 2*PI;
  }
   if(heading > 2*PI){
    heading -= 2*PI;
   }

    headingDegrees = heading * 180/PI;
    return headingDegrees;
}

/**
Returns the heading around Z axis using
magnetometer data.

**/
float SherwinsMPU9250::find_heading(){
   getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);

    return Calculate_heading(mag_x, mag_y);
}

/**
Combines he magnetometer and gyroscope
data to calculate the most accurate yaw.

PARAMETERS:
    1) magnetic field on X axis
    2) magnetic field on Y axis
    3) angular velocity around Z axis

NOTE: Tolerance is +/- 6 degrees. The function
finds the average of 5 readings.
**/
float SherwinsMPU9250::Calculate_yaw(float xMag, float yMag, float zGyr){

   Mangle = Calculate_heading(xMag,yMag);
   addedGangle = zGyr * getDt();
   Yyaw  = .02 * (addedGangle + Yyaw) + .98 * Mangle;

  return Yyaw;
}

/**
*Combines the magnetometer and gyroscope
*data to return the most accurate yaw.
*
*NOTE: Tolerance is +/- 6 degrees.
**/
float SherwinsMPU9250::find_yaw(){
  getBurstRead(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z,&mag_x, &mag_y, &mag_z);
  return Calculate_yaw(mag_x, mag_y, gyro_z);
}



