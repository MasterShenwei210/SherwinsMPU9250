#ifndef SHERWINSMPU9250_H
#define SHERWINSMPU9250_H

#include "Arduino.h"
#include "Wire.h"

#define MPU9250_ADDRESS     0x68
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_CONFIG_REG    0x1C
#define POWER_MGN1_REG      0X6B
#define ACCEL_XOUT_H        0X3B
#define ACCEL_XOUT_L        0X3C
#define ACCEL_YOUT_H        0X3D
#define ACCEL_YOUT_L        0X3E
#define ACCEL_ZOUT_H        0X3F
#define ACCEL_ZOUT_L        0X40
#define GYRO_XOUT_H         0X43
#define GYRO_XOUT_L         0X44
#define GYRO_YOUT_H         0X45
#define GYRO_YOUT_L         0X46
#define GYRO_ZOUT_H         0X47
#define GYRO_ZOUT_L         0X48
#define INT_PIN_CFG         0x37
#define MAG_ADDRESS	        0x0C
#define MAG_ENABLE          0X0A
#define MAG_XOUT_L		    0x03
#define MAG_XOUT_H		    0x04
#define MAG_YOUT_L		    0x05
#define MAG_YOUT_H		    0x06
#define MAG_ZOUT_L		    0x07
#define MAG_ZOUT_H		    0x08

class SherwinsMPU9250{
    public:
        SherwinsMPU9250(int accelScale, int gyroScale, float decl = 0);
        void initializeEverything();
        void initializeWithoutMagnetometer();
        float getDt();
        void getBurstRead(float* Ax, float* Ay, float* Az, float* Gx, float* Gy, float* Gz, float* Mx, float* My, float* Mz);

        float find_z_aAngle();
        float find_y_aAngle();
        float find_x_aAngle();
        float find_z_gAngle();
        float find_y_gAngle();
        float find_x_gAngle();
        float findFiltered_zAngle();
        float findFiltered_yAngle();
        float findFiltered_xAngle();

        float Calculate_z_aAngle(float xAcc, float yAcc);
        float Calculate_y_aAngle(float xAcc, float zAcc);
        float Calculate_x_aAngle(float yAcc, float zAcc);
        float Calculate_z_gAngle(float zGyr);
        float Calculate_y_gAngle(float yGyr);
        float Calculate_x_gAngle(float xGyr);
        float Calculate_filtered_zAngle(float xAcc, float yAcc, float zGyr);
        float Calculate_filtered_yAngle(float xAcc, float yGyr, float zAcc);
        float Calculate_filtered_xAngle(float xGyr, float yAcc, float zAcc);

        float hardIron_xMag(float xMag);
        float hardIron_yMag(float yMag);
        float hardIron_zMag(float xMag);
        float softIron_xMag(float xMag);
        float softIron_yMag(float yMag);
        float softIron_zMag(float zMag);

        float xMagCorrection(float xMag);
        float yMagCorrection(float yMag);
        float zMagCorrection(float zMag);

        float find_heading();
        float Calculate_heading(float xMag, float yMag);
        float find_yaw();
        float Calculate_yaw(float xMag, float yMag, float zGyr);



    private:
        void readRaw9axis(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz);
        void calibrateMinMax();
        void findScales();
        void initialize_MPU9250();

        void initializeAngles();
        void magnitometerCalibration();
        void initializeDt();
        void writeByte(uint16_t address, uint8_t data);
        void readByte(uint16_t address, uint8_t *dataOut);
        void writeMagByte(uint16_t address, uint8_t data);
        void readMagByte(uint16_t address, uint8_t *dataOut);

        uint8_t accelConfigByte;
        uint8_t gyroConfigByte;
        uint8_t buffer[20];
        float accelConvert;
        float gyroConvert;
        float magConvert = .15;
        float axn,ayn,azn,gxn,gyn,gzn,mxn,myn,mzn;

        double Aangle;
        double Gangle;
        double angle;
        double addedGangle;

        float dt;
        int i;
        int n;
        unsigned long mil;
        int nowMil;
        int lastMil;
        int milDifference;

        float accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z;
        float zAngle, yAngle, xAngle;

        float xmaxMag = 0;
        float xminMag = 0;
        float ymaxMag = 0;
        float yminMag = 0;
        float zmaxMag = 0;
        float zminMag = 0;
        float fixedMaxx,fixedMaxy,fixedMaxz;
        float fixedMinx,fixedMiny,fixedMinz;
        float xmagCorrection,ymagCorrection,zmagCorrection;

        float avgRad;
        float Xscale,Yscale,Zscale;
        float Mangle;
        float heading;
        float declination;
        float headingDeclination;

        float headingDegrees,headingFiltered;
        float Yyaw;
        float avgsX, avgsZ, avgsY;
        int accelScale, gyroScale;
        float decl;
        uint16_t address;
        uint8_t data, dataOut;
        float Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz;
        float xAcc, yAcc, zAcc, xMag,yMag,zMag,xGyr,yGyr,zGyr;
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t mx, my, mz;

};

#endif // SHERWINSMPU9250_H
