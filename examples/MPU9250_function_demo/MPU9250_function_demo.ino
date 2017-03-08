/*
 * Find functions read from the sensor and
 * calculate the angles. Every function reads
 * from the sensor on its own and can be used
 * independently.
 * 
 * IMPORTANT: Need to rotate the device around
 * the z axis during the initializeEverything().
 * Lasts for around 5 seconds. 
 * 
 * NOTE: Heading function only works when device 
 * is flat with the z axis perpendicular to the
 * ground
 */

#include <SherwinsMPU9250.h>

SherwinsMPU9250 accelGyroMag(2, 250, 11.52);//Constructor takes desired accelerometer and gyroscope scales and 
                                            //declination angle as argument for magnitometer calibration. 
                                            //You can find your declination angle at: http://www.ngdc.noaa.gov/geomag-web/
void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  accelGyroMag.initializeEverything(); //make sure to rotate sensor a lot around z axis for min/max calibration.
  
  //accelGyroMag.initializeWithoutMagnetometer(); //use instead if magnetometer is not being used     
}

void loop() {
  Serial.print("Accel z angle: "); Serial.println(accelGyroMag.find_z_aAngle());
  Serial.print("Accel y angle: "); Serial.println(accelGyroMag.find_y_aAngle());
  Serial.print("Accel x angle: "); Serial.println(accelGyroMag.find_x_aAngle());
  Serial.println();

  Serial.print("Gyro z angle: "); Serial.println(accelGyroMag.find_z_gAngle());
  Serial.print("Gyro y angle: "); Serial.println(accelGyroMag.find_y_gAngle());
  Serial.print("Gyro x angle: "); Serial.println(accelGyroMag.find_x_gAngle());
  Serial.println();

  Serial.print("filtered z angle: "); Serial.println(accelGyroMag.findFiltered_zAngle());
  Serial.print("filtered y angle: "); Serial.println(accelGyroMag.findFiltered_yAngle());
  Serial.print("filtered x angle: "); Serial.println(accelGyroMag.findFiltered_xAngle());
  Serial.println();

  Serial.print("heading: "); Serial.println(accelGyroMag.find_heading());
  Serial.print("filtered heading: "); Serial.println(accelGyroMag.find_yaw());
  Serial.println();
 
}
