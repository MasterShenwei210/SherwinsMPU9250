/*
 * Calculation functions require the sensor data as
 * parameters to calulate angle. getBurstRead function
 * needs to be called to extract calibrated sensor data.
 * Much faster than find functions if many things are 
 * being calculated at once since they can be found with 
 * only one sensor reading.
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

SherwinsMPU9250 accelGyroMag(11.52);//Constructor takes optional declination angle as argument for magnitometer calibration. 
                                    //You can find your declination angle at: http://www.ngdc.noaa.gov/geomag-web/
float ax,ay,az,gx,gy,gz,mx,my,mz;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  accelGyroMag.initializeEverything(); //make sure to rotate sensor a lot around z axis for min/max calibration.
  //accelGyroMag.initializeWithoutMagnetometer(); //use instead if magnetometer is not being used     
}

void loop() {
 accelGyroMag.getBurstRead(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mx);

 Serial.print("Accel z angle: "); Serial.println(accelGyroMag.Calculate_z_aAngle(ax,ay));
 Serial.print("Accel y angle: "); Serial.println(accelGyroMag.Calculate_y_aAngle(ax,az));
 Serial.print("Accel x angle: "); Serial.println(accelGyroMag.Calculate_x_aAngle(ay,az));
 Serial.println();

 Serial.print("Gyro z angle: "); Serial.println(accelGyroMag.Calculate_z_gAngle(gz));
 Serial.print("Gyro y angle: "); Serial.println(accelGyroMag.Calculate_y_gAngle(gy));
 Serial.print("Gyro x angle: "); Serial.println(accelGyroMag.Calculate_x_gAngle(gx));
 Serial.println();

 Serial.print("filtered z angle: "); Serial.println(accelGyroMag.Calculate_filtered_zAngle(ax,ay,gz));
 Serial.print("filtered y angle: "); Serial.println(accelGyroMag.Calculate_filtered_yAngle(ax,gy,az));
 Serial.print("filtered x angle: "); Serial.println(accelGyroMag.Calculate_filtered_xAngle(gx,ay,az));
 Serial.println();

 Serial.print("heading: "); Serial.println(accelGyroMag.find_heading());
 Serial.print("filtered heading: "); Serial.println(accelGyroMag.find_yaw());
 Serial.println();
 
}
