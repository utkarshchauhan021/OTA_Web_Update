/*
 * CODE NAME : PARADRONE CODE  
 * DESIGNER  : HARSH BENAHALKAR 
 * DATE      : 27-06-2021
 * LIBRARY   : https://github.com/jarzebski/Arduino-MPU6050
 * "Download above library for Header Files of IMU MPU6050"
 */

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int static_pos = 10;    // variable to store the servo position
int triggered_pos = 90;
float norm_acc, min_acc = 10;
float acceleration_threshold = 1.0;
int i, safety_threshold = 5, iterations = 1000;


void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(115200);
  
  myservo.attach(9);
  myservo.write(static_pos);

  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // If you want, you can set accelerometer offsets
  // mpu.setAccelOffsetX();
  // mpu.setAccelOffsetY();
  // mpu.setAccelOffsetZ();
  
  checkSettings();
}


void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
}


void deploy_parachute(){
  myservo.write(triggered_pos);  
  Serial.println("SERVO TRIGGERED!!!!!");
  digitalWrite(LED_BUILTIN, HIGH);
  while(1){}     
}


float give_norm_acceleration(){
  Vector normAccel = mpu.readNormalizeAccel();
  float normacc = 0, ax, ay, az;
  
  ax = normAccel.XAxis;
  Serial.print(" Xnorm = ");
  Serial.print(ax);

  ay = normAccel.YAxis;
  Serial.print(" Ynorm = ");
  Serial.print(ay);

  az = normAccel.ZAxis;
  Serial.print(" Znorm = ");
  Serial.print(az);

  normacc = sqrt( ax*ax + ay*ay + az*az ); 
  Serial.print(" Net acceleration = ");
  Serial.println(normacc);
  
  return normacc;
}


float give_raw_acceleration(){
  Vector rawAccel = mpu.readRawAccel();
  Serial.print(" Xraw = ");
  Serial.print(rawAccel.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(rawAccel.YAxis);
  Serial.print(" Zraw = ");
  Serial.println(rawAccel.ZAxis);
}


float min_acceleration(float min_acc, float raw_acc){
  if(raw_acc < min_acc){
    min_acc = raw_acc;
  }
  
  Serial.print("Minimum acceleration : ");
  Serial.println(min_acc);
  return min_acc;
  
}

void loop()
{
  int flags = 0;
  for(i=0; i<iterations; i++){
    norm_acc = give_norm_acceleration();
    min_acc = min_acceleration(min_acc, norm_acc); 
    if(norm_acc < acceleration_threshold){
      flags++;
    } 
    Serial.print("Loop number :");
    Serial.print(i);
    Serial.print(" flag :");
    Serial.println(flags);
    if(flags >= safety_threshold){
      deploy_parachute(); 
      
    }
    delay(10);
  }
  
  
  
}
