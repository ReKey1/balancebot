#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
SoftwareSerial BT (12, 13);
MPU6050 mpu(0x68);  // Change to 0x69 if your MPU uses that address
void setup() {
  Serial.begin(9600);
  Wire.begin();  // Explicitly set I2C pins for R4 Minima
  while (!Serial) {
    // Optional: blink LED to show waiting status (for boards with native USB)
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  

 // if (!mpu.testConnection()) {
 //   Serial.println("MPU6050 connection failed!");
 //   while (true); // Stop if connection fails
//  } else {
 //   Serial.println("MPU6050 connection successful!");
 // }
 
}

void loop() {
  
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Read raw data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print data to Serial Monitor
  //Serial.print("Accel X: "); Serial.print(ax);
  //Serial.print(" | Y: "); Serial.print(ay);
  //Serial.print(" | Z: "); Serial.print(az);
  //Serial.print(" | Gyro X: "); Serial.print(gx);
  //Serial.print(" | Y: "); Serial.print(gy);
  //Serial.print("Z, "); Serial.println(gz);
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  Serial.println(pitch);


}
