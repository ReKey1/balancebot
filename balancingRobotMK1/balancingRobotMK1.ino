#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
MPU6050 mpu(0x68);  // Change to 0x69 if your MPU uses that address
SoftwareSerial BT (12, 13); //serial port for bluetooth
int state = 0; //for bluetooth
// Motor A connections
int enA = 9;
int in1 = 4;
int in2 = 5;
// Motor B connections
int enB = 10;
int in3 = 6;
int in4 = 7;
const unsigned long motorControlInterval = 3;  // Run motor control code every 10 ms (adjust as needed)
unsigned long lastMotorControlTime = 0;

//MPU6050
// For a ±250°/s range:
double sensitivity = 1.0 / 131.0;  // ~0.00763 °/s per LSB
double gyroBias = 570;         // gyro offset

int16_t ax, ay, az;
int16_t gx, gy, gz;


///////////////////////////////////////////////// PID //////////////////////////////////////////////////////////////////
const double balancedAngle = -1.6;//54
double Kp = 131.19;
double Ki = 0.01;
double Kd = 0.07;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double targetAngle = balancedAngle; 
const double smoothingFactor = 0.02;  // Lower = smoother, Higher = more responsive
const double maxChangeRate = 100;  // Limit how fast power changes per update
double integral = 0;
double derivative = 0;
double prevAngleDif = 0;
double angleDif = 0;
double prevElapsed = 0;
double dt;

// Kalman filter tuning parameters
double Q_angle   = 0.12;   // Process noise variance for the accelerometer 0.3
double Q_bias    = 0.001;   // Process noise variance for the gyro bias0.0001
double R_measure = 0.06;    // Measurement noise variance 0.06


// Kalman filter state variables
double kalAngle = 0.0;  // Filtered angle estimate
double kalBias  = 0.0;  // Estimated gyro bias
double rate;            // Unbiased rate (gyro reading minus bias)

// Error covariance matrix - initialized to zero
double P[2][2] = {
  {0.0, 0.0},
  {0.0, 0.0}
};

// kalmanFilter function fuses the accelerometer (newAngle) and gyro (newRate) data.
// dt is the elapsed time (in seconds) since the last update.
double kalmanFilter(double newAngle, double newRate, double dt) {
  // Predict step:
  // Remove the bias from the gyro measurement and predict the new angle.
  rate = newRate - kalBias;
  kalAngle += dt * rate;
  
  // Update the error covariance matrix.
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Measurement update:
  // Compute the Kalman gain.
  double S = P[0][0] + R_measure;
  double K0 = P[0][0] / S;
  double K1 = P[1][0] / S;

  // Calculate the difference between the measured angle and predicted angle.
  double y = newAngle - kalAngle;

  // Update the angle estimate and bias.
  kalAngle += K0 * y;
  kalBias  += K1 * y;

  // Update the error covariance matrix.
  double P00_temp = P[0][0];
  double P01_temp = P[0][1];

  P[0][0] -= K0 * P00_temp;
  P[0][1] -= K0 * P01_temp;
  P[1][0] -= K1 * P00_temp;
  P[1][1] -= K1 * P01_temp;

  return kalAngle;
}
double pid(double targetAngle, double currentAngle, double dt) {
  double angleDif = targetAngle - currentAngle;

  // Integral term with anti-windup (clamping)
  integral += angleDif * dt;
  integral = constrain(integral, -5, 5); // Adjust limits as needed

  // Derivative term
  double derivative = (angleDif - prevAngleDif) / dt;

  // PID output
  double power = angleDif * Kp + integral * Ki + derivative * Kd;

  // Store previous error
  prevAngleDif = angleDif;

  return power;
}
double previousPower = 0;  // Store last motor power
double smoothMotorPower(double rawPower) {
  // Apply exponential smoothing
  double smoothedPower = previousPower * (1 - smoothingFactor) + rawPower * smoothingFactor;
  // Apply rate limiting (prevents sudden jumps)
  double powerDifference = smoothedPower - previousPower;
  if (powerDifference > maxChangeRate) smoothedPower = previousPower + maxChangeRate;
  if (powerDifference < -maxChangeRate) smoothedPower = previousPower - maxChangeRate;

  previousPower = smoothedPower;  // Store for next iteration
  return smoothedPower;
}
void power(int m1Power, int m2Power) {
  m1(m1Power);
  m2(m2Power);
}
void m1(int power) { //control motor 1
  if (power > 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (power < -0) {
    power = power * -1; //fix negative "power"
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(enA, power);
}
void m2(int power) { //control motor 2
  if (power > 0){
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (power < -0) {
    power = power * -1; //fix negative "power"
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  analogWrite(enB, power);
}
void motorsOff() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void setup() {
  Wire.begin();
  mpu.initialize();
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  motorsOff();
  Serial.begin(9600);
  BT.begin(9600);
  while(state != 'A') { //wait until start button is pressed
    if(BT.available() > 0){
      state = BT.read();
    }
  }
}
void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastMotorControlTime >= motorControlInterval) {
    lastMotorControlTime = currentTime;
    // Read raw data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0/PI); //degrees
    // Convert raw reading to angular rate in °/s:
    double angularRate = (gy - gyroBias) * sensitivity;

    //pid
    dt = (millis() - prevElapsed) / 1000.0; //in seconds
    prevElapsed = millis();
    double filteredAngle = kalmanFilter(pitch, angularRate, dt);

    double motorPower = pid(targetAngle, filteredAngle, dt);
    double smoothed = smoothMotorPower(motorPower);

  
    power(smoothed, smoothed);
    /*
    Serial.print(pitch);
    Serial.print(", ");
    Serial.print(angularRate);
    Serial.print(", ");
    Serial.println(filteredAngle);
    */
  }
  
  if(BT.available() > 0){
    state = BT.read();
  }
  if (state == 'F') {
    double integral = 0;
    double Kp = 135.19;
    targetAngle = balancedAngle + 0.25;
  }
  if (state == 'B') {
    double integral = 0;
    double Kp = 135.19;
    targetAngle = balancedAngle - 0.5;
  }
  if (state == 'P') {
    BT.print("P: ");
    BT.print(Kp);
    BT.print(", I: ");
    BT.print(Ki);
    BT.print(", D: ");
    BT.println(Kd);
    BT.print("PITCH: ");
    BT.println(atan2(-ax, sqrt(ay * ay + az * az)) * (180.0/PI));
    motorsOff();
  }
  if (state == '0') {
    double Kp = 131.19;
    targetAngle = balancedAngle;
  }
  

    //PID DEBUG
  /*
  if(BT.available() > 0){ // Checks whether data is comming from the serial port
    state = BT.read(); 
    BT.println("data came in");
  }   
  if (state == 'F') {
    Kp += 0.01;     
  }
  if (state == 'B') {
    Kp -= 0.01;
  } 
  if (state == 'T') {
    Ki += 0.0001;     
  }
  if (state == 'X') {
    Ki -= 0.0001;
  } 
  if (state =='D') { //give data
    BT.print("P: ");
    BT.print(Kp);
    BT.print(", I: ");
    BT.println(Ki);
  }
  */
  

  

}