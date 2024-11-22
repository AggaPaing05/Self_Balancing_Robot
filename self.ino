#include <Wire.h>
#include <Kalman.h>
#include <MPU6050.h>

MPU6050 mpu;

// Sensor data variables
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float pitch;

// Kalman filter variables
float Q_angle = 0.001; // Process noise variance for the accelerometer
float Q_bias = 0.003;  // Process noise variance for the gyro bias
float R_measure = 0.03; // Measurement noise variance

float angle = 0;       // The angle calculated by the Kalman filter
float bias = 0;        // Gyro bias
float rate;            // Unbiased rate

float P[2][2] = {{0, 0}, {0, 0}}; // Error covariance matrix

unsigned long lastTime;
float dt;
;
int set = -4;
int my;
// Define motor control pins
#define BIN1 7
#define BIN2 8
#define PWMB 9
#define AIN1 4
#define AIN2 5
#define PWMA 3
#define STBY 6

float error = 0;
float error_previous = 0;
float error_integral = 0;
float error_derivative = 0;

float kp =  0.822;//52;//78;//75;//55.0;
float ki =  1.5;//0.00001;//0.0005;
float kd = 0.0025;//0.01;//7.8;//7.3;//6.5;
float pidCalculate;
// float dt = 0.01;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  lastTime = millis();
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Initialize motors to stop
  digitalWrite(STBY, HIGH);  // Standby mode
}

void loop() {
  // Read MPU6050 data
  accX = mpu.getAccelerationX();
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationY();
  gyroZ = mpu.getRotationZ();

  // Calculate pitch from accelerometer
  float accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // Convert gyro to deg/s
  float gyroRateY = gyroY / 131.0;

  // Calculate delta time
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Apply Kalman filter to pitch
  pitch = kalmanFilter(accAngleY, gyroRateY);
  my = pitch;
  pid_cal();
  // Serial.print(millis());
  // Serial.print(",");
  Serial.print(pitch);
  // Serial.print(",");
  // Serial.print(pidCalculate);
  // Serial.print(",");
  // Serial.println(error);
  //delay(5); // Small delay to match desired loop rate
}

float kalmanFilter(float newAngle, float newRate) {
  // Prediction
  rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Correction
  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle; // Angle difference
  angle += K[0] * y;
  bias += K[1] * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}


void forward()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  //analogWrite(PWMA, pid);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  //analogWrite(PWMB, pid);
  analogWrite(PWMA, pidCalculate);
  analogWrite(PWMB, pidCalculate);
}

void backward()
{
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  //analogWrite(PWMA, pid);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  //analogWrite(PWMB, pid);
  analogWrite(PWMA, pidCalculate);
  analogWrite(PWMB, pidCalculate);
}

void pid_cal()
{
  error = my - set;
  error_integral += error*dt;
  error_integral += error*dt;
  error_derivative = (error - error_previous)/dt;
  error_previous = error;
  pidCalculate = kp*error + ki*error_integral + kd*error_derivative;

  if (pidCalculate > 250)  pidCalculate = 250;
  if (pidCalculate < -250) pidCalculate = 250;
  Serial.print("    Error : ");
  Serial.println(pidCalculate);
  if ( pidCalculate < set )
  {
    // backward();  
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  //analogWrite(PWMA, pid);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  //analogWrite(PWMB, pid);
  analogWrite(PWMA, pidCalculate);
  analogWrite(PWMB, pidCalculate) ;
  }
  else if ( pidCalculate > set )
  {

    digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  //analogWrite(PWMA, pid);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  //analogWrite(PWMB, pid);
 
  analogWrite(PWMA, -1*pidCalculate);
  analogWrite(PWMB, -1*pidCalculate);
  }
  // else
  // {
  //   digitalWrite(STBY, LOW);
  // }
}
