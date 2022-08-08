#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "NXP_FXOS_FXAS.h" 

float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;

#define NUMBER_SAMPLES 1000

float gx_off = 0;
float gy_off = 0;
float gz_off = 0;

int debug = 0; 
const int power_pin = D6;




float ax, ay, az;       // Stores the real accel value in g's
float gyrox, gyroy, gyroz; // Stores the real gyro value in degrees per seconds
float gyrox_cal, gyroy_cal, gyroz_cal;  
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate
float pitch, yaw, roll;
// parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (0.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval


void setup() {
  pinMode(power_pin, OUTPUT);
  digitalWrite(power_pin, LOW);
  delay(100);
  digitalWrite(power_pin, HIGH);
  delay(100);
  Serial.begin(115200);
  while (!Serial) yield();

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  sensors_event_t accel, gyro, mag;
  float x, y, z;
  for (uint16_t sample = 0; sample < NUMBER_SAMPLES; sample++) {
    gyroscope->getEvent(&gyro);
    x = gyro.gyro.x;
    y = gyro.gyro.y;
    z = gyro.gyro.z;
    Serial.print(F("Gyro: ("));
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(z); Serial.print(")");

    min_x = min(min_x, x);
    min_y = min(min_y, y);
    min_z = min(min_z, z);
  
    max_x = max(max_x, x);
    max_y = max(max_y, y);
    max_z = max(max_z, z);
  
    mid_x = (max_x + min_x) / 2;
    mid_y = (max_y + min_y) / 2;
    mid_z = (max_z + min_z) / 2;

    Serial.print(F(" Zero rate offset: ("));
    Serial.print(mid_x, 4); Serial.print(", ");
    Serial.print(mid_y, 4); Serial.print(", ");
    Serial.print(mid_z, 4); Serial.print(")");  
  
    Serial.print(F(" rad/s noise: ("));
    Serial.print(max_x - min_x, 3); Serial.print(", ");
    Serial.print(max_y - min_y, 3); Serial.print(", ");
    Serial.print(max_z - min_z, 3); Serial.println(")");   
    delay(10);
  }
  Serial.println(F("\n\nFinal zero rate offset in radians/s: "));
  Serial.print(mid_x, 4); Serial.print(", ");
  Serial.print(mid_y, 4); Serial.print(", ");
  Serial.println(mid_z, 4);
  gx_off = mid_x;
  gy_off = mid_y;
  gz_off = mid_z;
  



  Wire.setClock(400000); // 400KHz
}


void loop() {
  float roll, pitch, heading;
  float gx, gy, gz;
  static uint8_t counter = 0;


  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);




  // Gyroscope needs to be converted from Rad/s to Degree/s
  gyrox_cal = gyro.gyro.x - gx_off;
  gyroy_cal = gyro.gyro.y - gy_off;
  gyroz_cal = gyro.gyro.z - gz_off;

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  if(lastUpdate - firstUpdate > 10000000uL) {
    GyroMeasError = PI * (0.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
    beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
    GyroMeasDrift = PI * (0.045f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; 
  }
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyrox_cal, gyroy_cal, gyroz_cal);

  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  if (delt_t > 50) { // 
    
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    roll  *= 180.0f / PI;

     
    Serial.print("YPRAxAyAzGxGyGz=");
    Serial.print(yaw); Serial.print(",");
    Serial.print(pitch); Serial.print(",");
    Serial.print(roll); Serial.print(",");
  
    Serial.print(accel.acceleration.x, 4); Serial.print(",");
    Serial.print(accel.acceleration.y, 4); Serial.print(",");
    Serial.print(accel.acceleration.z, 4); Serial.print(",");

    Serial.print(gyrox_cal, 3); Serial.print(",");
    Serial.print(gyroy_cal, 3); Serial.print(",");
    Serial.print(gyrox_cal, 3); Serial.println();

    

    if (debug == 1){
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(yaw, 2);
      Serial.print(", ");
      Serial.print(pitch, 2);
      Serial.print(", ");
      Serial.println(roll, 2);

      Serial.print("average rate = "); Serial.print(1.0f/deltat, 2); Serial.println(" Hz");

      Serial.println(" x\t  y\t  z  ");

      Serial.print((int)(1000 * accel.acceleration.x)); Serial.print('\t');
      Serial.print((int)(1000 * accel.acceleration.y)); Serial.print('\t');
      Serial.print(accel.acceleration.z);
      Serial.println(" mg");


      Serial.print((int)(gyrox_cal)*SENSORS_RADS_TO_DPS); Serial.print('\t');
      Serial.print((int)(gyroy_cal)*SENSORS_RADS_TO_DPS); Serial.print('\t');
      Serial.print((int)(gyroz_cal)*SENSORS_RADS_TO_DPS);
      Serial.println(" o/s");

      Serial.print((int)(yaw)); Serial.print('\t');
      Serial.print((int)(pitch)); Serial.print('\t');
      Serial.print((int)(roll));
      Serial.println(" ypr");

      Serial.print("rt: "); Serial.print(1.0f / deltat, 2); Serial.println(" Hz");

      
    }
    count = millis();
  }
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
  
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gyrox -= gbiasx;
    gyroy -= gbiasy;
    gyroz -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
    qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
    qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
    qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}
