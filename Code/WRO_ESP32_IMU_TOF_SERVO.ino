/*─────────────────────────────────────────────────────────────────────────────
  TOF, Servo and BNO086 IMU

  Serial output (115200 baud) – one CSV line per tick at PRINT_HZ:

  d0,d1,d2,ax,ay,az,gx,gy,gz,roll,pitch,yaw

  d0/d1/d2 : ToF distances in mm
  ax/ay/az : linear acceleration  in m/s²  
  gx/gy/gz : gyroscope (rad/s) 
  roll, pitch, yaw : Euler's angles.          

  Commands received from RPi over USB Serial:
  a<angle>   – move servo to angle 0-180
  start      – trigger servo sequence
 ─────────────────────────────────────────────────────────────────────────────*/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL53L1X.h>

#include <ESP32Servo.h>

#include <SparkFun_BNO08x_Arduino_Library.h>

#define SERVO_PIN    13
#define XSHUT_L0X_0  25   // sensor[0] VL53L0X
#define XSHUT_L1X    26   // sensor[1] VL53L1X
#define XSHUT_L0X_2  27   // sensor[2] VL53L0X

#define BNO086_ADDR  0x4A

#define ADDR_L0X_0   0x30
#define ADDR_L1X     0x29
#define ADDR_L0X_2   0x32

#define FILTER_WINDOW   5      // ToF moving-average window
#define PRINT_HZ        50     // Serial output rate (Hz)

// IMU report interval in microseconds – 10 000 µs = 100 Hz
#define IMU_REPORT_INTERVAL_US  10000

// BNO086
BNO08x imu;
bool imuOK = false;

float imu_ax = 0, imu_ay = 0, imu_az = 0;   // linear acceleration
float imu_gx = 0, imu_gy = 0, imu_gz = 0;   // gyroscope
float imu_roll = 0, imu_pitch = 0, imu_yaw = 0; //Euler's angles

void imu_setup() {
  if (!imu.begin(BNO086_ADDR, Wire, -1, -1)) {
    Serial.println("[ERROR] BNO086 not found – check wiring and I2C address");
    imuOK = false;
    return;
  }

  imu.enableLinearAccelerometer(IMU_REPORT_INTERVAL_US);

  imu.enableGyro(IMU_REPORT_INTERVAL_US);

  imu.enableRotationVector(IMU_REPORT_INTERVAL_US);

  imuOK = true;
  Serial.printf("[OK]    BNO086 @ 0x%02X  (accel + gyro + rot @ %u µs)\n",
                BNO086_ADDR, IMU_REPORT_INTERVAL_US);
}

void imu_read() {
  if (!imuOK) return;
  while (imu.getSensorEvent()) {
    if(imu.getSensorEventID() == SENSOR_REPORTID_LINEAR_ACCELERATION) {
      imu_ax = imu.getLinAccelX();
      imu_ay = imu.getLinAccelY();
      imu_az = imu.getLinAccelZ();
    }

    if(imu.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED){
      imu_gx = imu.getGyroX();
      imu_gy = imu.getGyroY();
      imu_gz = imu.getGyroZ();
    }

    if (imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR){
      imu_roll = (imu.getRoll()) * 180.0 / PI;
      imu_pitch = (imu.getPitch()) * 180.0 / PI;
      imu_yaw = (imu.getYaw()) * 180.0 / PI;
    }
  }
}

Adafruit_VL53L0X l0x_0;
Adafruit_VL53L1X l1x = Adafruit_VL53L1X(XSHUT_L1X);
Adafruit_VL53L0X l0x_2;

bool sensorOK[3] = {false, false, false};
uint16_t distances[3] = {INVALID_MM, INVALID_MM, INVALID_MM};

uint16_t filterBuf[3][FILTER_WINDOW] = {};
uint8_t  filterIdx[3]  = {0, 0, 0};
uint8_t  filterFill[3] = {0, 0, 0};

uint16_t movingAverage(uint8_t s, uint16_t val) {
  filterBuf[s][filterIdx[s]] = val;
  filterIdx[s] = (filterIdx[s] + 1) % FILTER_WINDOW;
  if (filterFill[s] < FILTER_WINDOW) filterFill[s]++;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < filterFill[s]; i++) {
    uint8_t pos = (filterIdx[s] + FILTER_WINDOW - 1 - i) % FILTER_WINDOW;
    sum += filterBuf[s][pos];
  }
  return (uint16_t)(sum / filterFill[s]);
}

static bool reassignL1XAddress(uint8_t newAddr7bit) {
  Wire.beginTransmission(0x29);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x01);
  Wire.write((uint8_t)(newAddr7bit << 1));
  if (Wire.endTransmission() != 0) return false;
  delay(5);
  return true;
}

void tof_setup() {
  Serial.println("[BOOT] Initialising TOF sensors...");

  pinMode(XSHUT_L0X_0, OUTPUT); digitalWrite(XSHUT_L0X_0, LOW);
  pinMode(XSHUT_L1X,   OUTPUT); digitalWrite(XSHUT_L1X,   LOW);
  pinMode(XSHUT_L0X_2, OUTPUT); digitalWrite(XSHUT_L0X_2, LOW);
  delay(10);

  digitalWrite(XSHUT_L0X_0, HIGH);
  delay(15);
  if (!l0x_0.begin(0x29, false, &Wire)) {
    Serial.println("[ERROR] sensor[0] VL53L0X");
  } else {
    l0x_0.setAddress(ADDR_L0X_0);
    l0x_0.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
    l0x_0.startRangeContinuous();
    sensorOK[0] = true;
    Serial.printf("[OK]    sensor[0] VL53L0X @ 0x%02X\n", ADDR_L0X_0);
  }

  digitalWrite(XSHUT_L0X_2, HIGH);
  delay(15);
  if (!l0x_2.begin(0x29, false, &Wire)) {
    Serial.println("[ERROR] sensor[2] VL53L0X");
  } else {
    l0x_2.setAddress(ADDR_L0X_2);
    l0x_2.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
    l0x_2.startRangeContinuous();
    sensorOK[2] = true;
    Serial.printf("[OK]    sensor[2] VL53L0X @ 0x%02X\n", ADDR_L0X_2);
  }

  digitalWrite(XSHUT_L1X, HIGH);
  delay(150);
  if (!l1x.begin(0x29, &Wire)) {
    Serial.print("[ERROR] sensor[1] VL53L1X, status: ");
    Serial.println(l1x.vl_status);
    sensorOK[1] = false;
  } else {
    l1x.setTimingBudget(20);
    l1x.startRanging();
    sensorOK[1] = true;
    Serial.println("[OK]    sensor[1] VL53L1X @ 0x29");
  }
}

inline void readL0X(uint8_t idx, Adafruit_VL53L0X &sensor) {
  if (!sensorOK[idx] || !sensor.isRangeComplete()) return;
  uint16_t raw = sensor.readRange();
  distances[idx] = (raw == 0 || raw >= 8190) ? INVALID_MM : movingAverage(idx, raw);
}

inline void readL1X() {
  if (!sensorOK[1]) return;
  if (l1x.dataReady()) {
    int16_t raw = l1x.distance();
    l1x.clearInterrupt();
    distances[1] = (raw <= 0 || raw > 4000) ? INVALID_MM : movingAverage(1, (uint16_t)raw);
  }
}

// Servo
Servo servo;
int servoPosition = 105;
bool servoSequenceActive = false;
unsigned long servoTimer = 0;
int servoStep = 0;

void servo_setup() {
  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  servo.write(servoPosition);
}

void updateServoSequence() {
  if (!servoSequenceActive) return;
  unsigned long now = millis();
  switch (servoStep) {
    case 0:
      servo.write(0);
      servoTimer = now;
      servoStep  = 1;
      break;
    case 1:
      if (now - servoTimer >= 200) {
        servo.write(105);
        servoTimer = now;
        servoStep  = 2;
      }
      break;
    case 2:
      if (now - servoTimer >= 300) {
        servoSequenceActive = false;
        servoStep = 0;
        Serial.println("[SERVO] done");
      }
      break;
  }
}

// Serial command handling

String serialBuffer = "";

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("a")) {
    int angle = constrain(cmd.substring(1).toInt(), 0, 180);
    servo.write(angle);
    servoPosition = angle;
  } else if (cmd.equalsIgnoreCase("start")) {
    servoSequenceActive = true;
    servoStep = 0;
    Serial.println("[SERVO] start");
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length()) {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(XSHUT_L0X_0, OUTPUT); digitalWrite(XSHUT_L0X_0, LOW);
  pinMode(XSHUT_L1X,   OUTPUT); digitalWrite(XSHUT_L1X,   LOW);
  pinMode(XSHUT_L0X_2, OUTPUT); digitalWrite(XSHUT_L0X_2, LOW);

  Wire.begin(21, 22);       // SDA=21, SCL=22
  Wire.setClock(400000);    

  tof_setup();
  imu_setup();
  servo_setup();

  Serial.println("[READY]");
}

static unsigned long lastPrint = 0;

void loop() {
  handleSerial();
  updateServoSequence();

  readL0X(0, l0x_0);
  readL1X();
  readL0X(2, l0x_2);
  imu_read();

  unsigned long now = millis();
  if (now - lastPrint >= (1000UL / PRINT_HZ)) {
    lastPrint = now;
    Serial.print(distances[0]); Serial.print(',');
    Serial.print(distances[1]); Serial.print(',');
    Serial.print(distances[2]); Serial.print(',');
    Serial.print(imu_ax, 2);    Serial.print(',');
    Serial.print(imu_ay, 2);    Serial.print(',');
    Serial.print(imu_az, 2);    Serial.print(',');
    Serial.print(imu_gx, 4);    Serial.print(',');
    Serial.print(imu_gy, 4);    Serial.print(',');
    Serial.print(imu_gz, 4);    Serial.print(',');
    Serial.print(imu_roll, 1);  Serial.print(',');
    Serial.print(imu_pitch, 1); Serial.print(',');
    Serial.println(imu_yaw, 1);   
  }
}