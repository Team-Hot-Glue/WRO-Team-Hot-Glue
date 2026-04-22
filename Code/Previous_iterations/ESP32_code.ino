#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>

#define NUM_SENSORS 3
#define SERVO_PIN   18

#define FILTER_WINDOW 3

const uint8_t xshutPins[NUM_SENSORS] = {25, 26, 27};

const uint8_t sensorAddr[NUM_SENSORS] = {0x30, 0x31, 0x32};

Adafruit_VL53L0X sensors[NUM_SENSORS];

uint16_t distances[NUM_SENSORS] = {0};


uint16_t filterBuffer[NUM_SENSORS][FILTER_WINDOW] = {0};
uint8_t filterIndex = 0;

Servo servo;
int servoPosition = 105;

String serialBuffer = "";

bool servoSequenceActive = false;
unsigned long servoTimer = 0;
int servoStep = 0;

uint16_t movingAverage(uint8_t s, uint16_t val) {
  filterBuffer[s][filterIndex] = val;

  uint32_t sum = 0;
  for (int i = 0; i < FILTER_WINDOW; i++) {
    sum += filterBuffer[s][i];
  }
  return sum / FILTER_WINDOW;
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

void processCommand(String cmd) {
  cmd.trim();

  if (cmd.equalsIgnoreCase("start")) {
    servoSequenceActive = true;
    servoStep = 0;
    Serial.println("[SERVO] start");
  }
  else if (cmd.startsWith("a")) {
    int angle = constrain(cmd.substring(1).toInt(), 0, 180);
    servo.write(angle);
    servoPosition = angle;
  }
}

void updateServoSequence() {
  if (!servoSequenceActive) return;

  unsigned long now = millis();

  if (servoStep == 0) {
    servo.write(0);
    servoTimer = now;
    servoStep = 1;
  }
  else if (servoStep == 1 && now - servoTimer >= 200) {
    servo.write(105);
    servoTimer = now;
    servoStep = 2;
  }
  else if (servoStep == 2 && now - servoTimer >= 300) {
    servoSequenceActive = false;
    servoStep = 0;
    Serial.println("[SERVO] done");
  }
}

void tof_setup() {
  Serial.println("[BOOT] Init VL53L0X (no mux)...");

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  delay(50);

  for (int i = 0; i < NUM_SENSORS; i++) {

    // Turn ON this sensor only
    digitalWrite(xshutPins[i], HIGH);
    delay(50);

    if (!sensors[i].begin(0x29, false, &Wire)) {
      Serial.print("[ERROR] Sensor ");
      Serial.println(i);
      continue;
    }

    sensors[i].setAddress(sensorAddr[i]);

    sensors[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);

    sensors[i].startRangeContinuous();

    Serial.print("[OK] Sensor ");
    Serial.print(i);
    Serial.print(" addr: 0x");
    Serial.println(sensorAddr[i], HEX);
  }
}

void servo_setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  tof_setup();
  servo_setup();

  Serial.println("[READY]");
}

void loop() {
  handleSerial();
  updateServoSequence();

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {

    if (sensors[i].isRangeComplete()) {
      uint16_t raw = sensors[i].readRange();

      if (raw > 0) {
        distances[i] = movingAverage(i, raw);
      }
    }
  }

  filterIndex = (filterIndex + 1) % FILTER_WINDOW;

  Serial.print(distances[2]);
  Serial.print(",");
  Serial.print(distances[1]);
  Serial.print(",");
  Serial.println(distances[0]);
}