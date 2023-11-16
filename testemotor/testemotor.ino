#include <ESP32_ESC.h>
#include <EEPROM.h>
#include <FastIMU.h>
#include <BMP280.h>

MPU6050 IMU;
BMP280 bmp280;

AccelData accelData;
GyroData gyroData;
MagData magData;
calData calib = { 0 };

#define MOTOR_CALIB_ADDRESS (0x0)
#define MOTOR_LIFT_ADDRESS (0x0 + sizeOf(byte))
#define EEPROM_SIZE (12)

#define MOTOR_ARM_DUTY_CYCLE 0.5f
#define MOTOR_MIN_DUTY_CYCLE 0.52f
#define MOTOR_MAX_DUTY_CYCLE 1.0f

#define SEA_LEVEL_PRESSURE (1016f)

int testingMaxSpeed = 1100;

clock_t epoch = 0;
clock_t deltaTime = 0;

int altitude = 0;

ESC Motor0(18, MOTOR_MIN_DUTY_CYCLE * 2000, MOTOR_MAX_DUTY_CYCLE * 2000, MOTOR_ARM_DUTY_CYCLE * 2000);
ESC Motor1(5, MOTOR_MIN_DUTY_CYCLE * 2000, MOTOR_MAX_DUTY_CYCLE * 2000, MOTOR_ARM_DUTY_CYCLE * 2000);
ESC Motor2(17, MOTOR_MIN_DUTY_CYCLE * 2000, MOTOR_MAX_DUTY_CYCLE * 2000, MOTOR_ARM_DUTY_CYCLE * 2000);
ESC Motor3(16, MOTOR_MIN_DUTY_CYCLE * 2000, MOTOR_MAX_DUTY_CYCLE * 2000, MOTOR_ARM_DUTY_CYCLE * 2000);

ESC *motors[4] = { &Motor0, &Motor1, &Motor2, &Motor3 };

void atualizaAltitude(int *alt) {
  *alt = bmp280.calAltitude(bmp280.getPressure(), SEA_LEVEL_PRESSURE);
}

void updateDeltaTime(clock_t *pt_epoch, clock_t *pt_delta) {
  *pt_delta = (*pt_epoch - millis()) * 0.001;
  *pt_epoch = millis();
}

void startMotor(int speedMax) {
  if (speedMax < MOTOR_MIN_DUTY_CYCLE * 2000) {
    speedMax = MOTOR_MIN_DUTY_CYCLE * 2000;
  }
  if (speedMax > MOTOR_MAX_DUTY_CYCLE * 2000) {
    speedMax = MOTOR_MAX_DUTY_CYCLE * 2000;
  }

  int motorSpeed = Motor0.getCurrentSpeed();
  if (speedMax == motorSpeed) return;

  int ramp = 10;
  if (speedMax % ramp != 0) {
    speedMax += (ramp - speedMax % ramp);
  }
  if (speedMax < motorSpeed) {
    ramp = -10;
  }
  for (; motorSpeed != speedMax; motorSpeed += ramp) {
    Motor0.speed(motorSpeed);
    Motor1.speed(motorSpeed);
    Motor2.speed(motorSpeed);
    Motor3.speed(motorSpeed);
    delay(100);
  }
}

void calibrateMotors() {
  Serial.println("calibrating motors");
  for (int i = 0; i < 4; i++) {
    (*motors[i]).calib();
    (*motors[i]).stop();
  }
  Serial.println("motors calibrated");
}

void armMotors() {
  Serial.println("arming motors");
  for (int i = 0; i < 4; i++) {
    (*motors[i]).arm();
  }
  Serial.println("motors armed");
  delay(2000);
}

bool overrideCalibration = false;

void setup() {
  Wire.begin();
  EEPROM.begin(EEPROM_SIZE);
  byte motor_calibrated = EEPROM.read(MOTOR_CALIB_ADDRESS) == 1;
  Serial.begin(115200);
  delay(5000);
  bmp280.begin();
  /*bool err = IMU.init(calib, 0x68);
  if (err) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) { ; }
  }*/
  Serial.print(motor_calibrated);
  Serial.println(": input ok");
  while (true) {
    while (Serial.available() == 0) {}
    String teststr = Serial.readString();
    teststr.trim();
    if (teststr == "ok") {
      break;
    }
  }
  if (!motor_calibrated || overrideCalibration) {
    calibrateMotors();
    EEPROM.write(MOTOR_CALIB_ADDRESS, 1);
    EEPROM.commit();
    EEPROM.end();
  }
  armMotors();
  Serial.println("starting");
  startMotor(testingMaxSpeed);
}

void loop() {
  // put your main code here, to run repeatedly
  while (Serial.available() == 0) {};
  String speedStr = "";
  String teststr = Serial.readString();
  teststr.trim();
  if (teststr == "stop") {
    Motor0.stop();
    Motor1.stop();
  }
  if (teststr == "start") {
    startMotor(testingMaxSpeed);
  }
  if (teststr == "speed") {
    Serial.println("input new max speed (pulse width in uS): ");
    while (Serial.available() == 0) {};
    speedStr = Serial.readString();
    speedStr.trim();
    Serial.print("setting new max speed to: ");
    Serial.println(speedStr);
    testingMaxSpeed = speedStr.toInt();
    startMotor(testingMaxSpeed);
  }
  if (teststr == "alt") {
    atualizaAltitude(&altitude);
    Serial.print("altitude: ");
    Serial.println(altitude);
    Serial.print("temperatura: ");
    Serial.println(bmp280.getTemperature());
  }
  teststr = "";
  speedStr = "";
}
