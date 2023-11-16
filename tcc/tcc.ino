#include "tcc.h"

// Pino, valor mínimo, valor máximo, valor de armação, todos em uS
// Se a largura de pulso do PWM vai abaixo do valor de armação, o motor desliga
ESC Motor0(9, 500, 2000, 500);
ESC Motor1(10, 500, 2000, 500);
ESC Motor2(11, 500, 2000, 500);
ESC Motor3(12, 500, 2000, 500);

// Array pra iterar sobre todos os motores
ESC *motors[4] = { &Motor0, &Motor1, &Motor2, &Motor3 };

MPU9250 IMU;
BMP280 bmp280;

AccelData accelData;
GyroData gyroData;
MagData magData;
SpeedData speedData;
calData calib = { 0 };

clock_t epoch = 0;
clock_t deltaTime = 0;

int altitude = 0;
int heading = 0;
int initialHeading = 0;

void atualizaIMU(AccelData *accel, GyroData *gyro, MagData *mag) {
  IMU.update();
  IMU.getAccel(accel);
  IMU.getGyro(gyro);
  IMU.getMag(mag);
}

void atualizaAzimute(int *azmt) {
  *azmt = atan2(magData.magY, magData.magX) * 180 / M_PI;
}

void atualizaVelocidade(SpeedData *speed, AccelData *accel) {
  speed->speedX += accel->accelX * deltaTime;
  speed->speedY += accel->accelY * deltaTime;
  speed->speedZ += accel->accelZ * deltaTime;
  speed->speedNormalized = sqrt(speed->speedX ^ 2 + speed->speedY ^ 2 + speed->speedZ ^ 2);
}

void atualizaAltitude(int *alt) {
  *alt = bmp280.calAltitude(bmp280.getPressure(), SEA_LEVEL_PRESSURE);
}

void updateDeltaTime(clock_t *pt_epoch, clock_t *pt_delta) {
  *pt_delta = (*pt_epoch - millis()) * 0.001;
  *pt_epoch = millis();
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

void pedal(int direction, float throttle) {
  if (direction != 1 || direction != -1) return; // -1 pra esquerda, 1 pra direita
  for (int i = 0; i < 4; i++) {
    int motorSpeed = motor.getCurrentSpeed;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  SerialBT.begin("TrôneP32");
  while (!Serial) { ; }
  while (!SerialBT) { ; }

  bmp280.begin();
  bool err = IMU.init(calib, IMU_I2C_ADDRESS);
  if (err) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
  }

  if (!EEPROM.read(ESC_MEMORY_ADDRESS)) {
    calibrateMotors();
    EEPROM.write(ESC_MEMORY_ADDRESS, (byte) 1);
  }
  armMotors();
  atualizaIMU(&accelData, &gyroData, &magData);
  atualizaAzimute(&heading);
  initialHeading = heading;
  setpointHeading = heading;
  atualizaAltitude(&altitude);
  for (int speed = 500; speed < 2000 && accelData.accelY <= 0; speed++) {
    for (int i = 0; i < 4; i++) {
      (*motors[i]).speed(speed);
    }
    delay(10);  // espera 10ms para regular a velocidade
    if (accelData.accelY > 0) {
      EEPROM.write(LIFT_MEMORY_ADDRESS, speed);
    }
  }

  updateDeltaTime(&epoch, &deltaTime);
}

void loop() {
  updateDeltaTime(&epoch, &deltaTime);
  atualizaIMU(&accelData, &gyroData, &magData);
  atualizaVelocidade(&speedData, &accelData);
  atualizaAltitude(&altitude);
  atualizaAzimute(&heading);
  delay(50);  // atualiza dados a cada 50ms
}
