// Gerenciamento de memória, utilizado para processar a fila de waypoints (controle de rota)
#include <malloc.h>

// Protocolos de comunicação I2C
#include <Wire.h>

// Usada para controle dos motores via ESC, a partir de um PWM 50Hz variando pulso de 5ms a 20ms
// Modificação da biblioteca RC_ESC v1.1.0 de Eric Nantel, para funcionar utilizando ESP32
#include <ESP32_ESC.h>

// Usada para salvar valores de calibaração na memória do microcontrolador
#include <EEPROM.h>

// Usadas para leitura dos sensores medidos de inércia (acelerometro, giroscopio, magnetometro)
#include <FastIMU.h>

// Valores de setpoint do PID
struct setpointValues {
  float x = 0;
  float y = 0;
  float z = 0;
};

struct setpointValues setpointGyro;
struct setpointValues setpointAccel; 

float desiredHeading = 0;

// Usada para leitura do sensor de pressão
#include <BMP280.h>

// Wrapper do FreeRTOS para ESP32, usado para rodar tasks de atualização e controle em paralelo entre os núcleos
#include <CleanRTOS.h>

// Controle Serial via Bluetooth
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

// Endereços da I2C, valores padrão retirados dos datasheets dos componentes
#define IMU_I2C_ADDRESS (0x68)
#define BMP_I2C_ADDRESS (0x77)
#define MAG_I2C_ADDRESS (0xD)

#define ESC_MEMORY_ADDRESS (0x0)
#define LIFT_MEMORY_ADDRESS (0x8)

// Pressão atmosférica no nível do mar em Novo Hamburgo, em pA
#define SEA_LEVEL_PRESSURE (1016)

// Limites dos motores
#define MOTOR_ARM_DUTY_CYCLE 0.5f
#define MOTOR_MIN_DUTY_CYCLE 0.52f
#define MOTOR_MAX_DUTY_CYCLE 1.0f

// LED onboard da placa, precisa ser definido explicitamente pro ESP32
#define LED_BUILTIN (2)

// Declinação magnética (17° 45' W +- 26' na Liberato)
// TODO: substituir por cálculo usando GPS
#define MAG_DECLINATION (17.75)

// Inicializa o FreeRTOS
namespace crt {
MainInits mainInits;
}

// Struct para armazenar velocidade
typedef struct SpeedData {
  long speedX;
  long speedY;
  long speedZ;
  long speedNormalized;
} SpeedData;
