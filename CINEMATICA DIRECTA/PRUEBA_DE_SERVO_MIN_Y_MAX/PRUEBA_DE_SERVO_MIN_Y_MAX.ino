#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVONUM 0   // Canal del PCA9685 donde está conectado el servo

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia para servos

  Serial.println("Comenzando ajuste de SERVOMIN y SERVOMAX...");

  // Mover al mínimo
  Serial.println("Moviendo al mínimo...");
  pwm.setPWM(SERVONUM, 0, 102); // Ajusta este valor
  delay(2000);

  // Mover al máximo
  Serial.println("Moviendo al máximo...");
  pwm.setPWM(SERVONUM, 0, 522); // Ajusta este valor
  delay(2000);
}

void loop() {
  // Nada aquí
}
