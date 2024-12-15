#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 120 // Valor mínimo corregido
#define SERVOMAX 540 // Valor máximo corregido

// Pines de retroalimentación (potenciómetros internos de los servos)
#define POT1_PIN 36 // Servo 1 - Retroalimentación en GPIO36
#define POT2_PIN 39 // Servo 2 - Retroalimentación en GPIO39
#define POT3_PIN 34 // Servo 3 - Retroalimentación en GPIO34

// Pines para botones
#define BUTTON_PIN 26 // Botón para iniciar el movimiento
#define SDA_PIN 21     // Pin SDA del I2C
#define SCL_PIN 22     // Pin SCL del I2C

// Variables para los servos
int servoChannels[3] = {0, 1, 2}; // Canales del PCA9685 para los servos
int potPins[3] = {POT1_PIN, POT2_PIN, POT3_PIN}; // Pines de retroalimentación
int targetAngle = 90; // Ángulo al que deben moverse los servos

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Configurar pines I2C
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia para servos

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Configurar pines de retroalimentación como entrada
  for (int i = 0; i < 3; i++) {
    pinMode(potPins[i], INPUT);
  }

  Serial.println("Sistema iniciado. Listo para mantener posición.");
}

void loop() {
  // Verificar si se presionó el botón para iniciar el movimiento
  if (digitalRead(BUTTON_PIN) == LOW) {
    moverServosATarget();
    delay(300); // Debounce para evitar múltiples lecturas
  }
}

// Función para mover los servos a un ángulo específico
void moverServosATarget() {
  Serial.println("Moviendo servos a 90°...");
  for (int i = 0; i < 3; i++) {
    int currentAngle = leerAngulo(i); // Leer posición actual del servo
    Serial.print("Servo ");
    Serial.print(i + 1);
    Serial.print(" - Ángulo actual: ");
    Serial.print(currentAngle);
    Serial.println("°");

    moverServo(i, targetAngle); // Mover al ángulo objetivo (90°)
    delay(1000); // Pausa de 1 segundo entre movimientos
  }
}

// Función para leer el ángulo actual de un servo
int leerAngulo(int servoIndex) {
  int potValue = analogRead(potPins[servoIndex]); // Leer valor del potenciómetro
  int angle = map(potValue, 0, 4095, 0, 180); // Mapear valor a rango de grados
  return angle;
}

// Función para mover un servo a un ángulo específico
void moverServo(int servoIndex, int angle) {
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoChannels[servoIndex], 0, pulseLength);
  Serial.print("Moviendo servo ");
  Serial.print(servoIndex + 1);
  Serial.print(" a: ");
  Serial.print(angle);
  Serial.println("°");
}
