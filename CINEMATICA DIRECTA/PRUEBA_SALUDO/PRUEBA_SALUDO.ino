#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 120 // Valor mínimo corregido
#define SERVOMAX 540 // Valor máximo corregido

// Pines de retroalimentación (potenciómetros internos de los servos)
#define POT1_PIN 36 // Servo 1 - Retroalimentación en GPIO36
#define POT2_PIN 39 // Servo 2 - Retroalimentación en GPIO39
#define POT3_PIN 34 // Servo 3 - Retroalimentación en GPIO34
#define POT4_PIN 35 // Servo 4 - Retroalimentación en GPIO35

// Pines para botones
#define BUTTON_PIN 26 // Botón para iniciar el movimiento
#define SDA_PIN 21     // Pin SDA del I2C
#define SCL_PIN 22     // Pin SCL del I2C

// Variables para los servos
int servoChannels[4] = {0, 1, 2, 3}; // Canales del PCA9685 para los servos
int potPins[4] = {POT1_PIN, POT2_PIN, POT3_PIN, POT4_PIN}; // Pines de retroalimentación

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Configurar pines I2C
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia para servos

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Sistema iniciado. Listo para saludar.");
}

void loop() {
  // Verificar si se presionó el botón para iniciar el saludo
  if (digitalRead(BUTTON_PIN) == LOW) {
    realizarSaludo();
    delay(300); // Debounce para evitar múltiples lecturas
  }
}

// Función para realizar el saludo
void realizarSaludo() {
  Serial.println("Iniciando saludo...");

  // Mover Servo 2 a 45°
  moverServo(1, 135); // Servo 2 en el canal 1
  delay(1000);       // Esperar 1 segundo

  // Mover Servo 1 a 90°
  moverServo(0, 90); // Servo 1 en el canal 0
  delay(1000);       // Esperar 1 segundo

  // Mover Servo 3 en un bucle de 45° a 135° cinco veces
  for (int i = 0; i < 5; i++) {
    moverServo(2, 135); // Servo 3 en el canal 2
    delay(500);         // Pausa de 0.5 segundos
    moverServo(2, 45);  // Volver a 45°
    delay(500);
  }

  // Al final, dejar el Servo 3 en 45°
  moverServo(2, 45);

  // Mover Servo 4 a 100°
  moverServo(3, 90); // Servo 4 en el canal 3
  delay(1000);        // Esperar 1 segundo

  Serial.println("Saludo completado.");
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
