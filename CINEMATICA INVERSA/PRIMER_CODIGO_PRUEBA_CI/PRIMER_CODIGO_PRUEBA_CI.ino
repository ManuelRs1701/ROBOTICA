#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 120 // Valor mínimo del servo
#define SERVOMAX 540 // Valor máximo del servo

// Pines para botones
#define BUTTON_START_PIN 26 // Botón para iniciar movimientos
#define BUTTON_RELEASE_PIN 27 // Botón para liberar los servos
#define SDA_PIN 21            // Pin SDA del I2C
#define SCL_PIN 22            // Pin SCL del I2C

// Variables para los servos
int servoChannels[4] = {0, 1, 2, 3}; // Canales del PCA9685 para los servos
float servoRadians[4] = {1.5708, 2.35619, 0.785398, 1.5708}; // Ángulos en radianes para cada servo

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Configurar pines I2C
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia para servos

  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RELEASE_PIN, INPUT_PULLUP);

  Serial.println("Sistema iniciado. Listo para recibir comandos.");
}

void loop() {
  // Iniciar movimientos si se presiona el botón de inicio
  if (digitalRead(BUTTON_START_PIN) == LOW) {
    iniciarMovimientos();
    delay(300); // Debounce para evitar múltiples lecturas
  }

  // Liberar servos si se presiona el botón de liberar
  if (digitalRead(BUTTON_RELEASE_PIN) == LOW) {
    liberarServos();
    delay(300); // Debounce para evitar múltiples lecturas
  }
}

// Función para mover los servos según los ángulos en radianes
void iniciarMovimientos() {
  Serial.println("Iniciando movimientos...");
  for (int i = 0; i < 4; i++) {
    moverServo(i, servoRadians[i]); // Mover cada servo con su ángulo en radianes
    delay(500); // Pausa de 0.5 segundos entre movimientos
  }
  Serial.println("Movimientos completados.");
}

// Función para mover un servo a un ángulo específico en radianes
void moverServo(int servoIndex, float angleRad) {
  // Convertir radianes a grados
  float angleDeg = angleRad * (180.0 / 3.14159265359);

  // Ajustar ángulo a rango positivo alrededor de 90°
  if (angleDeg < 0) {
    angleDeg = 90 - angleDeg; // Cambiado: Ahora suma el valor negativo
    if (angleDeg > 180) angleDeg = 180; // Limitar al máximo de 180°
  }

  // Mapear el ángulo al pulso del servo
  int pulseLength = map(angleDeg, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoChannels[servoIndex], 0, pulseLength);

  // Depuración
  Serial.print("Servo ");
  Serial.print(servoIndex + 1);
  Serial.print(" (Canal ");
  Serial.print(servoChannels[servoIndex]);
  Serial.print(") a: ");
  Serial.print(angleDeg);
  Serial.println("°");
}

// Función para liberar los servos
void liberarServos() {
  for (int i = 0; i < 4; i++) {
    pwm.setPWM(servoChannels[i], 0, 0); // Apagar el canal del servo
    Serial.print("Servo ");
    Serial.print(i + 1);
    Serial.println(" liberado.");
  }
  Serial.println("Todos los servos liberados.");
}
