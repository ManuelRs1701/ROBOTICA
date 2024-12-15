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

// Posición "home" (en radianes)
float homeRadians[4] = {1.5708, 2.35619, 0.785398, 1.5708};

// Secuencia proporcionada de ángulos (en radianes)
float trajectoryRadians[5][3] = {
    {0.7854, 1.1516, -2.2435},  // P1
    {1.1071, 0.8663, -1.6285},  // P2
    {0.7854, 0.5951, -1.0383},  // P3
    {0.4636, 0.8663, -1.6285},  // P4
    {0.7854, 1.1516, -2.2435}   // P1 (Regreso)
};

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
    iniciarSecuencia();
    delay(300); // Debounce para evitar múltiples lecturas
  }

  // Liberar servos si se presiona el botón de liberar
  if (digitalRead(BUTTON_RELEASE_PIN) == LOW) {
    liberarServos();
    delay(300); // Debounce para evitar múltiples lecturas
  }
}

// Función para iniciar la secuencia completa
void iniciarSecuencia() {
  Serial.println("Moviendo a posición 'home'...");

  // Mover a posición "home" respetando el orden mecánico q2, q1, q3, q4
  moverServo(1, homeRadians[1]); // q2: Servo 2
  delay(500);
  moverServo(0, homeRadians[0]); // q1: Servo 1
  delay(500);
  moverServo(2, homeRadians[2]); // q3: Servo 3
  delay(500);
  moverServo(3, homeRadians[3]); // q4: Servo 4
  delay(500);

  Serial.println("Posición 'home' alcanzada. Iniciando secuencia...");

  // Recorrer la trayectoria proporcionada con el orden q1, q2, q3, q4
  for (int i = 0; i < 5; i++) {
    Serial.print("Moviendo a punto P");
    Serial.println(i + 1);

    // Mover a los valores de cada punto respetando el orden q1, q2, q3, q4
    moverServo(0, trajectoryRadians[i][0]); // q1: Servo 1
    delay(300);
    moverServo(1, trajectoryRadians[i][1]); // q2: Servo 2
    delay(300);
    moverServo(2, trajectoryRadians[i][2]); // q3: Servo 3
    delay(300);
  }

  Serial.println("Secuencia completada.");
}

// Función para mover un servo a un ángulo específico en radianes
void moverServo(int servoIndex, float angleRad) {
  // Convertir radianes a grados
  float angleDeg = angleRad * (180.0 / 3.14159265359);

  // Ajustar ángulo a rango positivo alrededor de 90°
  if (angleDeg < 0) {
    angleDeg = 90 + angleDeg; // Cambiado: Ahora suma el valor negativo correctamente
    if (angleDeg > 180) angleDeg = 180; // Limitar al máximo de 180°
  }

  // Mapear el ángulo al pulso del servo
  int pulseLength = map(angleDeg, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoChannels[servoIndex], 0, pulseLength);

  // Depuración detallada
  Serial.print("Servo ");
  Serial.print(servoIndex + 1);
  Serial.print(" (Canal ");
  Serial.print(servoChannels[servoIndex]);
  Serial.print(") a: ");
  Serial.print(angleDeg);
  Serial.print("° (Rad: ");
  Serial.print(angleRad, 6);
  Serial.println(")");
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
