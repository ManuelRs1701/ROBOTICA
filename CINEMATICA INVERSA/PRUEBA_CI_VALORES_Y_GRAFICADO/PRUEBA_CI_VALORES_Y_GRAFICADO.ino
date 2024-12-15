#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 120 // Valor mínimo del servo
#define SERVOMAX 540 // Valor máximo del servo

// Pines para botones
#define BUTTON_START_PIN 26   // Botón para iniciar movimientos
#define BUTTON_RELEASE_PIN 27 // Botón para liberar los servos
#define SDA_PIN 21            // Pin SDA del I2C
#define SCL_PIN 22            // Pin SCL del I2C

// Pines para lectura de potenciómetros internos
#define ADC0 A0 // Potenciómetro del Servo 1
#define ADC3 A3 // Potenciómetro del Servo 2
#define ADC6 A6 // Potenciómetro del Servo 3
#define ADC7 A7 // Potenciómetro del Servo 4

// Variables para los servos
int servoChannels[4] = {0, 1, 2, 3}; // Canales del PCA9685 para los servos
float currentAngles[4] = {0.0, 0.0, 0.0, 0.0}; // Ángulos actuales de los servos

// Dimensiones del brazo
float l1 = 7.0; // Longitud del primer eslabón
float l2 = 13.0; // Longitud del segundo eslabón
float l3 = 10.0; // Longitud del tercer eslabón

// Vector de radianes para la posición "home"
float homeRadians[4] = {1.5708, 2.35619, 0.785398, 1.5708}; // Home (en radianes)

// Funciones prototipo
void calcularCinematicaInversa(float pos[3]);
void moverServo(int servoIndex, float angleRad);
void liberarServos();
void ejecutarSecuenciaCuadrado();
void leerAngulosReales(float realAngles[4]);
void calcularErrores(float targetAngles[4], float realAngles[4]);

// Función setup
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Configurar pines I2C
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia para servos

  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RELEASE_PIN, INPUT_PULLUP);

  // Configurar pines ADC para lectura de potenciómetros
  pinMode(ADC0, INPUT);
  pinMode(ADC3, INPUT);
  pinMode(ADC6, INPUT);
  pinMode(ADC7, INPUT);

  Serial.println("Sistema iniciado. Listo para recibir comandos.");
}

void loop() {
  // Leer entrada desde el monitor serie para recibir el vector XYZ
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    float target[3];
    sscanf(input.c_str(), "%f,%f,%f", &target[0], &target[1], &target[2]); // Parsear entrada

    Serial.println("Calculando cinemática inversa...");
    calcularCinematicaInversa(target);
  }

  // Ejecutar secuencia de cuadrado si se presiona el botón de inicio
  if (digitalRead(BUTTON_START_PIN) == LOW) {
    ejecutarSecuenciaCuadrado();
    delay(300); // Debounce para evitar múltiples lecturas
  }

  // Liberar servos si se presiona el botón de liberar
  if (digitalRead(BUTTON_RELEASE_PIN) == LOW) {
    liberarServos();
    delay(300); // Debounce para evitar múltiples lecturas
  }
}

// Función para calcular la cinemática inversa
void calcularCinematicaInversa(float pos[3]) {
  float px = pos[0];
  float py = pos[1];
  float pz = pos[2];

  // Cálculo de los ángulos articulares
  float D = (px * px + py * py + (pz - l1) * (pz - l1) - l2 * l2 - l3 * l3) / (2 * l2 * l3);
  float theta3 = atan2(-sqrt(1 - D * D), D);
  float theta2 = atan2(pz - l1, sqrt(px * px + py * py)) - atan2(l3 * sin(theta3), l2 + l3 * cos(theta3));
  float theta1 = atan2(py, px);
  float targetAngles[4] = {theta1, theta2, theta3, 0.0};

  // Mostrar resultados calculados
  Serial.println("Valores calculados para los motores:");
  Serial.print("Theta1 calculado: ");
  Serial.print(theta1 * 180 / 3.14159265359);
  Serial.println("°");
  Serial.print("Theta2 calculado: ");
  Serial.print(theta2 * 180 / 3.14159265359);
  Serial.println("°");
  Serial.print("Theta3 calculado: ");
  Serial.print(theta3 * 180 / 3.14159265359);
  Serial.println("°");

  // Mover los servos a las posiciones calculadas
  moverServo(0, theta1);
  delay(500);
  moverServo(1, theta2);
  delay(500);
  moverServo(2, theta3);
  delay(500);

  // Leer los ángulos reales de los potenciómetros
  float realAngles[4];
  leerAngulosReales(realAngles);

  // Calcular y mostrar errores
  calcularErrores(targetAngles, realAngles);

  Serial.println("Movimientos completados.");
}

// Función para leer los ángulos reales desde los potenciómetros
void leerAngulosReales(float realAngles[4]) {
  realAngles[0] = analogRead(ADC0) * (180.0 / 1023.0); // Servo 1
  realAngles[1] = analogRead(ADC3) * (180.0 / 1023.0); // Servo 2
  realAngles[2] = analogRead(ADC6) * (180.0 / 1023.0); // Servo 3
  realAngles[3] = analogRead(ADC7) * (180.0 / 1023.0); // Servo 4

  // Mostrar los ángulos reales
  Serial.println("Valores reales medidos:");
  Serial.print("Theta1 real: ");
  Serial.print(realAngles[0]);
  Serial.println("°");
  Serial.print("Theta2 real: ");
  Serial.print(realAngles[1]);
  Serial.println("°");
  Serial.print("Theta3 real: ");
  Serial.print(realAngles[2]);
  Serial.println("°");
  Serial.print("Theta4 real: ");
  Serial.print(realAngles[3]);
  Serial.println("°");
}

// Función para calcular y mostrar errores
void calcularErrores(float targetAngles[4], float realAngles[4]) {
  Serial.println("Errores entre valores calculados y reales:");
  for (int i = 0; i < 4; i++) {
    float error = abs(targetAngles[i] * 180 / 3.14159265359 - realAngles[i]);
    Serial.print("Error en Theta");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(error);
    Serial.println("°");
  }
}

// Función para ejecutar una secuencia de un cuadrado en el eje X-Y
void ejecutarSecuenciaCuadrado() {
  Serial.println("Ejecutando secuencia de cuadrado en X-Y...");

  // Mantener Z constante en 10
  float pz = 10.0;

  // Definir los puntos del cuadrado (en el plano X-Y)
  float puntos[4][2] = {
    {5, 5},  // Punto 1
    {10, 5}, // Punto 2
    {10, 10}, // Punto 3
    {5, 10}  // Punto 4
  };

  // Recorrer cada punto y mover el brazo
  for (int i = 0; i < 4; i++) {
    float pos[3] = {puntos[i][0], puntos[i][1], pz};
    calcularCinematicaInversa(pos);
    delay(1000); // Pausa de 1 segundo entre movimientos
  }

  // Volver al primer punto para cerrar el cuadrado
  float pos[3] = {puntos[0][0], puntos[0][1], pz};
  calcularCinematicaInversa(pos);

  Serial.println("Secuencia de cuadrado completada.");
}

// Función para mover un servo a un ángulo específico en radianes
void moverServo(int servoIndex, float angleRad) {
  // Convertir radianes a grados
  float angleDeg = angleRad * (180.0 / 3.14159265359);

  // Ajustar ángulo a rango positivo alrededor de 90°
  if (angleDeg < 0) {
    angleDeg = 90 - angleDeg;
    if (angleDeg > 180) angleDeg = 180;
  }

  // Mapear el ángulo al pulso del servo
  int pulseLength = map(angleDeg, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoChannels[servoIndex], 0, pulseLength);

  // Actualizar el ángulo actual
  currentAngles[servoIndex] = angleRad;

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
