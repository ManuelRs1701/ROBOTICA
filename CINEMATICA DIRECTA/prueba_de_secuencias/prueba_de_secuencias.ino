#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 102 // Valor mínimo corregido
#define SERVOMAX 512 // Valor máximo corregido
#define SERVONUM 0   // Canal del PCA9685 donde está conectado el servo

// Pines
#define POT_PIN 36      // Pin ADC0 (GPIO36) conectado al potenciómetro del servo
#define BUTTON1_PIN 25  // Botón 1 para guardar posición
#define BUTTON2_PIN 26  // Botón 2 para iniciar secuencia
#define BUTTON3_PIN 27  // Botón 3 para borrar posiciones
#define SDA_PIN 21      // Pin SDA del I2C
#define SCL_PIN 22      // Pin SCL del I2C

// Variables
int positions[3] = {0, 0, 0}; // Array para guardar posiciones
int positionIndex = 0;        // Índice actual de posición
bool playSequence = false;    // Bandera para iniciar secuencia

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Configurar pines I2C
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia para servos

  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);

  Serial.println("Sistema iniciado. ¡Listo para funcionar!");
}

void loop() {
  // Leer botones
  if (digitalRead(BUTTON1_PIN) == LOW) {
    desactivarServo(); // Permitir mover el servo manualmente
    delay(500);        // Tiempo para mover manualmente
    guardarPosicion();
    delay(300);        // Debounce para evitar múltiples lecturas
  }

  if (digitalRead(BUTTON2_PIN) == LOW) {
    iniciarSecuencia();
    delay(300);
  }

  if (digitalRead(BUTTON3_PIN) == LOW) {
    borrarPosiciones();
    delay(300);
  }

  if (playSequence) {
    reproducirSecuencia();
    playSequence = false;
  }
}

// Función para leer el ángulo del servo basado en el potenciómetro
int leerAngulo() {
  int potValue = analogRead(POT_PIN); // Leer valor del potenciómetro
  int angle = map(potValue, 0, 4095, 0, 180); // Mapear valor a rango de grados
  Serial.print("Ángulo leído: ");
  Serial.println(angle);
  return angle;
}

// Función para mover el servo a un ángulo específico
void moverServo(int angle) {
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(SERVONUM, 0, pulseLength);
  Serial.print("Moviendo servo a: ");
  Serial.println(angle);
}

// Función para desactivar el PWM del servo
void desactivarServo() {
  pwm.setPWM(SERVONUM, 0, 0); // Apagar el canal del servo
  Serial.println("Servo desactivado. Puedes moverlo manualmente.");
}

// Función para guardar una posición
void guardarPosicion() {
  if (positionIndex < 3) {
    int angle = leerAngulo();
    positions[positionIndex] = angle;
    Serial.print("Posición guardada: ");
    Serial.println(angle);
    positionIndex++;
  } else {
    Serial.println("Memoria llena. Usa el botón 3 para borrar posiciones.");
  }
}

// Función para iniciar la secuencia de posiciones
void iniciarSecuencia() {
  if (positionIndex > 0) {
    Serial.println("Iniciando secuencia...");
    playSequence = true;
  } else {
    Serial.println("No hay posiciones guardadas.");
  }
}

// Función para reproducir las posiciones guardadas
void reproducirSecuencia() {
  for (int i = 0; i < positionIndex; i++) {
    Serial.print("Reproduciendo posición: ");
    Serial.println(positions[i]); // Verificar posición guardada
    moverServo(positions[i]);
    delay(1000); // Pausa de 1 segundo entre movimientos
  }
}

// Función para borrar posiciones guardadas
void borrarPosiciones() {
  for (int i = 0; i < 3; i++) {
    positions[i] = 0;
  }
  positionIndex = 0;
  Serial.println("Memoria borrada. Puedes guardar nuevas posiciones.");
}
