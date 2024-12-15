#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 120 // Valor mínimo corregido
#define SERVOMAX 540 // Valor máximo corregido
#define SERVONUM 0   // Canal del PCA9685 donde está conectado el servo

// Pines
#define BUTTON2_PIN 26  // Botón para iniciar secuencia
#define BUTTON3_PIN 27  // Botón para borrar posiciones
#define SDA_PIN 21      // Pin SDA del I2C
#define SCL_PIN 22      // Pin SCL del I2C

// Variables
int positions[3] = {180, 0, 90}; // Ángulos definidos en el código
int positionIndex = 3;           // Número de posiciones guardadas
bool playSequence = false;       // Bandera para iniciar secuencia

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Configurar pines I2C
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia para servos

  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);

  desactivarServo(); // Asegurar que el servo esté desactivado al inicio

  Serial.println("Sistema iniciado. Servo desactivado.");
}

void loop() {
  // Leer botón para iniciar secuencia
  if (digitalRead(BUTTON2_PIN) == LOW) {
    iniciarSecuencia();
    delay(300);
  }

  // Leer botón para borrar posiciones
  if (digitalRead(BUTTON3_PIN) == LOW) {
    borrarPosiciones();
    delay(300);
  }

  // Ejecutar la secuencia si está activada
  if (playSequence) {
    reproducirSecuencia();
    playSequence = false;
  }
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
  Serial.println("Servo desactivado.");
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
  desactivarServo(); // Apagar el servo al finalizar la secuencia
}

// Función para borrar posiciones guardadas
void borrarPosiciones() {
  for (int i = 0; i < 3; i++) {
    positions[i] = 0;
  }
  positionIndex = 0;
  Serial.println("Memoria borrada. Configura nuevas posiciones en el código.");
}
