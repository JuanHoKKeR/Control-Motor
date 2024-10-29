/*
 * Control de Posición
 * Autor: Juan David Cruz Useche
 * Descripción: Este código permite controlar la posición de un motor DC con un encoder cuadraturo.
 * 
 * Componentes:
 * - Arduino UNO
 * - Motor DC
 * - Encoder cuadraturo
 * 
 * Licencia: Este código es de libre uso y distribución bajo la licencia MIT (modificar según sea necesario).
 */
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <TimerOne.h>
//Motor Grande Pines:
//Naranja Encoder_B
//Blanco Encoder_A
//Verde GND
//Gris 5V
#define Encoder      2 // Quadrature encoder A pin
#define Encoder_B    8 //Amarillo--> Cafe
#define IN1          3
#define IN2          11
#define END_stop1    13
//-------Variables Controlador-------------
float kp = 6;
float ki = 0.02, kd = 0.02;
float numero;
float input, output, setpoint;
float iTerm = 0, lastInput = 0, dInput = 0, error = 0;
int outMin = -200, outMax = 200;
int sampleTime = 10; // This value is in milliseconds
//----------------------------------------
volatile long encoderPos = 0;
volatile long lastEncoderPos = 0; // Variable to store the previous encoder position
//----------------------------------------
const int VALOR_PISO_1 = 1;
const int VALOR_PISO_2 = 2900;
const int VALOR_PISO_3 = 5600;
const int VALOR_PISO_4 = 8300;
int pisoActual = 1;
int pisoObjetivo = 1;
//----------------------------------------
float piso1=0;
float piso2=0;
float piso3=0;
float piso4=0;
int valor1=0;
int valor2=0;
int valor3=0;
int valor4=0;

void setup() {
  Serial.begin(115200);
  pinMode(Encoder, INPUT);
  pinMode(Encoder_B, INPUT);
  pinMode(END_stop1, INPUT);
  //----------------------------------------
  attachPCINT(digitalPinToPCINT(Encoder), encoder, CHANGE); // Detect both rising and falling edges
  attachPCINT(digitalPinToPCINT(Encoder_B), encoder, CHANGE);
  //----------------------------------------
  Timer1.initialize(sampleTime * 1000); // Setup Timer1
  Timer1.attachInterrupt(moveTo);
}

void loop() {
  Serial.print("Pasos: ");
  Serial.println(encoderPos);

  if (Serial.available() > 0) {
    int nuevoNumero = Serial.parseInt();
    if (nuevoNumero != 0) {
      numero = nuevoNumero;
      Serial.print("Número recibido: ");
      Serial.println(numero);
    }
  }
  Serial.print("Número:   ");
  Serial.println(numero);
  delay(100);
}

void encoder() {
  int MSB = digitalRead(Encoder);
  int LSB = digitalRead(Encoder_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoderPos << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos--;
  }

  lastEncoderPos = encoded; // Update the last encoder position
}

void moveTo() {
  setpoint = numero;
  input = encoderPos;
  error = setpoint - input;
  iTerm += ki * error * sampleTime;
  if (iTerm > outMax) iTerm = outMax;
  else if (iTerm < outMin) iTerm = outMin;
  dInput = (input - lastInput) / sampleTime;
  output = kp * error + iTerm - kd * dInput;
  if (output > outMax) output = outMax;
  else if (output < outMin) output = outMin;
  lastInput = input;
  pwmOut(output);
}

void pwmOut(int out) {
  if (out > 0) {
    analogWrite(IN1, out);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(out));
  }
}


