#include <PinChangeInterrupt.h>
#include <TimerOne.h>

#define Encoder 2
#define Encoder_B 8
#define IN1 3
#define IN2 11

float Kp_ = 3.11; //1.06;
float Ki_ = 33; //8.14;
float Kd_ = 0.00388; //0.0207;

float Kp = 2.68; //1.06;
float Ki = 40.9; //8.14;
float Kd = 0.00767; //0.0207;

float ts = 0.015;//0.025;
float q0 = (Kp+((Ki*ts)/2)+(Kd/ts));
float q1 = -(Kp-((Ki*ts)/2)+((2*Kd)/ts));
float q2 = (Kd/ts);

volatile float error[3]={0, 0, 0};
volatile float PID[3]={0, 0, 0};

volatile long pulseCount = 0; // Contador de pulsos del encoder
long lastPulseCount = 0; // Almacenar el último valor de pulseCount para calcular RPM
unsigned long lastRPMCalculation = 0; // Última vez que se calculó el RPM
unsigned long lastTimeCalculation = 0; 
volatile float rpm = 0; // Velocidad en RPM
int numero=0;
volatile long encoderPos = 0;
volatile long lastEncoderPos = 0;

int outMax = 255;
volatile float u_ = 0;
volatile float output = 0;

void setup() {
  Serial.begin(115200);
  pinMode(Encoder, INPUT_PULLUP);
  pinMode(Encoder_B, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachPCINT(digitalPinToPCINT(Encoder), encoder, CHANGE); // Detect both rising and falling edges
  attachPCINT(digitalPinToPCINT(Encoder_B), encoder, CHANGE);
  Timer1.initialize(ts*1000000); 
  Timer1.attachInterrupt(calculateRPM);
  pwmOut(0);
}

void loop() {
  if (Serial.available() > 0) {
    int nuevoNumero = Serial.parseInt();
    if (nuevoNumero != 0) {
      numero = nuevoNumero;
      Serial.print("Número recibido: ");
      Serial.println(numero);
    }
  }
  if (millis() - lastRPMCalculation >= ts*1000) {
    //Serial.print(0);Serial.print(",");Serial.print(rpm);Serial.print(",");Serial.print(numero);Serial.print(",");Serial.println(100);
    Serial.println(encoderPos);
    lastRPMCalculation = millis();
  }
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

void calculateRPM() {
  long pulses = encoderPos - lastPulseCount;
  rpm = (pulses * 60.0) / (4.0*2250.0*ts); // Reemplaza PULSES_PER_REVOLUTION con el número de pulsos por revolución de tu encoder 400 //2140
  lastPulseCount = encoderPos;
  ControladorV2();
}


void Controlador(){
  volatile float setpoint = numero;
  error[0] = setpoint - rpm;
  PID[2] = error[0]; 
  PID[1] = (PID[1]+(error[0]*ts));
  PID[0] = (error[0]-error[1])/ts;
  output = Kp_*PID[2]+Ki_*PID[1]+Kd_*PID[0]; //Kp*P + Ki*I + Kd*D
  error[1]=error[0];
  if(output>outMax) pwmOut(int(outMax));
  else if(output>=0 && output<= outMax) pwmOut(int(output));
  else pwmOut(int(0));
}

void ControladorV2(){
  float setpoint = numero;
  error[0] = setpoint - rpm;
  u_ = u_ + q0*error[0] + q1*error[1] + q2* error[2];
  error[2] = error[1];
  error[1] = error[0];
  if(u_>outMax) pwmOut(int(outMax));
  else if(u_>=0 && u_<= outMax) pwmOut(int(u_));
  else pwmOut(int(0));
}

void pwmOut(int out) {
  analogWrite(IN1, out);
  analogWrite(IN2, 0);
}