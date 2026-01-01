#include "PID_t.h"

const int resistorPin = A5;   // entrada analógica (fotoresistor)
const int ledPin = 3;         // saída PWM (LED)
const int potPin = A0;        // potenciômetro para definir setpoint

PID_t pid;

double Kp = 1;
double Ki = 4;
double Kd = 0.0;

void setup() {
  Serial.begin(57600);
  pinMode(resistorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(potPin, INPUT);

  pid.DefConstantes(Kp, Ki, Kd);
  pid.DefIntervalo(1);
  pid.DefModo(AUTOMATICO);
  pid.DefTipo(ABSOLUTO);
  pid.DefSentido(DIRETO);
  pid.DefEstilo(CLASSICO);
  pid.DefLimitesSaida(0, 255);
}

void loop() {
  int leitura = analogRead(resistorPin); // Luz 
  int leitura_padronizada = map(leitura, 0, 1023, 0, 255);

  int alvo = analogRead(potPin); // Potenciômetro define setpoint
  int alvo_padronizado = map(alvo, 0, 1023, 0, 255);

  pid.DefInput(leitura_padronizada);
  pid.DefSetpoint(alvo_padronizado);

  pid.Compute();

  analogWrite(ledPin, pid.LerOutput());

  // Depuração
  Serial.print("Entrada: ");
  Serial.print(leitura_padronizada);
  Serial.print(" | Setpoint: ");
  Serial.print(alvo_padronizado);
  Serial.print(" | Saída PID: ");
  Serial.println(pid.LerOutput());
}
