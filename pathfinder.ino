/* PROJETO BYTESIGHT - MANOPLA INTELIGENTE (GAUNTLET EDITION)
   Hardware: ESP32 DevKit V1 (Lado Esquerdo Otimizado)
   Sensores: 
     - Laser (Frente): Detecta obstáculos precisos a frente
     - Ultrassons (Laterais): Detectam aproximação na Esquerda/Direita
     - GPS + Botão: Localização e Pânico
   Atuadores: 1x Motor de Vibração
*/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <TinyGPS++.h>
#include "BluetoothSerial.h"

// ==========================================
// 📍 MAPEAMENTO DE PINOS (LADO ESQUERDO)
// ==========================================

// Ultrassom 1 (Lado Esquerdo da Manopla)
#define TRIG_US1 12
#define ECHO_US1 13  // Input (Divisor de Tensão!)

// Ultrassom 2 (Lado Direito da Manopla)
#define TRIG_US2 27
#define ECHO_US2 14  // Input (Divisor de Tensão!)

// Motor de Vibração (Transistor)
#define PIN_MOTOR 26 

// Botão de Pânico (Input Only)
#define PIN_BOTAO 34 

// Laser (I2C Customizado - FRENTE)
#define I2C_SDA 33
#define I2C_SCL 25

// GPS (Serial 2)
#define GPS_RX_PIN_ESP 35 // Liga no TX do GPS
#define GPS_TX_PIN_ESP 32 // Liga no RX do GPS

// ==========================================
// ⚙️ AJUSTES DE DISTÂNCIA (MODO MANOPLA)
// ==========================================

// Distância para o Laser (Frente) começar a apitar
// 600mm = 0,6 metros. Se algo estiver MENOR que isso, vibra pulsante.
const int DIST_FRENTE_MM = 600; 

// Distância para os Ultrassons (Laterais) avisarem
// 25cm. Se alguém chegar muito perto do lado, vibra contínuo.
const int DIST_LADOS_CM = 25; 

// ==========================================
// Ignora qualquer coisa menor que 4cm (geralmente são os fios ou erro do sensor)
const int DIST_MINIMA_FILTRO = 4;

// Objetos Globais
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); 
BluetoothSerial SerialBT;    

// Variáveis de Controle
unsigned long ultimoTempoVibra = 0;
bool estadoMotor = false;
bool botaoPressionadoAnteriormente = false;

void setup() {
  // 1. Iniciar Comunicações
  Serial.begin(115200);
  
  SerialBT.begin("ByteSight_Manopla"); 
  Serial.println("--- MODO MANOPLA INICIADO ---");

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN_ESP, GPS_TX_PIN_ESP);

  // 2. Configurar Pinos
  pinMode(TRIG_US1, OUTPUT); pinMode(ECHO_US1, INPUT);
  pinMode(TRIG_US2, OUTPUT); pinMode(ECHO_US2, INPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_BOTAO, INPUT); 

  // 3. Feedback Tátil de Inicialização (Vibra 3x rápido - Estilo Iron Man)
  for(int i=0; i<3; i++) {
    digitalWrite(PIN_MOTOR, HIGH); delay(80); 
    digitalWrite(PIN_MOTOR, LOW);  delay(80);
  }

  // 4. Iniciar Laser
  Wire.begin(I2C_SDA, I2C_SCL); 
  if (!lox.begin()) {
    Serial.println(F("ERRO: Laser nao encontrado! Verifique fios."));
  } else {
    Serial.println(F("Laser Online!"));
  }
}

void loop() {
  // --- PARTE A: LEITURA DOS SENSORES ---

  // 1. Ler Ultrassons (Laterais)
  int distEsq = lerUltrassom(TRIG_US1, ECHO_US1);
  delay(10); 
  int distDir = lerUltrassom(TRIG_US2, ECHO_US2);

  // 2. Ler Laser (Frente)
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  int distFrente = 9999; // Valor padrão "infinito"
  if (measure.RangeStatus != 4) {
    distFrente = measure.RangeMilliMeter;
  }

  // --- PARTE B: CÉREBRO DA MANOPLA (DETECÇÃO) ---

  // Lógica: 
  // 1. Tem que ser MENOR que o limite (Perto)
  // 2. Tem que ser MAIOR que o filtro (Para ignorar fios/erros de 0 a 4cm)

  // Perigo Frontal (Laser)
  // Nota: O Laser é em Milímetros. 4cm = 40mm.
  bool obstaculoFrente = (distFrente > (DIST_MINIMA_FILTRO * 10) && distFrente < DIST_FRENTE_MM);

  // Perigo Lateral (Esquerda OU Direita) em Centímetros
  bool obstaculoLados = (distEsq > DIST_MINIMA_FILTRO && distEsq < DIST_LADOS_CM) || 
                        (distDir > DIST_MINIMA_FILTRO && distDir < DIST_LADOS_CM);


  // --- PARTE C: FEEDBACK TÁTIL (MOTOR) ---

  if (obstaculoFrente) {
    // ALERTA VERMELHO: FRENTE BLOQUEADA
    // Padrão: Pulsante Rápido (Zzz.. Zzz.. Zzz..)
    unsigned long tempoAtual = millis();
    if (tempoAtual - ultimoTempoVibra >= 100) { 
      ultimoTempoVibra = tempoAtual;
      estadoMotor = !estadoMotor;
      digitalWrite(PIN_MOTOR, estadoMotor);
    }
  } 
  else if (obstaculoLados) {
    // ALERTA AMARELO: LATERAIS FECHADAS
    // Padrão: Contínuo (Zzzzzzzzzz)
    digitalWrite(PIN_MOTOR, HIGH);
  } 
  else {
    // CAMINHO LIVRE
    digitalWrite(PIN_MOTOR, LOW);
  }

  // --- PARTE D: GPS E BOTÃO ---

  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());

  bool botaoAgora = digitalRead(PIN_BOTAO);
  
  // Se apertou o botão
  if (botaoAgora == HIGH && botaoPressionadoAnteriormente == LOW) {
    enviarLocalizacaoBluetooth();
    delay(200); 
  }
  botaoPressionadoAnteriormente = botaoAgora;

  delay(20); 
}

// --- FUNÇÕES AUXILIARES ---

void enviarLocalizacaoBluetooth() {
  // Vibra confirmando envio
  digitalWrite(PIN_MOTOR, HIGH); delay(200); digitalWrite(PIN_MOTOR, LOW);

  if (gps.location.isValid()) {
    String link = "http://maps.google.com/?q=";
    link += String(gps.location.lat(), 6);
    link += ",";
    link += String(gps.location.lng(), 6);

    SerialBT.println("\n--- AJUDA MANOPLA ---");
    SerialBT.println(link);
    SerialBT.println("---------------------\n");
  } else {
    SerialBT.println("\n[MANOPLA] Botao ativado. Aguardando GPS...\n");
  }
}

int lerUltrassom(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 30000); 
  if (dur == 0) return 999; 
  return dur * 0.034 / 2;
}