/* * PROJETO BYTESIGHT - DISPOSITIVO HÁPTICO DE ASSISTÊNCIA (VERSÃO MANOPLA)
 * * Descrição:
 * Este firmware controla um dispositivo vestível baseado no microcontrolador ESP32,
 * destinado a auxiliar na navegação espacial de pessoas com deficiência visual.
 * O sistema integra sensores de distância (Laser e Ultrassom) para detecção de obstáculos
 * e um módulo GPS para geolocalização em situações de emergência.
 * * Hardware:
 * - Microcontrolador: ESP32 DevKit V1
 * - Sensor Frontal: VL53L0X (Time-of-Flight)
 * - Sensores Laterais: HC-SR04 (Ultrassônico)
 * - Atuador: Motor de Vibração DC (via Transistor)
 * - Interface: Botão Tátil e Módulo GPS NEO-6M
 * * Autor: João Paulo Pollesi Santana
 * Data: Fevereiro de 2026
 */

// ==========================================
// 1. INCLUSÃO DE BIBLIOTECAS
// ==========================================

// Wire.h: Responsável pela comunicação I2C (Inter-Integrated Circuit),
// necessária para o sensor a laser VL53L0X.
#include <Wire.h>

// Adafruit_VL53L0X.h: Driver de alto nível para controle do sensor VL53L0X.
// Abstrai a complexidade dos registros I2C do sensor.
#include <Adafruit_VL53L0X.h>

// TinyGPS++.h: Biblioteca para análise (parsing) de sentenças NMEA
// enviadas pelo módulo GPS, facilitando a extração de latitude e longitude.
#include <TinyGPS++.h>

// BluetoothSerial.h: Implementa a pilha Bluetooth Classic (SPP - Serial Port Profile)
// no ESP32, permitindo comunicação serial sem fio com smartphones.
#include "BluetoothSerial.h"

// ==========================================
// 2. DEFINIÇÃO DE HARDWARE E PINAGEM
// ==========================================

// Sensores Ultrassônicos (Detecção Lateral)
// O Trigger envia o pulso e o Echo recebe o retorno.
// Nota: O pino Echo requer um divisor de tensão (5V -> 3.3V) para proteção do ESP32.
#define PIN_TRIG_ESQ 12
#define PIN_ECHO_ESQ 13 

#define PIN_TRIG_DIR 27
#define PIN_ECHO_DIR 14 

// Atuadores e Interface
#define PIN_MOTOR_VIBRACAO 26 // Controlado via transistor (NPN)
#define PIN_BOTAO_PANICO 34   // Configurado como Input (Pull-down externo recomendado)

// Sensor Laser VL53L0X (Detecção Frontal)
// Pinos definidos para o barramento I2C secundário ou remapeado.
#define I2C_SDA_PIN 33
#define I2C_SCL_PIN 25

// Módulo GPS (Comunicação Serial UART)
// RX do ESP conecta ao TX do GPS e vice-versa.
#define PIN_GPS_RX 35 
#define PIN_GPS_TX 32 

// ==========================================
// 3. CONSTANTES E PARÂMETROS DE OPERAÇÃO
// ==========================================

// Limiares de Detecção (Thresholds)
// Define a distância máxima para considerar um obstáculo como perigoso.

// Distância frontal (em milímetros): 600mm = 0.6 metros.
// Valor calibrado para detecção ao alcance do braço estendido.
const int LIMIAR_FRONTAL_MM = 600; 

// Distância lateral (em centímetros): 25cm.
// Valor calibrado para proteção corporal em passagens estreitas.
const int LIMIAR_LATERAL_CM = 25; 

// Filtro de Ruído Mínimo (em centímetros): 4cm.
// Leituras abaixo deste valor são descartadas para evitar falsos positivos
// causados por interferência mecânica (fios) ou ruído do sensor.
const int DISTANCIA_MINIMA_FILTRO_CM = 4;

// Parâmetros de Tempo e Física
// Velocidade do som no ar (ao nível do mar, ~20°C) em cm/us.
// Utilizado para cálculo de distância do sensor ultrassônico.
const float VELOCIDADE_SOM = 0.034; 

// Intervalo de vibração pulsante (em milissegundos).
// Controla a frequência do feedback háptico para alertas frontais.
const unsigned long INTERVALO_VIBRACAO_MS = 100;

// ==========================================
// 4. INSTÂNCIA DE OBJETOS GLOBAIS
// ==========================================

Adafruit_VL53L0X sensorLaser = Adafruit_VL53L0X();
TinyGPSPlus gpsParser;
HardwareSerial gpsSerialPort(2); // Utiliza a UART2 do ESP32
BluetoothSerial interfaceBluetooth;    

// Variáveis de Controle de Estado
unsigned long ultimoTempoVibracao = 0; // Timestamp para controle não-bloqueante (millis)
bool estadoMotorAtivo = false;         // Estado atual do motor (Ligado/Desligado)
bool estadoBotaoAnterior = false;      // Para detecção de borda de subida (debounce simples)

// ==========================================
// 5. CONFIGURAÇÃO INICIAL (SETUP)
// ==========================================
void setup() {
  // Inicialização da Serial de Depuração (USB)
  // Baud rate de 115200 bps para comunicação rápida.
  Serial.begin(115200);
  
  // Inicialização do Bluetooth
  // O dispositivo será visível como "ByteSight_Manopla".
  interfaceBluetooth.begin("ByteSight_Manopla"); 
  Serial.println("--- SISTEMA INICIADO: MODO OPERACIONAL ---");

  // Inicialização da Serial do GPS
  // Baud rate de 9600 bps é o padrão para módulos NMEA (como o NEO-6M).
  gpsSerialPort.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  // Configuração dos Pinos de I/O
  pinMode(PIN_TRIG_ESQ, OUTPUT); pinMode(PIN_ECHO_ESQ, INPUT);
  pinMode(PIN_TRIG_DIR, OUTPUT); pinMode(PIN_ECHO_DIR, INPUT);
  pinMode(PIN_MOTOR_VIBRACAO, OUTPUT);
  pinMode(PIN_BOTAO_PANICO, INPUT); 

  // Sequência de Teste de Hardware (Feedback Tátil)
  // Vibra o motor 3 vezes rapidamente para confirmar a inicialização ao usuário.
  for(int i = 0; i < 3; i++) {
    digitalWrite(PIN_MOTOR_VIBRACAO, HIGH); delay(80); 
    digitalWrite(PIN_MOTOR_VIBRACAO, LOW);  delay(80);
  }

  // Inicialização do Sensor Laser (I2C)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
  if (!sensorLaser.begin()) {
    Serial.println(F("ERRO CRÍTICO: Sensor Laser não detectado. Verifique conexões I2C."));
  } else {
    Serial.println(F("Status: Sensor Laser Online."));
  }
}

// ==========================================
// 6. LOOP PRINCIPAL
// ==========================================
void loop() {
  // --- ETAPA A: AQUISIÇÃO DE DADOS ---

  // Leitura dos Sensores Ultrassônicos (Laterais)
  int distanciaEsq = medirDistanciaUltrassom(PIN_TRIG_ESQ, PIN_ECHO_ESQ);
  delay(10); // Pequeno atraso para evitar interferência acústica entre sensores
  int distanciaDir = medirDistanciaUltrassom(PIN_TRIG_DIR, PIN_ECHO_DIR);

  // Leitura do Sensor Laser (Frontal)
  VL53L0X_RangingMeasurementData_t dadosLaser;
  sensorLaser.rangingTest(&dadosLaser, false); // false = modo sem debug detalhado
  
  int distanciaFrente = 9999; // Valor inicial "infinito" (fora de alcance)
  
  // Verifica se a leitura é válida (Status 4 indica 'Out of Range' ou erro)
  if (dadosLaser.RangeStatus != 4) {
    distanciaFrente = dadosLaser.RangeMilliMeter;
  }

  // --- ETAPA B: PROCESSAMENTO E LÓGICA DE DECISÃO ---

  // Detecção de Obstáculo Frontal
  // Critério: Distância deve ser menor que o limiar máximo E maior que o filtro mínimo (em mm).
  bool obstaculoDetectadoFrente = (distanciaFrente > (DISTANCIA_MINIMA_FILTRO_CM * 10) && 
                                   distanciaFrente < LIMIAR_FRONTAL_MM);

  // Detecção de Obstáculo Lateral
  // Critério: Distância válida em qualquer um dos lados (Esquerda OU Direita).
  bool obstaculoDetectadoLados = (distanciaEsq > DISTANCIA_MINIMA_FILTRO_CM && distanciaEsq < LIMIAR_LATERAL_CM) || 
                                 (distanciaDir > DISTANCIA_MINIMA_FILTRO_CM && distanciaDir < LIMIAR_LATERAL_CM);


  // --- ETAPA C: CONTROLE DO ATUADOR (FEEDBACK HÁPTICO) ---

  if (obstaculoDetectadoFrente) {
    // Cenário: Obstrução Frontal
    // Ação: Vibração Pulsante (Alerta de Alta Prioridade).
    // Utiliza a função millis() para não bloquear o processador.
    unsigned long tempoAtual = millis();
    if (tempoAtual - ultimoTempoVibracao >= INTERVALO_VIBRACAO_MS) { 
      ultimoTempoVibracao = tempoAtual;
      estadoMotorAtivo = !estadoMotorAtivo; // Alterna estado (ON/OFF)
      digitalWrite(PIN_MOTOR_VIBRACAO, estadoMotorAtivo);
    }
  } 
  else if (obstaculoDetectadoLados) {
    // Cenário: Obstrução Lateral
    // Ação: Vibração Contínua (Alerta de Proximidade Periférica).
    digitalWrite(PIN_MOTOR_VIBRACAO, HIGH);
  } 
  else {
    // Cenário: Caminho Livre
    // Ação: Desligar motor.
    digitalWrite(PIN_MOTOR_VIBRACAO, LOW);
  }

  // --- ETAPA D: SISTEMA DE EMERGÊNCIA (GPS E BOTÃO) ---

  // Processamento contínuo dos dados do GPS
  while (gpsSerialPort.available() > 0) {
    gpsParser.encode(gpsSerialPort.read());
  }

  // Leitura do Botão de Pânico
  bool estadoBotaoAtual = digitalRead(PIN_BOTAO_PANICO);
  
  // Detecção de Borda de Subida (Pressionamento)
  if (estadoBotaoAtual == HIGH && estadoBotaoAnterior == LOW) {
    executarProtocoloEmergencia();
    delay(200); // Debounce simples via software
  }
  estadoBotaoAnterior = estadoBotaoAtual;

  delay(20); // Ciclo de estabilização do loop
}

// ==========================================
// 7. FUNÇÕES AUXILIARES
// ==========================================

/**
 * Função: executarProtocoloEmergencia
 * Descrição: Compila as coordenadas atuais do GPS e envia um link formatado
 * via Bluetooth para o dispositivo pareado. Fornece feedback tátil de envio.
 */
void executarProtocoloEmergencia() {
  // Feedback tátil curto: Confirmação de acionamento
  digitalWrite(PIN_MOTOR_VIBRACAO, HIGH); delay(200); digitalWrite(PIN_MOTOR_VIBRACAO, LOW);

  if (gpsParser.location.isValid()) {
    // Formatação do Link Google Maps
    String linkMapa = "http://maps.google.com/?q=";
    linkMapa += String(gpsParser.location.lat(), 6); // Precisão de 6 casas decimais
    linkMapa += ",";
    linkMapa += String(gpsParser.location.lng(), 6);

    interfaceBluetooth.println("\n--- ALERTA DE LOCALIZAÇÃO ---");
    interfaceBluetooth.println(linkMapa);
    interfaceBluetooth.println("-----------------------------\n");
  } else {
    // Caso o GPS ainda não tenha fixado satélites (Cold Start)
    interfaceBluetooth.println("\n[SISTEMA] Botão acionado. Aguardando fixação de satélites GPS...\n");
  }
}

/**
 * Função: medirDistanciaUltrassom
 * Parâmetros: trigPin (Pino de Gatilho), echoPin (Pino de Leitura)
 * Retorno: Distância medida em centímetros (int)
 * Descrição: Envia um pulso ultrassônico de 10us e mede o tempo de retorno.
 */
int medirDistanciaUltrassom(int trigPin, int echoPin) {
  // Garante que o trigger está baixo
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  
  // Envia pulso de 10 microssegundos
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Mede a duração do pulso em nível ALTO no pino Echo
  // Timeout de 30ms (suficiente para ~5 metros)
  long duracao = pulseIn(echoPin, HIGH, 30000); 
  
  // Se houver timeout (duracao 0), retorna valor alto para indicar "sem obstáculo"
  if (duracao == 0) return 999; 
  
  // Cálculo: Distância = (Tempo * Velocidade do Som) / 2 (Ida e Volta)
  return duracao * VELOCIDADE_SOM / 2;
}
