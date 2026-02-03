# ByteSight: Dispositivo Vestível de Assistência e Segurança

## Autores
Este projeto foi desenvolvido como requisito avaliativo para a discuplina de Projeto Integrado de Computação I do curso de Engenharia de Computação - UFES.

* **João Paulo Pollesi** - Desenvolvimento de Hardware e Software
* **João Vitor Teixeira** - Desenvolvimento de Hardware e Software
* **Djalma Salvador** - Desenvolvimento de Hardware e Software
* 
* **Prof. Jadir Eduardo Souza Lucas** - Orientador

## Resumo
O ByteSight é um dispositivo vestível (wearable) desenvolvido na forma de uma manopla, projetado para auxiliar pessoas com deficiência visual na navegação espacial e na segurança pessoal. O sistema integra sensores de distância baseados em ultrassom e laser (Time-of-Flight) para fornecer feedback háptico (vibração) sobre obstáculos, além de um sistema de geolocalização e comunicação via Bluetooth para envio de alertas de emergência.

## Tabela de Conteúdos
1. [Conceito e Motivação](#conceito-e-motivação)
2. [Funcionalidades](#funcionalidades)
3. [Arquitetura de Hardware](#arquitetura-de-hardware)
4. [Lista de Materiais](#lista-de-materiais)
5. [Evolução do Projeto e Desafios](#evolução-do-projeto-e-desafios)
6. [Instruções de Uso](#instruções-de-uso)
7. [Autores](#autores)

## Conceito e Motivação
A mobilidade urbana para pessoas com deficiência visual apresenta desafios que vão além do que a bengala tátil tradicional pode detectar, especialmente obstáculos na altura do tronco e da cabeça (como orelhões, placas e galhos de árvores). Além disso, a segurança pessoal é uma preocupação constante.

O ByteSight foi concebido para atuar como um complemento à bengala, oferecendo:
* **Percepção Estendida:** Detecção de obstáculos aéreos e laterais que a bengala não toca.
* **Segurança Ativa:** Um mecanismo rápido para comunicar localização em situações de perigo ou desorientação.

## Funcionalidades

### 1. Detecção de Obstáculos Híbrida
O sistema utiliza dois tipos de tecnologia para mapear o ambiente:
* **Detecção Frontal de Alta Precisão:** Um sensor Laser (LiDAR/ToF) monitora obstáculos à frente com precisão milimétrica e resposta rápida.
* **Detecção Lateral Periférica:** Sensores ultrassônicos monitoram as laterais do usuário, auxiliando na passagem por portas e corredores estreitos.

### 2. Feedback Háptico
A comunicação com o usuário é não-visual e não-auditiva (para não bloquear a audição, sentido crucial para cegos). Um motor de vibração atua com intensidades ou padrões diferentes dependendo da proximidade do objeto, criando um "tato virtual" à distância.

### 3. Sistema de Pânico e Geolocalização
Em caso de emergência, o usuário pode acionar um botão físico na manopla. O sistema captura as coordenadas globais (Latitude/Longitude) via satélite (GPS) e as transmite via Bluetooth para um aplicativo no smartphone, gerando um link de localização imediato (Google Maps).

## Arquitetura de Hardware

O projeto é controlado por um microcontrolador ESP32, escolhido por sua capacidade de processamento dual-core e conectividade Bluetooth/Wi-Fi integrada.

### Pinagem (Pinout)
*Consulte o código fonte (`.ino`) para a definição exata dos pinos GPIO utilizados na versão final.*

* **Processamento:** ESP32 DevKit V1
* **Sensor Frontal:** VL53L0X (Protocolo I2C)
* **Sensores Laterais:** 2x HC-SR04 (Protocolo Digital Trigger/Echo)
* **Geolocalização:** Módulo GPS NEO-6M (Protocolo Serial/UART)
* **Atuador:** Motor de Vibração DC (controlado via Transistor NPN)
* **Interface:** Botão Tátil (Push Button) com resistor Pull-down

## Lista de Materiais

| Componente | Quantidade | Descrição |
| :--- | :---: | :--- |
| ESP32 DevKit V1 | 1 | Microcontrolador principal |
| Sensor VL53L0X | 1 | Sensor de distância a Laser (Time-of-Flight) |
| Sensor HC-SR04 | 2 | Sensor de distância Ultrassônico |
| Módulo GPS NEO-6M | 1 | Receptor de satélite para geolocalização |
| Motor de Vibração | 1 | Feedback háptico (tipo moeda ou cilíndrico) |
| Transistor NPN | 1 | BD139 (Driver do motor) |
| Diodo Retificador | 1 | 1N4007 (Proteção contra corrente reversa) |
| Resistores | Diversos | Divisores de tensão e pull-down |
| Push Button | 1 | Acionamento de emergência |
| Power Bank | 1 | Fonte de alimentação portátil (5V) |
| Protoboard/PCB | 1 | Base para conexão dos componentes |

## Evolução do Projeto e Desafios

Durante o desenvolvimento, o projeto sofreu iterações importantes para garantir confiabilidade:

### 1. Substituição do Sensor Principal
Inicialmente, planejou-se o uso exclusivo de sensores ultrassônicos. No entanto, testes práticos mostraram que o ultrassom sofria com reflexões imprecisas em superfícies irregulares e tinha um cone de abertura muito amplo para a detecção frontal precisa. A solução foi adotar o sensor a laser **VL53L0X**, que oferece um feixe diretivo e maior estabilidade de leitura.

### 2. Filtragem de Ruído (Debounce e Threshold)
Um dos maiores desafios técnicos foi o "falso positivo" gerado por fios do próprio circuito entrando no campo de visão dos sensores ou por ruído elétrico. Foi implementada uma lógica de software para ignorar leituras abaixo de uma distância mínima de segurança (filtro de ruído) e reduzir a sensibilidade máxima para evitar a detecção do solo quando o usuário está com o braço relaxado.

### 3. Gestão de Energia
A integração de múltiplos sensores (especialmente o GPS e o Laser) exigiu um planejamento cuidadoso da distribuição de corrente na protoboard, necessitando de pontes de alimentação robustas para evitar quedas de tensão que reiniciavam o microcontrolador.

## Instruções de Uso

1.  **Inicialização:** Conecte o cabo USB do Power Bank ao ESP32. O sistema vibrará três vezes para indicar que foi iniciado com sucesso.
2.  **Conexão Bluetooth:** No smartphone, pareie com o dispositivo `ByteSight_Manopla` e utilize um terminal Bluetooth serial.
3.  **Operação Normal:** Aponte a manopla para o caminho. O dispositivo vibrará ao detectar obstáculos dentro do raio de ação (configurado para ~60cm frontal e ~25cm lateral).
4.  **Modo de Emergência:** Pressione o botão de pânico por 1 segundo. Uma vibração longa confirmará o envio das coordenadas GPS para o terminal conectado.

---
