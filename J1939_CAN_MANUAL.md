# Manual Completo: CAN Bus e J1939
## Guia Definitivo para Veículos Pesados e Maquinário Agrícola

---

# PARTE 1: CAN BUS — FUNDAMENTOS COMPLETOS

## 1.1 O que é o CAN Bus

O **Controller Area Network (CAN)** é um protocolo de comunicação serial desenvolvido pela Bosch em 1986, projetado especificamente para ambientes automotivos e industriais ruidosos. Tornou-se o padrão dominante para comunicação entre ECUs (Unidades de Controle Eletrônico) em veículos.

### Por que CAN e não Ethernet ou RS-485?

| Característica | CAN | RS-485 | Ethernet |
|---------------|-----|--------|---------|
| Topologia | Multi-drop bus | Multi-drop | Estrela/Switch |
| Arbitração | Sem destruição de dados | Colisão = dados perdidos | CSMA/CD |
| Tempo real | Determinístico | Semi | Não determinístico |
| Tolerância a falha | Alta (bus-off, error frames) | Baixa | Baixa |
| Taxa típica | 125k–1M bps | 100k–10M bps | 10M–10G bps |
| Custo (cabling) | Muito baixo (2 fios) | Baixo | Alto |

---

## 1.2 Camada Física (Physical Layer) — ISO 11898-2

### Diferencial e Topologia

O CAN usa sinalização diferencial em dois fios:
- **CAN_H** (CAN High)
- **CAN_L** (CAN Low)

```
          Terminador         Terminador
          120 Ω              120 Ω
    ┌──────────┐                ┌──────────┐
    │          │                │          │
ECU1│CAN_H ────┼────────────────┼─── CAN_H │ECU5
    │CAN_L ────┼────────────────┼─── CAN_L │
    └──────────┘   │    │    │  └──────────┘
                ECU2  ECU3  ECU4
```

### Níveis de Tensão

```
Estado DOMINANTE (bit '0'):
  CAN_H = 3.5V
  CAN_L = 1.5V
  Diferencial = +2.0V

Estado RECESSIVO (bit '1'):
  CAN_H = 2.5V (terminadores)
  CAN_L = 2.5V
  Diferencial ≈ 0V
```

**Importante**: O barramento em REPOUSO está no estado RECESSIVO. Um bit DOMINANTE sempre "vence" sobre um RECESSIVO — essa é a base da arbitração.

### Terminadores

OBRIGATÓRIO: Dois terminadores de 120Ω nos extremos físicos do barramento.

```
Resistência equivalente: 120 // 120 = 60Ω
Sem terminator: reflexões causam corrupção de dados
Terminator incorreto: comunicação não funciona
```

### Velocidades e Comprimentos Máximos

| Velocidade | Comp. Máximo | Aplicação |
|-----------|-------------|-----------|
| 1 Mbit/s  | 40 m        | ECU interno (J1939 não usa) |
| 500 kbit/s| 100 m       | HS-CAN automotivo (J1939 padrão) |
| 250 kbit/s| 250 m       | J1939 agrícola/off-highway |
| 125 kbit/s| 500 m       | LowSpeed CAN, carroçaria |
| 50 kbit/s | 1000 m      | Indústria, longos cabos |
| 25 kbit/s | 2000 m      | Aplicações especiais |

---

## 1.3 Formato do Frame CAN 2.0

### Frame de Dados (Data Frame)

```
┌─────┬────┬────┬───────────────────┬─────┬────┬─────┬─────┬────┐
│ SOF │ ID │ RTR│ Control (DLC+IDE) │DATA │CRC │ ACK │ EOF │IFS │
└─────┴────┴────┴───────────────────┴─────┴────┴─────┴─────┴────┘
  1b   11b  1b        6b            0-64b  16b   2b    7b   3b
```

Para CAN 2.0B (29-bit Extended ID, usado pelo J1939):
```
┌─────┬────────┬────┬────┬────────┬───┬────┬─────┬────┬─────┬────┐
│ SOF │ ID[28-18]│SRR│IDE│ID[17-0]│RTR│r1 │DLC │DATA│CRC │ACK │EOF│
└─────┴────────┴────┴────┴────────┴───┴────┴─────┴────┴─────┴────┴────┘
  1b    11b     1b   1b   18b     1b   1b   4b   0-64b 15b  2b  7b
```

**Campos:**
- **SOF** (Start Of Frame): Bit dominante — indica início do frame
- **ID**: Identificador (11-bit padrão ou 29-bit estendido)
- **RTR** (Remote Transmission Request): 0=dados, 1=request remoto
- **IDE** (Identifier Extension): 0=11-bit, 1=29-bit
- **DLC** (Data Length Code): 0-8 bytes de dados
- **DATA**: Payload de 0 a 8 bytes
- **CRC**: CRC-15 para detecção de erros
- **ACK**: Todos nós receptores puxam para dominante se receberam OK
- **EOF**: 7 bits recessivos — fim do frame
- **IFS** (Inter-Frame Space): 3 bits mínimo entre frames

### Bits por Frame (CAN 2.0B, 8 bytes dados, sem bit stuffing)

```
SOF:1 + ID29:29 + outros headers:6 + DLC:4 + DATA:64 + CRC:16 + ACK:2 + EOF:7 + IFS:3 = 132 bits
```
Com bit stuffing médio: ≈ 148 bits por frame.

**A 500 kbit/s**: ~500000/148 ≈ **3378 frames/segundo máximo**.

---

## 1.4 Arbitração — Como Múltiplos Nós Compartilham o Barramento

O CAN usa **CSMA/CD com arbitração não-destrutiva baseada em prioridade**.

```
Processo de transmissão:
1. Nó aguarda barramento recessivo (IFS + Bus Idle)
2. Nó começa a transmitir bit a bit
3. Enquanto transmite, MONITORA o barramento
4. Se o bit enviado = bit lido → continua
5. Se enviou RECESSIVO mas leu DOMINANTE → PERDEU arbitração
6. Nó perdedor para transmissão imediatamente (sem dano ao frame)
7. Nó vencedor continua até o fim do frame
```

**Exemplo de arbitração:**

```
Tempo →     b1  b2  b3  b4  b5  b6  b7  ...
Nó A (ID=0x0CF004):  0   0   1   1   0   0   0  ← Transmite
Nó B (ID=0x18FEF0):  0   1   ...                 ← Transmite b2=1, barramento=0 → perde!
Barramento:          0   0   ...                 ← Bit 2 dominante (Nó A vence)
```

**Regra**: ID **menor** = **maior prioridade**.
- 0x0000000 = máxima prioridade
- 0x1FFFFFFF = mínima prioridade

---

## 1.5 Tipos de Frames CAN

### Data Frame
Frame normal com dados.

### Remote Frame (RTR=1)
Solicita que outro nó transmita dados. Não contém payload.

### Error Frame
Gerado automaticamente pelo hardware quando detecta erro:
```
├── Error Flag: 6 bits dominantes (ativo) ou recessivos (passivo)
└── Error Delimiter: 8 bits recessivos
```

### Overload Frame
Indica que o receptor precisa de mais tempo antes do próximo frame.

---

## 1.6 Detecção e Confinamento de Erros

### Tipos de Erro Detectados

| Tipo | Descrição | Detectado por |
|------|-----------|--------------|
| **Bit Error** | Nó monitora e vê bit diferente do enviado | Transmissor |
| **Stuff Error** | 6+ bits consecutivos do mesmo nível | Todos |
| **CRC Error** | CRC calculado ≠ CRC recebido | Receptor |
| **Form Error** | Campo EOF/Ack com bit dominante | Todos |
| **ACK Error** | Nenhum nó reconheceu o frame | Transmissor |

### Contadores de Erro (ISO 11898-1)

Cada nó mantém dois contadores:
- **TEC** (Transmit Error Counter): Incrementa em erros TX
- **REC** (Receive Error Counter): Incrementa em erros RX

```
Regras de incremento/decremento:
TX Error:  TEC += 8  (por cada erro de transmissão)
RX Error:  REC += 1  (por cada erro de recepção)
Sucesso:   TEC -= 1, REC -= 1  (por frame sem erro)
```

### Estados do Nó (Error Confinement)

```
                TEC < 128 AND REC < 128
                        │
                   ERROR ACTIVE  ◄────────────────────┐
              (envia Active Error Flag)                 │
                        │ TEC ≥ 128 OR REC ≥ 128       │ TEC diminui
                        ▼                               │ < 128
                   ERROR PASSIVE ◄─────────────────────┐│
              (envia Passive Error Flag)                 ││
                        │ TEC ≥ 256                     ││
                        ▼                               ││
                      BUS-OFF  ────────────────────────┘┘
              (nó desconecta do barramento)
              Recovery: 128 × 11 bits recessivos
```

---

## 1.7 Bit Stuffing

Para sincronização, o CAN insere um bit oposto após 5 bits consecutivos do mesmo nível:

```
Dados originais: 0 0 0 0 0 1 1 1 1 1 1
Após stuffing:   0 0 0 0 0 1(stuff) 1 1 1 1 1 0(stuff)
```

**Pior caso**: Cada 5 bits → 1 stuff bit = 20% overhead adicional.

---

# PARTE 2: J1939 — PROTOCOLO COMPLETO

## 2.1 Visão Geral do J1939

**J1939** é um conjunto de padrões SAE (Society of Automotive Engineers) que definem:
1. Como usar o CAN para comunicação entre ECUs em veículos pesados
2. Formato de mensagens (PGNs)
3. Parâmetros de engenharia (SPNs)
4. Gerenciamento de rede (endereçamento, diagnóstico)
5. Protocolo de transporte para mensagens > 8 bytes

### Stack de Protocolos J1939

```
┌─────────────────────────────────────────────────┐
│     APLICAÇÃO: PGNs, SPNs, Diagnóstico          │ SAE J1939-71, 73
├─────────────────────────────────────────────────┤
│     GERENCIAMENTO DE REDE: Endereçamento        │ SAE J1939-81
├─────────────────────────────────────────────────┤
│     PROTOCOLO DE TRANSPORTE: Frames > 8 bytes   │ SAE J1939-21
├─────────────────────────────────────────────────┤
│     DATA LINK: CAN 2.0B, 29-bit ID              │ SAE J1939-21
├─────────────────────────────────────────────────┤
│     FÍSICO: CAN ISO 11898, 250/500 kbit/s       │ SAE J1939-11, 15
└─────────────────────────────────────────────────┘
```

---

## 2.2 Estrutura do CAN ID de 29 bits no J1939

```
Bit:   28 27 26 | 25 | 24 | 23 22 21 20 19 18 17 16 | 15 14 13 12 11 10 9 8 | 7 6 5 4 3 2 1 0
       ─────────────────────────────────────────────────────────────────────────────────────────
       Priority  | R  | DP |      PF (PDU Format)      |      PS (PDU Specific)  |     SA
         3 bits  |1b  |1b  |           8 bits           |          8 bits         |    8 bits
```

### Campos Detalhados

#### Priority (Prioridade) — bits 28-26
- **0** = Mais alta prioridade (controle de segurança crítico)
- **3** = Prioridade padrão para parâmetros de controle
- **6** = Prioridade padrão para parâmetros informativos
- **7** = Menor prioridade

```
Prioridade 0: Controle de freio, safety-critical
Prioridade 3: Engine Speed, Throttle (controle)
Prioridade 6: Temperatura, pressão (informativo)
Prioridade 7: Diagnóstico, configuração
```

#### Reserved (R) — bit 25
Sempre 0. Reservado para uso futuro.

#### Data Page (DP) — bit 24
Seleciona entre duas "páginas" de PGNs:
- **DP=0**: PGNs 0-65279 (página 0 — maioria das mensagens)
- **DP=1**: PGNs 65536-131071 (página 1 — extensões J1939)

#### PF (PDU Format) — bits 23-16

| PF Value | Tipo PDU | Descrição |
|----------|----------|-----------|
| 0x00-0xEF | PDU1 | Mensagem peer-to-peer (endereçada) |
| 0xF0-0xFF | PDU2 | Mensagem broadcast (difusão) |

#### PS (PDU Specific) — bits 15-8

- Se **PDU1 (PF < 0xF0)**: PS = **Endereço de Destino (DA)** → mensagem vai para um SA específico
- Se **PDU2 (PF ≥ 0xF0)**: PS = **Group Extension (GE)** → parte do PGN

#### SA (Source Address) — bits 7-0
Endereço do nó que está transmitindo (0x00-0xFD).

### Cálculo do PGN

```
Para PDU1 (PF < 0xF0):
  PGN = (DP << 17) | (PF << 8)  [PS não incluído — é o DA]

Para PDU2 (PF ≥ 0xF0):
  PGN = (DP << 17) | (PF << 8) | PS  [PS é extensão do PGN]
```

### Exemplos Práticos

```
EEC1 (Engine Speed): PGN = 61444 = 0xF004
  CAN ID: 0CF00400
  ├── Priority = 0x0C >> 2 & 0x7 = 3
  ├── DP = 0
  ├── PF = 0xF0 (=240 ≥ 240 → PDU2)
  ├── PS (Group Ext) = 0x04
  └── SA = 0x00 (ECM #1)
  
  PGN = 0x00 | (0xF0 << 8) | 0x04 = 0xF004 = 61444 ✓

Request para DM1: PGN = 59904 = 0xEA00 (PDU1)
  Destino 0xFF (broadcast request)
  CAN ID: 18EAFF00
  ├── Priority = 6
  ├── PF = 0xEA (< 240 → PDU1)
  ├── PS = 0xFF (DA = broadcast)
  └── SA = 0x00
  
  PGN = (0xEA << 8) = 0xEA00 = 59904 ✓
```

---

## 2.3 Endereços de Fonte (Source Addresses)

J1939 define endereços para cada tipo de unidade de controle:

### Endereços Reservados Importantes

| SA (Hex) | Descrição |
|---------|-----------|
| 0x00 | Engine #1 (ECM) |
| 0x01 | Engine #2 |
| 0x02 | Turbocharger |
| 0x03 | Transmission #1 (TCM) |
| 0x04 | Transmission #2 |
| 0x05 | Shift Console |
| 0x06 | Power Takeoff |
| 0x07 | Axle Steering #1 |
| 0x08 | Axle Drive #1 (Drive Axle #1, Steer) |
| 0x09 | Axle Drive #1 (Drive Axle #1, Drive) |
| 0x0A | Brakes — System Controller |
| 0x0B | Brakes — ABS |
| 0x0C | Retarder — Engine |
| 0x0D | Retarder — Driveline |
| 0x0E | Cruise Control |
| 0x0F | Fuel System |
| 0x10 | Steering Controller |
| 0x11 | Suspension Controller |
| 0x12 | Instrument Cluster |
| 0x13 | Trip Recorder |
| 0x14 | Cab Climate |
| 0x15 | Multiplex Lighting |
| 0x16 | Non-Container Refrigeration |
| 0x17 | Frame (body module) |
| 0x18 | Headway Controller |
| 0x19 | On Board Diagnostic Unit |
| 0x1A | Data Logger |
| 0x1B | PC Keyboard |
| 0x1C | Safety Restraint System (Airbag) / Instrument Display |
| 0x1D | Turbocharger Compressor Bypass |
| 0x1E | Turbocharger Wastegate / Hitch/PTO (agricutural) |
| 0x1F | Throttle |
| 0x20 | Headway (distance control) |
| 0x21 | Body Controller |
| 0x22 | Auxiliary Valve Control 1 |
| 0x23 | Auxiliary Valve Control 2 |
| 0x24 | Auxiliary Heater 1 |
| 0x25 | Auxiliary Heater 2 / Implement Controller |
| 0x26 | ISOBUS Virtual Terminal |
| 0x27 | Cab Display / Body Control Module |
| 0x28 | Armrest Control Unit |
| 0x29-0x7D | Outros (varia por OEM e aplicação) |
| 0x7E | Dynamic — atribuído por Address Claiming |
| 0x7F | Implement (ISOBUS) |
| 0xF0-0xFD | Dynamic — atribuído por Address Claiming |
| 0xFE | Null Address (usado durante Address Claiming) |
| 0xFF | Broadcast (destino global) |

---

## 2.4 NAME — Identificador Único do Nó (64-bit)

Cada ECU tem um NAME único de 64 bits, definido pela SAE J1939-81:

```
Bit 63-61: Industry Group (3 bits)
  000 = Global (non-industry specific)
  001 = On-Highway
  010 = Agricultural / Off-Highway
  011 = Construction
  100 = Marine
  101 = Industrial

Bit 60:    Arbitrary Address Capable (1 bit)
  0 = Endereço fixo
  1 = Pode tentar novo endereço se conflito

Bits 59-56: Vehicle System Instance (4 bits)
  Para máquinas com múltiplos sistemas idênticos

Bits 55-49: Vehicle System (7 bits)
  Define o tipo de veículo/máquina

Bit 48: Reserved (=0)

Bits 47-42: Function (6 bits)
  Função específica do ECU

Bits 41-39: Function Instance (3 bits)
  Para múltiplos ECUs de mesma função

Bits 38-35: ECU Instance (4 bits)
  Para múltiplos ECUs no mesmo nó físico

Bits 34-21: Manufacturer Code (11 bits)
  Código registrado na SAE (Bosch=5, Deere=98, etc.)

Bits 20-0: Identity Number (21 bits)
  Número de série único do fabricante
```

### Exemplos de NAME

```
ECM (Engine Control Module) — John Deere:
  Industry: 010 (Agricultural)
  Vehicle System: 0000001 (Engine)
  Function: 000000 (Engine Management)
  Manufacturer: 0001100010 (John Deere = 0x062)
  Identity: 000000000000000000001

NAME em hex: 0x0006200000620001
```

---

## 2.5 Procedimento de Address Claiming (J1939-81)

O Address Claiming garante que não há dois nós com o mesmo SA no barramento.

### Sequência Completa

```
PASSO 1: POWER-ON
  ┌─────────────────────────────────────────────────────┐
  │ ECU liga, executa self-test                          │
  │ Lê endereço desejado da EEPROM (ex: SA=0x00 para ECM)│
  └─────────────────────────────────────────────────────┘
                          │
                          ▼
PASSO 2: TRANSMIT CANNOT CLAIM (opcional — com SA=0xFE)
  ┌─────────────────────────────────────────────────────┐
  │ Frame PGN 60928 (0xEE00), SA=0xFE, DA=0xFF          │
  │ Dados = meu NAME (8 bytes)                          │
  │ Indica: "Estou acordando, ainda não tenho endereço"  │
  └─────────────────────────────────────────────────────┘
                          │
                          ▼
PASSO 3: TRANSMIT ADDRESS CLAIM
  ┌─────────────────────────────────────────────────────┐
  │ Frame PGN 60928 (0xEE00), SA=meu_sa, DA=0xFF        │
  │ Dados = meu NAME (8 bytes)                          │
  │ "Quero usar SA=0x00, meu NAME é 0x0006200000620001" │
  └─────────────────────────────────────────────────────┘
                          │
                          ▼
PASSO 4: AGUARDA 250 ms
  ┌─────────────────────────────────────────────────────┐
  │ Monitora barramento por conflitos                   │
  │ Se outro nó reclamar o mesmo SA:                    │
  │   → Compara NAMEs                                   │
  │   → NAME menor (numericamente) VENCE                │
  │   → Perdedor tenta SA diferente ou vai para 0xFE    │
  └─────────────────────────────────────────────────────┘
                          │
             ┌────────────┴────────────┐
         Sem conflito              Conflito
             │                        │
             ▼                        ▼
PASSO 5: ONLINE         PASSO 5b: NOVO ENDEREÇO
  ECU está online          Tenta SA alternativo
  Começa a transmitir      ou fica em 0xFE (null)
  mensagens periódicas
```

### Frame de Address Claim

```
CAN ID: 18EEFF00 (para SA=0x00)
  Priority = 6 (0x18 >> 2 & 0x7 = 6)
  PGN = 0xEE00 = 60928
  DA = 0xFF (broadcast — todos precisam saber)
  SA = 0x00 (endereço que estou reivindicando)

Data (8 bytes) = NAME (LSB primeiro):
  Byte 0: Identity[0:7]
  Byte 1: Identity[8:15]
  Byte 2: Identity[16:20] | ECU_Instance[21:24]
  Byte 3: Function_Instance[25:27] | Function[28:33]
  Byte 4: Reserved=0 | Vehicle_System[35:41]
  Byte 5: Vehicle_System_Instance[42:45] | Industry_Group[46:48] | AA_Cap[49]
  Byte 6: Manufacturer_Code[50:57] (LSB)
  Byte 7: Manufacturer_Code[58:60] (MSB)
```

---

## 2.6 PGN — Parameter Group Number

Um **PGN** agrupa parâmetros relacionados em um frame de 8 bytes.

### Estrutura de um PGN

```
PGN = número único que identifica o conjunto de parâmetros
Nome = mnemônico (ex: EEC1, ET1, LFE)
Taxa TX = com que frequência é transmitido (ms)
SPNs = lista de parâmetros dentro dos 8 bytes
```

### Tipos de Transmissão

| Tipo | Descrição |
|------|-----------|
| **Cyclic** | Transmitido a intervalos fixos (ex: EEC1 a 10ms) |
| **On Change** | Transmitido apenas quando valor muda |
| **On Request** | Transmitido apenas após receber Request PGN (59904) |
| **Broadcast** | DA=0xFF, todos recebem |
| **Unicast** | DA=SA específico, peer-to-peer |

---

## 2.7 PGNs Completos — Heavy Machinery

### Grupo Engine (ECM)

#### PGN 61444 — EEC1 (Electronic Engine Control 1)
**Taxa**: 10 ms | **Prioridade**: 3 | **Tipo**: Broadcast

| Byte | Bits | SPN | Nome | Factor | Offset | Range | Unit |
|------|------|-----|------|--------|--------|-------|------|
| 0 | 0-3 | 899 | Engine Torque Mode | 1 | 0 | 0-15 | — |
| 0 | 4-7 | 512 | Driver's Demand % (high nibble) | — | — | — | — |
| 1 | 0-7 | 512 | Driver's Demand Engine Torque | 1 | -125 | -125..+125 | % |
| 2 | 0-7 | 513 | Actual Engine Torque | 1 | -125 | -125..+125 | % |
| 3-4 | 0-15 | 190 | Engine Speed | 0.125 | 0 | 0..8031.875 | rpm |
| 5 | 0-7 | 1483 | Source Address Override | 1 | 0 | 0..255 | — |
| 6 | 0-3 | 1675 | Engine Starter Mode | 1 | 0 | 0..15 | — |
| 7 | — | — | Reservado | — | — | — | — |

**Valores SPN 899 (Engine Torque Mode):**
```
0 = Low Idle
1 = Operator Demanded
2 = Remote Demanded
3 = Speed Control
4 = High Speed Governor
...
```

**Valores SPN 1675 (Engine Starter Mode):**
```
0 = No Request
1 = Starter Active
2 = Starter Not Active (inhibited)
...
```

---

#### PGN 61445 — EEC2 (Electronic Engine Control 2)
**Taxa**: 50 ms | **Prioridade**: 3

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0 | — | Accel Pedal 1 Low Idle Switch | — | — | — |
| 1 | 558/91 | Accelerator Pedal Position 1 | 0.4 | 0 | % |
| 2 | 92 | Engine Percent Load At Current Speed | 0.4 | 0 | % |
| 3 | 974 | Remote Accelerator Pedal Position | 0.4 | 0 | % |
| 4 | — | Accel Pedal 2 Position | 0.4 | 0 | % |
| 5 | 1437 | Remote Throttle Valve Position | 0.4 | 0 | % |
| 6-7 | — | Reservado | — | — | — |

---

#### PGN 65247 — EEC3 (Electronic Engine Control 3)
**Taxa**: 250 ms | **Prioridade**: 6

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0-1 | 518 | Nominal Friction Torque (%) | 0.125 | -125 | % |
| 2-3 | 514 | Desired Operating Speed | 0.125 | 0 | rpm |
| 4 | 515 | Desired Operating Speed Asymmetry | 1 | 0 | rpm |
| 5-7 | — | Reservado | — | — | — |

---

#### PGN 65262 — ET1 (Engine Temperature 1)
**Taxa**: 1000 ms | **Prioridade**: 6

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0 | 110 | Engine Coolant Temperature | 1 | -40 | °C |
| 1 | 174 | Fuel Temperature | 1 | -40 | °C |
| 2-3 | 175 | Engine Oil Temperature 1 | 0.03125 | -273 | °C |
| 4-5 | 176 | Turbocharger Oil Temperature | 0.03125 | -273 | °C |
| 6 | 52 | Engine Intercooler Temperature | 1 | -40 | °C |
| 7 | 1134 | Engine Intercooler Thermostat Opening | 0.4 | 0 | % |

---

#### PGN 65263 — EFL/P1 (Engine Fluid Level/Pressure 1)
**Taxa**: 500 ms | **Prioridade**: 6

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0 | 94 | Fuel Delivery Pressure | 4 | 0 | kPa |
| 1-2 | 22 | Extended Range Fuel Pressure | 0.1 | 0 | kPa |
| 3 | 98 | Engine Oil Level | 0.4 | 0 | % |
| 4 | 100 | Engine Oil Pressure | 4 | 0 | kPa |
| 5 | 101 | Crankcase Blow-By Pressure | 0.05 | 0 | kPa |
| 6 | 109 | Engine Coolant Pressure | 2 | 0 | kPa |
| 7 | 111 | Engine Coolant Level | 0.4 | 0 | % |

---

#### PGN 65270 — IC1 (Inlet/Exhaust Conditions 1)
**Taxa**: 500 ms | **Prioridade**: 6

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0 | 102 | Engine Intake Manifold 1 Pressure | 2 | 0 | kPa |
| 1 | 105 | Engine Intake Manifold 1 Temperature | 1 | -40 | °C |
| 2 | 106 | Engine Air Inlet Pressure | 2 | 0 | kPa |
| 3 | 107 | Engine Air Filter 1 Differential Pressure | 0.5 | 0 | kPa |
| 4-5 | 173 | Engine Exhaust Gas Temperature | 0.03125 | -273 | °C |
| 6 | 1127 | Engine Coolant Thermostat Opening | 0.4 | 0 | % |
| 7 | 108 | Engine Percent Fan Speed | 0.4 | 0 | % |

---

#### PGN 65266 — LFE (Fuel Economy)
**Taxa**: 100 ms | **Prioridade**: 6

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0-1 | 183 | Engine Fuel Rate | 0.05 | 0 | L/h |
| 2-3 | 184 | Engine Instantaneous Fuel Economy | 1/512 | 0 | km/L |
| 4-5 | 185 | Engine Average Fuel Economy | 1/512 | 0 | km/L |
| 6 | 51 | Engine Throttle Valve 1 Position | 0.4 | 0 | % |
| 7 | — | Reservado | — | — | — |

---

#### PGN 65253 — HOURS (Engine Hours/Revolutions)
**Taxa**: 1000 ms | **Prioridade**: 6

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0-3 | 247 | Total Engine Hours | 0.05 | 0 | h |
| 4-7 | 249 | Total Engine Revolutions | 1000 | 0 | rev |

---

### Grupo Transmission (TCM)

#### PGN 61442 — ETC1 (Electronic Transmission Controller 1)
**Taxa**: 20 ms | **Prioridade**: 3

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0 | 560 | Transmission Driveline Engaged | 2bits | — | — |
| 0 | 573 | TC Lockup Engaged | 2bits | — | — |
| 0 | 574 | Progressive Shift Disable | 2bits | — | — |
| 1-2 | 161 | Transmission Input Shaft Speed | 0.125 | 0 | rpm |
| 3-4 | 191 | Transmission Output Shaft Speed | 0.125 | 0 | rpm |
| 5 | 1783 | Current Gear | 1 | -125 | — |
| 6 | 1784 | Selected Gear | 1 | -125 | — |
| 7 | 573 | TC Engagement (full byte) | 0.4 | 0 | % |

---

#### PGN 61443 — ETC2 (Electronic Transmission Controller 2)
**Taxa**: 20 ms | **Prioridade**: 3

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0 | 524 | Transmission Selected Gear | 1 | -125 | — |
| 1-2 | 526 | Transmission Actual Gear Ratio | 0.001 | 0 | — |
| 3 | 523 | Transmission Current Gear | 1 | -125 | — |
| 4 | 525 | Transmission Requested Range | — | — | — |
| 5 | 577 | Transmission Current Range | — | — | — |
| 6 | 1854 | Transmission Range Attained | — | — | — |
| 7 | 1855 | Transmission Range Selected | — | — | — |

---

### Grupo Vehicle (CCVS, VD)

#### PGN 65265 — CCVS (Cruise Control/Vehicle Speed)
**Taxa**: 100 ms | **Prioridade**: 6

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0 | 69 | Two-Speed Axle Switch | 2bits | — | — |
| 0 | 70 | Parking Brake Switch | 2bits | — | — |
| 0 | 1633 | CC Pause Switch | 2bits | — | — |
| 1-2 | 84 | Wheel-Based Vehicle Speed | 1/256 | 0 | km/h |
| 3 | 595 | CC Enable Switch | 2bits | — | — |
| 3 | 596 | Clutch Switch | 2bits | — | — |
| 3 | 597 | Brake Switch | 2bits | — | — |
| 3 | 85 | CC Active | 2bits | — | — |
| 4 | 86 | CC Set Speed | 1 | 0 | km/h |
| 5 | 976 | PTO Set Speed | 1 | 0 | rpm |
| 6 | 527 | CC Speed Set Limit | 1 | 0 | km/h |
| 7 | — | CC High/Low Set Limit | — | — | — |

---

#### PGN 65248 — VD (Vehicle Distance)
**Taxa**: 1000 ms | **Prioridade**: 6

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0-3 | 244 | Trip Distance | 0.125 | 0 | km |
| 4-7 | 245 | Total Vehicle Distance | 0.125 | 0 | km |

---

### Grupo ISOBUS / Implements

#### PGN 65093 — PTO (Power Take-Off Information)
**Taxa**: 100 ms | **Prioridade**: 6

| Byte | Bits | SPN | Nome | Factor | Offset | Unit |
|------|------|-----|------|--------|--------|------|
| 0 | 0-4 | 1691 | Rear PTO State | 1 | 0 | enum |
| 0 | 5-7 | 1692 | Front PTO State | 1 | 0 | enum |
| 1 | 0-1 | 900 | PTO Engagement Control | 1 | 0 | enum |
| 2-3 | 0-15 | 1693 | Rear PTO Output Shaft Speed | 0.125 | 0 | rpm |
| 4-5 | 0-15 | 1694 | Front PTO Output Shaft Speed | 0.125 | 0 | rpm |
| 6 | 0-7 | 1696 | Rear PTO Economy Mode | 0.4 | 0 | % |
| 7 | — | — | Reservado | — | — | — |

**Valores SPN 1691 (Rear PTO State):**
```
0 = Off/Disabled
1 = Hold (brake)
2 = Remote — Position 1
3 = Remote — Position 2
4 = Nominal 540
5 = Nominal 1000
6 = External Speed
...
```

---

#### PGN 65091 — HITCH (Hitch Status, agricultural)
**Taxa**: 100 ms | **Prioridade**: 6

| Byte | SPN | Nome | Factor | Offset | Unit |
|------|-----|------|--------|--------|------|
| 0 | 1871 | Rear Hitch Position | 0.4 | 0 | % |
| 1 | 1872 | Rear Hitch In-Work Indicator | 2bits | — | — |
| 2-3 | 1873 | Rear Hitch Nominal Lower Link Force | 0.125 | -2000 | Nm |
| 4 | 1874 | Rear Hitch Draft | 0.125 | -200 | kN |
| 5 | 1875 | Rear Nominal Lower Link Force Limit | 1 | 0 | % |
| 6 | 1876 | Rear Draft Limit | 1 | 0 | % |
| 7 | — | Reservado | — | — | — |

---

### Grupo Diagnóstico (DM1-DM35)

#### PGN 65226 — DM1 (Active Diagnostic Trouble Codes)
**Taxa**: 1 Hz (ou ao mudar) | **Prioridade**: 6

```
Byte 0:
  Bits 6-7: MIL (Malfunction Indicator Lamp) Status
  Bits 4-5: Red Stop Lamp Status
  Bits 2-3: Amber Warning Lamp Status
  Bits 0-1: Protect Lamp Status

  Lamp Values: 00=off, 01=on, 10=flash, 11=not defined

Byte 1: Reserved (=0xFF when not supported)

Bytes 2-5: First DTC (se existir):
  Bytes 2-3: SPN[0-7], SPN[8-15]
  Byte 4:
    Bits 7: SPN Conversion Method (CM)
    Bits 5-3: FMI (Failure Mode Identifier, 0-31)
    Bits 2-0: SPN[16-18]
  Byte 5: Occurrence Count (0-126; 127=125+)

Bytes 6-9: Second DTC (se existir — requer Transport Protocol para múltiplos DTCs)
```

#### PGN 65227 — DM2 (Previously Active DTCs)
Mesmo formato que DM1, mas contém DTCs que foram ativos anteriormente e já se resolveram.

#### PGN 65228 — DM3 (Diagnostic Data Clear/Reset)
Solicitação para limpar todos os DTCs stored.
```
Transmitir frame vazio para SA da ECU com PGN DM3.
ECU deve limpar seus DTCs e transmitir DM2 zerado.
```

#### PGN 65235 — DM11 (Diagnostic Data Clear)
Limpa todos os DTCs ativos (DM1) e armazenados (DM2).

#### PGN 65230 — DM5 (Diagnostic Readiness)
Indica o status de prontidão dos monitores OBD:
```
Byte 0: Active Fault Count
Byte 1: Previously Active Fault Count
Byte 2: OBD Compliance
Byte 3-7: Monitor Ready Status bits
```

---

## 2.8 SPN — Suspect Parameter Number

Cada **SPN** é um parâmetro individual dentro de um PGN.

### Estrutura de um SPN

```
SPN Number: Identificador único global (1-524287)
Nome: Descrição do parâmetro
Comprimento: em bits (1-32 bits típico)
Fator de resolução: multiplicador para converter raw → engenharia
Offset: valor a somar após multiplicação
Unidade: km/h, kPa, °C, rpm, etc.
Range: faixa válida de valores
Indicador de erro: all-bits-1 = parâmetro não disponível
```

### Cálculo do Valor Físico

```
Valor_Físico = Valor_Raw × Fator + Offset

Exemplos:
  SPN 190 (Engine Speed):
    Raw = 0x1100 = 4352 (little-endian 2 bytes)
    Factor = 0.125 rpm/bit
    Valor = 4352 × 0.125 = 544 rpm

  SPN 110 (Coolant Temp):
    Raw = 0x84 = 132
    Factor = 1.0 °C/bit
    Offset = -40
    Valor = 132 × 1 + (-40) = 92 °C
    
  SPN 100 (Oil Pressure):
    Raw = 0x58 = 88
    Factor = 4.0 kPa/bit
    Valor = 88 × 4 = 352 kPa
```

### Indicador de Não Disponível

O valor máximo (todos bits = 1) indica que o parâmetro não está disponível:

```
SPN 190 (16 bits): 0xFFFF = não disponível
SPN 110 (8 bits):  0xFF   = não disponível
SPN 100 (8 bits):  0xFF   = não disponível
```

---

## 2.9 FMI — Failure Mode Identifier

O FMI descreve o TIPO de falha detectada para um SPN:

| FMI | Código SAE | Descrição |
|-----|-----------|-----------|
| 0 | ABOVE_NORMAL_MOST_SEVERE | Dado válido mas acima do normal — crítico |
| 1 | BELOW_NORMAL_MOST_SEVERE | Dado válido mas abaixo do normal — crítico |
| 2 | ERRATIC | Dado errático, intermitente ou incorreto |
| 3 | VOLTAGE_HIGH | Tensão acima do normal ou curto para + |
| 4 | VOLTAGE_LOW | Tensão abaixo do normal ou curto para - |
| 5 | CURRENT_BELOW | Corrente abaixo do normal ou circuito aberto |
| 6 | CURRENT_ABOVE | Corrente acima do normal ou curto para terra |
| 7 | MECHANICAL | Sistema mecânico não responde ou fora de ajuste |
| 8 | ABNORMAL_FREQ | Frequência, largura de pulso ou período anormal |
| 9 | ABNORMAL_UPDATE | Taxa de atualização anormal |
| 10 | RATE_OF_CHANGE | Taxa de variação anormal |
| 11 | ROOT_CAUSE_UNKNOWN | Causa raiz desconhecida |
| 12 | BAD_DEVICE | Dispositivo ou componente defeituoso |
| 13 | OUT_OF_CALIBRATION | Fora de calibração |
| 14 | SPECIAL_INSTRUCTIONS | Instruções especiais (ver documentação) |
| 15 | ABOVE_NORMAL_LEAST | Acima do normal — menos severo |
| 16 | ABOVE_NORMAL_MODERATE | Acima do normal — moderadamente severo |
| 17 | BELOW_NORMAL_LEAST | Abaixo do normal — menos severo |
| 18 | BELOW_NORMAL_MODERATE | Abaixo do normal — moderadamente severo |
| 19 | NETWORK_DATA_ERROR | Dados de rede recebidos com erro |
| 20-30 | RESERVED | Reservado |
| 31 | CONDITION_EXISTS | Condição existe — sem dado anormal |

### Exemplos de DTC com FMI

```
SPN 110, FMI 0 → Temperatura do refrigerante ACIMA do normal — mais severo
  (Coolant overheating — Red Stop condition)

SPN 100, FMI 1 → Pressão do óleo ABAIXO do normal — mais severo
  (Low oil pressure — Red Stop condition)

SPN 94, FMI 4 → Pressão de combustível TENSÃO BAIXA
  (Fuel pressure sensor shorted to ground)

SPN 190, FMI 9 → Velocidade do motor TAXA DE ATUALIZAÇÃO ANORMAL
  (Engine speed signal lost — bus timeout)

SPN 3361, FMI 17 → Nível DEF ABAIXO do normal — menos severo
  (DEF level low — Amber warning)
```

---

## 2.10 Protocolo de Transporte (J1939-21)

Para mensagens com mais de 8 bytes, o J1939 usa um protocolo de transporte.

### Protocolo BAM (Broadcast Announce Message)

Para transmitir para múltiplos receptores (broadcast):

```
PASSO 1: TP.CM_BAM (PGN 0xEC00 = 60160)
  Byte 0: 0x20 (BAM control)
  Bytes 1-2: Total message size (bytes, LE)
  Byte 3: Total number of packets
  Byte 4: 0xFF (reserved for BAM)
  Bytes 5-7: PGN being transported (LE, 3 bytes)

PASSO 2: TP.DT (PGN 0xEB00 = 60416), pacotes consecutivos
  Byte 0: Sequence number (1-255)
  Bytes 1-7: Data (7 bytes per packet, last packet may use 0xFF padding)

INTERVALO ENTRE PACOTES: 50-200 ms
```

**Exemplo: DM1 com 3 DTCs (12 bytes de dados)**

```
Frame 1 - TP.CM_BAM:
  CAN ID: 1CECFF00 (Priority=7, PGN=EC00, SA=00, DA=FF)
  Data: 20 0C 00 02 FF CA FE 00
       [20=BAM] [0x000C=12 bytes] [02=2 packets] [FF] [PGN 0x00FECA in LE]

Frame 2 - TP.DT Packet 1:
  CAN ID: 1CEBFF00
  Data: 01 40 FF FF 00 10 01 00
       [seq=1] [byte1..7 of message]

Frame 3 - TP.DT Packet 2:
  CAN ID: 1CEBFF00
  Data: 02 20 00 01 05 FF FF FF
       [seq=2] [byte8..14 (padded)]
```

### Protocolo CMDT (Connection Mode Data Transfer)

Para peer-to-peer (unicast) com controle de fluxo:

```
1. Transmissor → TP.CM_RTS (Request to Send)
2. Receptor   ← TP.CM_CTS (Clear to Send) — especifica quantos pacotes
3. Transmissor → TP.DT (Data Transfer), N pacotes
4. Receptor   ← TP.CM_CTS ou TP.CM_EndOfMsgAck
5. Repete até concluir
```

---

## 2.11 ISOBUS (ISO 11783) — Para Máquinas Agrícolas

O ISOBUS é um perfil do J1939 para equipamentos agrícolas. Adiciona:

### Working Set Management

```
Working Set Master: controla um conjunto de ECUs de implemento
Working Set Member: cada ECU subordinada

Mensagens especiais:
  PGN 0xFEFE = Working Set Master
  PGN 0xFEFD = Working Set Member
  PGN 0xEE00 = Address Claim (mesmo do J1939)
```

### Task Controller (TC)

O Task Controller gerencia tarefas de campo:
- Controle de dosagem de sementes/fertilizantes
- Mapeamento de colheita
- Controle de seção

### Virtual Terminal (VT)

O Virtual Terminal é o display do trator:
- Apresenta informações do implemento
- Permite configuração
- Usa PGNs específicos para atualizar a tela

### ECU Address Ranges ISOBUS

```
0x80-0xFF: Dynamic (Address Claiming)
0x7F:      Implement (especial)
0x0D:      Task Controller
0x26:      Virtual Terminal
0x27:      Primary Navigation Display
```

---

# PARTE 3: DIAGNÓSTICO — COMO FAZER

## 3.1 Equipamentos Necessários

### Ferramentas de Hardware

| Ferramenta | Uso | Custo (aprox.) |
|-----------|-----|--------|
| Multímetro | Verificar tensões, resistência | R$ 50-500 |
| Osciloscópio | Análise de sinal CAN | R$ 500-5000 |
| CAN Analyzer (Kvaser/PEAK) | Monitor de frames J1939 | R$ 500-3000 |
| Diagnostic Tool (OBD/J1939) | Leitura de DTCs | R$ 200-5000 |
| Terminador de teste (60Ω) | Verificar circuito terminador | R$ 20 |

### Software

- **PEAK PCAN View**: Monitor CAN gratuito
- **Kvaser CanKing**: Análise de frames
- **ScanTool/CANalyzer**: Suite completa (comercial)
- **Python + python-can**: Scripting diagnóstico
- **Rust + socketcan**: Diagnóstico programático

---

## 3.2 Verificação da Camada Física

### Passo 1: Verificar Resistência do Barramento

Com barramento sem energia:
```
Medir entre CAN_H e CAN_L:
  Esperado: ~60 Ω (dois terminadores de 120Ω em paralelo)
  
  > 120 Ω: Um terminador faltando ou aberto
  < 50 Ω:  Terminador extra ou curto parcial
  ~0 Ω:   Curto entre CAN_H e CAN_L → comunicação impossível
  ∞ Ω:    Barramento interrompido (wire break)
```

### Passo 2: Verificar Tensões em Operação

Com barramento energizado e ECUs ativas:
```
Modo Recessivo (sem transmissão):
  CAN_H: ~2.5V ±0.1V
  CAN_L: ~2.5V ±0.1V
  Diferencial: ~0V

Modo Dominante (durante transmissão):
  CAN_H: 2.75-4.5V (típico 3.5V)
  CAN_L: 0.5-2.25V (típico 1.5V)
  Diferencial: 1.5-3.0V (típico 2.0V)
```

### Passo 3: Verificar com Osciloscópio

```
Configuração:
  Modo diferencial (CH1 = CAN_H, CH2 = CAN_L, Math = CH1-CH2)
  Escala: 1V/div, 20µs/div (para 500kbps)
  Trigger: Edge, descida, CH1 ou diferencial

Sinais normais:
  ┌─────────────────────────────────────────────────────┐
  │  CAN_H:   ─────┐   ┌───────┐   ┌─────              │
  │           2.5V │   │ 3.5V  │   │                   │
  │                └───┘       └───┘                   │
  │  CAN_L:   ─────┐   ┌───────┐   ┌─────              │
  │           2.5V │   │ 1.5V  │   │                   │
  │                └───┘       └───┘                   │
  │  Diferencial: ~0V  2.0V  ~0V  2.0V  ~0V            │
  └─────────────────────────────────────────────────────┘

Problemas comuns no osciloscópio:
  - Reflexões: Terminadores insuficientes
  - Sinal "sujo": Interferência EMI, aterramento ruim
  - Bits cortados: Problemas de temporização, clock
  - Assimetria H/L: Curto parcial, resistência de série
```

---

## 3.3 Diagnóstico de DTCs

### Leitura de DTCs via J1939

**Método 1: Monitor passivo (escutar DM1)**
```
PGN 65226 (0xFECA) é transmitido periodicamente
Escute passivamente no barramento
Qualquer ECU com falha ativa transmitirá DM1
Decodifique: SA, SPN, FMI → identifique o problema
```

**Método 2: Request DM1**
```
Transmita Request (PGN 59904 = 0xEA00):
  Data: 3 bytes = PGN desejado em LE
  Para DM1: Data = CA FE 00 (0xFECA em LE)
  DA = SA da ECU alvo (ou 0xFF para todas)

ECU responderá com DM1
```

**Método 3: Request DM2 (DTCs armazenados)**
```
Mesmo que DM1 mas com PGN 65227 (0xFECB):
  Data: CB FE 00
```

### Interpretação dos DTCs

```
Exemplo de DM1 recebido:
  SA: 0x00 (ECM)
  Data: 44 FF 00 68 08 03 FF FF

  Byte 0: 0x44 = 0100 0100
    Bits 7-6: 01 = Protect Lamp ON
    Bits 5-4: 00 = Red Stop OFF
    Bits 3-2: 01 = Amber Warning ON
    Bits 1-0: 00 = MIL OFF

  Bytes 2-5: First DTC
    Byte 2: 0x00 = SPN[7:0] = 0
    Byte 3: 0x68 = SPN[15:8] = 104  → SPN[15:0] = 0x6800 >> 0 = wait...
    
    Correctly parsing DTC bytes 2-5 (0x00, 0x68, 0x08, 0x03):
    SPN bits = data[2][7:0] | data[3][7:0] << 8 | data[4][2:0] << 16
             = 0x00 | (0x68 << 8) | ((0x08 & 0x07) << 16)
             = 0x006800 = SPN 26624? 
    
    Mais correto (J1939 byte order):
    SPN = ((data[4] & 0x07) << 16) | (data[3] << 8) | data[2]
        = ((0x08 & 0x07) << 16) | (0x68 << 8) | 0x00
        = 0 | 0x6800 | 0x00 = 26624... 
    
    Vamos usar o exemplo real:
    data[2] = 0x64 = SPN LSB = 100
    data[3] = 0x00 = SPN mid = 0
    data[4] = 0x24 = 0010 0100:
      bits[7] = CM = 0
      bits[6:3] = FMI = 0100 = 4 (FMI 4 = Voltage Low)
      bits[2:0] = SPN[18:16] = 100 = 0
    data[5] = 0x01 = Occurrence Count = 1
    
    SPN = 0x006400 = 100... wait
    
    Final: SPN = 100 (Oil Pressure), FMI = 4 (Voltage Low/Shorted to Ground)
    → Sensor de pressão de óleo com circuito aberto ou curto para terra
```

### Processo de Diagnóstico

```
1. IDENTIFICAR: Qual SPN? Qual FMI?
   SPN → Identifica o PARÂMETRO ou SENSOR
   FMI → Identifica o TIPO DE FALHA

2. VERIFICAR: Condição atual
   Leia o valor atual do SPN pelo barramento
   Compare com range esperado

3. ISOLAR: Hardware vs Software
   FMI 3/4/5/6 → Problema de fiação/sensor
   FMI 0/1     → Valor real fora do range
   FMI 11/12   → Problema interno do ECU
   FMI 9       → Timeout de comunicação

4. REPARAR: Baseado no tipo de falha
   FMI 3: Medir tensão no sensor, verificar circuito
   FMI 4: Verificar curto para massa
   FMI 5: Verificar circuito aberto, resistência
   FMI 0/1: Verificar sistema mecânico/fluido

5. CONFIRMAR: Após reparo
   Clear DTCs (DM3)
   Restart sistema
   Verificar se DTC retorna
```

---

## 3.4 Programação e Reprogramação de ECUs

### Flashing via CAN (J1939-21)

A maioria dos ECUs modernos suporta reprogramação via CAN (em campo):

```
Protocolo: SAE J1939-21 Memory Access

PGN 49152 (0xC000) — DM16 Binary Data Transfer
  Transfere blocos de dados

PGN 49408 (0xC100) — DM15 Memory Access Response
  Resposta do ECU sobre sucesso/falha

Sequência típica:
1. Tool → ECU: DM16 com bloco de dados (via TP se > 8 bytes)
2. ECU → Tool: DM15 com status
3. Repetir para todos os blocos
4. Tool → ECU: Comando de boot/reset
5. ECU reinicia com novo firmware
```

### Calibração de Parâmetros (EEPROM Write)

Alguns ECUs permitem alterar parâmetros via CAN:

```
Protocolo OEM-específico (Proprietary PGNs)

Exemplo genérico:
1. Tool envia "Unlock" com seed/key authentication
2. ECU retorna "unlocked" status
3. Tool envia PGN com parâmetro + valor
4. ECU salva em EEPROM, confirma
5. Tool envia "Lock" para proteger
```

---

# PARTE 4: IMPLEMENTAÇÃO PRÁTICA

## 4.1 Decodificação de Frame J1939 em Rust

```rust
pub fn decode_j1939_id(raw_id: u32) -> J1939Fields {
    let priority = ((raw_id >> 26) & 0x07) as u8;
    let _reserved = (raw_id >> 25) & 0x01;
    let dp        = (raw_id >> 24) & 0x01;
    let pf        = ((raw_id >> 16) & 0xFF) as u8;
    let ps        = ((raw_id >>  8) & 0xFF) as u8;
    let sa        = (raw_id & 0xFF) as u8;

    let (pgn, da) = if pf < 0xF0 {
        // PDU1: PS é o Destination Address
        let pgn = (dp << 17) | ((pf as u32) << 8);
        (pgn, ps)
    } else {
        // PDU2: PS é Group Extension (parte do PGN)
        let pgn = (dp << 17) | ((pf as u32) << 8) | (ps as u32);
        (pgn, 0xFF)
    };

    J1939Fields { priority, pgn, sa, da }
}

pub fn encode_j1939_id(priority: u8, pgn: u32, sa: u8, da: u8) -> u32 {
    let dp = (pgn >> 17) & 0x01;
    let pf = (pgn >> 8) & 0xFF;
    let ps = if pf < 0xF0 { da as u32 } else { pgn & 0xFF };
    
    ((priority as u32 & 0x07) << 26)
    | (dp << 24)
    | (pf << 16)
    | (ps << 8)
    | (sa as u32)
}
```

## 4.2 Extraindo Bits de um SPN

```rust
/// Extrai valor raw de N bits de um array de bytes, iniciando em byte_offset:bit_offset
fn extract_spn_bits(data: &[u8], byte_offset: usize, bit_offset: u8, length: u8) -> u64 {
    let mut result: u64 = 0;
    for i in 0..length as usize {
        let absolute_bit = byte_offset * 8 + bit_offset as usize + i;
        let byte_idx = absolute_bit / 8;
        let bit_idx  = absolute_bit % 8;
        if byte_idx < data.len() {
            if (data[byte_idx] >> bit_idx) & 1 == 1 {
                result |= 1u64 << i;
            }
        }
    }
    result
}

/// Converte raw para valor físico
fn raw_to_physical(raw: u64, factor: f64, offset: f64) -> f64 {
    raw as f64 * factor + offset
}

// Uso:
// Engine Speed (SPN 190): byte_offset=3, bit_offset=0, length=16, factor=0.125, offset=0
let rpm_raw     = extract_spn_bits(&frame_data, 3, 0, 16);
let rpm_physical = raw_to_physical(rpm_raw, 0.125, 0.0); // ex: 2200.0 rpm
```

## 4.3 Transmitindo Frame J1939

```rust
// Engine Speed broadcast (EEC1)
fn build_eec1_frame(rpm: f64, throttle_pct: f64, sa: u8) -> [u8; 8] {
    let mut data = [0xFFu8; 8]; // inicializa com "não disponível"
    
    // SPN 512: Driver Demand Torque % (offset -125)
    data[1] = (throttle_pct + 125.0).clamp(0.0, 250.0) as u8;
    
    // SPN 513: Actual Engine Torque % (offset -125)
    data[2] = (throttle_pct * 0.8 + 125.0).clamp(0.0, 250.0) as u8;
    
    // SPN 190: Engine Speed (factor 0.125 rpm/bit)
    let spd_raw = (rpm / 0.125) as u16;
    data[3] = (spd_raw & 0xFF) as u8;        // LSB
    data[4] = ((spd_raw >> 8) & 0xFF) as u8;  // MSB
    
    // SPN 1675: Starter Mode = 1 (active) or 0 (not cranking)
    data[6] = 0xF0; // outros bits = não disponível
    
    data
}

let raw_id = encode_j1939_id(3, 61444, 0x00, 0xFF); // priority=3, PGN=EEC1, SA=ECM
// Agora enviar raw_id + data pelo hardware CAN...
```

## 4.4 Implementando Address Claiming

```rust
use std::time::{Duration, Instant};

enum ClaimState {
    SendingClaim,
    Waiting { timer: Instant },
    Claimed { sa: u8 },
    CannotClaim,
}

struct AddressClaimer {
    desired_sa: u8,
    my_name:    u64,
    state:      ClaimState,
}

impl AddressClaimer {
    fn tick(&mut self, received_claims: &[(u8, u64)]) -> Option<[u8; 8]> {
        match &self.state {
            ClaimState::SendingClaim => {
                // Transmitir AC frame
                let mut name_bytes = [0u8; 8];
                for i in 0..8 { name_bytes[i] = ((self.my_name >> (i*8)) & 0xFF) as u8; }
                self.state = ClaimState::Waiting { timer: Instant::now() };
                Some(name_bytes)
            }
            ClaimState::Waiting { timer } => {
                // Verificar conflitos por 250ms
                for &(claimed_sa, claimed_name) in received_claims {
                    if claimed_sa == self.desired_sa && claimed_name != self.my_name {
                        if claimed_name < self.my_name {
                            // Outro nó tem NAME menor → venceu, eu perco
                            // Tentar próximo SA disponível ou ir para null
                            self.desired_sa = self.find_next_sa();
                            self.state = ClaimState::SendingClaim;
                        }
                        // Se meu NAME < claimed_name → venci, continuo esperando
                    }
                }
                if timer.elapsed() >= Duration::from_millis(250) {
                    self.state = ClaimState::Claimed { sa: self.desired_sa };
                }
                None
            }
            ClaimState::Claimed { .. } | ClaimState::CannotClaim => None,
        }
    }
    
    fn find_next_sa(&self) -> u8 {
        // Lógica para encontrar SA alternativo...
        self.desired_sa.wrapping_add(1)
    }
}
```

---

# PARTE 5: REFERÊNCIA RÁPIDA

## 5.1 PGNs Mais Usados em Heavy Machinery

| PGN (dec) | PGN (hex) | Nome | Taxa | Prioridade |
|-----------|-----------|------|------|-----------|
| 61441 | 0xF001 | EBC1 — Electronic Brake Controller 1 | 10ms | 2 |
| 61442 | 0xF002 | ETC1 — Electronic Trans Controller 1 | 20ms | 3 |
| 61443 | 0xF003 | ETC2 — Electronic Trans Controller 2 | 20ms | 3 |
| 61444 | 0xF004 | EEC1 — Engine Speed/Torque | 10ms | 3 |
| 61445 | 0xF005 | EEC2 — Throttle/Load | 50ms | 3 |
| 59904 | 0xEA00 | RQST — Request PGN | On demand | 6 |
| 60928 | 0xEE00 | AC — Address Claim | Boot | 6 |
| 60160 | 0xEC00 | TP.CM — Transport Protocol CM | As needed | 7 |
| 60416 | 0xEB00 | TP.DT — Transport Protocol DT | As needed | 7 |
| 65093 | 0xFE65 | PTO — Power Take-Off | 100ms | 6 |
| 65091 | 0xFE63 | HITCH — Hitch Status | 100ms | 6 |
| 65164 | 0xFEAC | FD — Fan Drive | 500ms | 6 |
| 65226 | 0xFECA | DM1 — Active DTCs | 1Hz | 6 |
| 65227 | 0xFECB | DM2 — Stored DTCs | On request | 6 |
| 65228 | 0xFECC | DM3 — Clear DTCs | On request | 6 |
| 65235 | 0xFED3 | DM11 — Reset | On request | 6 |
| 65247 | 0xFEDF | EEC3 — Engine Control 3 | 250ms | 6 |
| 65248 | 0xFEE0 | VD — Vehicle Distance | 1Hz | 6 |
| 65249 | 0xFEE1 | RCFG — Retarder Config | On request | 6 |
| 65253 | 0xFEE5 | HOURS — Engine Hours | 1Hz | 6 |
| 65257 | 0xFEE9 | FUEL1 — Fuel Consumption | 1Hz | 6 |
| 65262 | 0xFEEE | ET1 — Engine Temperature 1 | 1Hz | 6 |
| 65263 | 0xFEEF | EFL/P1 — Oil/Fuel Press/Level | 500ms | 6 |
| 65265 | 0xFEF1 | CCVS — Vehicle Speed/CC | 100ms | 6 |
| 65266 | 0xFEF2 | LFE — Fuel Economy | 100ms | 6 |
| 65269 | 0xFEF5 | AMB — Ambient Conditions | 1Hz | 6 |
| 65270 | 0xFEF6 | IC1 — Inlet/Exhaust Conditions | 500ms | 6 |

## 5.2 SPNs Mais Importantes

| SPN | Nome | PGN | Factor | Offset | Unit |
|-----|------|-----|--------|--------|------|
| 51 | Throttle Position | LFE | 0.4 | 0 | % |
| 84 | Vehicle Speed | CCVS | 1/256 | 0 | km/h |
| 91 | Accelerator Pedal Position 1 | EEC2 | 0.4 | 0 | % |
| 92 | Engine Load | EEC2 | 0.4 | 0 | % |
| 94 | Fuel Delivery Pressure | EFL/P1 | 4 | 0 | kPa |
| 100 | Engine Oil Pressure | EFL/P1 | 4 | 0 | kPa |
| 102 | Boost Pressure | IC1 | 2 | 0 | kPa |
| 105 | Intake Manifold Temperature | IC1 | 1 | -40 | °C |
| 110 | Engine Coolant Temperature | ET1 | 1 | -40 | °C |
| 161 | Trans Input Shaft Speed | ETC1 | 0.125 | 0 | rpm |
| 173 | Exhaust Gas Temperature | IC1 | 0.03125 | -273 | °C |
| 174 | Fuel Temperature | ET1 | 1 | -40 | °C |
| 175 | Engine Oil Temperature | ET1 | 0.03125 | -273 | °C |
| 183 | Engine Fuel Rate | LFE | 0.05 | 0 | L/h |
| 190 | Engine Speed | EEC1 | 0.125 | 0 | rpm |
| 191 | Trans Output Speed | ETC1 | 0.125 | 0 | rpm |
| 244 | Trip Distance | VD | 0.125 | 0 | km |
| 245 | Total Distance | VD | 0.125 | 0 | km |
| 247 | Total Engine Hours | HOURS | 0.05 | 0 | h |
| 512 | Driver Demand Torque | EEC1 | 1 | -125 | % |
| 513 | Actual Engine Torque | EEC1 | 1 | -125 | % |
| 523 | Trans Current Gear | ETC2 | 1 | -125 | — |
| 524 | Trans Selected Gear | ETC2 | 1 | -125 | — |
| 558 | Accel Pedal 1 Position | EEC2 | 0.4 | 0 | % |
| 1691 | Rear PTO State | PTO | 1 | 0 | enum |
| 1693 | Rear PTO Speed | PTO | 0.125 | 0 | rpm |
| 3251 | DPF Soot Load | Various | 0.4 | 0 | % |
| 3361 | DEF Level | Various | 0.4 | 0 | % |

## 5.3 Tabela de Troubleshooting Rápido

| Sintoma | Causa Provável | Diagnóstico | Solução |
|---------|---------------|-------------|---------|
| Nenhum ECU responde | Sem alimentação, sem terminadores | Meça 60Ω entre H/L, verifique tensão | Adicione terminadores, verifique fuses |
| DM1 contínuo SPN 110 FMI 0 | Refrigerante quente | Leia temperatura atual, inspecione circuito | Verifique radiador, ventoinha, nível |
| DM1 SPN 100 FMI 1 | Pressão de óleo baixa | Verifique nível, leia pressão | Troque óleo, verifique bomba |
| DM1 SPN 94 FMI 4 | Sensor pressão combustível baixo | Medir tensão no sensor | Substitua sensor ou verifique cabeamento |
| TCM não responde | Timeout de heartbeat | Escute barramento por ETC1 | Verifique alimentação TCM, CAN connector |
| Address Conflict | Dois ECUs mesmo SA | Observe AC frames no barramento | Reconfigure SA de um dos ECUs |
| Bus load > 80% | Muitas mensagens, loop | Monitor de frames, identifique fonte | Reduzir taxa de TX, verificar rogue ECU |
| CRC errors frequentes | EMI, mau aterramento | Osciloscópio, verificar blindagem | Melhorar aterramento, separar cabos |
| Bits corrompidos | Terminadores errados, cabo longo | Meça H-L diferencial no osciloscópio | Adicione terminadores corretos |

---

# PARTE 6: GLOSSÁRIO

| Termo | Definição |
|-------|-----------|
| **ACK** | Acknowledgment — confirmação de recebimento |
| **BAM** | Broadcast Announce Message — protocolo de transporte broadcast |
| **BSFC** | Brake Specific Fuel Consumption — consumo específico de combustível |
| **CAN** | Controller Area Network — protocolo de rede serial |
| **CMDT** | Connection Mode Data Transfer — protocolo de transporte unicast |
| **DA** | Destination Address — endereço de destino |
| **DEF** | Diesel Exhaust Fluid (AdBlue) — fluido de redução de NOx |
| **DLC** | Data Length Code — número de bytes no payload |
| **DM** | Diagnostic Message — mensagem de diagnóstico J1939 |
| **DP** | Data Page — seletor de página de PGNs |
| **DPF** | Diesel Particulate Filter — filtro de partículas |
| **DTC** | Diagnostic Trouble Code — código de falha |
| **ECM** | Engine Control Module — módulo de controle do motor |
| **ECU** | Electronic Control Unit — unidade de controle eletrônico |
| **EGR** | Exhaust Gas Recirculation — recirculação de gases |
| **EMI** | Electromagnetic Interference — interferência eletromagnética |
| **EOF** | End Of Frame — fim do frame CAN |
| **FMI** | Failure Mode Identifier — identificador do modo de falha |
| **GE** | Group Extension — extensão de grupo no PGN |
| **HCM** | Hydraulic Control Module |
| **ICM** | Instrument Cluster Module |
| **IFS** | Inter-Frame Space — espaço entre frames |
| **ISOBUS** | ISO 11783 — barramento para máquinas agrícolas |
| **J1939** | SAE standard para redes em veículos pesados |
| **KAM** | Keep Alive Memory — memória mantida sem ignição |
| **LS** | Load Sensing — sensoriamento de carga (hidráulico) |
| **MIL** | Malfunction Indicator Lamp — lâmpada de aviso |
| **NAME** | Identificador único de 64 bits por J1939-81 |
| **NOx** | Óxidos de nitrogênio — gases de emissão regulados |
| **PF** | PDU Format — byte do formato PDU |
| **PGN** | Parameter Group Number — número do grupo de parâmetros |
| **PRV** | Pressure Relief Valve — válvula de alívio de pressão |
| **PS** | PDU Specific — DA ou Group Extension |
| **PTO** | Power Take-Off — tomada de força |
| **REC** | Receive Error Counter — contador de erros de recepção |
| **RTR** | Remote Transmission Request — solicitação de transmissão remota |
| **SA** | Source Address — endereço da fonte |
| **SAE** | Society of Automotive Engineers |
| **SCR** | Selective Catalytic Reduction — redução catalítica seletiva |
| **SOF** | Start Of Frame — início do frame |
| **SPN** | Suspect Parameter Number — número do parâmetro suspeito |
| **TC** | Task Controller (ISOBUS) |
| **TCM** | Transmission Control Module |
| **TEC** | Transmit Error Counter — contador de erros de transmissão |
| **TP** | Transport Protocol — protocolo de transporte J1939 |
| **VGT** | Variable Geometry Turbocharger — turbo de geometria variável |
| **VT** | Virtual Terminal (ISOBUS) |

---

*Manual gerado para o projeto AutoBreaking ECU Simulation Bench*  
*Versão 1.0 — 2026*  
*Baseado em: SAE J1939-21, J1939-71, J1939-73, J1939-81, ISO 11898, ISO 11783*
