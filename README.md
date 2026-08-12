# SimuBench

SimuBench e uma bancada visual de simulacao, diagnostico e engenharia para ECUs de maquinas pesadas, escrita em Rust.

Em um unico aplicativo, o projeto junta:
- simulacao multi-ECU em tempo real;
- painel desktop com tabs operacionais e de diagnostico;
- CAN/J1939, UDS e captura ECM Live;
- Leak Physics Lab com predicao, Monte Carlo e calibracao via CSV de bancada;
- camada de I/O com politicas de seguranca para uso em hardware real.

## Visao Rapida

| Area | O que entrega |
|---|---|
| Simulacao | ECM, TCM, ABS/ESP/TCS, BCM, HCM, VCM, sensores e telematica rodando juntos |
| Diagnostico | Trace CAN, sinais J1939, faults, boot, UDS e rede ECU |
| Dados live | Detect, connect, retrieve e export de dados ECM |
| Engenharia | Leak Lab para risco, ruptura, tuning e calibracao automatica |
| Seguranca | Allowlist, rate limit, dry-run e gates explicitos para escrita |

## Comece Aqui

### 1. Rodar em simulacao

```powershell
cargo run -- --hw-mode=sim
```

### 2. Rodar o binario principal explicitamente

```powershell
cargo run --bin auto_breaking -- --hw-mode=sim
```

### 3. Rodar o runner headless

```powershell
cargo run --bin simulator_cli -- --seed 7 --steps 20000 --model reduced
```

### 4. Validar qualidade

```powershell
cargo check
cargo clippy --workspace --all-targets
cargo test --workspace
```

## O Que Voce Vai Ver Na UI

O app foi dividido em duas frentes:

### Operacao

- `Cluster`: velocidade, RPM, marcha, lamps e sinais principais
- `Engine`: torque, carga, temperaturas, aftertreatment e DTCs ativos
- `Implements`: PTO, hitch, loader, auxiliares e hidraulica
- `Sensors`: GPS, IMU, radar, lidar, camera e coerencia de percepcao
- `Autonomous`: ACC, AEB, LKA, TJA e saidas de comando
- `V2X`: V2V, SPaT, work zones, telematica e OTA
- `Leak Lab`: laboratorio visual para leak/risco/ruptura
- `Plots`: comparacao temporal rapida dos principais canais

### Diagnostico

- `CAN Bus`: sinais, trace e estado dos barramentos
- `Events`: historico de eventos e correlacao temporal
- `ECU Net`: rede de ECUs, presenca e comportamento de boot
- `Faults`: injecao de falhas e observacao de DTCs/DM1
- `Boot`: sequencia de ignicao, interlocks e readiness
- `UDS`: console de servicos ISO 14229
- `ECM Live`: fluxo real/simulado de captura de parametros
- `Help`: leitura guiada do produto e da operacao

## Fluxo Recomendado

Se voce esta chegando agora, use esta ordem:

1. `Help` para entender a navegacao.
2. `Cluster` e `Engine` para validar se a maquina liga, troca marcha e se move.
3. `CAN Bus` e `Events` para ver rede e correlacao temporal.
4. `Faults` para injetar falha controlada e acompanhar DTC/DM1.
5. `UDS` para exercitar sessao, seguranca, leitura e limpeza.
6. `ECM Live` para capturar/exportar sinais.
7. `Leak Lab` para engenharia, risco e calibracao.

## Principais Capacidades

### Simulacao multi-ECU

O orchestrator central integra os modulos de powertrain, chassis, hidraulica, sensores, telematica e diagnostico.

Arquivos principais:
- [src/lib.rs](src/lib.rs)
- [src/main.rs](src/main.rs)

### CAN, J1939 e UDS

Cobertura da pilha de rede:
- gateway e multi-bus CAN;
- estados de saude e injecao de erro;
- decode J1939;
- servidor UDS para ECM e TCM;
- log visual no desktop.

Arquivos principais:
- [src/can_gateway.rs](src/can_gateway.rs)
- [src/can_network.rs](src/can_network.rs)
- [src/j1939.rs](src/j1939.rs)
- [src/uds.rs](src/uds.rs)

### Sensores e autonomia

O projeto inclui sensores sinteticos e fusao para cenarios AD:
- GPS
- IMU
- radar
- lidar
- camera
- fusao de sensores
- controlador autonomo

Arquivos principais:
- [src/gps.rs](src/gps.rs)
- [src/imu.rs](src/imu.rs)
- [src/radar.rs](src/radar.rs)
- [src/lidar.rs](src/lidar.rs)
- [src/camera.rs](src/camera.rs)
- [src/autonomous.rs](src/autonomous.rs)

### Leak Physics Lab

Ambiente de engenharia para circuitos hidraulicos e vedacao:
- runtime por circuito;
- predicao por horizonte e `dt`;
- Monte Carlo;
- export runtime/prediction/catalogos;
- calibracao automatica com CSV de bancada;
- ASCII CAD e visualizacoes de apoio.

Arquivo principal:
- [src/leak_physics.rs](src/leak_physics.rs)

## Leak Lab Em 30 Segundos

### O que ele faz

- mostra leak atual por circuito;
- estima risco de ruptura;
- projeta cenarios;
- roda Monte Carlo;
- calibra coeficientes com dados reais.

### Fluxo rapido

1. Selecione um circuito.
2. Ajuste parametros manuais, se necessario.
3. Clique em `Aplicar + Rodar Cenario` ou `Aplicar + Rodar Monte Carlo`.
4. Leia o `Scenario Ranking`.
5. Use export CSV/JSON para rastreabilidade.

### CSV esperado para calibracao

```text
timestamp_s,circuit_name,pressure_bar,delta_p_bar,temp_c,cycles_per_s,duty_01,fluid_density_kg_m3,measured_leak_lpm,observed_rupture
```

Campos:
- `timestamp_s`: tempo da amostra
- `circuit_name`: nome do circuito
- `pressure_bar`: pressao instantanea
- `delta_p_bar`: diferencial de pressao
- `temp_c`: temperatura do fluido
- `cycles_per_s`: frequencia de ciclagem
- `duty_01`: duty entre `0` e `1`
- `fluid_density_kg_m3`: densidade do fluido
- `measured_leak_lpm`: vazao medida
- `observed_rupture`: ruptura observada (`true/false`)

## Modo Live e Seguranca

O projeto foi desenhado para ser seguro por padrao quando ha I/O real.

### Flags importantes

```text
--hw-mode=sim|live
--vendor-name=cat_comm
--serial-port
--serial-baud
--can-if
--enable-write
--allowlist
--noninteractive-approved
--dry-run / --dry-run=false
--rate-limit-global
--rate-limit-per-id
--log-dir
```

### Gates de escrita fisica

- `enable-write`
- allowlist valida
- `noninteractive-approved`
- `dry-run` explicitamente desabilitado

Arquivos principais:
- [src/io/hw.rs](src/io/hw.rs)
- [src/io/live_runner.rs](src/io/live_runner.rs)
- [docs/ECM-Data.md](docs/ECM-Data.md)

## Binarios

O repositorio possui dois executaveis:

- `auto_breaking`: aplicacao desktop principal
- `simulator_cli`: runner headless para reproducao e carga

Arquivo principal do CLI:
- [src/bin/simulator_cli.rs](src/bin/simulator_cli.rs)

## Estrutura Rapida do Repositorio

```text
AutoBreaking/
├── README.md
├── Cargo.toml
├── src/
│   ├── lib.rs
│   ├── main.rs
│   ├── leak_physics.rs
│   └── io/
├── tests/
├── docs/
├── reports/
└── artifacts/
```

Arquivos que valem abrir primeiro:
- [src/main.rs](src/main.rs): UI desktop, tabs e fila de comandos
- [src/lib.rs](src/lib.rs): orchestrator principal
- [src/leak_physics.rs](src/leak_physics.rs): Leak Lab
- [src/io/hw.rs](src/io/hw.rs): configuracao e politicas de hardware
- [src/io/live_runner.rs](src/io/live_runner.rs): fluxo ECM live

## Qualidade e Testes

Comandos recomendados:

```powershell
cargo fmt
cargo clippy --workspace --all-targets
cargo clippy --test io_mock
cargo test --locked --workspace
cargo test --test io_mock
cargo check
```

Suites importantes:
- [tests/io_mock.rs](tests/io_mock.rs)
- [tests/leak_system_integration.rs](tests/leak_system_integration.rs)
- [tests/property_invariants.rs](tests/property_invariants.rs)
- [tests/speed_regression.rs](tests/speed_regression.rs)
- [tests/system_failure_scenarios.rs](tests/system_failure_scenarios.rs)

## Documentacao Auxiliar

- [TESTING.md](TESTING.md): guia de testes praticos
- [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md): resumo tecnico do projeto
- [CODE_EXAMPLES.md](CODE_EXAMPLES.md): exemplos e referencia rapida
- [J1939_CAN_MANUAL.md](J1939_CAN_MANUAL.md): material focado em rede
- [AUDIT_FULL_2026-08-12.md](AUDIT_FULL_2026-08-12.md): auditoria consolidada

## Estado Atual

O projeto esta organizado como uma bancada integrada de simulacao + diagnostico + engenharia, com UI desktop, fluxo headless, cobertura de testes e runbook para operacao simulada ou live.

Se a sua entrada no projeto for pratica, use primeiro:
- [README.md](README.md)
- [TESTING.md](TESTING.md)
- [docs/ECM-Data.md](docs/ECM-Data.md)

Se a sua entrada for tecnica, abra primeiro:
- [src/lib.rs](src/lib.rs)
- [src/main.rs](src/main.rs)
- [src/leak_physics.rs](src/leak_physics.rs)


