# SimuBench

SimuBench e uma bancada completa de simulacao e diagnostico de ECUs para maquinas pesadas, escrita em Rust.

O projeto integra em um unico app:
- simulacao multi-ECU em tempo real;
- monitoramento CAN/J1939 e UDS;
- fluxo de dados ECM live com exportacao;
- Leak Physics Lab com predicao, Monte Carlo e calibracao automatica por CSV de bancada;
- camada de I/O com politicas de seguranca (allowlist, rate limit, dry-run e auditoria).

## Sumario

- [Visao geral rapida](#visao-geral-rapida)
- [Arquitetura do projeto](#arquitetura-do-projeto)
- [Executaveis](#executaveis)
- [Como rodar](#como-rodar)
- [Fluxo da interface (tabs)](#fluxo-da-interface-tabs)
- [Leak Lab e modo calibracao](#leak-lab-e-modo-calibracao)
- [Seguranca para I/O real](#seguranca-para-io-real)
- [Testes e qualidade](#testes-e-qualidade)
- [Mapa de arquivos](#mapa-de-arquivos)

## Visao geral rapida

- Linguagem: Rust (edition 2021)
- UI desktop: eframe/egui
- Plataforma foco: Windows (com suporte de modo sim completo)
- Branch principal: master

Entrypoints principais:
- Orquestrador e modulos: [src/lib.rs](src/lib.rs)
- Aplicacao desktop: [src/main.rs](src/main.rs)
- Fisica de vazamento/calibracao: [src/leak_physics.rs](src/leak_physics.rs)
- I/O live e seguranca: [src/io/hw.rs](src/io/hw.rs)

## Arquitetura do projeto

### 1. Core de simulacao multi-ECU

O orchestrator central instancia e integra ECM, TCM, BCM, ICM, HCM, ABS/ESP, VCM, sensores e telematica.

Referencia:
- [src/lib.rs](src/lib.rs)

### 2. Rede CAN/J1939 e UDS

Cobertura de monitoramento e diagnostico de rede:
- transporte e gateway;
- multi-bus CAN com estados de saude;
- decode J1939;
- servidor UDS para fluxos de diagnostico.

Referencias:
- [src/can_gateway.rs](src/can_gateway.rs)
- [src/can_network.rs](src/can_network.rs)
- [src/j1939.rs](src/j1939.rs)
- [src/uds.rs](src/uds.rs)

### 3. Pilha de sensores/autonomia

Inclui GPS, IMU, radar, lidar, camera e fusao para cenarios AD.

Referencias:
- [src/gps.rs](src/gps.rs)
- [src/imu.rs](src/imu.rs)
- [src/radar.rs](src/radar.rs)
- [src/lidar.rs](src/lidar.rs)
- [src/camera.rs](src/camera.rs)
- [src/autonomous.rs](src/autonomous.rs)

### 4. Leak Physics Lab

Ambiente de engenharia para circuitos hidraulicos e de vedacao:
- simulacao de degradacao e risco de ruptura;
- predicao por cenario;
- Monte Carlo;
- exportacao runtime/prediction/catalogos;
- calibracao automatica com dados reais de bancada (CSV).

Referencia:
- [src/leak_physics.rs](src/leak_physics.rs)

## Executaveis

O repositorio possui dois bins:
- auto_breaking: app desktop principal
- simulator_cli: runner headless

Referencia:
- [src/bin/simulator_cli.rs](src/bin/simulator_cli.rs)

## Como rodar

### Rodar app desktop (modo simulacao)

```powershell
cargo run -- --hw-mode=sim
```

### Rodar binario explicito

```powershell
cargo run --bin auto_breaking -- --hw-mode=sim
```

### Rodar CLI headless

```powershell
cargo run --bin simulator_cli -- --seed 7 --steps 20000 --model reduced
```

### Build com feature Windows vendor

```powershell
cargo build --release --features "vendor-windows"
```

## Fluxo da interface (tabs)

Tabs principais no app ([src/main.rs](src/main.rs)):
1. Cluster
2. CAN Bus
3. Events
4. ECU Net
5. Engine
6. Faults
7. Boot
8. Implements
9. Params
10. Sensors
11. Autonomous
12. V2X
13. UDS
14. ECM Live
15. Leak Lab
16. Plots

Sugestao de fluxo de uso rapido:
1. Inicie em modo sim.
2. Use Cluster/Engine para validar dinamica basica.
3. Va para CAN Bus + Events para diagnostico.
4. Use UDS para exercitar servicos de diagnostico.
5. Use ECM Live para captura e exportacao de historico.
6. Finalize no Leak Lab para analise de risco e calibracao por CSV.

## Leak Lab e modo calibracao

### O que o Leak Lab entrega

- runtime em tempo real por circuito;
- predicao por horizonte e dt;
- Monte Carlo;
- export CSV/JSON de runtime e prediction;
- export de catalogos de materiais e oleos;
- calibracao por CSV de bancada com ajuste automatico de coeficientes.

### Fluxo de calibracao (UI)

No tab Leak Lab:
1. Clique em Select CSV
2. Escolha o arquivo de bancada
3. Clique em Run Auto Calibration
4. Veja o resumo por circuito (RMSE, MAPE, acuracia de ruptura)
5. Exporte o relatorio em CSV/JSON

### CSV esperado para calibracao

Cabecalho esperado:

```text
timestamp_s,circuit_name,pressure_bar,delta_p_bar,temp_c,cycles_per_s,duty_01,fluid_density_kg_m3,measured_leak_lpm,observed_rupture
```

Descricao rapida de campos:
- timestamp_s: tempo da amostra
- circuit_name: nome do circuito (deve bater com circuito conhecido)
- pressure_bar: pressao instantanea
- delta_p_bar: diferencial de pressao
- temp_c: temperatura do fluido
- cycles_per_s: frequencia de ciclagem
- duty_01: duty cycle entre 0 e 1
- fluid_density_kg_m3: densidade
- measured_leak_lpm: vazao medida de leak
- observed_rupture: true/false opcional

### O que e ajustado automaticamente

A calibracao faz busca deterministica (grid search) por circuito para ajustar:
- damage_rate_scale
- extrusion_rate_scale
- thermal_rate_scale
- flow_rate_scale
- rupture_area_scale

Saidas do relatorio:
- por circuito;
- agregado por material;
- agregado por oleo.

## Seguranca para I/O real

A camada de I/O foi desenhada com default seguro.

Diretorio:
- [src/io](src/io)

Gates para escrita fisica:
- enable-write
- allowlist valida
- noninteractive-approved
- dry-run desabilitado de forma explicita

Flags importantes:
- --hw-mode=sim|live
- --vendor-name=cat_comm
- --serial-port
- --serial-baud
- --can-if
- --enable-write
- --allowlist
- --noninteractive-approved
- --dry-run / --dry-run=false
- --rate-limit-global
- --rate-limit-per-id
- --log-dir

## Testes e qualidade

Comandos recomendados:

```powershell
cargo fmt
cargo clippy --workspace --all-targets
cargo clippy --test io_mock
cargo test --locked --workspace
cargo test --test io_mock
cargo check
```

Suites de referencia:
- [tests/io_mock.rs](tests/io_mock.rs)
- [tests/leak_system_integration.rs](tests/leak_system_integration.rs)
- [tests/property_invariants.rs](tests/property_invariants.rs)
- [tests/system_failure_scenarios.rs](tests/system_failure_scenarios.rs)

## Mapa de arquivos

- [src/main.rs](src/main.rs): UI desktop, tabs e fila de comandos
- [src/lib.rs](src/lib.rs): orchestrator principal e API publica
- [src/leak_physics.rs](src/leak_physics.rs): fisica de leak, predicoes e calibracao
- [src/io/hw.rs](src/io/hw.rs): configuracao de hardware e politicas de runtime
- [src/io/live_runner.rs](src/io/live_runner.rs): ciclo live ECM
- [src/io/ecm_params.rs](src/io/ecm_params.rs): decode de parametros live
- [docs/ECM-Data.md](docs/ECM-Data.md): runbook de dados ECM
- [CHANGELOG.md](CHANGELOG.md): historico de mudancas


