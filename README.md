
![Rust](https://img.shields.io/badge/Rust-000000?style=for-the-badge&logo=rust&logoColor=white)

![Top language](https://img.shields.io/github/languages/top/kriptomad/SimuBench)


SimuBench e uma bancada visual completa de simulacao, diagnostico, calibracao e engenharia para ECUs de maquinas pesadas, escrita em Rust.

Desenvolvida como ambiente de engenharia real — cada modulo implementa protocolos, fisicas e comportamentos do hardware fisico correspondente, incluindo J1939, UDS/ISO 14229, CAN, sistemas hidraulicos, ABS/ESP, motores diesel Tier 4 e transmissoes powershift.

---

## O que o SimuBench entrega

| Dominio | Capacidades |
|---|---|
| **Simulacao multi-ECU** | ECM, TCM, ABS/ESP/TCS, BCM, HCM, VCM, ICM rodando juntos em tempo real a 60 Hz |
| **Motor diesel** | Torque curve realista, BSFC variavel com carga, turbo, termica, alternador, CRDI, DPF/SCR/EGR |
| **Transmissao** | Powershift 16F/16R, clutch fill/modulate/lock, auto-shift adaptativo, shuttle eletronico D-R real |
| **Hidraulica** | Bomba load-sensing, atuadores com fisica de orificio, acumulador, temperatura de fluido |
| **ABS/ESP/TCS** | Fisica de roda com inercia rotacional, v_ref por rodas nao bloqueadas, rollover 0,4 g |
| **IMU realista** | Madgwick AHRS, vibracao de motor, bias de giroscopio, roll/pitch/yaw com fisica correta |
| **Diagnostico CAN/J1939** | Trace, sinais decodificados, filtro, pausar, arvore por PGN/SA, export |
| **UDS / ISO 14229** | Console UDS, flash de firmware, leitura DID, security access, DTC management |
| **ECM Live** | Detect ECMs em rede, connect, snapshot de parametros, export |
| **Remap ECU profissional** | Mapas 3D/2D heat-map + projecao isometrica, editor HEX ROM 64 KB, patches DPF/EGR/speed delete, bit fields, live cursor |
| **Leak Physics Lab** | Modelo fisico de ruptura, Monte Carlo, calibracao via CSV, predicao de risco temporal |
| **Producao/Flash** | Live runner UDS, allowlist, preflight, conformance report, vendor bridge template |
| **CI/CD** | GitHub Actions: check, testes, mock I/O, vendor bridge E2E, nightly Monte Carlo |

---

## Arquitetura

```
src/
+-- lib.rs                  HeavyMachinery: tick master, integracao de todos os ECUs
+-- main.rs                 UI desktop egui, 20+ tabs
+-- ecu_ecm.rs              Motor diesel Tier 4, J1939 EEC1/EEC2/ET1/LFE/DM1
+-- ecu_tcm.rs              Transmissao powershift, shuttle eletronico, clutch pack
+-- ecu_abs.rs              ABS/ESP/TCS, fisica de roda, rollover, hill hold
+-- ecu_bcm.rs              BCM, eletrica, HVAC, iluminacao, bateria/alternador
+-- ecu_hcm.rs              Hidraulica load-sensing, 3-point hitch, loader
+-- ecu_rom.rs              ROM 64 KB, 6 mapas 3D, 6 curvas 2D, patches, bit fields
+-- electrical.rs           Rede eletrica com resistencias, fusiveis, reles
+-- imu.rs                  IMU Madgwick AHRS, vibracao, bias, gravidade projetada
+-- chassis.rs              Dinamica de chassi auxiliar (ABS, TCS, ESP)
+-- j1939.rs                Builder/decoder J1939 todos os PGNs principais
+-- uds.rs                  Stack UDS completo (0x10/0x22/0x27/0x34/0x36/0x37)
+-- io/
|   +-- hw.rs               HwConfig, HwCapabilities, probe_live_adapter
|   +-- live_runner.rs      Flash ECM real, UDS transport, conformance
|   +-- vendor_cat_comm.rs  Adapter Cat Comm (vendor-windows feature)
|   +-- production_program.rs 12 fases de producao, gate, relatorio JSON/MD
|   +-- allowlist.rs        Whitelist CAN/serial com rate limiting
|   +-- mock.rs             Mock adapter para testes sem hardware
|   +-- serial_adapter.rs   Serial real (Linux/Windows)
|   +-- socketcan_adapter.rs SocketCAN Linux
|   +-- replay.rs           Record/replay de trafego CAN
|   +-- metrics.rs          Metricas de transporte (ack, seq_err, fc_timeout)
+-- bin/
    +-- simulator_cli.rs    CLI: validate-bridge, run-production-phases, flash
    +-- cat_comm_bridge.rs  Bridge de referencia stdio-JSONL para template vendor
```

---

## Compiling Section

Todas as formas possiveis de compilar, inicializar, testar e empacotar o SimuBench.

### Pre-requisitos

```powershell
# Instalar Rust (se nao tiver)
winget install Rustlang.Rustup          # Windows
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh  # Linux/macOS

# Verificar versao minima (edition 2021, estavel)
rustc --version    # recomendado >= 1.78
cargo --version
```

---

### Build — Debug (desenvolvimento)

```powershell
# Compilar sem rodar (rapido, sem otimizacoes)
cargo build

# Compilar com feature vendor-windows (adapter Cat Comm)
cargo build --features vendor-windows

# Compilar com observabilidade avancada (tracing estruturado)
cargo build --features advanced_observability

# Compilar todas as features juntas
cargo build --features vendor-windows,advanced_observability

# Compilar apenas a biblioteca (sem binarios)
cargo build --lib
```

---

### Build — Release (producao / distribuicao)

```powershell
# Release otimizado (LTO, sem debug info) — binario final
cargo build --release

# Release com vendor-windows
cargo build --release --features vendor-windows

# Release com todas as features
cargo build --release --features vendor-windows,advanced_observability

# Caminho do binario gerado
# Windows: target\release\auto_breaking.exe
# Linux:   target/release/auto_breaking
```

---

### Executar — GUI Desktop

```powershell
# Modo padrao (simulacao pura)
cargo run

# Modo release (mais rapido, sem debug info)
cargo run --release

# Passar argumentos de hardware ao GUI
cargo run -- --hw-mode=sim
cargo run --features vendor-windows -- --hw-mode=live --vendor-name=cat_comm --vendor-template-dir=C:/cat/template
```

---

### Executar — CLI (simulator_cli)

```powershell
# Validar bridge Cat Comm (handshake + capabilities)
cargo run --features vendor-windows --bin simulator_cli -- `
  --validate-cat-bridge `
  --hw-mode=live `
  --vendor-name=cat_comm `
  --vendor-template-dir=C:/cat/template

# Executar 12 fases de producao em modo simulado
cargo run --bin simulator_cli -- `
  --run-production-phases `
  --hw-mode=sim `
  --noninteractive-approved `
  --phase-report-dir=reports

# Executar 12 fases de producao com flash em hardware real
cargo run --features vendor-windows --bin simulator_cli -- `
  --run-production-phases `
  --hw-mode=live `
  --vendor-name=cat_comm `
  --vendor-template-dir=C:/cat/template `
  --enable-write `
  --noninteractive-approved `
  --dry-run=false `
  --allowlist=allowlist.example.json `
  --target-sa=00 `
  --firmware=firmware.bin `
  --phase-report-dir=reports `
  --execute-flash

# Versao release da CLI (producao)
cargo run --release --features vendor-windows --bin simulator_cli -- `
  --run-production-phases --hw-mode=live `
  --vendor-name=cat_comm --vendor-template-dir=C:/cat/template `
  --enable-write --noninteractive-approved --dry-run=false `
  --allowlist=allowlist.example.json --target-sa=00 `
  --firmware=firmware.bin --phase-report-dir=reports --execute-flash
```

---

### Executar — Bridge de Referencia

```powershell
# Rodar o bridge de referencia diretamente (stdio-JSONL)
cargo run --bin cat_comm_bridge

# Com feature vendor-windows ativo
cargo run --features vendor-windows --bin cat_comm_bridge

# Compilar e copiar para diretorio de template
cargo build --features vendor-windows --bin cat_comm_bridge
Copy-Item target\debug\cat_comm_bridge.exe C:\cat\template\

# Script completo de build + stage
powershell -NoProfile -ExecutionPolicy Bypass `
  -File scripts/build_cat_comm_template_bridge.ps1
```

---

### Testes

```powershell
# Todos os testes (workspace completo)
cargo test --workspace

# Testes da lib principal com output visivel
cargo test --lib -- --nocapture

# Suite especifica
cargo test --test io_mock
cargo test --test speed_regression
cargo test --test system_failure_scenarios
cargo test --test property_invariants
cargo test --test leak_system_integration

# E2E vendor bridge (requer feature + Windows)
cargo test --features vendor-windows --test vendor_bridge_e2e

# Testes com feature observabilidade
cargo test --workspace --features advanced_observability

# Rodar um teste especifico pelo nome
cargo test --workspace nome_do_teste -- --nocapture
```

---

### Verificacao de Compilacao (sem build completo)

```powershell
# Check padrao (mais rapido que build)
cargo check

# Check com feature vendor-windows
cargo check --features vendor-windows

# Check com todas as features
cargo check --features vendor-windows,advanced_observability

# Check de todos os targets (bins + tests + examples)
cargo check --all-targets

# Clippy (linter)
cargo clippy
cargo clippy --features vendor-windows -- -D warnings
```

---

### Automacao Windows Completa

```powershell
# Build + stage do bridge + validar handshake + 12 fases + flash + relatorio
powershell -NoProfile -ExecutionPolicy Bypass `
  -File scripts/run_windows_template_e2e.ps1

# Apenas build + stage do bridge
powershell -NoProfile -ExecutionPolicy Bypass `
  -File scripts/build_cat_comm_template_bridge.ps1
```

---

### Limpeza

```powershell
# Remover artefatos de build
cargo clean

# Remover apenas artefatos debug (manter release)
Remove-Item -Recurse -Force target\debug

# Rebuild completo a partir do zero
cargo clean; cargo build
```

---

### Resumo Rapido

| Objetivo | Comando |
|---|---|
| Compilar debug | `cargo build` |
| Compilar release | `cargo build --release` |
| Rodar GUI (sim) | `cargo run` |
| Rodar GUI (release) | `cargo run --release` |
| CLI sim 12 fases | `cargo run --bin simulator_cli -- --run-production-phases --hw-mode=sim --noninteractive-approved --phase-report-dir=reports` |
| CLI live + flash | `cargo run --features vendor-windows --bin simulator_cli -- --run-production-phases --hw-mode=live --vendor-name=cat_comm --vendor-template-dir=C:/cat/template --enable-write --noninteractive-approved --dry-run=false --allowlist=allowlist.example.json --target-sa=00 --firmware=firmware.bin --phase-report-dir=reports --execute-flash` |
| Validar bridge | `cargo run --features vendor-windows --bin simulator_cli -- --validate-cat-bridge --hw-mode=live --vendor-name=cat_comm --vendor-template-dir=C:/cat/template` |
| Todos os testes | `cargo test --workspace` |
| E2E vendor bridge | `cargo test --features vendor-windows --test vendor_bridge_e2e` |
| Automacao Windows | `powershell -File scripts/run_windows_template_e2e.ps1` |
| Lint / check | `cargo check --features vendor-windows` |
| Limpar build | `cargo clean` |

---

## Quickstart

```powershell
# Simulacao desktop
cargo run

# Verificar compilacao
cargo check
cargo check --features vendor-windows

# Testes completos
cargo test --workspace

# E2E vendor bridge
cargo test --features vendor-windows --test vendor_bridge_e2e

# CLI: validar bridge Cat Comm
cargo run --features vendor-windows --bin simulator_cli -- `
  --validate-cat-bridge --hw-mode=live --vendor-name=cat_comm `
  --vendor-template-dir=C:/cat/template

# CLI: 12 fases de producao com flash
cargo run --features vendor-windows --bin simulator_cli -- `
  --run-production-phases --hw-mode=live --vendor-name=cat_comm `
  --vendor-template-dir=C:/cat/template --enable-write `
  --noninteractive-approved --dry-run=false `
  --allowlist=allowlist.example.json `
  --target-sa=00 --firmware=firmware.bin `
  --phase-report-dir=reports --execute-flash

# Automacao Windows completa (build + stage + E2E)
powershell -NoProfile -ExecutionPolicy Bypass `
  -File scripts/run_windows_template_e2e.ps1
```

---

## Features Cargo

| Feature | Ativa |
|---|---|
| `default` | Simulacao pura sem dependencias externas |
| `vendor-windows` | Adapter Cat Comm, path de template vendor, E2E bridge |
| `advanced_observability` | Tracing estruturado com tracing/tracing-subscriber |

---

## Tabs da Interface

| Tab | Conteudo |
|---|---|
| CLUSTER | Painel de instrumentos: velocidade, RPM, temperatura, pressoes |
| ELECTRICAL | Rede eletrica com branches, fusiveis, reles, fault injection |
| ENGINE | Motor, turbo, aftertreatment, consumo, DPF/SCR/EGR |
| IMPL | Implementos, hitch 3 pontos, loader, PTO |
| AD | Autonomous Driving: ACC, LKA, AEB, sensor fusion |
| LEAK LAB | Fisica de ruptura hidraulica, Monte Carlo, calibracao via CSV |
| PLOTS | Graficos temporais: RPM, velocidade, torque, boost, DPF, coolant |
| CAN | Trace CAN/J1939 decode por PGN, sinalizacao, export |
| EVENTS | Log estruturado de eventos com filtro e nivel |
| ECU NET | Rede J1939, status online, source addresses |
| FAULTS | DTC ativo e historico, severidade, clear |
| BOOT | Boot sequence, estado de ignicao, cranking |
| PARAMS | Sliders de parametros de simulacao |
| SENSORS | IMU, GPS, radar, camera, LIDAR, fusao |
| V2X | V2X / telematica com KPIs e trajetoria |
| UDS | Console UDS, flash de firmware, DID, security access |
| ECM LIVE | Detect/connect/snapshot de ECM em hardware real |
| REMAP | Remap profissional: mapas 3D/2D, editor HEX, patches, ROM bits, live cal |

---

## Painel de Remap ECU

Replica o ambiente de ferramentas profissionais (ECU Titanium, WinOLS, Hondata):

- **MAPS 3D** — 6 mapas 3D (Combustivel, Avanco, VGT Boost, Lambda, VE, EGR) com heat-map
  colorido (blue->green->red) e projecao isometrica 3D interativa. Cursor live no ponto de
  operacao atual.
- **MAPS 2D** — 6 curvas (Idle Speed, Torque Limit, Boost Limit, Pilot Injection, Rail Pressure,
  Cold Start Advance) com grafico editavel e tabela de pontos.
- **HEX ROM** — Editor HEX de ROM virtual 64 KB com 16 bytes/linha, enderecos, hex, ASCII,
  navegacao por regiao nomeada, escrita de byte individual.
- **PATCHES** — 16 patches com endereco real, valor original e patched: DPF delete, EGR delete,
  NOx delete, AdBlue delete, Speed limiter remove, Torque derate disable e mais.
- **ROM BITS** — 13 bit fields editaveis individualmente com endereco e mascara.
- **LIVE CAL** — Sliders em tempo real para todos os parametros de motor e transmissao, com
  flags de delete aplicadas ao ECM ao vivo.

---

## Shuttle Eletronico D-R Real

Sequencia real de inversao de direcao em powershift:

1. Solicitacao de R em D: **inibido** se velocidade > limiar (padrao 4 km/h, configuravel)
2. Motor mantém idle; rodas freiam naturalmente
3. Ao atingir o limiar: embreagem OPEN, direcao vai para Neutro
4. **Dwell timer** (0,40 s padrao) para proteger os discos molhados
5. Apos dwell: begin_shift para R-A1 (primeira marcha de re)
6. Embreagem: FILL -> MODULANDO -> LOCKED

Todos os parametros do shuttle sao editaveis no painel Remap.

---

## Correcoes de Realismo Fisico e Protocolo

| Area | Correcao |
|---|---|
| ABS/ESP steering | ESP nao recebe mais clutch_slip_pct como angulo de direcao |
| ABS lateral g | lateral_g = 0 substituido por valor real do IMU |
| ESP yaw rate | velocidade x 0,01 substituido por imu.gyro_z real |
| Rollover threshold | 0,8 g -> 0,4 g (maquinario pesado capota entre 0,35-0,45 g) |
| ABS v_ref | Derivado das 2 rodas mais rapidas nao bloqueadas (nao mais do TCM) |
| ABS fisica | Inercia rotacional real (J=6 kg.m2) em vez de formula empirica |
| Temperatura de oleo | Lag termico de ~5 min (massa propria), nao mais instantanea |
| Resfriamento motor | Motor agora esfria em direcao ao ambiente apos desligamento |
| Temperatura turbo | Oleo do turbo aprox. oleo do motor (nao 60% da exaustao = 384C) |
| Temperatura admissao | Aquecimento apenas por boost gauge (nao pressao absoluta) |
| BSFC | Penalidade em carga leve agora quadratica realista (~420 g/kWh a 10%) |
| DPF regeneracao | 33 min para zerar (nao 33 segundos) |
| DPF acumulacao | Taxa aumentada 3x para intervalo realista (regen a cada 8-10 h) |
| DPF threshold | 50% (nao 75%) |
| SCR eficiencia | Reduz em carga alta (estava invertido) |
| NOx baseline | 150 ppm em idle (era 800 ppm) |
| EEC1 taxa J1939 | 100 ms (era 10 ms — 10x excessivo) |
| ETC2 taxa J1939 | 100 ms (era 20 ms) |
| IMU roll/pitch | Derivados de aceleracao lateral/longitudinal (nao zero fixo) |
| IMU temperatura | Ambiente + auto-aquecimento (nao correlacionada ao RPM) |
| Gravidade em rampa | Projetada por cos(pitch) no eixo Z |
| Acumulador hidraulico | 3 bar/s com sangria ao parar (nao 20 bar/s) |
| Pressao piloto | Redutor regulado a 38 bar (nao system/10) |
| Alternador | Lag de campo ~200 ms, corrente proporcional a carga/SoC |
| Tensao bateria | Inclui queda por carga (Rint = 12 mOhm) |
| Master cylinder | 220 bar (maquina pesada, nao 150 bar de carro) |
| Massa do veiculo | F=ma (20 t, resistencia de rolamento, arrasto) |
| Raio do pneu | 0,655 m (440/80R24 real, era 0,80 m: +22% erro de velocidade) |
| Temperatura cab | Drift para temperatura ambiente sempre ativo |

---

## Relatorio de Producao

Apos execucao das 12 fases, o runner gera JSON + Markdown:

```json
{
  "overall_passed": true,
  "mode": "live",
  "execute_flash": true,
  "conformance_summary": {
    "passed": true,
    "services": [
      { "service_sid": "0x27", "passed": true, "evidence": "seed_ok=true unlock_ok=true" },
      { "service_sid": "0x34", "passed": true, "evidence": "request_download_positive=true" },
      { "service_sid": "0x36", "passed": true, "evidence": "acked=9/9 seq_err=0 fc_timeout=0" },
      { "service_sid": "0x37", "passed": true, "evidence": "request_transfer_exit_positive=true" }
    ]
  }
}
```

---

## Versao

**v0.3.0** — Agosto 2026

- Motor diesel Tier 4 Final com DPF/SCR/EGR completo
- Shuttle eletronico com sequencia real D-R
- Remap ECU profissional (mapas 3D/2D, HEX, patches, ROM bits)
- 27 correcoes de realismo fisico (ABS, ESP, IMU, hidraulica, termica, protocolo J1939)
- Vendor bridge Cat Comm com E2E validado
- 12 fases de producao com conformance report UDS
- Analise intrinsica e correcao de anomalias vs. comportamento de maquinario real
