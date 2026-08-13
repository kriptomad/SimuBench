# Production Program Report

- generated_at_ms: 1786622112533
- mode: sim
- target_sa: None
- overall_passed: false

## Phase Results

- Fase 1 - Baseline e Compliance: Passed (runbook e protocolo presentes)
- Fase 2 - Plataforma Fisica Real: Passed (sim_harness=true: elapsed=6.00s steps=360 speed=0.00kmh)
- Fase 3 - Conexao Confiavel: Passed (can_health=0.924 errors=0)
- Fase 4 - Leitura e Identidade ECM: Passed (vin_ok=true sw_ok=true vin_len=20 sw_len=15)
- Fase 5 - Validacao de Artefato: Passed (64 bytes crc32=0x2880FB99)
- Fase 6 - Flash Seguro (Preflight): Passed (battery_v=12.94)
- Fase 7 - Diagnostico e Debug: Failed (uds_retry=3 p2=1000 p2*= 5000 runtime=false: session_sid=0x7F vin_sid=0x62 tester_sid=0x7E)
- Fase 8 - Governanca e Seguranca: Passed (allowlist_present=true noninteractive_approved=true write_effective=false)
- Fase 9 - Confiabilidade e Stress: Passed (rate_global=100 rate_per_id=10 reconnect=500..30000 ms stress=true: max_speed=0.00 fuel=84.92%)
- Fase 10 - Industrializacao: Passed (pipeline de build/test encontrado)
- Fase 11 - Homologacao Final: Passed (gerando dossie json/markdown)
- Fase 12 - Gate de Producao: Failed (ha falhas bloqueantes nas fases anteriores)
