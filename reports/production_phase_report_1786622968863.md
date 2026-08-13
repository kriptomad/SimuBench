# Production Program Report

- generated_at_ms: 1786622968863
- mode: sim
- target_sa: None
- overall_passed: false

## UDS Conformance Summary

- passed: true
- SID 0x27: passed=true (flash path not executed)
- SID 0x34: passed=true (flash path not executed)
- SID 0x36: passed=true (flash path not executed)
- SID 0x37: passed=true (flash path not executed)

## Phase Results

- Fase 1 - Baseline e Compliance: Passed (runbook e protocolo presentes)
- Fase 2 - Plataforma Fisica Real: Passed (sim_harness=true: elapsed=6.00s steps=360 speed=0.00kmh)
- Fase 3 - Conexao Confiavel: Passed (can_health=0.924 errors=0)
- Fase 4 - Leitura e Identidade ECM: Passed (vin_ok=true sw_ok=true vin_len=20 sw_len=15)
- Fase 5 - Validacao de Artefato: Blocked (forneca --firmware=<arquivo.bin>)
- Fase 6 - Flash Seguro (Preflight): Passed (battery_v=12.94)
- Fase 7 - Diagnostico e Debug: Passed (uds_retry=3 p2=1000 p2*= 5000 runtime=true: session_sid=0x50 vin_sid=0x62 tester_sid=0x7E)
- Fase 8 - Governanca e Seguranca: Failed (allowlist_present=false noninteractive_approved=false write_effective=false)
- Fase 9 - Confiabilidade e Stress: Passed (rate_global=100 rate_per_id=10 reconnect=500..30000 ms stress=true: max_speed=0.00 fuel=84.92%)
- Fase 10 - Industrializacao: Passed (pipeline de build/test encontrado)
- Fase 11 - Homologacao Final: Passed (gerando dossie json/markdown)
- Fase 12 - Gate de Producao: Failed (ha falhas bloqueantes nas fases anteriores)
