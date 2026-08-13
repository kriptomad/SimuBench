# Production Program Report

- generated_at_ms: 1786621709828
- mode: sim
- target_sa: None
- overall_passed: false

## Phase Results

- Fase 1 - Baseline e Compliance: Passed (runbook e protocolo presentes)
- Fase 2 - Plataforma Fisica Real: Blocked (requer --hw-mode=live)
- Fase 3 - Conexao Confiavel: Blocked (hardware live indisponivel)
- Fase 4 - Leitura e Identidade ECM: Blocked (requer modo live)
- Fase 5 - Validacao de Artefato: Blocked (forneca --firmware=<arquivo.bin>)
- Fase 6 - Flash Seguro (Preflight): Blocked (requer modo live)
- Fase 7 - Diagnostico e Debug: Passed (uds_retry=3 p2=1000 p2*= 5000)
- Fase 8 - Governanca e Seguranca: Failed (allowlist_present=false noninteractive_approved=false)
- Fase 9 - Confiabilidade e Stress: Passed (rate_global=100 rate_per_id=10 reconnect=500..30000 ms)
- Fase 10 - Industrializacao: Passed (pipeline de build/test encontrado)
- Fase 11 - Homologacao Final: Passed (dossie json/markdown sera gerado)
- Fase 12 - Gate de Producao: Failed (ha falhas bloqueantes nas fases anteriores)
