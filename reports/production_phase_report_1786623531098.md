# Production Program Report

- generated_at_ms: 1786623531098
- mode: live
- target_sa: Some(0)
- overall_passed: true

## UDS Conformance Summary

- passed: true
- SID 0x27: passed=true (seed_ok=true unlock_ok=true)
- SID 0x34: passed=true (request_download_positive=true)
- SID 0x36: passed=true (acked=17/17 seq_err=0 fc_timeout=0 stmin_ms=Some(5) bs=Some(8))
- SID 0x37: passed=true (request_transfer_exit_positive=true)

## Phase Results

- Fase 1 - Baseline e Compliance: Passed (runbook e protocolo presentes)
- Fase 2 - Plataforma Fisica Real: Passed (capabilities: can_raw=false isotp=true j1939=false uds_flash=true vendor_bridge=true)
- Fase 3 - Conexao Confiavel: Passed (ECMs detectados: 1)
- Fase 4 - Leitura e Identidade ECM: Passed (fingerprint=true allow_untrusted_ecu=false)
- Fase 5 - Validacao de Artefato: Passed (1024 bytes crc32=0x7BE4DFD0)
- Fase 6 - Flash Seguro (Preflight): Passed (supply_v=12.50 trusted_fp=true | flash_ok bytes=1024 blocks=17 crc32=0x7BE4DFD0 | td_ack=17/17 seq_err=0 fc_timeout=0)
- Fase 7 - Diagnostico e Debug: Passed (uds_retry=3 p2=1000 p2*= 5000 runtime=true: session_sid=0x50 vin_sid=0x62 tester_sid=0x7E)
- Fase 8 - Governanca e Seguranca: Passed (allowlist_present=true noninteractive_approved=true write_effective=true)
- Fase 9 - Confiabilidade e Stress: Passed (rate_global=100 rate_per_id=10 reconnect=500..30000 ms stress=true: max_speed=0.00 fuel=84.92%)
- Fase 10 - Industrializacao: Passed (pipeline de build/test encontrado)
- Fase 11 - Homologacao Final: Passed (gerando dossie json/markdown)
- Fase 12 - Gate de Producao: Passed (gates criticos satisfeitos para o contexto atual)
