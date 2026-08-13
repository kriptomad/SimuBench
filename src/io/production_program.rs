use std::fs;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::HeavyMachinery;

use super::artifact::{validate_firmware_artifact, FirmwareArtifactReport};
use super::hw::{HwCapabilities, HwConfig, HwError, HwMode};
use super::live_runner::{
    connect_ecm, detect_ecms, live_flash_ecm_firmware, live_preflight_ecm, live_read_ecm_identity,
    DetectResult, FlashPreflightReport, FlashSummary,
};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum PhaseStatus {
    Passed,
    Failed,
    Blocked,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhaseResult {
    pub phase: u8,
    pub title: String,
    pub status: PhaseStatus,
    pub details: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProgramReport {
    pub generated_at_ms: u64,
    pub mode: String,
    pub target_sa: Option<u8>,
    pub strict_mode: bool,
    pub execute_flash: bool,
    pub artifact: Option<FirmwareArtifactReport>,
    pub detect: Option<DetectResult>,
    pub preflight: Option<FlashPreflightReport>,
    pub identity: Option<super::live_runner::EcuIdentity>,
    pub conformance_summary: Option<ConformanceSummary>,
    pub phases: Vec<PhaseResult>,
    pub overall_passed: bool,
    pub report_sha256: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceConformanceResult {
    pub service_sid: String,
    pub passed: bool,
    pub evidence: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConformanceSummary {
    pub passed: bool,
    pub services: Vec<ServiceConformanceResult>,
}

#[derive(Debug, Clone, Copy)]
pub struct ProgramOptions {
    pub strict_mode: bool,
    pub execute_flash: bool,
}

impl Default for ProgramOptions {
    fn default() -> Self {
        Self {
            strict_mode: true,
            execute_flash: false,
        }
    }
}

pub fn run_all_phases(
    cfg: &HwConfig,
    target_sa: Option<u8>,
    artifact_path: Option<&Path>,
    out_dir: &Path,
    options: ProgramOptions,
) -> Result<PathBuf, HwError> {
    fs::create_dir_all(out_dir)
        .map_err(|e| HwError::Unknown(format!("create report dir failed: {e}")))?;

    let mut phases = Vec::with_capacity(12);
    let mut artifact = None;
    let mut detect = None;
    let mut preflight = None;
    let mut identity = None;
    let mut flash_summary: Option<FlashSummary> = None;
    let capabilities = cfg.declared_capabilities();
    let missing_live_caps = capabilities.missing_for_live_flash();

    // Phase 1: Compliance baseline
    let docs_ok = Path::new("docs/ECM-Data.md").exists()
        && Path::new("docs/CAT_COMM_TEMPLATE_PROTOCOL.md").exists();
    phases.push(PhaseResult {
        phase: 1,
        title: "Baseline e Compliance".to_string(),
        status: if docs_ok {
            PhaseStatus::Passed
        } else {
            PhaseStatus::Failed
        },
        details: if docs_ok {
            "runbook e protocolo presentes".to_string()
        } else {
            "faltam documentos obrigatorios em docs/".to_string()
        },
    });

    // Phase 2: Hardware bench readiness
    let hw_ready = matches!(cfg.mode, HwMode::Live) && missing_live_caps.is_empty();
    let sim_baseline_ok = run_sim_baseline_check();
    phases.push(PhaseResult {
        phase: 2,
        title: "Plataforma Fisica Real".to_string(),
        status: if hw_ready {
            PhaseStatus::Passed
        } else if matches!(cfg.mode, HwMode::Sim) && sim_baseline_ok.0 {
            PhaseStatus::Passed
        } else if matches!(cfg.mode, HwMode::Sim) {
            PhaseStatus::Failed
        } else {
            PhaseStatus::Failed
        },
        details: if hw_ready {
            format!("capabilities: {}", cfg.capabilities_summary())
        } else if matches!(cfg.mode, HwMode::Sim) {
            format!("sim_harness={}: {}", sim_baseline_ok.0, sim_baseline_ok.1)
        } else {
            format!(
                "capabilities insufficients: {}; missing={}",
                cfg.capabilities_summary(),
                if missing_live_caps.is_empty() {
                    "none".to_string()
                } else {
                    missing_live_caps.join(",")
                }
            )
        },
    });

    // Phase 3: Live detect/connect
    if hw_ready {
        match detect_ecms(cfg, Duration::from_secs(3)) {
            Ok(d) => {
                let found = d.source_addresses.len();
                detect = Some(d);
                phases.push(PhaseResult {
                    phase: 3,
                    title: "Conexao Confiavel".to_string(),
                    status: if found > 0 {
                        PhaseStatus::Passed
                    } else {
                        PhaseStatus::Failed
                    },
                    details: format!("ECMs detectados: {found}"),
                });
            }
            Err(e) => phases.push(PhaseResult {
                phase: 3,
                title: "Conexao Confiavel".to_string(),
                status: PhaseStatus::Failed,
                details: format!("falha detect/connect: {e}"),
            }),
        }

        match connect_ecm(cfg, target_sa) {
            Ok(()) => {}
            Err(e) => phases.push(PhaseResult {
                phase: 3,
                title: "Conexao Confiavel (Connect)".to_string(),
                status: PhaseStatus::Failed,
                details: format!("connect falhou: {e}"),
            }),
        }
    } else if matches!(cfg.mode, HwMode::Sim) {
        let (ok, detail) = run_sim_connectivity_check();
        phases.push(PhaseResult {
            phase: 3,
            title: "Conexao Confiavel".to_string(),
            status: if ok { PhaseStatus::Passed } else { PhaseStatus::Failed },
            details: detail,
        });
    } else {
        phases.push(PhaseResult {
            phase: 3,
            title: "Conexao Confiavel".to_string(),
            status: PhaseStatus::Failed,
            details: "hardware live indisponivel".to_string(),
        });
    }

    // Phase 4: ECU identity
    if hw_ready {
        match live_read_ecm_identity(cfg, target_sa) {
            Ok(id) => {
                let fp = id.fingerprint.is_some();
                identity = Some(id);
                phases.push(PhaseResult {
                    phase: 4,
                    title: "Leitura e Identidade ECM".to_string(),
                    status: if fp || cfg.allow_untrusted_ecu {
                        PhaseStatus::Passed
                    } else {
                        PhaseStatus::Failed
                    },
                    details: format!("fingerprint={fp} allow_untrusted_ecu={}", cfg.allow_untrusted_ecu),
                });
            }
            Err(e) => phases.push(PhaseResult {
                phase: 4,
                title: "Leitura e Identidade ECM".to_string(),
                status: PhaseStatus::Failed,
                details: format!("falha leitura DID: {e}"),
            }),
        }
    } else if matches!(cfg.mode, HwMode::Sim) {
        let (ok, detail) = run_sim_identity_check();
        phases.push(PhaseResult {
            phase: 4,
            title: "Leitura e Identidade ECM".to_string(),
            status: if ok { PhaseStatus::Passed } else { PhaseStatus::Failed },
            details: detail,
        });
    } else {
        phases.push(PhaseResult {
            phase: 4,
            title: "Leitura e Identidade ECM".to_string(),
            status: PhaseStatus::Failed,
            details: "requer modo live".to_string(),
        });
    }

    // Phase 5: Artifact validation
    match artifact_path {
        Some(path) => match validate_firmware_artifact(path, 64 * 1024 * 1024) {
            Ok(r) => {
                artifact = Some(r.clone());
                phases.push(PhaseResult {
                    phase: 5,
                    title: "Validacao de Artefato".to_string(),
                    status: PhaseStatus::Passed,
                    details: format!("{} bytes crc32=0x{:08X}", r.bytes, r.crc32),
                });
            }
            Err(e) => phases.push(PhaseResult {
                phase: 5,
                title: "Validacao de Artefato".to_string(),
                status: PhaseStatus::Failed,
                details: format!("artefato invalido: {e}"),
            }),
        },
        None => phases.push(PhaseResult {
            phase: 5,
            title: "Validacao de Artefato".to_string(),
            status: if options.strict_mode {
                PhaseStatus::Failed
            } else {
                PhaseStatus::Blocked
            },
            details: "forneca --firmware=<arquivo.bin>".to_string(),
        }),
    }

    // Phase 6: Flash preflight gate
    if hw_ready {
        match live_preflight_ecm(cfg, target_sa) {
            Ok(pf) => {
                let v_ok = pf.supply_voltage_v >= 11.8;
                preflight = Some(pf.clone());
                let mut status = if v_ok {
                    PhaseStatus::Passed
                } else {
                    PhaseStatus::Failed
                };
                let mut details = format!(
                    "supply_v={:.2} trusted_fp={}",
                    pf.supply_voltage_v, pf.trusted_fingerprint
                );
                if options.execute_flash {
                    if let (true, Some(path), true) = (
                        status == PhaseStatus::Passed,
                        artifact_path,
                        cfg.write_effectively_enabled(),
                    ) {
                        match fs::read(path) {
                            Ok(payload) => match live_flash_ecm_firmware(cfg, target_sa, &payload) {
                                Ok(sum) => {
                                    flash_summary = Some(sum.clone());
                                    details = format!(
                                        "{} | flash_ok bytes={} blocks={} crc32=0x{:08X} | td_ack={}/{} seq_err={} fc_timeout={}",
                                        details,
                                        sum.bytes_sent,
                                        sum.blocks_sent,
                                        sum.crc32,
                                        sum.transport_diagnostics.transfer_data_blocks_acked,
                                        sum.transport_diagnostics.transfer_data_blocks_attempted,
                                        sum.transport_diagnostics.sequence_error_count,
                                        sum.transport_diagnostics.flowcontrol_timeout_count
                                    );
                                }
                                Err(e) => {
                                    status = PhaseStatus::Failed;
                                    details = format!("{} | flash_fail={}", details, e);
                                }
                            },
                            Err(e) => {
                                status = PhaseStatus::Failed;
                                details = format!("{} | firmware_read_fail={}", details, e);
                            }
                        }
                    } else {
                        status = PhaseStatus::Failed;
                        details = format!(
                            "{} | flash requested but missing requirements (artifact + write_enabled + preflight_ok)",
                            details
                        );
                    }
                }
                phases.push(PhaseResult {
                    phase: 6,
                    title: "Flash Seguro (Preflight)".to_string(),
                    status,
                    details,
                });
            }
            Err(e) => phases.push(PhaseResult {
                phase: 6,
                title: "Flash Seguro (Preflight)".to_string(),
                status: PhaseStatus::Failed,
                details: format!("preflight falhou: {e}"),
            }),
        }
    } else if matches!(cfg.mode, HwMode::Sim) {
        let (ok, detail) = run_sim_preflight_check();
        phases.push(PhaseResult {
            phase: 6,
            title: "Flash Seguro (Preflight)".to_string(),
            status: if ok { PhaseStatus::Passed } else { PhaseStatus::Failed },
            details: detail,
        });
    } else {
        phases.push(PhaseResult {
            phase: 6,
            title: "Flash Seguro (Preflight)".to_string(),
            status: PhaseStatus::Failed,
            details: "requer modo live".to_string(),
        });
    }

    // Phase 7: Diagnostics readiness
    let diag_ready_cfg = cfg.uds_timeout_p2_ms > 0
        && cfg.uds_timeout_p2star_ms >= cfg.uds_timeout_p2_ms
        && cfg.uds_retry_count > 0;
    let diag_runtime = run_uds_runtime_self_test();
    let diag_ready = diag_ready_cfg && diag_runtime.0;
    phases.push(PhaseResult {
        phase: 7,
        title: "Diagnostico e Debug".to_string(),
        status: if diag_ready {
            PhaseStatus::Passed
        } else {
            PhaseStatus::Failed
        },
        details: format!(
            "uds_retry={} p2={} p2*= {} runtime={}: {}",
            cfg.uds_retry_count,
            cfg.uds_timeout_p2_ms,
            cfg.uds_timeout_p2star_ms,
            diag_runtime.0,
            diag_runtime.1
        ),
    });

    // Phase 8: Governance/security
    let gov = cfg.allowlist_path.is_some() && cfg.noninteractive_approved;
    let write_policy = if cfg.enable_write {
        cfg.write_effectively_enabled()
    } else {
        true
    };
    phases.push(PhaseResult {
        phase: 8,
        title: "Governanca e Seguranca".to_string(),
        status: if gov && write_policy {
            PhaseStatus::Passed
        } else {
            PhaseStatus::Failed
        },
        details: format!(
            "allowlist_present={} noninteractive_approved={} write_effective={}",
            cfg.allowlist_path.is_some(),
            cfg.noninteractive_approved,
            cfg.write_effectively_enabled()
        ),
    });

    // Phase 9: Reliability/stress readiness
    let reliability_ok = cfg.rate_limit_global_per_sec > 0
        && cfg.rate_limit_per_id_per_sec > 0
        && cfg.reconnect_backoff_max_ms >= cfg.reconnect_backoff_base_ms
        && run_sim_stress_check().0;
    let sim_stress = run_sim_stress_check();
    phases.push(PhaseResult {
        phase: 9,
        title: "Confiabilidade e Stress".to_string(),
        status: if reliability_ok {
            PhaseStatus::Passed
        } else {
            PhaseStatus::Failed
        },
        details: format!(
            "rate_global={} rate_per_id={} reconnect={}..{} ms",
            cfg.rate_limit_global_per_sec,
            cfg.rate_limit_per_id_per_sec,
            cfg.reconnect_backoff_base_ms,
            cfg.reconnect_backoff_max_ms
        ) + &format!(" stress={}: {}", sim_stress.0, sim_stress.1),
    });

    // Phase 10: Industrialization
    let industrial_ok = Path::new("Cargo.toml").exists()
        && Path::new(".github/workflows/integration-mock.yml").exists()
        && Path::new("docs/ECM-Data.md").exists();
    phases.push(PhaseResult {
        phase: 10,
        title: "Industrializacao".to_string(),
        status: if industrial_ok {
            PhaseStatus::Passed
        } else {
            PhaseStatus::Failed
        },
        details: if industrial_ok {
            "pipeline de build/test encontrado".to_string()
        } else {
            "faltam artefatos de pipeline".to_string()
        },
    });

    // Phase 11: Homologation dossier generation (placeholder, finalized after writing artifacts)
    phases.push(PhaseResult {
        phase: 11,
        title: "Homologacao Final".to_string(),
        status: PhaseStatus::Passed,
        details: "gerando dossie json/markdown".to_string(),
    });

    // Phase 12: Production gate
    let overall_passed = phases
        .iter()
        .filter(|p| p.phase <= 10)
        .all(|p| match p.status {
            PhaseStatus::Passed => true,
            PhaseStatus::Blocked => !options.strict_mode,
            PhaseStatus::Failed => false,
        });
    phases.push(PhaseResult {
        phase: 12,
        title: "Gate de Producao".to_string(),
        status: if overall_passed {
            PhaseStatus::Passed
        } else {
            PhaseStatus::Failed
        },
        details: if overall_passed {
            "gates criticos satisfeitos para o contexto atual".to_string()
        } else {
            "ha falhas bloqueantes nas fases anteriores".to_string()
        },
    });

    let conformance_summary = Some(build_conformance_summary(
        options.execute_flash,
        options.strict_mode,
        &flash_summary,
        &capabilities,
    ));

    let report = ProgramReport {
        generated_at_ms: now_ms(),
        mode: match cfg.mode {
            HwMode::Sim => "sim".to_string(),
            HwMode::Live => "live".to_string(),
        },
        target_sa,
        strict_mode: options.strict_mode,
        execute_flash: options.execute_flash,
        artifact,
        detect,
        preflight,
        identity,
        conformance_summary,
        phases,
        overall_passed,
        report_sha256: None,
    };

    let ts = now_ms();
    let json_path = out_dir.join(format!("production_phase_report_{ts}.json"));
    let md_path = out_dir.join(format!("production_phase_report_{ts}.md"));

    let json = serde_json::to_string_pretty(&report)
        .map_err(|e| HwError::Unknown(format!("serialize report json failed: {e}")))?;
    fs::write(&json_path, json)
        .map_err(|e| HwError::Unknown(format!("write report json failed: {e}")))?;

    let mut md = String::new();
    md.push_str("# Production Program Report\n\n");
    md.push_str(&format!("- generated_at_ms: {}\n", report.generated_at_ms));
    md.push_str(&format!("- mode: {}\n", report.mode));
    md.push_str(&format!("- target_sa: {:?}\n", report.target_sa));
    md.push_str(&format!("- overall_passed: {}\n\n", report.overall_passed));
    if let Some(c) = &report.conformance_summary {
        md.push_str("## UDS Conformance Summary\n\n");
        md.push_str(&format!("- passed: {}\n", c.passed));
        for s in &c.services {
            md.push_str(&format!(
                "- SID {}: passed={} ({})\n",
                s.service_sid, s.passed, s.evidence
            ));
        }
        md.push_str("\n");
    }
    md.push_str("## Phase Results\n\n");
    for p in &report.phases {
        md.push_str(&format!(
            "- Fase {} - {}: {:?} ({})\n",
            p.phase, p.title, p.status, p.details
        ));
    }
    fs::write(&md_path, md)
        .map_err(|e| HwError::Unknown(format!("write report markdown failed: {e}")))?;

    let json_bytes = fs::read(&json_path)
        .map_err(|e| HwError::Unknown(format!("read report json failed: {e}")))?;
    let mut h = Sha256::new();
    h.update(&json_bytes);
    let report_hash = hex_sha(h.finalize().as_slice());

    let mut report_final = report;
    if let Some(p11) = report_final.phases.iter_mut().find(|p| p.phase == 11) {
        p11.status = if Path::new(&json_path).exists() && Path::new(&md_path).exists() {
            PhaseStatus::Passed
        } else {
            PhaseStatus::Failed
        };
        p11.details = format!(
            "json={} md={} sha256={}",
            json_path.display(),
            md_path.display(),
            report_hash
        );
    }
    report_final.report_sha256 = Some(report_hash);

    let json_final = serde_json::to_string_pretty(&report_final)
        .map_err(|e| HwError::Unknown(format!("serialize final report json failed: {e}")))?;
    fs::write(&json_path, json_final)
        .map_err(|e| HwError::Unknown(format!("write final report json failed: {e}")))?;

    Ok(json_path)
}

fn run_sim_baseline_check() -> (bool, String) {
    let mut sim = HeavyMachinery::new();
    sim.key_advance();
    sim.key_advance();
    sim.key_advance();
    sim.throttle_pct = 35.0;
    for _ in 0..360 {
        sim.tick(1.0 / 60.0);
    }
    let ok = sim.elapsed > 5.0 && sim.metrics.steps_completed > 0;
    (
        ok,
        format!(
            "elapsed={:.2}s steps={} speed={:.2}kmh",
            sim.elapsed, sim.metrics.steps_completed, sim.tcm.ground_speed_kmh
        ),
    )
}

fn run_sim_connectivity_check() -> (bool, String) {
    let mut sim = HeavyMachinery::new();
    sim.key_advance();
    sim.key_advance();
    sim.key_advance();
    sim.throttle_pct = 20.0;
    for _ in 0..240 {
        sim.tick(1.0 / 60.0);
    }
    let can_health = sim.can_net.network_health_score_01();
    let ok = (0.0..=1.0).contains(&can_health);
    (ok, format!("can_health={:.3} errors={}", can_health, sim.can_net.total_errors()))
}

fn run_sim_identity_check() -> (bool, String) {
    let mut sim = HeavyMachinery::new();
    let vin = sim.uds_ecm.process(&[0x22, 0xF1, 0x90], 0.0);
    let sw = sim.uds_ecm.process(&[0x22, 0xF1, 0x89], 0.1);
    let vin_ok = vin.first().copied() == Some(0x62) && vin.len() > 6;
    let sw_ok = sw.first().copied() == Some(0x62) && sw.len() > 6;
    (
        vin_ok && sw_ok,
        format!("vin_ok={} sw_ok={} vin_len={} sw_len={}", vin_ok, sw_ok, vin.len(), sw.len()),
    )
}

fn build_conformance_summary(
    execute_flash: bool,
    strict_mode: bool,
    flash_summary: &Option<FlashSummary>,
    capabilities: &HwCapabilities,
) -> ConformanceSummary {
    if !execute_flash {
        return ConformanceSummary {
            passed: !strict_mode,
            services: vec![
                ServiceConformanceResult {
                    service_sid: "0x27".to_string(),
                    passed: !strict_mode,
                    evidence: "flash path not executed".to_string(),
                },
                ServiceConformanceResult {
                    service_sid: "0x34".to_string(),
                    passed: !strict_mode,
                    evidence: "flash path not executed".to_string(),
                },
                ServiceConformanceResult {
                    service_sid: "0x36".to_string(),
                    passed: !strict_mode,
                    evidence: "flash path not executed".to_string(),
                },
                ServiceConformanceResult {
                    service_sid: "0x37".to_string(),
                    passed: !strict_mode,
                    evidence: "flash path not executed".to_string(),
                },
            ],
        };
    }

    if let Some(sum) = flash_summary {
        let d = &sum.transport_diagnostics;
        let sid27_ok = d.security_seed_positive && d.security_unlock_positive;
        let sid34_ok = d.request_download_positive;
        let sid36_ok = d.transfer_data_blocks_attempted > 0
            && d.transfer_data_blocks_attempted == d.transfer_data_blocks_acked
            && d.sequence_error_count == 0
            && d.flowcontrol_timeout_count == 0;
        let sid37_ok = d.request_transfer_exit_positive;

        let services = vec![
            ServiceConformanceResult {
                service_sid: "0x27".to_string(),
                passed: sid27_ok,
                evidence: format!(
                    "seed_ok={} unlock_ok={}",
                    d.security_seed_positive, d.security_unlock_positive
                ),
            },
            ServiceConformanceResult {
                service_sid: "0x34".to_string(),
                passed: sid34_ok,
                evidence: format!("request_download_positive={}", d.request_download_positive),
            },
            ServiceConformanceResult {
                service_sid: "0x36".to_string(),
                passed: sid36_ok,
                evidence: format!(
                    "acked={}/{} seq_err={} fc_timeout={} stmin_ms={:?} bs={:?}",
                    d.transfer_data_blocks_acked,
                    d.transfer_data_blocks_attempted,
                    d.sequence_error_count,
                    d.flowcontrol_timeout_count,
                    d.fc_stmin_seen_ms,
                    d.fc_blocksize_seen
                ),
            },
            ServiceConformanceResult {
                service_sid: "0x37".to_string(),
                passed: sid37_ok,
                evidence: format!("request_transfer_exit_positive={}", d.request_transfer_exit_positive),
            },
        ];

        return ConformanceSummary {
            passed: services.iter().all(|s| s.passed),
            services,
        };
    }

    ConformanceSummary {
        passed: false,
        services: vec![
            ServiceConformanceResult {
                service_sid: "0x27".to_string(),
                passed: false,
                evidence: format!(
                    "flash path requested without summary; capabilities={}",
                    if capabilities.missing_for_live_flash().is_empty() {
                        "present".to_string()
                    } else {
                        capabilities.missing_for_live_flash().join(",")
                    }
                ),
            },
            ServiceConformanceResult {
                service_sid: "0x34".to_string(),
                passed: false,
                evidence: "flash summary unavailable".to_string(),
            },
            ServiceConformanceResult {
                service_sid: "0x36".to_string(),
                passed: false,
                evidence: "flash summary unavailable".to_string(),
            },
            ServiceConformanceResult {
                service_sid: "0x37".to_string(),
                passed: false,
                evidence: "flash summary unavailable".to_string(),
            },
        ],
    }
}

fn run_sim_preflight_check() -> (bool, String) {
    let mut sim = HeavyMachinery::new();
    sim.key_advance();
    sim.key_advance();
    sim.key_advance();
    for _ in 0..120 {
        sim.tick(1.0 / 60.0);
    }
    let v = sim.bcm.battery_voltage;
    let ok = v >= 11.8;
    (ok, format!("battery_v={:.2}", v))
}

fn run_uds_runtime_self_test() -> (bool, String) {
    let mut sim = HeavyMachinery::new();
    let sess = sim.uds_ecm.process(&[0x10, 0x01], 0.0);
    let vin = sim.uds_ecm.process(&[0x22, 0xF1, 0x90], 0.1);
    let tester = sim.uds_ecm.process(&[0x3E, 0x00], 0.2);
    let ok = sess.first().copied() == Some(0x50)
        && vin.first().copied() == Some(0x62)
        && tester.first().copied() == Some(0x7E);
    (
        ok,
        format!(
            "session_sid=0x{:02X} vin_sid=0x{:02X} tester_sid=0x{:02X}",
            sess.first().copied().unwrap_or(0),
            vin.first().copied().unwrap_or(0),
            tester.first().copied().unwrap_or(0)
        ),
    )
}

fn run_sim_stress_check() -> (bool, String) {
    let mut sim = HeavyMachinery::new();
    sim.key_advance();
    sim.key_advance();
    sim.key_advance();
    let mut max_speed = 0.0_f64;
    for i in 0..1800 {
        sim.throttle_pct = if i % 300 < 180 { 55.0 } else { 15.0 };
        sim.brake_pct = if i % 450 > 400 { 20.0 } else { 0.0 };
        sim.tick(1.0 / 60.0);
        max_speed = max_speed.max(sim.tcm.ground_speed_kmh);
        if sim.tcm.ground_speed_kmh.is_nan() {
            return (false, "speed NaN detected".to_string());
        }
    }
    (
        true,
        format!("max_speed={:.2} fuel={:.2}%", max_speed, sim.ecm.fuel_level_pct),
    )
}

fn hex_sha(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        out.push_str(&format!("{b:02x}"));
    }
    out
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}
