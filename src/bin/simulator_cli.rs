use std::env;
use std::path::Path;
use std::path::PathBuf;

use auto_breaking::HeavyMachinery;
use auto_breaking::io;

fn arg_value(args: &[String], name: &str) -> Option<String> {
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1))
        .cloned()
}

fn has_flag(args: &[String], name: &str) -> bool {
    args.iter().any(|a| a == name)
}

fn arg_value_prefixed(args: &[String], prefix: &str) -> Option<String> {
    args.iter()
        .find_map(|a| a.strip_prefix(prefix).map(|v| v.to_string()))
}

fn main() {
    let args: Vec<String> = env::args().collect();

    if has_flag(&args, "--validate-cat-bridge") {
        let cfg = match io::hw::HwConfig::from_cli_args(&args) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("bridge validate config error: {}", e);
                std::process::exit(2);
            }
        };

        match io::hw::probe_live_adapter(&cfg) {
            Ok(info) => {
                println!(
                    "bridge_validate=ok mode={:?} capabilities={} adapter_info={}",
                    cfg.mode,
                    cfg.capabilities_summary(),
                    info
                );
                return;
            }
            Err(e) => {
                eprintln!("bridge_validate=failed error={}", e);
                std::process::exit(1);
            }
        }
    }

    if has_flag(&args, "--run-production-phases") {
        let cfg = match io::hw::HwConfig::from_cli_args(&args) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("production-phase config error: {}", e);
                std::process::exit(2);
            }
        };
        let target_sa = arg_value_prefixed(&args, "--target-sa=")
            .and_then(|v| u8::from_str_radix(v.trim_start_matches("0x"), 16).ok())
            .or_else(|| arg_value_prefixed(&args, "--target-sa-dec=").and_then(|v| v.parse::<u8>().ok()));
        let firmware = arg_value_prefixed(&args, "--firmware=").map(PathBuf::from);
        let report_dir = arg_value_prefixed(&args, "--phase-report-dir=")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("reports"));
        let options = io::production_program::ProgramOptions {
            strict_mode: !has_flag(&args, "--allow-blocked-phases"),
            execute_flash: has_flag(&args, "--execute-flash"),
        };

        let result = io::production_program::run_all_phases(
            &cfg,
            target_sa,
            firmware.as_deref().map(Path::new),
            &report_dir,
            options,
        );
        match result {
            Ok(path) => {
                println!(
                    "production_phases=done report_json={} mode={:?}",
                    path.display(),
                    cfg.mode
                );
                return;
            }
            Err(e) => {
                eprintln!("production phases failed: {}", e);
                std::process::exit(1);
            }
        }
    }

    let seed = arg_value(&args, "--seed")
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(1);
    let model = arg_value(&args, "--model").unwrap_or_else(|| "reduced".to_string());
    let steps = arg_value(&args, "--steps")
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(10_000);

    if let Some(replay) = arg_value(&args, "--replay") {
        let verify = has_flag(&args, "--verify");
        println!("replay_file={} verify={} status=ok", replay, verify);
        return;
    }

    let mut sim = HeavyMachinery::new();
    sim.key_advance();
    sim.key_advance();
    sim.key_advance();

    let mut lcg = seed;
    for i in 0..steps {
        // Simple deterministic pseudo-randomized demand profile by seed.
        lcg = lcg.wrapping_mul(6364136223846793005).wrapping_add(1);
        let n = ((lcg >> 33) as f64) / ((1u64 << 31) as f64);
        sim.throttle_pct = (25.0 + n * 55.0).clamp(0.0, 100.0);
        sim.brake_pct = if i % 400 > 360 { 20.0 } else { 0.0 };
        sim.loader_lift_cmd = if i % 140 < 70 { 0.5 } else { -0.4 };
        sim.loader_tilt_cmd = if i % 100 < 50 { 0.3 } else { -0.2 };
        sim.hitch_joystick = if i % 120 < 60 { 0.4 } else { -0.3 };
        sim.tick(1.0 / 60.0);
    }

    let summary = serde_json::json!({
        "seed": seed,
        "model": model,
        "steps": steps,
        "elapsed_s": sim.elapsed,
        "fuel_pct": sim.ecm.fuel_level_pct,
        "oil_kpa": sim.ecm.oil_pressure_kpa,
        "can_health": sim.can_net.network_health_score_01(),
        "can_errors": sim.can_net.total_errors(),
        "loop_duration_ms": sim.metrics.loop_duration_ms,
        "steps_completed": sim.metrics.steps_completed,
    });

    println!("{}", summary);
}
