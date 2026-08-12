use auto_breaking::{HeavyMachinery, ManualCircuitParams, OilType};

#[test]
fn e2e_hydraulic_plus_ac_stress_generates_reports() {
    let mut bench = HeavyMachinery::new();

    bench.apply_leak_manual_params(
        "HYD_MAIN",
        ManualCircuitParams {
            oil_type: Some(OilType::HydraulicIso68),
            piston_pressure_bar: Some(210.0),
            operation_pressure_bar: Some(180.0),
            pressure_min_bar: Some(45.0),
            pressure_mean_bar: Some(155.0),
            pressure_ideal_bar: Some(175.0),
            pressure_max_bar: Some(230.0),
            pressure_rupture_bar: Some(280.0),
            oring_squeeze_pct: Some(19.0),
            compression_set_pct: Some(14.0),
            base_leak_area_mm2: Some(0.0009),
            max_supported_temp_c: Some(115.0),
        },
    );

    bench.bcm.hvac_on = true;
    bench.key_advance();
    bench.key_advance();
    bench.key_advance();

    for i in 0..900 {
        bench.throttle_pct = if i % 120 < 60 { 72.0 } else { 48.0 };
        bench.brake_pct = if i % 180 > 150 { 22.0 } else { 0.0 };
        bench.loader_lift_cmd = if i % 90 < 45 { 0.8 } else { -0.6 };
        bench.loader_tilt_cmd = if i % 70 < 35 { 0.5 } else { -0.4 };
        bench.hitch_joystick = if i % 80 < 40 { 0.7 } else { -0.3 };
        bench.tick(1.0 / 60.0);
    }

    assert!(!bench.leak_reports.is_empty());

    let pred = bench.predict_leak_scenarios(300.0, 0.05);
    assert!(!pred.is_empty());

    let root = std::env::temp_dir().join("autobreaking_e2e");
    bench
        .export_leak_report_json(root.join("runtime_report.json"))
        .expect("runtime json export");
    bench
        .export_leak_report_csv(root.join("runtime_report.csv"))
        .expect("runtime csv export");
    bench
        .export_leak_predictions_json(root.join("pred_report.json"), &pred)
        .expect("pred json export");
    bench
        .export_leak_predictions_csv(root.join("pred_report.csv"), &pred)
        .expect("pred csv export");
}

#[test]
fn monte_carlo_distribution_has_valid_bounds() {
    let bench = HeavyMachinery::new();
    let sims = bench.monte_carlo_leak_predictions(40, 180.0, 0.05, 30.0);

    assert!(!sims.is_empty());
    for s in sims.iter().take(20) {
        assert!((0.0..=100.0).contains(&s.final_rupture_probability_pct));
        assert!(s.peak_pressure_bar >= 0.0);
    }
}
