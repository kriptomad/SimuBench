use auto_breaking::HeavyMachinery;

fn boot_running(sim: &mut HeavyMachinery) {
    sim.key_advance();
    sim.key_advance();
    sim.key_advance();
}

#[test]
fn valve_stuck_scenario_keeps_hydraulic_bounds() {
    let mut sim = HeavyMachinery::new();
    boot_running(&mut sim);

    sim.loader_lift_cmd = 1.0;
    for _ in 0..300 {
        sim.loader_lift_cmd = 1.0;
        sim.tick(1.0 / 60.0);
    }

    assert!(sim.hcm.system_pressure_bar >= 0.0);
    assert!((0.0..=100.0).contains(&sim.hcm.fluid_level_pct));
}

#[test]
fn sensor_bias_scenario_keeps_controller_stable() {
    let mut sim = HeavyMachinery::new();
    boot_running(&mut sim);

    for _ in 0..200 {
        sim.ecm.coolant_temp_c += 0.03; // persistent positive bias
        sim.tick(1.0 / 60.0);
    }

    assert!(sim.ecm.rpm >= 0.0);
    assert!(sim.ecm.coolant_temp_c < 140.0);
}

#[test]
fn can_message_loss_fault_transitions_bus_state() {
    let mut sim = HeavyMachinery::new();
    boot_running(&mut sim);

    sim.can_net
        .inject_bus_off_once(auto_breaking::BusId::PowertrainHs);
    sim.tick(1.0 / 60.0);

    assert!(sim.can_net.powertrain.state != auto_breaking::CanBusState::ErrorActive);
}

#[test]
fn message_storm_scenario_degrades_but_bounds_health() {
    let mut sim = HeavyMachinery::new();
    boot_running(&mut sim);

    sim.can_net.inject_error_once(
        auto_breaking::BusId::PowertrainHs,
        auto_breaking::CanErrorKind::BablingIdiot,
    );
    for _ in 0..180 {
        sim.tick(1.0 / 60.0);
    }

    let h = sim.can_net.network_health_score_01();
    assert!((0.0..=1.0).contains(&h));
}

#[test]
fn rapid_setpoint_change_preserves_runtime_safety_bounds() {
    let mut sim = HeavyMachinery::new();
    boot_running(&mut sim);

    for i in 0..500 {
        sim.throttle_pct = if i % 20 < 10 { 90.0 } else { 5.0 };
        sim.brake_pct = if i % 35 < 5 { 35.0 } else { 0.0 };
        sim.tick(1.0 / 60.0);
    }

    assert!((0.0..=100.0).contains(&sim.ecm.fuel_level_pct));
    assert!(sim.hcm.system_pressure_bar >= 0.0);
    assert!(sim.ecm.oil_pressure_kpa >= 0.0);
}
