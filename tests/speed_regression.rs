use auto_breaking::ecu_tcm::Direction;
use auto_breaking::HeavyMachinery;

fn bootstrap_running_machine(sim: &mut HeavyMachinery) {
    // OFF -> ACC -> ON -> START/RUN sequence
    sim.key_advance();
    sim.key_advance();
    sim.key_advance();
    sim.tcm.set_direction(Direction::Forward);
}

#[test]
fn speed_increases_with_throttle_in_forward() {
    let mut sim = HeavyMachinery::new();
    bootstrap_running_machine(&mut sim);

    sim.throttle_pct = 55.0;
    sim.brake_pct = 0.0;

    for _ in 0..600 {
        sim.tick(1.0 / 60.0);
    }

    assert!(sim.tcm.ground_speed_kmh > 3.0);
}

#[test]
fn partial_throttle_launch_moves_machine_promptly() {
    let mut sim = HeavyMachinery::new();
    bootstrap_running_machine(&mut sim);

    sim.throttle_pct = 30.0;
    sim.brake_pct = 0.0;

    for _ in 0..240 {
        sim.tick(1.0 / 60.0);
    }

    assert!(
        sim.tcm.ground_speed_kmh > 2.0,
        "partial throttle should still move the machine visibly from launch"
    );
}

#[test]
fn brake_reduces_speed_after_acceleration() {
    let mut sim = HeavyMachinery::new();
    bootstrap_running_machine(&mut sim);

    sim.throttle_pct = 65.0;
    sim.brake_pct = 0.0;
    for _ in 0..500 {
        sim.tick(1.0 / 60.0);
    }
    let v_before = sim.tcm.ground_speed_kmh;

    sim.throttle_pct = 0.0;
    sim.brake_pct = 70.0;
    for _ in 0..250 {
        sim.tick(1.0 / 60.0);
    }
    let v_after = sim.tcm.ground_speed_kmh;

    assert!(v_before > 4.0);
    assert!(v_after < v_before);
}
