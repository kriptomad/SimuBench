use auto_breaking::HeavyMachinery;
use proptest::prelude::*;

proptest! {
    #[test]
    fn physical_levels_stay_within_bounds(throttle in 0.0f64..100.0, brake in 0.0f64..100.0, steps in 1usize..180usize) {
        let mut sim = HeavyMachinery::new();
        sim.key_advance();
        sim.key_advance();
        sim.key_advance();

        for _ in 0..steps {
            sim.throttle_pct = throttle;
            sim.brake_pct = brake;
            sim.tick(1.0 / 60.0);

            prop_assert!((0.0..=100.0).contains(&sim.ecm.fuel_level_pct));
            prop_assert!((0.0..=100.0).contains(&sim.ecm.def_level_pct));
            prop_assert!((0.0..=100.0).contains(&sim.hcm.fluid_level_pct));
            prop_assert!(sim.hcm.system_pressure_bar >= 0.0);
            prop_assert!(sim.ecm.oil_pressure_kpa >= 0.0);
        }
    }

    #[test]
    fn can_health_score_is_always_bounded(steps in 1usize..240usize) {
        let mut sim = HeavyMachinery::new();
        sim.key_advance();
        sim.key_advance();
        sim.key_advance();

        for _ in 0..steps {
            sim.tick(1.0 / 60.0);
            let h = sim.can_net.network_health_score_01();
            prop_assert!((0.0..=1.0).contains(&h));
        }
    }
}
