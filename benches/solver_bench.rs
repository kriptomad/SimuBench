use criterion::{criterion_group, criterion_main, Criterion};

fn bench_tick(c: &mut Criterion) {
    c.bench_function("tick_1k_steps", |b| {
        b.iter(|| {
            let mut sim = auto_breaking::HeavyMachinery::new();
            sim.key_advance();
            sim.key_advance();
            sim.key_advance();
            for _ in 0..1000 {
                sim.throttle_pct = 40.0;
                sim.tick(1.0 / 60.0);
            }
        })
    });

    c.bench_function("tick_10k_steps", |b| {
        b.iter(|| {
            let mut sim = auto_breaking::HeavyMachinery::new();
            sim.key_advance();
            sim.key_advance();
            sim.key_advance();
            for _ in 0..10_000 {
                sim.throttle_pct = 35.0;
                sim.tick(1.0 / 60.0);
            }
        })
    });
}

criterion_group!(benches, bench_tick);
criterion_main!(benches);
