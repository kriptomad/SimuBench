//! Incremental architecture layer for solver/integrator responsibilities.
//! This module is API-preserving: public simulation APIs can migrate internally.

pub trait SimState {
    fn len(&self) -> usize;
    fn get(&self, idx: usize) -> f64;
    fn set(&mut self, idx: usize, value: f64);
}

pub trait DynamicsModel {
    fn derivative(&self, x: &[f64], t: f64) -> Vec<f64>;
}

pub trait Integrator {
    fn step(&self, model: &dyn DynamicsModel, state: &mut dyn SimState, t: f64, dt: f64);
}

#[derive(Debug, Clone, Copy, Default)]
pub struct EulerIntegrator;

impl Integrator for EulerIntegrator {
    fn step(&self, model: &dyn DynamicsModel, state: &mut dyn SimState, t: f64, dt: f64) {
        let mut x = Vec::with_capacity(state.len());
        for i in 0..state.len() {
            x.push(state.get(i));
        }
        let dx = model.derivative(&x, t);
        for (i, dxi) in dx.iter().enumerate().take(state.len()) {
            state.set(i, state.get(i) + dxi * dt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct VecState(Vec<f64>);
    impl SimState for VecState {
        fn len(&self) -> usize {
            self.0.len()
        }
        fn get(&self, idx: usize) -> f64 {
            self.0[idx]
        }
        fn set(&mut self, idx: usize, value: f64) {
            self.0[idx] = value;
        }
    }

    struct UnitDrift;
    impl DynamicsModel for UnitDrift {
        fn derivative(&self, x: &[f64], _t: f64) -> Vec<f64> {
            vec![1.0; x.len()]
        }
    }

    #[test]
    fn euler_integrator_advances_state() {
        let model = UnitDrift;
        let integ = EulerIntegrator;
        let mut state = VecState(vec![0.0, 2.0, -1.0]);
        integ.step(&model, &mut state, 0.0, 0.2);
        assert!((state.get(0) - 0.2).abs() < 1e-9);
        assert!((state.get(1) - 2.2).abs() < 1e-9);
        assert!((state.get(2) - -0.8).abs() < 1e-9);
    }
}
