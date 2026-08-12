# 💻 Exemplos de Código - Sistema de Freio com ABS

Coleção de snippets de código úteis e exemplos de extensão.

---

## 1. Exemplo: Modificar Frequência ABS

### Localização
[src/lib.rs](src/lib.rs#L130)

### Código Original
```rust
pub struct ABSController {
    // ...
    pulse_frequency: f64,  // 8.0 Hz
}
```

### Modificação: Aumentar para 10 Hz
```rust
impl ABSController {
    pub fn new() -> Self {
        Self {
            brake_system: BrakeSystem::new(),
            vehicle_velocity: 0.0,
            abs_active: false,
            current_pressure: 0.0,
            abs_cycle: 0.0,
            pulse_frequency: 10.0,  // ← Alterado para 10 Hz
        }
    }
}
```

### Recompilar e Testar
```bash
cargo build
cargo run
# Selecione Emergency Brake (tecla 2)
# Observe ciclos ABS acontecerem ~25% mais rápido
```

---

## 2. Exemplo: Ajustar Limiar de Travamento

### Localização
[src/lib.rs](src/lib.rs#L88)

### Código Original
```rust
let velocity_diff = vehicle_velocity - self.velocity;
if velocity_diff > 5.0 && self.brake_pressure > 0.3 {  // 5 km/h threshold
    self.state = WheelState::Skidding;
}
```

### Modificação: Mais Sensível (4 km/h)
```rust
// Detecta travamento em 4 km/h em vez de 5 km/h
if velocity_diff > 4.0 && self.brake_pressure > 0.3 {
    self.state = WheelState::Skidding;
}
```

### Impacto
- **ABS mais cedo**: Ativa ~0.5s mais rápido
- **Mais ciclos**: ~10-15% mais ciclos ABS
- **Parada mais suave**: Menos "pulsação" perceptível

---

## 3. Exemplo: Mudar Pressão de Liberação ABS

### Localização
[src/lib.rs](src/lib.rs#L162)

### Código Original
```rust
if self.abs_cycle < 0.5 {
    self.current_pressure = base_pressure * 0.3;  // 30% liberação
} else {
    self.current_pressure = base_pressure * 0.9;  // 90% aplicação
}
```

### Modificação: Mais Agressivo
```rust
if self.abs_cycle < 0.5 {
    self.current_pressure = base_pressure * 0.15;  // 15% (muito agressivo)
} else {
    self.current_pressure = base_pressure * 0.95;  // 95% (mais pressão)
}
```

### Modificação: Mais Conservador
```rust
if self.abs_cycle < 0.5 {
    self.current_pressure = base_pressure * 0.4;   // 40% (menos agressivo)
} else {
    self.current_pressure = base_pressure * 0.85;  // 85% (menos pressão)
}
```

---

## 4. Exemplo: Adicionar Novo Cenário

### Passo 1: Adicionar Variante de Enum
[src/main.rs](src/main.rs#L103)

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
enum SimulationScenario {
    Manual,
    EmergencyBrake,
    HighSpeed,
    RepeatedBraking,
    WetRoad,        // ← NOVO
}
```

### Passo 2: Implementar Display
[src/main.rs](src/main.rs#L113)

```rust
impl std::fmt::Display for SimulationScenario {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            // ... casos existentes ...
            SimulationScenario::WetRoad => write!(f, "WET ROAD"),
        }
    }
}
```

### Passo 3: Implementar Lógica
[src/main.rs](src/main.rs#L49)

```rust
match scenario {
    SimulationScenario::WetRoad => {
        scenario_time += DT;
        if scenario_time < 2.0 {
            throttle = 1.0;  // Acelera
        } else if scenario_time < 3.0 {
            throttle = 0.0;
            brake_request = 0.5;  // Freio moderado em piso molhado
        } else if scenario_time > 8.0 {
            brake_request = 0.0;
        }
    }
    // ... outros casos ...
}
```

### Passo 4: Adicionar Tecla de Ativação
[src/main.rs](src/main.rs#L35)

```rust
KeyCode::Char('5') => {
    scenario = SimulationScenario::WetRoad;
    scenario_time = 0.0;
    simulator.reset();
}
```

### Passo 5: Atualizar Controles (comentário)
[src/main.rs](src/main.rs#L304)

```rust
output.push_str("│ 1: Manual  2: Emergency  │  3: HighSpeed  4: Repeated Brake │\r\n");
output.push_str("│ 5: Wet Road              │  ...                              │\r\n");
```

---

## 5. Exemplo: Implementar Função de Análise

### Novo Módulo: metrics.rs

```rust
// src/metrics.rs

#[derive(Debug, Clone)]
pub struct BrakingMetrics {
    pub initial_velocity: f64,
    pub final_velocity: f64,
    pub stopping_distance: f64,
    pub stopping_time: f64,
    pub abs_cycles: u32,
    pub max_wheel_slip: f64,
    pub avg_deceleration: f64,
}

impl BrakingMetrics {
    pub fn calculate(
        initial_v: f64,
        final_v: f64,
        distance: f64,
        time: f64,
        abs_cycles: u32,
        max_slip: f64,
    ) -> Self {
        let avg_decel = (initial_v - final_v) / time * 3.6;  // m/s²
        
        BrakingMetrics {
            initial_velocity: initial_v,
            final_velocity: final_v,
            stopping_distance: distance,
            stopping_time: time,
            abs_cycles,
            max_wheel_slip: max_slip,
            avg_deceleration: avg_decel,
        }
    }
    
    pub fn print_report(&self) {
        println!("╔═══════════════════════════════════════╗");
        println!("║     BRAKING PERFORMANCE REPORT        ║");
        println!("╚═══════════════════════════════════════╝");
        println!("Initial velocity:  {:.1} km/h", self.initial_velocity);
        println!("Final velocity:    {:.1} km/h", self.final_velocity);
        println!("Stopping distance: {:.1} m", self.stopping_distance);
        println!("Stopping time:     {:.2} s", self.stopping_time);
        println!("ABS cycles:        {}", self.abs_cycles);
        println!("Max wheel slip:    {:.1} km/h", self.max_wheel_slip);
        println!("Avg deceleration:  {:.2} m/s²", self.avg_deceleration);
    }
}
```

### Usar no main.rs

```rust
// No fim da simulação
let metrics = BrakingMetrics::calculate(
    100.0,  // initial velocity
    0.0,    // final velocity
    55.0,   // estimated distance
    2.9,    // stopping time
    23,     // abs cycles
    5.1,    // max wheel slip
);

metrics.print_report();
```

---

## 6. Exemplo: Exportar Dados para CSV

### Novo Módulo: data_export.rs

```rust
// src/data_export.rs

use std::fs::File;
use std::io::Write;

pub struct DataExporter {
    filename: String,
}

impl DataExporter {
    pub fn new(filename: &str) -> Self {
        DataExporter {
            filename: filename.to_string(),
        }
    }
    
    pub fn export_history(
        &self,
        velocities: &[f64],
        abs_cycles: &[u32],
        times: &[f64],
    ) -> std::io::Result<()> {
        let mut file = File::create(&self.filename)?;
        
        // Header
        writeln!(file, "time_s,velocity_kmh,abs_cycles")?;
        
        // Data
        for (i, &vel) in velocities.iter().enumerate() {
            if i < abs_cycles.len() && i < times.len() {
                writeln!(
                    file,
                    "{:.3},{:.1},{}",
                    times[i],
                    vel,
                    abs_cycles[i]
                )?;
            }
        }
        
        println!("Data exported to: {}", self.filename);
        Ok(())
    }
}
```

### Usar

```rust
let exporter = DataExporter::new("simulation_data.csv");
exporter.export_history(&velocity_history, &abs_cycles, &times)?;
```

---

## 7. Exemplo: Sistema de Configuração Customizado

### Novo tipo: Config

```rust
// src/config.rs

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SimulatorConfig {
    pub abs_frequency: f64,
    pub abs_pressure_low: f64,
    pub abs_pressure_high: f64,
    pub skidding_threshold: f64,
    pub max_velocity: f64,
    pub max_deceleration: f64,
    pub sensor_noise: f64,
}

impl Default for SimulatorConfig {
    fn default() -> Self {
        SimulatorConfig {
            abs_frequency: 8.0,
            abs_pressure_low: 0.3,
            abs_pressure_high: 0.9,
            skidding_threshold: 5.0,
            max_velocity: 200.0,
            max_deceleration: 10.0,
            sensor_noise: 0.5,
        }
    }
}

impl SimulatorConfig {
    pub fn from_json(json_str: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json_str)
    }
    
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
}
```

### Arquivo config.json

```json
{
  "abs_frequency": 10.0,
  "abs_pressure_low": 0.25,
  "abs_pressure_high": 0.95,
  "skidding_threshold": 4.5,
  "max_velocity": 200.0,
  "max_deceleration": 12.0,
  "sensor_noise": 0.3
}
```

---

## 8. Exemplo: Teste Unitário Customizado

### Adicionar em lib.rs

```rust
#[cfg(test)]
mod custom_tests {
    use super::*;
    
    #[test]
    fn test_abs_activation_delay() {
        let mut sim = VehicleSimulator::new();
        sim.velocity = 100.0;
        
        // Simular 100 frames de frenagem
        for _ in 0..100 {
            sim.update(0.0, 1.0, DT);  // Freio máximo
        }
        
        // Verificar que ABS foi ativado
        assert!(sim.get_abs_active());
        assert!(sim.get_abs_cycles() > 0);
    }
    
    #[test]
    fn test_sensor_accuracy() {
        let mut sensor = SpeedSensor::new();
        sensor.update(100.0);
        
        let measured = sensor.get_measured_velocity();
        let error = (measured - 100.0).abs();
        
        // Erro deve estar dentro de ±1.5 km/h (3σ)
        assert!(error <= 1.5, "Sensor error too large: {}", error);
    }
    
    #[test]
    fn test_stopping_distance_reasonable() {
        let mut sim = VehicleSimulator::new();
        sim.velocity = 100.0;
        
        let initial_v = sim.velocity;
        let mut frame_count = 0;
        
        // Simular até parada
        while sim.velocity > 0.1 && frame_count < 1000 {
            sim.update(0.0, 1.0, DT);
            frame_count += 1;
        }
        
        // Distância teórica: d = v²/(2a) = (27.78)²/20 ≈ 38.6 m
        // Com dt=0.016 e 1000 frames: distância estimada ≈ v_media * t
        let time_elapsed = frame_count as f64 * DT;
        let distance = initial_v * time_elapsed * 0.3;  // Fator aproximado
        
        println!("Distance: {:.1} m in {:.1} s", distance, time_elapsed);
        assert!(distance > 30.0 && distance < 70.0);
    }
}
```

---

## 9. Exemplo: Criar Curva de Performance

### Função: Testar em Múltiplas Velocidades

```rust
pub fn test_braking_performance() {
    let velocities = vec![30.0, 50.0, 70.0, 100.0, 130.0, 160.0];
    
    println!("╔═════════════════════════════════════════════╗");
    println!("║ BRAKING PERFORMANCE vs INITIAL VELOCITY     ║");
    println!("╠═══════════╦══════════╦═════════╦═══════════╣");
    println!("║ V_init    ║ Time (s) ║ Cycles  ║ Distance  ║");
    println!("╠═══════════╬══════════╬═════════╬═══════════╣");
    
    for &v_init in &velocities {
        let mut sim = VehicleSimulator::new();
        sim.velocity = v_init;
        
        let mut time = 0.0;
        let mut frames = 0;
        
        while sim.velocity > 0.1 && frames < 2000 {
            sim.update(0.0, 1.0, 0.016);
            time += 0.016;
            frames += 1;
        }
        
        let distance = v_init * time * 0.3;  // Estimativa
        let cycles = sim.get_abs_cycles();
        
        println!(
            "║ {:6.1}    ║ {:6.2}   ║ {:6}   ║ {:6.1}   ║",
            v_init, time, cycles, distance
        );
    }
    
    println!("╚═════════════════════════════════════════════╝");
}
```

---

## 10. Exemplo: Integração com Hardware Simulado

### Pseudocódigo: Conexão CAN Bus

```rust
// src/can_interface.rs (pseudocódigo)

pub trait CANInterface {
    fn send_wheel_velocities(&self, velocities: [f64; 4]) -> Result<(), String>;
    fn send_brake_pressure(&self, pressure: f64) -> Result<(), String>;
    fn send_abs_status(&self, active: bool) -> Result<(), String>;
    fn receive_brake_request(&self) -> Result<f64, String>;
}

pub struct SimulatedCANBus {
    // Implementação simulada
}

impl CANInterface for SimulatedCANBus {
    fn send_wheel_velocities(&self, velocities: [f64; 4]) -> Result<(), String> {
        // Simularia envio via CAN
        println!("CAN TX: Wheel velocities: {:?}", velocities);
        Ok(())
    }
    
    fn send_brake_pressure(&self, pressure: f64) -> Result<(), String> {
        println!("CAN TX: Brake pressure: {:.1}%", pressure * 100.0);
        Ok(())
    }
    
    fn send_abs_status(&self, active: bool) -> Result<(), String> {
        println!("CAN TX: ABS {}", if active { "ACTIVE" } else { "IDLE" });
        Ok(())
    }
    
    fn receive_brake_request(&self) -> Result<f64, String> {
        Ok(0.5)  // Simulado
    }
}
```

---

## 📋 Checklist de Customização

Quando personalizar o código:

- [ ] Entender o módulo a ser alterado
- [ ] Compilar `cargo build` (verificar erros)
- [ ] Testar novo comportamento
- [ ] Validar contra expectativas
- [ ] Documentar mudanças
- [ ] Considerar efeitos colaterais
- [ ] Executar testes `cargo test`
- [ ] Atualizar comentários

---

## 🔗 Referências Rápidas

### Arquivos Principais
- [lib.rs](src/lib.rs) - Lógica de simulação
- [main.rs](src/main.rs) - Interface
- [Cargo.toml](Cargo.toml) - Dependências

### Documentação
- [ARCHITECTURE.md](ARCHITECTURE.md) - Como estender
- [ANALYSIS.md](ANALYSIS.md) - Física e matemática
- [TUTORIALS.md](TUTORIALS.md) - Exemplos práticos

---

**Versão**: 0.1.0  
**Data**: 2026-08-11
