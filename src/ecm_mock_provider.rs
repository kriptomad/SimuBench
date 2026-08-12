use auto_breaking::HeavyMachinery;

use crate::io::ecm_params::EcmSnapshot;

#[derive(Clone, Debug)]
pub struct MockParam {
    pub key: &'static str,
    pub value: f64,
    pub unit: &'static str,
}

#[derive(Clone, Debug)]
pub struct MockFunction {
    pub name: &'static str,
    pub params: Vec<MockParam>,
}

#[derive(Clone, Debug)]
pub struct MockEcmNode {
    pub name: &'static str,
    pub source_address: u8,
    pub functions: Vec<MockFunction>,
}

pub fn build_mock_ecm_tree(bench: &HeavyMachinery, snap: &EcmSnapshot) -> Vec<MockEcmNode> {
    let e = &bench.ecm;
    let h = &bench.hcm;
    let t = &bench.tcm;

    let ecm = MockEcmNode {
        name: "Engine ECM",
        source_address: snap.source_address.unwrap_or(0x00),
        functions: vec![
            MockFunction {
                name: "Engine Performance",
                params: vec![
                    MockParam {
                        key: "engine_speed_rpm",
                        value: snap.engine_speed_rpm.unwrap_or(e.rpm),
                        unit: "rpm",
                    },
                    MockParam {
                        key: "accel_pedal_pct",
                        value: snap.accel_pedal_pct.unwrap_or(e.active_throttle),
                        unit: "%",
                    },
                    MockParam {
                        key: "actual_torque_nm",
                        value: e.actual_torque_nm,
                        unit: "Nm",
                    },
                    MockParam {
                        key: "vehicle_speed",
                        value: t.ground_speed_kmh,
                        unit: "km/h",
                    },
                ],
            },
            MockFunction {
                name: "Pressures and Temperatures",
                params: vec![
                    MockParam {
                        key: "coolant_temp_c",
                        value: snap.coolant_temp_c.unwrap_or(e.coolant_temp_c),
                        unit: "C",
                    },
                    MockParam {
                        key: "fuel_temp_c",
                        value: snap.fuel_temp_c.unwrap_or(e.fuel_temp_c),
                        unit: "C",
                    },
                    MockParam {
                        key: "oil_pressure_kpa",
                        value: snap.oil_pressure_kpa.unwrap_or(e.oil_pressure_kpa),
                        unit: "kPa",
                    },
                    MockParam {
                        key: "boost_pressure_kpa",
                        value: e.boost_pressure_kpa,
                        unit: "kPa",
                    },
                ],
            },
            MockFunction {
                name: "Aftertreatment",
                params: vec![
                    MockParam {
                        key: "def_level_pct",
                        value: e.def_level_pct,
                        unit: "%",
                    },
                    MockParam {
                        key: "dpf_soot_pct",
                        value: e.dpf_soot_pct,
                        unit: "%",
                    },
                    MockParam {
                        key: "scr_efficiency_pct",
                        value: e.scr_efficiency_pct,
                        unit: "%",
                    },
                    MockParam {
                        key: "nox_tailpipe_ppm",
                        value: e.nox_tailpipe_ppm,
                        unit: "ppm",
                    },
                ],
            },
            MockFunction {
                name: "Diagnostics",
                params: vec![
                    MockParam {
                        key: "active_dtcs",
                        value: e.active_dtcs.len() as f64,
                        unit: "count",
                    },
                    MockParam {
                        key: "red_lamp",
                        value: if e.red_lamp { 1.0 } else { 0.0 },
                        unit: "bool",
                    },
                    MockParam {
                        key: "amber_lamp",
                        value: if e.amber_lamp { 1.0 } else { 0.0 },
                        unit: "bool",
                    },
                    MockParam {
                        key: "mil_active",
                        value: if e.mil_active { 1.0 } else { 0.0 },
                        unit: "bool",
                    },
                ],
            },
        ],
    };

    let hcm = MockEcmNode {
        name: "Hydraulic HCM",
        source_address: 0x1E,
        functions: vec![MockFunction {
            name: "Hydraulic",
            params: vec![
                MockParam {
                    key: "system_pressure_bar",
                    value: h.system_pressure_bar,
                    unit: "bar",
                },
                MockParam {
                    key: "pump_flow_lpm",
                    value: h.pump_flow_lpm,
                    unit: "L/min",
                },
                MockParam {
                    key: "fluid_temp_c",
                    value: h.fluid_temp_c,
                    unit: "C",
                },
                MockParam {
                    key: "filter_dp_bar",
                    value: h.filter_dp_bar,
                    unit: "bar",
                },
            ],
        }],
    };

    let tcm = MockEcmNode {
        name: "Transmission TCM",
        source_address: 0x03,
        functions: vec![MockFunction {
            name: "Driveline",
            params: vec![
                MockParam {
                    key: "ground_speed_kmh",
                    value: t.ground_speed_kmh,
                    unit: "km/h",
                },
                MockParam {
                    key: "speed_step",
                    value: t.speed_step as f64,
                    unit: "idx",
                },
                MockParam {
                    key: "clutch_slip_pct",
                    value: t.clutch_slip_pct,
                    unit: "%",
                },
                MockParam {
                    key: "gear_ratio",
                    value: t.gear_ratio,
                    unit: "ratio",
                },
            ],
        }],
    };

    vec![ecm, tcm, hcm]
}
