//! Custom egui widgets: arc gauges, bar gauges, warning lamps, LED indicators.
#![allow(float_literal_f32_fallback)]

use eframe::egui::{
    self, Align2, Color32, FontId, Pos2, Rect, Response, Sense, Shape, Stroke, Ui, Vec2,
};
use std::f32::consts::PI;

// ── Arc Gauge (tachometer / speedometer style) ──────────────────────────────

/// Draws a semicircular arc gauge.
///   • size       = diameter in pixels (suggest 140–200)
///   • warn/crit  = thresholds that change color (0.0 = ignore)
///   • low_warn   = optional low-side warning (e.g. oil pressure)
pub fn arc_gauge(
    ui: &mut Ui,
    value: f64,
    min: f64,
    max: f64,
    label: &str,
    unit: &str,
    size: f32,
    warn: f64,
    crit: f64,
    low_warn: Option<f64>,
) -> Response {
    let (rect, response) = ui.allocate_exact_size(Vec2::splat(size), Sense::hover());
    if !ui.is_rect_visible(rect) {
        return response;
    }

    let painter = ui.painter_at(rect);
    let center = rect.center();
    let r = size * 0.35;
    let frac = ((value - min) / (max - min).max(0.001)).clamp(0.0, 1.0) as f32;

    // Color logic
    let value_color = if value >= crit {
        Color32::RED
    } else if value >= warn && warn > 0.0 {
        Color32::YELLOW
    } else if let Some(lo) = low_warn {
        if value <= lo {
            Color32::RED
        } else {
            Color32::from_rgb(30, 210, 90)
        }
    } else {
        Color32::from_rgb(30, 210, 90)
    };

    // Arc: 135° → 405° (270° clockwise sweep)
    const START: f32 = PI * 0.75;
    const SWEEP: f32 = PI * 1.50;
    const N: usize = 80;

    // Background arc
    for i in 0..N {
        let a0 = START + (i as f32 / N as f32) * SWEEP;
        let a1 = START + ((i + 1) as f32 / N as f32) * SWEEP;
        let p0 = center + Vec2::new(a0.cos() * r, a0.sin() * r);
        let p1 = center + Vec2::new(a1.cos() * r, a1.sin() * r);
        painter.line_segment([p0, p1], Stroke::new(4.0, Color32::from_gray(42)));
    }

    // Value arc (gradient green → yellow → red)
    let filled = ((frac * N as f32).floor() as usize).min(N);
    for i in 0..filled {
        let t = i as f32 / N as f32;
        let seg = if t < 0.60 {
            Color32::from_rgb(30, 210, 90)
        } else if t < 0.82 {
            Color32::YELLOW
        } else {
            Color32::RED
        };
        let a0 = START + t * SWEEP;
        let a1 = START + ((i + 1) as f32 / N as f32) * SWEEP;
        let p0 = center + Vec2::new(a0.cos() * r, a0.sin() * r);
        let p1 = center + Vec2::new(a1.cos() * r, a1.sin() * r);
        painter.line_segment([p0, p1], Stroke::new(5.5, seg));
    }

    // Warning tick mark
    if warn > min && warn < max {
        let wf = ((warn - min) / (max - min)) as f32;
        let wa = START + wf * SWEEP;
        let ti: Pos2 = center + Vec2::new(wa.cos() * (r - 8.0), wa.sin() * (r - 8.0));
        let to: Pos2 = center + Vec2::new(wa.cos() * (r + 4.0), wa.sin() * (r + 4.0));
        painter.line_segment([ti, to], Stroke::new(2.0, Color32::YELLOW));
    }
    if crit > min && crit < max {
        let cf = ((crit - min) / (max - min)) as f32;
        let ca = START + cf * SWEEP;
        let ti: Pos2 = center + Vec2::new(ca.cos() * (r - 8.0), ca.sin() * (r - 8.0));
        let to: Pos2 = center + Vec2::new(ca.cos() * (r + 4.0), ca.sin() * (r + 4.0));
        painter.line_segment([ti, to], Stroke::new(2.0, Color32::RED));
    }

    // Needle
    let na = START + frac * SWEEP;
    let tip: Pos2 = center + Vec2::new(na.cos() * r * 0.80, na.sin() * r * 0.80);
    let perp_a = na + PI * 0.5;
    let bl: Pos2 = center + Vec2::new(perp_a.cos() * 4.0, perp_a.sin() * 4.0);
    let br: Pos2 = center - Vec2::new(perp_a.cos() * 4.0, perp_a.sin() * 4.0);
    painter.add(Shape::convex_polygon(
        vec![tip, bl, br],
        Color32::WHITE,
        Stroke::NONE,
    ));

    // Center cap
    painter.circle_filled(center, 7.0, Color32::from_gray(68));
    painter.circle_stroke(center, 7.0, Stroke::new(1.5, Color32::from_gray(120)));

    // Value text
    painter.text(
        center + Vec2::new(0.0, r * 0.28),
        Align2::CENTER_CENTER,
        format!("{:.0}", value),
        FontId::proportional(size * 0.165),
        value_color,
    );
    // Unit
    painter.text(
        center + Vec2::new(0.0, r * 0.56),
        Align2::CENTER_CENTER,
        unit,
        FontId::proportional(size * 0.080),
        Color32::from_gray(155),
    );
    // Label
    painter.text(
        center + Vec2::new(0.0, r * 0.85),
        Align2::CENTER_CENTER,
        label,
        FontId::proportional(size * 0.095),
        Color32::from_gray(200),
    );

    // Scale min/max
    let min_p: Pos2 = center + Vec2::new(START.cos() * (r + 14.0), START.sin() * (r + 14.0));
    let max_a = START + SWEEP;
    let max_p: Pos2 = center + Vec2::new(max_a.cos() * (r + 14.0), max_a.sin() * (r + 14.0));
    painter.text(
        min_p,
        Align2::CENTER_CENTER,
        format!("{:.0}", min),
        FontId::proportional(size * 0.075),
        Color32::from_gray(90),
    );
    painter.text(
        max_p,
        Align2::CENTER_CENTER,
        format!("{:.0}", max),
        FontId::proportional(size * 0.075),
        Color32::from_gray(90),
    );

    response
}

// ── Bar Gauge ────────────────────────────────────────────────────────────────

pub fn bar_gauge(
    ui: &mut Ui,
    label: &str,
    value: f64,
    max: f64,
    unit: &str,
    bar_width: f32,
    color: Color32,
) {
    let frac = (value / max.max(0.001)).clamp(0.0, 1.0) as f32;
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(format!("{:<14}", label))
                .size(11.0)
                .monospace()
                .color(Color32::from_gray(175)),
        );
        let (rect, _) = ui.allocate_exact_size(Vec2::new(bar_width, 14.0), Sense::hover());
        let p = ui.painter_at(rect);
        p.rect_filled(rect, 3.0, Color32::from_gray(22));
        if frac > 0.0 {
            let fill = Rect::from_min_size(rect.min, Vec2::new(rect.width() * frac, rect.height()));
            p.rect_filled(fill, 3.0, color);
        }
        p.rect_stroke(rect, 3.0, Stroke::new(0.7, Color32::from_gray(65)));
        ui.label(
            egui::RichText::new(format!("{:>8.1} {}", value, unit))
                .size(11.0)
                .monospace()
                .color(color),
        );
    });
}

// ── Warning Lamp (LED style) ─────────────────────────────────────────────────

/// Returns Response so caller can check `.clicked()` or `.hovered()`.
pub fn warning_lamp(ui: &mut Ui, active: bool, label: &str, color: Color32) -> Response {
    let (rect, response) = ui.allocate_exact_size(Vec2::new(68.0, 48.0), Sense::click());
    if !ui.is_rect_visible(rect) {
        return response;
    }
    let p = ui.painter_at(rect);
    let lc = Pos2::new(rect.center().x, rect.top() + 16.0);
    let lr = 11.0_f32;

    if active {
        let gc = Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 38);
        p.circle_filled(lc, lr + 6.0, gc);
    }
    p.circle_filled(
        lc,
        lr,
        if active {
            color
        } else {
            Color32::from_gray(28)
        },
    );
    p.circle_stroke(
        lc,
        lr,
        Stroke::new(
            1.0,
            if active {
                color
            } else {
                Color32::from_gray(55)
            },
        ),
    );
    if active {
        p.circle_filled(
            lc - Vec2::new(3.0, 3.0),
            3.5,
            Color32::from_rgba_unmultiplied(255, 255, 255, 70),
        );
    }
    let lbl_col = if active {
        color
    } else {
        Color32::from_gray(72)
    };
    p.text(
        Pos2::new(rect.center().x, rect.bottom() - 3.0),
        Align2::CENTER_BOTTOM,
        label,
        FontId::proportional(9.5),
        lbl_col,
    );

    response.on_hover_text(label)
}

// ── Digital Readout ──────────────────────────────────────────────────────────

pub fn digital_readout(ui: &mut Ui, label: &str, value: &str, color: Color32) {
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(format!("{:<16}", label))
                .size(11.0)
                .monospace()
                .color(Color32::from_gray(140)),
        );
        ui.label(
            egui::RichText::new(value)
                .size(12.0)
                .monospace()
                .color(color)
                .strong(),
        );
    });
}

// ── Direction Selector ───────────────────────────────────────────────────────

/// Returns which button was clicked: 'F', 'N', 'R', 'P' or None
pub fn direction_selector(ui: &mut Ui, current: &str) -> Option<char> {
    let mut clicked = None;
    ui.horizontal(|ui| {
        for (label, key, color_on, color_off) in [
            (
                "P",
                'P',
                Color32::from_rgb(200, 200, 80),
                Color32::from_gray(50),
            ),
            (
                "R",
                'R',
                Color32::from_rgb(220, 80, 80),
                Color32::from_gray(50),
            ),
            ("N", 'N', Color32::YELLOW, Color32::from_gray(50)),
            (
                "F",
                'F',
                Color32::from_rgb(80, 210, 80),
                Color32::from_gray(50),
            ),
        ] {
            let active = current == label;
            let fill = if active { color_on } else { color_off };
            let btn = egui::Button::new(
                egui::RichText::new(label)
                    .size(16.0)
                    .color(if active {
                        Color32::BLACK
                    } else {
                        Color32::GRAY
                    })
                    .strong(),
            )
            .min_size(Vec2::new(36.0, 36.0))
            .fill(fill);
            if ui.add(btn).clicked() {
                clicked = Some(key);
            }
        }
    });
    clicked
}
