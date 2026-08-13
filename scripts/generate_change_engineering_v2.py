from __future__ import annotations

import datetime as dt
import re
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT_MD = ROOT / "docs" / "FULL_BOOK_V2_CHANGE_ENGINEERING.md"
OUT_PDF = ROOT / "docs" / "FULL_BOOK_V2_CHANGE_ENGINEERING.pdf"

INCLUDE_DIRS = [ROOT / "src", ROOT / "tests", ROOT / "scripts", ROOT / ".github"]
INCLUDE_EXTS = {".rs", ".ps1", ".sh", ".yml", ".yaml", ".toml", ".md"}


RISK_RULES = [
    ("critical", ["flash", "uds", "run_all_phases", "open_real_adapter", "vendor", "transfer_data", "request_download", "request_transfer_exit", "security_access"]),
    ("high", ["allowlist", "preflight", "connect", "probe", "rate_limit", "production", "report"]),
    ("medium", ["decode", "encode", "sim", "network", "sensor", "ecu", "j1939"]),
]

TEST_COMMANDS = {
    "critical": [
        "cargo check",
        "cargo check --features vendor-windows",
        "cargo test --workspace",
        "cargo test --features vendor-windows --test vendor_bridge_e2e",
        "powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1",
    ],
    "high": [
        "cargo check",
        "cargo test --workspace",
        "cargo test --test io_mock",
    ],
    "medium": [
        "cargo check",
        "cargo test --workspace",
    ],
    "low": [
        "cargo check",
    ],
}


def rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def source_files() -> list[Path]:
    files: list[Path] = []
    for d in INCLUDE_DIRS:
        if not d.exists():
            continue
        for p in d.rglob("*"):
            if p.is_file() and p.suffix.lower() in INCLUDE_EXTS and "target" not in p.parts:
                files.append(p)
    files.sort(key=lambda p: p.as_posix())
    return files


def read(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8", errors="replace").splitlines()


def find_symbols(path: Path, lines: list[str]) -> list[dict]:
    symbols = []
    if path.suffix.lower() == ".rs":
        pat = re.compile(r"^\s*(pub\s+)?(async\s+)?fn\s+([A-Za-z_][A-Za-z0-9_]*)")
        for i, line in enumerate(lines, start=1):
            m = pat.search(line)
            if m:
                symbols.append({"name": m.group(3), "line": i, "file": path})
    return symbols


def classify_risk(name: str, context: str) -> str:
    hay = f"{name} {context}".lower()
    for level, needles in RISK_RULES:
        if any(n in hay for n in needles):
            return level
    return "low"


def risk_rank(level: str) -> int:
    return {"critical": 0, "high": 1, "medium": 2, "low": 3}.get(level, 4)


def risk_reason(name: str, file_rel: str) -> str:
    n = name.lower()
    if any(k in n for k in ["flash", "transfer", "download"]):
        return "impacts firmware write path, transfer integrity, and production gate outcome"
    if "uds" in n or "security" in n:
        return "impacts diagnostic protocol contract and ECU session state"
    if "run_all_phases" in n or "production" in file_rel:
        return "impacts release gate and conformance reporting"
    if "adapter" in n or "vendor" in file_rel or "hw" in file_rel:
        return "impacts hardware transport abstraction and live connectivity"
    if "allowlist" in n:
        return "impacts write authorization and operational safety"
    if "report" in n:
        return "impacts evidence schema consumed by automation and approvals"
    return "impacts runtime behavior and downstream callers"


def extract_context(lines: list[str], line: int) -> str:
    s = max(1, line - 2)
    e = min(len(lines), line + 2)
    return " ".join(lines[s - 1 : e]).lower()


def build_call_index(files: list[Path]) -> dict[str, list[dict]]:
    index: dict[str, list[dict]] = defaultdict(list)
    call_pat = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)\s*\(")
    for f in files:
        lines = read(f)
        for i, line in enumerate(lines, start=1):
            for m in call_pat.finditer(line):
                index[m.group(1)].append({"file": f, "line": i, "code": line.strip()})
    return index


def severity_from_risk(risk: str) -> str:
    return {
        "critical": "critica",
        "high": "alta",
        "medium": "media",
        "low": "baixa",
    }.get(risk, "media")


def render_pdf(md_text: str, out_pdf: Path) -> None:
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas

    width, height = A4
    left = 10 * mm
    top = height - 10 * mm
    bottom = 10 * mm
    line_h = 4.0 * mm

    c = canvas.Canvas(str(out_pdf), pagesize=A4)
    c.setTitle("AutoBreaking V2 Change Engineering")
    c.setFont("Courier", 8)

    y = top

    def new_page() -> None:
        nonlocal y
        c.showPage()
        c.setFont("Courier", 8)
        y = top

    for raw in md_text.splitlines():
        line = raw.expandtabs(4)
        while len(line) > 135:
            chunk = line[:135]
            if y < bottom:
                new_page()
            c.drawString(left, y, chunk)
            y -= line_h
            line = line[135:]
        if y < bottom:
            new_page()
        c.drawString(left, y, line)
        y -= line_h

    c.save()


def generate() -> str:
    files = source_files()
    rust_files = [f for f in files if f.suffix.lower() == ".rs"]

    symbols = []
    lines_cache = {}
    for f in rust_files:
        lines = read(f)
        lines_cache[f] = lines
        symbols.extend(find_symbols(f, lines))

    for s in symbols:
        ctx = extract_context(lines_cache[s["file"]], s["line"])
        s["risk"] = classify_risk(s["name"], ctx + " " + rel(s["file"]))
        s["reason"] = risk_reason(s["name"], rel(s["file"]))

    symbols.sort(key=lambda x: (risk_rank(x["risk"]), rel(x["file"]), x["line"]))
    call_index = build_call_index(rust_files)

    critical_subset = [s for s in symbols if s["risk"] in {"critical", "high"}]
    critical_subset = critical_subset[:220]
    risk_counts = defaultdict(int)
    for s in symbols:
        risk_counts[s["risk"]] += 1

    out = []
    out.append("# AutoBreaking V2 Change Engineering Book")
    out.append("")
    out.append(f"Gerado em: {dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    out.append("")
    out.append("## Executive Summary")
    out.append("| Metric | Value |")
    out.append("|---|---:|")
    out.append(f"| Rust files scanned | {len(rust_files)} |")
    out.append(f"| Functions detected | {len(symbols)} |")
    out.append(f"| Critical risk functions | {risk_counts['critical']} |")
    out.append(f"| High risk functions | {risk_counts['high']} |")
    out.append(f"| Medium risk functions | {risk_counts['medium']} |")
    out.append(f"| Low risk functions | {risk_counts['low']} |")
    out.append("")

    out.append("## Visual Roadmap")
    out.append("```mermaid")
    out.append("flowchart LR")
    out.append("  W1[Wave 1\\nProtocol Contracts] --> W2[Wave 2\\nHardware Abstraction]")
    out.append("  W2 --> W3[Wave 3\\nReport Schema Stabilization]")
    out.append("  W3 --> W4[Wave 4\\nPerformance and Observability]")
    out.append("  W4 --> W5[Wave 5\\nCI and Evidence Hardening]")
    out.append("```")
    out.append("")

    out.append("## 1. Matriz de Risco por Funcao Critica")
    out.append("")
    out.append("<details>")
    out.append("<summary>Expand full critical and high risk matrix</summary>")
    out.append("")
    out.append("| Risco | Funcao | Local | Linha | Justificativa | Testes Minimos |")
    out.append("|---|---|---|---:|---|---|")
    for s in critical_subset:
        f = rel(s["file"])
        t = " ; ".join(TEST_COMMANDS.get(s["risk"], TEST_COMMANDS["medium"]))
        out.append(
            f"| {s['risk']} | {s['name']} | {f} | {s['line']} | {s['reason']} | {t} |"
        )
    out.append("")
    out.append("</details>")
    out.append("")

    out.append("## 2. Se Mudar X Entao Quebra Y")
    out.append("")
    out.append("<details>")
    out.append("<summary>Expand full X->Y impact table with line references</summary>")
    out.append("")
    out.append("| X alterado | Onde | Risco | Y afetado | Linhas de referencia | Efeito esperado |")
    out.append("|---|---|---|---|---|---|")
    for s in critical_subset[:160]:
        refs = call_index.get(s["name"], [])
        refs = [r for r in refs if r["file"] != s["file"]][:6]
        if not refs:
            y = "chamadas diretas nao detectadas por varredura estatica"
            lines = f"{rel(s['file'])}:{s['line']}"
            effect = "impacto local com potencial efeito indireto via fluxo de controle"
            out.append(
                f"| {s['name']} | {rel(s['file'])} | {severity_from_risk(s['risk'])} | {y} | {lines} | {effect} |"
            )
            continue

        y_parts = []
        line_parts = [f"{rel(s['file'])}:{s['line']}"]
        for r in refs:
            y_parts.append(rel(r["file"]))
            line_parts.append(f"{rel(r['file'])}:{r['line']}")
        y_join = " ; ".join(y_parts)
        l_join = " ; ".join(line_parts)

        if s["risk"] == "critical":
            effect = "pode quebrar protocolo, gate de producao ou fluxo de flash"
        elif s["risk"] == "high":
            effect = "pode quebrar readiness, relatorio ou integracao de hardware"
        else:
            effect = "pode alterar comportamento funcional e cobertura de testes"

        out.append(
            f"| {s['name']} | {rel(s['file'])} | {severity_from_risk(s['risk'])} | {y_join} | {l_join} | {effect} |"
        )
    out.append("")
    out.append("</details>")
    out.append("")

    out.append("## 3. Roadmap de Refatoracao Segura por Ondas")
    out.append("")
    out.append("### Wave 1 - Blindagem de Contratos de Protocolo")
    out.append("- Escopo: src/io/live_runner.rs, src/io/vendor_cat_comm.rs, src/uds.rs")
    out.append("- Mudancas: centralizar validacao de SID positivo, tipar NRC, consolidar parse de DIDs")
    out.append("- Checklist de testes:")
    for c in [
        "cargo check",
        "cargo test --workspace",
        "cargo test --features vendor-windows --test vendor_bridge_e2e",
    ]:
        out.append(f"  - [ ] {c}")
    out.append("")

    out.append("### Wave 2 - Refino de Abstracao de Hardware")
    out.append("- Escopo: src/io/hw.rs, src/io/socketcan_adapter.rs, src/io/serial_adapter.rs")
    out.append("- Mudancas: contratos de capability com traits menores, erros padronizados por camada")
    out.append("- Checklist de testes:")
    for c in [
        "cargo check --features vendor-windows",
        "cargo test --workspace",
        "cargo test --test io_mock",
    ]:
        out.append(f"  - [ ] {c}")
    out.append("")

    out.append("### Wave 3 - Estabilizacao de Schema de Relatorio")
    out.append("- Escopo: src/io/production_program.rs, src/bin/simulator_cli.rs, docs/ECM-Data.md")
    out.append("- Mudancas: versionamento de schema, compat adapters para consumidores antigos")
    out.append("- Checklist de testes:")
    for c in [
        "cargo check",
        "cargo test --workspace",
        "powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1",
    ]:
        out.append(f"  - [ ] {c}")
    out.append("")

    out.append("### Wave 4 - Performance e Observabilidade")
    out.append("- Escopo: src/main.rs, src/observability.rs, src/io/metrics.rs")
    out.append("- Mudancas: reduzir jitter de render e padronizar eventos de telemetria")
    out.append("- Checklist de testes:")
    for c in [
        "cargo check",
        "cargo test --workspace",
    ]:
        out.append(f"  - [ ] {c}")
    out.append("")

    out.append("### Wave 5 - Harden de CI e Evidencia")
    out.append("- Escopo: .github/workflows/*.yml, scripts/*.ps1, scripts/*.sh")
    out.append("- Mudancas: gates obrigatorios por risco, artefatos de report/hash sempre anexados")
    out.append("- Checklist de testes:")
    for c in [
        "cargo check",
        "cargo test --workspace",
        "cargo test --features vendor-windows --test vendor_bridge_e2e",
    ]:
        out.append(f"  - [ ] {c}")
    out.append("")

    out.append("## Cobertura e Limites")
    out.append("- Esta edicao prioriza funcoes de alto impacto e rastreabilidade de chamadas por varredura estatica.")
    out.append("- Chamadas dinamicas e dispatch indireto podem nao aparecer na tabela X->Y.")

    return "\n".join(out)


def main() -> None:
    md = generate()
    OUT_MD.write_text(md, encoding="utf-8")
    render_pdf(md, OUT_PDF)
    print(f"Wrote {OUT_MD}")
    print(f"Wrote {OUT_PDF}")


if __name__ == "__main__":
    main()
