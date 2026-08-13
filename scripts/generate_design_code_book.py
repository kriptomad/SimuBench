from __future__ import annotations

import datetime as dt
import re
from pathlib import Path
from typing import Iterable

ROOT = Path(__file__).resolve().parents[1]
OUT_MD = ROOT / "docs" / "FULL_DESIGN_CODE_BOOK.md"
OUT_PDF = ROOT / "docs" / "FULL_DESIGN_CODE_BOOK.pdf"

RUST_GLOBS = [
    "src/**/*.rs",
    "tests/**/*.rs",
]

HEADER = """# AutoBreaking Full Design-Book and Code-Book

> Generated automatically from the current repository snapshot.
> Goal: provide deep technical traceability with architecture, call-flow links, symbol map, and line-level impact notes.

"""


def scan_files() -> list[Path]:
    files: list[Path] = []
    for pattern in RUST_GLOBS:
        files.extend(ROOT.glob(pattern))
    files = [p for p in files if p.is_file()]
    files.sort(key=lambda p: p.as_posix())
    return files


def rel(p: Path) -> str:
    return p.relative_to(ROOT).as_posix()


def classify_line(line: str) -> str:
    s = line.strip()
    if not s:
        return "line is blank; visual separation only."
    if s.startswith("//"):
        return "comment line; documentation intent, no runtime effect."
    if s.startswith("use "):
        return "imports symbols; changing path/name affects compile-time resolution."
    if s.startswith("mod ") or s.startswith("pub mod "):
        return "declares a module boundary; changing module name/path breaks references."
    if s.startswith("pub struct ") or s.startswith("struct "):
        return "declares data layout; field changes may break serialization, tests, and callers."
    if s.startswith("pub enum ") or s.startswith("enum "):
        return "declares state variants; adding/removing variants impacts pattern matches and logic branches."
    if s.startswith("pub trait ") or s.startswith("trait "):
        return "defines interface contract; signature changes impact all implementers and call sites."
    if re.match(r"^(pub\s+)?(async\s+)?fn\s+", s):
        return "declares executable behavior; signature/body changes alter API, control flow, and outputs."
    if s.startswith("impl "):
        return "implementation block; method behavior and trait conformance are defined here."
    if s.startswith("match "):
        return "branch dispatch; changing arms alters behavior for variants and input classes."
    if s.startswith("if ") or s.startswith("if let"):
        return "conditional gate; changing condition shifts runtime path and side effects."
    if s.startswith("for ") or s.startswith("while ") or s.startswith("loop"):
        return "iteration control; changing bounds/body affects throughput, timing, and possibly safety guarantees."
    if "?" in s:
        return "error-propagating operation; changing it impacts failure semantics and caller recovery paths."
    if "unwrap(" in s or "expect(" in s:
        return "panic-prone operation; changing/removing affects crash behavior and robustness."
    if "serde" in s or "Serialize" in s or "Deserialize" in s:
        return "serialization contract touchpoint; edits can break report compatibility and tooling."
    if "send" in s or "write" in s or "flash" in s or "uds" in s.lower():
        return "I/O or protocol-critical logic; edits can affect integration, timing, and diagnostic outcomes."
    if s in ["{", "}", "};"]:
        return "scope delimiter; structure-only line, but misplaced edits can change ownership and lifetime scopes."
    return "executable/support line; behavior impact depends on surrounding block context."


def extract_symbols(lines: list[str]) -> list[tuple[int, str, str]]:
    symbols: list[tuple[int, str, str]] = []
    patterns = [
        ("module", re.compile(r"^\s*(pub\s+)?mod\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("struct", re.compile(r"^\s*(pub\s+)?struct\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("enum", re.compile(r"^\s*(pub\s+)?enum\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("trait", re.compile(r"^\s*(pub\s+)?trait\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("impl", re.compile(r"^\s*impl\s+([A-Za-z_][A-Za-z0-9_:<> ,]*)")),
        ("fn", re.compile(r"^\s*(pub\s+)?(async\s+)?fn\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("const", re.compile(r"^\s*(pub\s+)?const\s+([A-Za-z_][A-Za-z0-9_]*)")),
    ]
    for i, line in enumerate(lines, start=1):
        for kind, pat in patterns:
            m = pat.search(line)
            if m:
                name = m.group(m.lastindex or 1)
                symbols.append((i, kind, name.strip()))
                break
    return symbols


def file_role(path: Path) -> str:
    s = rel(path)
    if s.startswith("src/io/"):
        return "I/O and diagnostics stack: adapters, allowlist, flashing, replay, metrics, production phases."
    if s.startswith("src/bin/"):
        return "Executable entrypoint for CLI/bridge workflow."
    if s == "src/main.rs":
        return "Desktop GUI runtime orchestration and presentation layer."
    if s == "src/lib.rs":
        return "Library root and module graph for the simulation and protocol ecosystem."
    if s.startswith("tests/"):
        return "Automated regression and integration validation surface."
    return "Domain module in simulation/control/protocol stack."


def count_nonempty(lines: Iterable[str]) -> int:
    return sum(1 for x in lines if x.strip())


def markdown_escape_pipe(s: str) -> str:
    return s.replace("|", "\\|")


def generate_md() -> str:
    now = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    files = scan_files()

    out: list[str] = []
    out.append(HEADER)
    out.append(f"Generated at: **{now}**\n")
    out.append("## Scope\n")
    out.append("- Full repository code map for Rust source and tests.")
    out.append("- Architecture and runtime flow linking major modules.")
    out.append("- Per-file symbol inventory with line references.")
    out.append("- Line-by-line impact commentary for change-risk analysis.\n")

    out.append("## Build and Feature Context\n")
    cargo = (ROOT / "Cargo.toml").read_text(encoding="utf-8", errors="replace").splitlines()
    for line in cargo:
        if line.startswith("name") or line.startswith("version") or line.startswith("edition"):
            out.append(f"- {line}")
    out.append("- Key features: default, advanced_observability, new_integrator, vendor-windows.\n")

    out.append("## System Call-Flow (High Level)\n")
    out.append("1. UI path: src/main.rs drives simulation ticks, visualizations, and control state updates.")
    out.append("2. CLI path: src/bin/simulator_cli.rs parses hardware/phase flags and calls io::production_program::run_all_phases.")
    out.append("3. Live flash path: io::production_program -> io::live_runner -> io::hw selected adapter -> io::vendor_cat_comm or socketcan/serial adapters.")
    out.append("4. UDS protocol path: io::live_runner and uds.rs coordinate 0x27/0x34/0x36/0x37 services, transport accounting, and conformance evidence.")
    out.append("5. Report path: production report JSON/Markdown emitted with gate and conformance summary.\n")

    out.append("## File Inventory\n")
    out.append(f"Total files covered: **{len(files)}**\n")
    for f in files:
        out.append(f"- {rel(f)}")
    out.append("")

    for f in files:
        text = f.read_text(encoding="utf-8", errors="replace")
        lines = text.splitlines()
        symbols = extract_symbols(lines)
        total = len(lines)
        nonempty = count_nonempty(lines)

        out.append(f"## File: {rel(f)}\n")
        out.append(f"Role: {file_role(f)}")
        out.append(f"Lines: {total} | Non-empty: {nonempty} | Symbols: {len(symbols)}\n")

        out.append("### Symbol Inventory\n")
        if symbols:
            out.append("| Line | Kind | Symbol | Change Impact |")
            out.append("|---:|---|---|---|")
            for line_no, kind, name in symbols:
                impact = "signature/API coupling" if kind in {"fn", "trait"} else "data/structure coupling"
                if kind == "impl":
                    impact = "behavior binding and method semantics"
                out.append(
                    f"| {line_no} | {kind} | {markdown_escape_pipe(name)} | {impact} |"
                )
        else:
            out.append("No top-level symbols detected by static regex scan.")
        out.append("")

        out.append("### Line-by-Line Impact Map\n")
        out.append("| Line | Source | Operational Impact If Changed |")
        out.append("|---:|---|---|")
        for i, line in enumerate(lines, start=1):
            src = markdown_escape_pipe(line.rstrip())
            if src == "":
                src = "(blank)"
            impact = classify_line(line)
            out.append(f"| {i} | {src} | {impact} |")
        out.append("")

    out.append("## Change Safety Guide\n")
    out.append("- Prefer additive changes in protocol structs; preserve serde field names for backward compatibility.")
    out.append("- In flashing paths, preserve positive-response validation and do not weaken gate thresholds.")
    out.append("- For trait/interface changes in io/hw, run full workspace tests plus vendor-windows E2E.")
    out.append("- For any change touching line maps in live_runner/vendor_cat_comm/production_program, re-validate conformance evidence output.")
    out.append("")

    return "\n".join(out)


def render_pdf_from_markdown(md_text: str, out_pdf: Path) -> None:
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas

    width, height = A4
    left = 12 * mm
    right = width - 12 * mm
    top = height - 12 * mm
    bottom = 12 * mm
    line_h = 4.2 * mm

    c = canvas.Canvas(str(out_pdf), pagesize=A4)
    c.setTitle("AutoBreaking Full Design-Book and Code-Book")
    c.setFont("Courier", 8)

    y = top

    def new_page() -> None:
        nonlocal y
        c.showPage()
        c.setFont("Courier", 8)
        y = top

    for raw in md_text.splitlines():
        line = raw.expandtabs(4)
        while len(line) > 120:
            chunk = line[:120]
            if y < bottom:
                new_page()
            c.drawString(left, y, chunk)
            y -= line_h
            line = line[120:]
        if y < bottom:
            new_page()
        c.drawString(left, y, line)
        y -= line_h

    c.save()


def main() -> None:
    md = generate_md()
    OUT_MD.write_text(md, encoding="utf-8")
    render_pdf_from_markdown(md, OUT_PDF)
    print(f"Wrote {OUT_MD}")
    print(f"Wrote {OUT_PDF}")


if __name__ == "__main__":
    main()
