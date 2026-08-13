from __future__ import annotations

import datetime as dt
import hashlib
import json
import re
import subprocess
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable

ROOT = Path(__file__).resolve().parents[1]
OUT_MD = ROOT / "docs" / "FULL_BOOK_ULTIMATE.md"
OUT_PDF = ROOT / "docs" / "FULL_BOOK_ULTIMATE.pdf"

EXCLUDE_DIRS = {
    ".git",
    "target",
    ".vscode",
    "node_modules",
    "__pycache__",
}

INCLUDE_EXTS = {
    ".rs",
    ".toml",
    ".md",
    ".yml",
    ".yaml",
    ".ps1",
    ".sh",
    ".json",
}

LINE_MAP_EXTS = {
    ".rs",
    ".toml",
    ".yml",
    ".yaml",
    ".ps1",
    ".sh",
}


def run_cmd(cmd: list[str]) -> str:
    try:
        p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=False, check=False)
        stdout = p.stdout.decode("utf-8", errors="replace") if p.stdout else ""
        stderr = p.stderr.decode("utf-8", errors="replace") if p.stderr else ""
        if p.returncode == 0:
            return stdout.strip()
        return (stdout + "\n" + stderr).strip()
    except Exception as exc:
        return f"<command failed: {' '.join(cmd)} | {exc}>"


def sanitize_text(raw: str) -> str:
    # Defensive cleanup for occasional mojibake from external tools.
    cleaned = raw.replace("\ufffd", "?")
    cleaned = cleaned.replace("â”‚", "|").replace("â”œ", "|-").replace("â”€", "-")
    cleaned = cleaned.replace("â””", "`-").replace("Ã—", "x")
    return cleaned


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def all_files() -> list[Path]:
    out: list[Path] = []
    for p in ROOT.rglob("*"):
        if not p.is_file():
            continue
        parts = set(p.relative_to(ROOT).parts)
        if parts & EXCLUDE_DIRS:
            continue
        if p.suffix.lower() in INCLUDE_EXTS:
            out.append(p)
    out.sort(key=lambda x: x.as_posix())
    return out


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def classify_line(line: str) -> str:
    s = line.strip()
    if not s:
        return "blank separator; no runtime behavior."
    if s.startswith("//") or s.startswith("#"):
        return "comment/documentation; changing affects understanding and intent traceability."
    if s.startswith("use "):
        return "import wiring; changing path/name can break compilation and module linkage."
    if s.startswith("mod ") or s.startswith("pub mod "):
        return "module declaration; changing can break namespace graph."
    if s.startswith("pub struct ") or s.startswith("struct "):
        return "data layout contract; field changes can break callers/serde/tests."
    if s.startswith("pub enum ") or s.startswith("enum "):
        return "state machine variants; changes ripple through matches and business logic."
    if s.startswith("pub trait ") or s.startswith("trait "):
        return "interface contract; changes impact all implementations and users."
    if re.match(r"^(pub\s+)?(async\s+)?fn\s+", s):
        return "function signature/body; changes affect API behavior and control-flow."
    if s.startswith("impl "):
        return "implementation binding; behavior semantics and trait conformance live here."
    if s.startswith("match "):
        return "branch dispatcher; arm changes alter behavior class handling."
    if s.startswith("if ") or s.startswith("if let"):
        return "runtime gate; condition changes can enable/disable critical paths."
    if s.startswith("for ") or s.startswith("while ") or s.startswith("loop"):
        return "iteration logic; may alter timing, throughput, and safety constraints."
    if "?" in s:
        return "error propagation point; changes alter failure propagation semantics."
    if "unwrap(" in s or "expect(" in s:
        return "panic boundary; edits alter crash behavior and resilience."
    if "flash" in s.lower() or "uds" in s.lower() or "send" in s or "write" in s:
        return "protocol or I/O critical; changes may impact external integration correctness."
    if s in {"{", "}", "};"}:
        return "scope delimiter; structural only, but wrong placement changes ownership/lifetimes."
    return "support logic; impact depends on neighboring block and data flow."


def role_of(path: Path) -> str:
    p = rel(path)
    if p.startswith("src/io/"):
        return "hardware/protocol integration, live flashing, diagnostics, and production gating"
    if p.startswith("src/bin/"):
        return "operational executable entrypoint"
    if p.startswith("src/"):
        return "core simulation, domain logic, and ECU/network behavior"
    if p.startswith("tests/"):
        return "automated quality gate and regression coverage"
    if p.startswith("docs/"):
        return "human-facing architecture, protocols, runbooks, and design records"
    if p.startswith("scripts/"):
        return "automation and reproducible operational workflows"
    if p.startswith(".github/"):
        return "CI/CD governance and continuous validation"
    return "project support artifact"


def extract_rs_symbols(lines: list[str]) -> list[tuple[int, str, str]]:
    pats = [
        ("module", re.compile(r"^\s*(pub\s+)?mod\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("struct", re.compile(r"^\s*(pub\s+)?struct\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("enum", re.compile(r"^\s*(pub\s+)?enum\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("trait", re.compile(r"^\s*(pub\s+)?trait\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("impl", re.compile(r"^\s*impl\s+([A-Za-z_][A-Za-z0-9_:<> ,]*)")),
        ("fn", re.compile(r"^\s*(pub\s+)?(async\s+)?fn\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("const", re.compile(r"^\s*(pub\s+)?const\s+([A-Za-z_][A-Za-z0-9_]*)")),
        ("type", re.compile(r"^\s*(pub\s+)?type\s+([A-Za-z_][A-Za-z0-9_]*)")),
    ]
    out: list[tuple[int, str, str]] = []
    for i, ln in enumerate(lines, start=1):
        for kind, pat in pats:
            m = pat.search(ln)
            if m:
                name = m.group(m.lastindex or 1)
                out.append((i, kind, name.strip()))
                break
    return out


def extract_ps_symbols(lines: list[str]) -> list[tuple[int, str, str]]:
    pat = re.compile(r"^\s*function\s+([A-Za-z_][A-Za-z0-9_-]*)")
    out: list[tuple[int, str, str]] = []
    for i, ln in enumerate(lines, start=1):
        m = pat.search(ln)
        if m:
            out.append((i, "function", m.group(1)))
    return out


def extract_md_sections(lines: list[str]) -> list[tuple[int, str, str]]:
    pat = re.compile(r"^(#+)\s+(.+)$")
    out: list[tuple[int, str, str]] = []
    for i, ln in enumerate(lines, start=1):
        m = pat.search(ln)
        if m:
            out.append((i, f"h{len(m.group(1))}", m.group(2).strip()))
    return out


def extract_symbols(path: Path, lines: list[str]) -> list[tuple[int, str, str]]:
    ext = path.suffix.lower()
    if ext == ".rs":
        return extract_rs_symbols(lines)
    if ext == ".ps1" or ext == ".sh":
        return extract_ps_symbols(lines)
    if ext == ".md":
        return extract_md_sections(lines)
    return []


def module_name_for_src(path: Path) -> str:
    p = rel(path)
    stem = path.stem
    if p == "src/lib.rs":
        return "lib_root"
    if p == "src/main.rs":
        return "main_app"
    if p.startswith("src/io/"):
        return f"io_{stem}"
    if p.startswith("src/bin/"):
        return f"bin_{stem}"
    return stem


def count_nonempty(lines: Iterable[str]) -> int:
    return sum(1 for x in lines if x.strip())


def md_pipe_escape(s: str) -> str:
    return s.replace("|", "\\|")


def collect_function_index(rs_files: list[Path]) -> set[str]:
    funcs: set[str] = set()
    pat = re.compile(r"^\s*(pub\s+)?(async\s+)?fn\s+([A-Za-z_][A-Za-z0-9_]*)")
    for f in rs_files:
        for ln in read_text(f).splitlines():
            m = pat.search(ln)
            if m:
                funcs.add(m.group(3))
    return funcs


def call_refs_for_file(path: Path, known_funcs: set[str]) -> Counter[str]:
    c: Counter[str] = Counter()
    text = read_text(path)
    for name in re.findall(r"\b([A-Za-z_][A-Za-z0-9_]*)\s*\(", text):
        if name in known_funcs:
            c[name] += 1
    return c


def crate_use_edges(rs_files: list[Path]) -> dict[str, set[str]]:
    edges: dict[str, set[str]] = defaultdict(set)
    use_pat = re.compile(r"^\s*use\s+crate::([A-Za-z_][A-Za-z0-9_]*)(?:::.*)?;")
    for f in rs_files:
        src = module_name_for_src(f)
        for ln in read_text(f).splitlines():
            m = use_pat.search(ln)
            if m:
                edges[src].add(m.group(1))
    return edges


def render_pdf(md_text: str, out_pdf: Path) -> None:
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas

    width, height = A4
    left = 10 * mm
    top = height - 10 * mm
    bottom = 10 * mm
    line_h = 4 * mm

    c = canvas.Canvas(str(out_pdf), pagesize=A4)
    c.setTitle("AutoBreaking Full Ultimate Book")
    c.setFont("Courier", 7.5)

    y = top

    def np() -> None:
        nonlocal y
        c.showPage()
        c.setFont("Courier", 7.5)
        y = top

    for raw in md_text.splitlines():
        line = raw.expandtabs(4)
        while len(line) > 145:
            chunk = line[:145]
            if y < bottom:
                np()
            c.drawString(left, y, chunk)
            y -= line_h
            line = line[145:]
        if y < bottom:
            np()
        c.drawString(left, y, line)
        y -= line_h

    c.save()


def generate_markdown() -> str:
    now = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    files = all_files()
    rs_files = [f for f in files if f.suffix.lower() == ".rs"]
    funcs = collect_function_index(rs_files)
    edges = crate_use_edges(rs_files)

    git_head = run_cmd(["git", "rev-parse", "--short", "HEAD"])
    git_status = run_cmd(["git", "status", "--short"])
    cargo_tree = sanitize_text(run_cmd(["cargo", "tree", "--charset", "ascii"]))

    out: list[str] = []
    out.append("# AutoBreaking Ultimate Full Book")
    out.append("")
    out.append("> Complete design-book and code-book generated from repository state with maximal practical coverage.")
    out.append("")
    out.append("## Quick Navigation")
    out.append("- Meta")
    out.append("- Architecture Narrative")
    out.append("- Module Coupling Graph")
    out.append("- Dependency Tree Snapshot")
    out.append("- Master File Catalog")
    out.append("- Per-file Deep Dive")
    out.append("- Cross File Traceability")
    out.append("- Change Simulation Playbook")
    out.append("")
    out.append("## Meta")
    out.append(f"- Generated at: {now}")
    out.append(f"- Git commit: {git_head}")
    out.append(f"- Files covered: {len(files)}")
    out.append(f"- Rust files covered: {len(rs_files)}")
    out.append("")

    out.append("## Architecture Narrative")
    out.append("1. Core simulation and ECU domain logic live under src and feed deterministic/runtime scenarios.")
    out.append("2. Live hardware and flashing orchestration live in src/io with strict protocol checks and gating.")
    out.append("3. Binary entrypoints in src/bin drive CLI workflows and template bridge interactions.")
    out.append("4. tests contains regression, property, and integration suites including vendor bridge e2e.")
    out.append("5. scripts and .github files define operational and CI automation.")
    out.append("")

    out.append("## Module Coupling Graph")
    out.append("```mermaid")
    out.append("graph LR")
    for src, dsts in sorted(edges.items()):
        for dst in sorted(dsts):
            out.append(f"  {src} --> {dst}")
    out.append("```")
    out.append("")

    out.append("## Dependency Tree Snapshot")
    out.append("<details>")
    out.append("<summary>Expand full dependency tree</summary>")
    out.append("")
    out.append("```text")
    out.append(cargo_tree if cargo_tree else "<no cargo tree output>")
    out.append("```")
    out.append("</details>")
    out.append("")

    out.append("## Working Tree Status")
    out.append("```text")
    out.append(sanitize_text(git_status) if git_status else "clean")
    out.append("```")
    out.append("")

    out.append("## Master File Catalog")
    out.append("<details>")
    out.append("<summary>Expand full file catalog with hashes</summary>")
    out.append("")
    for f in files:
        b = f.read_bytes()
        out.append(f"- {rel(f)} | {len(b)} bytes | sha256={sha256_bytes(b)}")
    out.append("")
    out.append("</details>")
    out.append("")

    per_file_call_refs: dict[str, Counter[str]] = {}
    for rf in rs_files:
        per_file_call_refs[rel(rf)] = call_refs_for_file(rf, funcs)

    for f in files:
        text = read_text(f)
        lines = text.splitlines()
        symbols = extract_symbols(f, lines)
        p = rel(f)

        out.append(f"## File Deep Dive: {p}")
        out.append(f"Role: {role_of(f)}")
        out.append(f"Line count: {len(lines)} | Non-empty: {count_nonempty(lines)} | Symbol count: {len(symbols)}")
        out.append("")

        if f.suffix.lower() == ".rs":
            refs = per_file_call_refs.get(p, Counter())
            top_refs = refs.most_common(25)
            out.append("### Function Call Hotspots")
            if top_refs:
                out.append("| Function | Approx Calls In File |")
                out.append("|---|---:|")
                for name, c in top_refs:
                    out.append(f"| {name} | {c} |")
            else:
                out.append("No call hotspots found by static regex pass.")
            out.append("")

        out.append("### Symbol Index")
        if symbols:
            out.append("| Line | Kind | Name | If Changed, Likely Impact |")
            out.append("|---:|---|---|---|")
            for ln, kind, name in symbols:
                if kind in {"fn", "trait"}:
                    impact = "API and behavior coupling to callers/implementers"
                elif kind in {"struct", "enum", "type"}:
                    impact = "data/schema coupling and match/serde break risk"
                elif kind == "impl":
                    impact = "method behavior and trait semantics"
                else:
                    impact = "namespace and compile-time linkage"
                out.append(f"| {ln} | {kind} | {md_pipe_escape(name)} | {impact} |")
        else:
            out.append("No symbols extracted for this file type.")
        out.append("")

        if f.suffix.lower() in LINE_MAP_EXTS:
            out.append("### Line by Line Change Impact")
            out.append("| Line | Code | What Happens If This Line Changes |")
            out.append("|---:|---|---|")
            for i, line in enumerate(lines, start=1):
                code = md_pipe_escape(line.rstrip())
                if code == "":
                    code = "(blank)"
                out.append(f"| {i} | {code} | {classify_line(line)} |")
            out.append("")
        else:
            out.append("### Content Snapshot")
            out.append("```text")
            out.extend(lines[:220])
            if len(lines) > 220:
                out.append(f"... truncated {len(lines) - 220} lines for this snapshot in markdown volume ...")
            out.append("```")
            out.append("")

    out.append("## Cross File Traceability")
    out.append("- Build root: Cargo.toml -> src/lib.rs, src/main.rs, src/bin/*")
    out.append("- Live flash execution chain: src/bin/simulator_cli.rs -> src/io/production_program.rs -> src/io/live_runner.rs -> src/io/hw.rs -> adapter implementations")
    out.append("- Vendor path: src/io/vendor_cat_comm.rs + src/bin/cat_comm_bridge.rs + tests/vendor_bridge_e2e.rs")
    out.append("- Quality gates: tests/*.rs + .github/workflows/*.yml")
    out.append("")

    out.append("## Change Simulation Playbook")
    out.append("1. If you change protocol validation in src/io/live_runner.rs, rerun cargo test --workspace and vendor bridge e2e.")
    out.append("2. If you change adapter contracts in src/io/hw.rs, validate every adapter implementation and cli/prod phase wiring.")
    out.append("3. If you change report schema fields, update consumers, docs, and assertions in tests/vendor_bridge_e2e.rs.")
    out.append("4. If you change Cargo features, verify ci workflows for feature matrix drift.")
    out.append("")

    out.append("## Full Raw Context Appendix")
    out.append("This book is machine-generated for maximal practical coverage, including source maps, symbols, call hotspots, and change-impact guidance.")

    return "\n".join(out)


def main() -> None:
    md = generate_markdown()
    OUT_MD.write_text(md, encoding="utf-8")
    render_pdf(md, OUT_PDF)
    print(f"Wrote {OUT_MD}")
    print(f"Wrote {OUT_PDF}")


if __name__ == "__main__":
    main()
