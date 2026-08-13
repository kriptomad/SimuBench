from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfgen.canvas import Canvas
from reportlab.platypus import Paragraph, Preformatted, SimpleDocTemplate, Spacer, Table, TableStyle

ROOT = Path(__file__).resolve().parents[1]


@dataclass
class BookTarget:
    md_path: Path
    pdf_path: Path
    title: str
    subtitle: str


TARGETS = [
    BookTarget(
        md_path=ROOT / "docs" / "FULL_BOOK_ULTIMATE.md",
        pdf_path=ROOT / "docs" / "FULL_BOOK_ULTIMATE_PREMIUM.pdf",
        title="AutoBreaking Ultimate Full Book",
        subtitle="Design, Architecture, Code and Traceability",
    ),
    BookTarget(
        md_path=ROOT / "docs" / "FULL_BOOK_V2_CHANGE_ENGINEERING.md",
        pdf_path=ROOT / "docs" / "FULL_BOOK_V2_CHANGE_ENGINEERING_PREMIUM.pdf",
        title="AutoBreaking Change Engineering V2",
        subtitle="Risk Matrix, Cascading Impact and Safe Refactor Waves",
    ),
]


def escape_html(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def page_decor(canvas: Canvas, doc: SimpleDocTemplate) -> None:
    w, h = A4
    canvas.saveState()

    # Header band
    canvas.setFillColor(colors.HexColor("#F0F4FA"))
    canvas.rect(0, h - 18 * mm, w, 18 * mm, stroke=0, fill=1)
    canvas.setFillColor(colors.HexColor("#1E3A5F"))
    canvas.setFont("Helvetica-Bold", 9)
    canvas.drawString(16 * mm, h - 11 * mm, "AutoBreaking Technical Manual")

    # Footer and page number
    canvas.setStrokeColor(colors.HexColor("#D8DEE9"))
    canvas.line(14 * mm, 12 * mm, w - 14 * mm, 12 * mm)
    canvas.setFillColor(colors.HexColor("#4C566A"))
    canvas.setFont("Helvetica", 8)
    canvas.drawString(16 * mm, 8 * mm, datetime.now().strftime("Generated %Y-%m-%d %H:%M"))
    canvas.drawRightString(w - 16 * mm, 8 * mm, f"Page {canvas.getPageNumber()}")

    canvas.restoreState()


def build_story(md_text: str, title: str, subtitle: str):
    styles = getSampleStyleSheet()

    h1 = ParagraphStyle(
        "H1",
        parent=styles["Heading1"],
        fontName="Helvetica-Bold",
        fontSize=19,
        leading=23,
        textColor=colors.HexColor("#1E3A5F"),
        spaceAfter=9,
    )
    h2 = ParagraphStyle(
        "H2",
        parent=styles["Heading2"],
        fontName="Helvetica-Bold",
        fontSize=14,
        leading=18,
        textColor=colors.HexColor("#204A73"),
        spaceBefore=10,
        spaceAfter=6,
    )
    h3 = ParagraphStyle(
        "H3",
        parent=styles["Heading3"],
        fontName="Helvetica-Bold",
        fontSize=11,
        leading=14,
        textColor=colors.HexColor("#2E3440"),
        spaceBefore=8,
        spaceAfter=4,
    )
    body = ParagraphStyle(
        "Body",
        parent=styles["BodyText"],
        fontName="Helvetica",
        fontSize=9.5,
        leading=13.5,
        textColor=colors.HexColor("#2E3440"),
        spaceAfter=4,
    )
    bullet = ParagraphStyle(
        "Bullet",
        parent=body,
        leftIndent=12,
        bulletIndent=2,
        spaceAfter=2,
    )
    cover_title = ParagraphStyle(
        "CoverTitle",
        parent=styles["Title"],
        fontName="Helvetica-Bold",
        fontSize=28,
        leading=32,
        alignment=TA_CENTER,
        textColor=colors.HexColor("#1E3A5F"),
    )
    cover_subtitle = ParagraphStyle(
        "CoverSub",
        parent=body,
        fontName="Helvetica",
        fontSize=12,
        leading=16,
        alignment=TA_CENTER,
        textColor=colors.HexColor("#4C566A"),
    )

    story = []

    # Cover
    story.append(Spacer(1, 45 * mm))
    story.append(Paragraph(escape_html(title), cover_title))
    story.append(Spacer(1, 7 * mm))
    story.append(Paragraph(escape_html(subtitle), cover_subtitle))
    story.append(Spacer(1, 9 * mm))
    story.append(Paragraph(escape_html(datetime.now().strftime("Generated on %Y-%m-%d at %H:%M")), cover_subtitle))
    story.append(Spacer(1, 100 * mm))

    banner = Table(
        [["Comprehensive Engineering Reference"]],
        colWidths=[170 * mm],
        rowHeights=[10 * mm],
    )
    banner.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), colors.HexColor("#EAF2FF")),
                ("TEXTCOLOR", (0, 0), (-1, -1), colors.HexColor("#1E3A5F")),
                ("ALIGN", (0, 0), (-1, -1), "CENTER"),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("FONTNAME", (0, 0), (-1, -1), "Helvetica-Bold"),
                ("FONTSIZE", (0, 0), (-1, -1), 10),
                ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#C5D6F2")),
            ]
        )
    )
    story.append(banner)
    story.append(Spacer(1, 16 * mm))
    story.append(Paragraph("<b>Table of Contents</b>", h2))

    lines = md_text.splitlines()
    toc_entries = []
    for line in lines:
        if line.startswith("# "):
            toc_entries.append((1, line[2:].strip()))
        elif line.startswith("## "):
            toc_entries.append((2, line[3:].strip()))
        elif line.startswith("### "):
            toc_entries.append((3, line[4:].strip()))

    for level, text in toc_entries[:140]:
        indent = "" if level == 1 else ("&nbsp;" * 6 if level == 2 else "&nbsp;" * 12)
        style = body if level <= 2 else bullet
        story.append(Paragraph(f"{indent}{escape_html(text)}", style))

    story.append(Spacer(1, 10 * mm))

    in_code = False
    code_acc = []

    def flush_code():
        nonlocal code_acc
        if not code_acc:
            return
        text = "\n".join(code_acc)
        story.append(
            Preformatted(
                text,
                ParagraphStyle(
                    "Code",
                    fontName="Courier",
                    fontSize=7.7,
                    leading=10,
                    backColor=colors.HexColor("#F6F8FB"),
                    borderColor=colors.HexColor("#D8DEE9"),
                    borderWidth=0.5,
                    borderPadding=5,
                ),
            )
        )
        story.append(Spacer(1, 2 * mm))
        code_acc = []

    for raw in lines:
        line = raw.rstrip("\n")

        if line.strip().startswith("```"):
            if in_code:
                in_code = False
                flush_code()
            else:
                in_code = True
            continue

        if in_code:
            code_acc.append(line)
            continue

        if line.startswith("# "):
            story.append(Spacer(1, 3 * mm))
            story.append(Paragraph(escape_html(line[2:].strip()), h1))
            continue
        if line.startswith("## "):
            story.append(Paragraph(escape_html(line[3:].strip()), h2))
            continue
        if line.startswith("### "):
            story.append(Paragraph(escape_html(line[4:].strip()), h3))
            continue

        if line.startswith("- "):
            story.append(Paragraph(escape_html(line[2:].strip()), bullet, bulletText="-"))
            continue

        if line.startswith("|") and line.endswith("|"):
            # Keep markdown tables readable as fixed text blocks.
            story.append(
                Preformatted(
                    line,
                    ParagraphStyle(
                        "TblLine",
                        fontName="Courier",
                        fontSize=7.6,
                        leading=9.4,
                        backColor=colors.HexColor("#FBFCFE"),
                    ),
                )
            )
            continue

        if not line.strip():
            story.append(Spacer(1, 1.2 * mm))
            continue

        story.append(Paragraph(escape_html(line), body))

    flush_code()
    return story


def render_book(target: BookTarget) -> None:
    md_text = target.md_path.read_text(encoding="utf-8", errors="replace")
    story = build_story(md_text, target.title, target.subtitle)

    doc = SimpleDocTemplate(
        str(target.pdf_path),
        pagesize=A4,
        leftMargin=16 * mm,
        rightMargin=16 * mm,
        topMargin=24 * mm,
        bottomMargin=16 * mm,
        title=target.title,
        author="AutoBreaking",
    )
    doc.build(story, onFirstPage=page_decor, onLaterPages=page_decor)


def main() -> None:
    for target in TARGETS:
        if not target.md_path.exists():
            print(f"SKIP: missing source markdown {target.md_path}")
            continue
        render_book(target)
        print(f"Wrote {target.pdf_path}")


if __name__ == "__main__":
    main()
