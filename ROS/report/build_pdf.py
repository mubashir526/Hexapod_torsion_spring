#!/usr/bin/env python3
"""Render experiment_report.md to a paginated PDF.

No pandoc or LaTeX is required: markdown -> HTML -> PDF via WeasyPrint. WeasyPrint has
no math engine, so the small subset of LaTeX used in the report ($k_x$, $|θ_0|$,
$q_{op}$, $\\sqrt{}$, sub/superscripts) is converted to styled HTML first. The converter
deliberately skips fenced and inline code so `code spans` are left alone.

Run:  python3 build_pdf.py
"""

from __future__ import annotations

import os
import re
import sys

import markdown
from weasyprint import HTML

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as D          # noqa: E402

SRC = os.path.join(D.HERE, "experiment_report.md")
OUT = os.path.join(D.HERE, "experiment_report.pdf")

CSS = """
@page {
  size: A4;
  margin: 17mm 14mm 16mm 14mm;
  @bottom-center { content: counter(page); font: 8pt "DejaVu Sans"; color: #777; }
  @top-right { content: "Passive Knee-Spring Assist — Simulation Study";
               font: 7pt "DejaVu Sans"; color: #999; }
}
@page :first { @bottom-center { content: none; } @top-right { content: none; } }

body { font-family: "DejaVu Serif", serif; font-size: 9.6pt; line-height: 1.42;
       color: #1a1a1a; text-align: justify; hyphens: auto; }

h1, h2, h3, h4 { font-family: "DejaVu Sans", sans-serif; color: #0b2545;
                 line-height: 1.22; text-align: left; }
h1 { font-size: 20pt; margin: 0 0 4pt 0; }
h2 { font-size: 13.5pt; margin: 0 0 8pt 0; padding-bottom: 3pt;
     border-bottom: 1.1pt solid #0b2545; page-break-before: always; }
h2.nobreak { page-break-before: avoid; }
h3 { font-size: 11pt; margin: 13pt 0 4pt 0; }
h4 { font-size: 9.8pt; margin: 10pt 0 3pt 0; }
p { margin: 0 0 6pt 0; }
strong { color: #0b2545; }

/* ---- title page ---- */
.title-page { page-break-after: always; text-align: center; padding-top: 42mm; }
.title-page .t { font-family: "DejaVu Sans", sans-serif; font-size: 25pt;
                 font-weight: bold; color: #0b2545; line-height: 1.2; }
.title-page .s { font-family: "DejaVu Sans", sans-serif; font-size: 12.5pt;
                 color: #444; margin-top: 8mm; }
.title-page .m { font-size: 10pt; color: #555; margin-top: 26mm; line-height: 1.8; }
.title-page hr { width: 45%; margin: 7mm auto; border: none;
                 border-top: 0.9pt solid #b8c4d4; }

/* ---- table of contents ---- */
.toc { page-break-after: always; }
.toc h2 { page-break-before: avoid; }
.toc ul { list-style: none; padding-left: 0; margin: 0; }
.toc ul ul { padding-left: 5.5mm; }
.toc li { font-family: "DejaVu Sans", sans-serif; font-size: 8.3pt; margin: 0.7pt 0;
          line-height: 1.25; }
.toc > ul > li { font-weight: bold; font-size: 9pt; margin-top: 2.6pt; }
.toc a { color: #0b2545; text-decoration: none; }

/* ---- tables ---- */
table { border-collapse: collapse; width: 100%; font-family: "DejaVu Sans", sans-serif;
        font-size: 7.4pt; margin: 3pt 0 9pt 0; }
th { background: #0b2545; color: #fff; font-weight: bold; text-align: left;
     padding: 2.6pt 3pt; border: 0.4pt solid #0b2545; }
td { padding: 2.2pt 3pt; border: 0.4pt solid #ccd4e0; vertical-align: top; }
tbody tr:nth-child(even) { background: #f2f5fa; }
thead { display: table-header-group; }
tr { page-break-inside: avoid; }

/* ---- figures ---- */
img { max-width: 100%; height: auto; display: block; margin: 4pt auto 2pt auto;
      page-break-inside: avoid; }
p { orphans: 2; widows: 2; }
em strong, strong em { font-family: "DejaVu Sans", sans-serif; }

/* figure captions: '***Figure N.*** ...' becomes <em><strong>..</strong></em> */
p > em:first-child > strong { color: #0b2545; }

code { font-family: "DejaVu Sans Mono", monospace; font-size: 7.9pt;
       background: #f0f2f6; padding: 0 1.5pt; }
pre { background: #f5f6f9; border-left: 2.2pt solid #b8c4d4; padding: 5pt 7pt;
      font-size: 8.2pt; page-break-inside: avoid; }
pre code { background: none; font-size: 8.2pt; }

hr { border: none; border-top: 0.5pt solid #d5dbe5; margin: 8pt 0; }
.math { font-family: "DejaVu Serif", serif; font-style: italic; }
.math sub, .math sup { font-style: normal; font-size: 0.72em; }
ol, ul { margin: 0 0 6pt 0; padding-left: 6mm; }
li { margin: 0 0 2.5pt 0; }
"""

# ------------------------------------------------------------------ math subset
_CMD = {
    r"\min": "min", r"\max": "max", r"\mathrm": "", r"\text": "",
    r"\,": "\u2009", r"\;": "\u2009", r"\!": "", r"\cdot": "·",
    r"\times": "×", r"\approx": "≈", r"\le": "≤", r"\ge": "≥",
    r"\in": "\u2208", r"\pm": "±",
}


def _math(expr: str) -> str:
    """Convert the small LaTeX subset used in this report to inline HTML."""
    s = expr
    s = re.sub(r"\\sqrt\{([^{}]*)\}", r"√(\1)", s)
    s = re.sub(r"\\(?:mathrm|text)\{([^{}]*)\}", r"\1", s)
    for k, v in _CMD.items():
        s = s.replace(k, v)
    s = s.replace(r"\{", "{").replace(r"\}", "}").replace(r"\|", "|")
    s = re.sub(r"_\{([^{}]*)\}", r"<sub>\1</sub>", s)
    s = re.sub(r"\^\{([^{}]*)\}", r"<sup>\1</sup>", s)
    s = re.sub(r"_([A-Za-z0-9])", r"<sub>\1</sub>", s)
    s = re.sub(r"\^([A-Za-z0-9*])", r"<sup>\1</sup>", s)
    s = s.replace("\\", "")
    return f'<span class="math">{s}</span>'


def convert_math(text: str) -> str:
    """Replace $...$ outside of fenced blocks and inline code."""
    out = []
    for i, block in enumerate(re.split(r"(```.*?```)", text, flags=re.S)):
        if i % 2:                                  # fenced code — leave alone
            out.append(block)
            continue
        parts = []
        for j, seg in enumerate(re.split(r"(`[^`]*`)", block)):
            if j % 2:                              # inline code — leave alone
                parts.append(seg)
                continue
            parts.append(re.sub(r"\$([^$\n]+)\$", lambda m: _math(m.group(1)), seg))
        out.append("".join(parts))
    return "".join(out)


def main() -> int:
    with open(SRC) as fh:
        text = fh.read()

    lines = text.splitlines()
    # The H1 block and the abstract become the title page; the rest is the body.
    try:
        body_start = next(i for i, l in enumerate(lines)
                          if l.startswith("## ") and "Abstract" not in l)
    except StopIteration:
        body_start = 0
    head = "\n".join(lines[:body_start])
    body = "\n".join(lines[body_start:])

    abstract = head.split("## Abstract", 1)[1] if "## Abstract" in head else ""
    abstract = abstract.strip().lstrip("-").strip()
    meta_lines = [l for l in lines[:12] if l.startswith("**")]

    md = markdown.Markdown(extensions=["tables", "fenced_code", "toc", "attr_list",
                                       "sane_lists"],
                           extension_configs={"toc": {"toc_depth": "2-3"}})
    body_html = md.convert(convert_math(body))
    toc_html = md.toc
    abstract_html = markdown.markdown(convert_math(abstract), extensions=["tables"])
    meta_html = markdown.markdown(convert_math("\n".join(meta_lines)))

    html = f"""<!DOCTYPE html><html><head><meta charset="utf-8">
<title>Passive Knee-Spring Assist — Simulation Study</title>
<style>{CSS}</style></head><body>

<div class="title-page">
  <div class="t">Passive Knee-Spring Assist<br>on a Quadruped</div>
  <hr>
  <div class="s">A Simulation Study across Five Experiment Phases</div>
  <div class="m">{meta_html}</div>
</div>

<div class="toc"><h2 class="nobreak">Contents</h2>{toc_html}</div>

<h2 class="nobreak">Abstract</h2>
{abstract_html}

{body_html}
</body></html>"""

    debug = os.path.join(D.HERE, "experiment_report.html")
    with open(debug, "w") as fh:
        fh.write(html)
    HTML(string=html, base_url=D.HERE).write_pdf(OUT)

    size_kb = os.path.getsize(OUT) / 1024
    print(f"wrote {OUT}  ({size_kb:.0f} kB)")
    print(f"      {debug}  (intermediate HTML, for debugging layout)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
