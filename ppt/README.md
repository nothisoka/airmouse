# NIDAR AirMouse — Progress Presentation & Technical Reports

This folder contains the complete presentation materials and technical documentation for the NIDAR AirMouse autonomous drone project.

---

### Files in this Directory

| File | Format | Description & Usage |
|---|---|---|
| **[`presentation.html`](presentation.html)** | Interactive HTML Deck | Self-contained, presentation-ready slide deck with dark aerospace styling, keyboard controls (`←`/`→`/`Space`), and fullscreen toggle. Can be opened directly in any browser. |
| **[`PRESENTATION.md`](PRESENTATION.md)** | Marp / Markdown | 10-slide standard presentation source compatible with Marp, Pandoc, and VS Code slide previewers. |
| **[`generate_pptx.py`](generate_pptx.py)** | Python Script | Generates an official Microsoft PowerPoint (`.pptx`) presentation file. Requires `pip install python-pptx`. |
| **[`AUTONOMY_SYSTEM_REPORT.md`](AUTONOMY_SYSTEM_REPORT.md)** | Technical Report | Deep, comprehensive technical report covering Phase A through Phase D progress, benchmarks, and physical hardware deployment blueprints. |

---

### How to Use

#### 1. Open the Interactive Slide Deck:
Simply open `ppt/presentation.html` in your web browser (Chrome, Firefox, Edge, etc.):
```bash
# Example (Linux)
google-chrome 
# or
xdg-open ppt/presentation.html
```

#### 2. Generate Native PowerPoint (`.pptx`):
```bash
pip install python-pptx
python3 ppt/generate_pptx.py
```
This generates `ppt/NIDAR_AirMouse_Progress_Presentation.pptx`.

