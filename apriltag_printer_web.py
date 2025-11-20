"""Flask UI for downloading AprilTag images and generating scaled PDFs.

This tool synchronizes tags from the official AprilTag image repository
(AprilRobotics/apriltag-imgs), exposes them via a simple web UI, and lets users
produce printable PDFs sized to a target edge length. The default selection
reflects the tag family, id, and edge size used by ``onics-t.py`` so that
operators can quickly print the expected landing tag.
"""
from __future__ import annotations

import io
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import requests
from flask import (
    Flask,
    Response,
    flash,
    redirect,
    render_template_string,
    request,
    send_file,
    url_for,
)
from reportlab.lib.units import inch
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader

APP = Flask(__name__)
APP.secret_key = "apriltag-printer-secret"

BASE_DIR = Path(__file__).resolve().parent
CACHE_DIR = BASE_DIR / "apriltag_cache"
CACHE_DIR.mkdir(exist_ok=True)

REPO_API_ROOT = "https://api.github.com/repos/AprilRobotics/apriltag-imgs/contents"
ONICS_T_PATH = BASE_DIR / "onics-t.py"


def mm_to_points(mm: float) -> float:
    return mm / 25.4 * 72.0


def read_onics_defaults() -> Dict[str, object]:
    """Extract AprilTag defaults from onics-t without importing heavy deps."""
    if not ONICS_T_PATH.exists():
        return {
            "family": "tagStandard41h12",
            "tag_id": 0,
            "edge_m": 0.144,
        }

    content = ONICS_T_PATH.read_text(encoding="utf-8")
    family_match = re.search(r"APRILTAG_FAMILY\s*=\s*'([^']+)'", content)
    tag_match = re.search(r"tag_landing_id\s*=\s*([0-9]+)", content)
    size_match = re.search(r"tag_landing_size\s*=\s*([0-9.]+)", content)

    return {
        "family": family_match.group(1) if family_match else "tagStandard41h12",
        "tag_id": int(tag_match.group(1)) if tag_match else 0,
        "edge_m": float(size_match.group(1)) if size_match else 0.144,
    }


def fetch_families() -> List[str]:
    resp = requests.get(REPO_API_ROOT, timeout=30)
    resp.raise_for_status()
    return [entry["name"] for entry in resp.json() if entry.get("type") == "dir"]


def fetch_family_entries(family: str) -> List[dict]:
    resp = requests.get(f"{REPO_API_ROOT}/{family}", timeout=30)
    resp.raise_for_status()
    return resp.json()


def download_family(family: str) -> List[Path]:
    family_dir = CACHE_DIR / family
    family_dir.mkdir(parents=True, exist_ok=True)

    downloaded: List[Path] = []
    for entry in fetch_family_entries(family):
        if entry.get("type") != "file" or not entry.get("name", "").lower().endswith(".png"):
            continue
        target = family_dir / entry["name"]
        if target.exists():
            continue
        img_resp = requests.get(entry["download_url"], timeout=30)
        img_resp.raise_for_status()
        target.write_bytes(img_resp.content)
        downloaded.append(target)
    return downloaded


def sync_all_tags() -> Dict[str, List[str]]:
    """Download every available AprilTag image into the local cache."""
    options: Dict[str, List[str]] = {}
    for family in fetch_families():
        download_family(family)
        tag_files = sorted((CACHE_DIR / family).glob("*.png"))
        options[family] = [path.name for path in tag_files]
    return options


@dataclass
class TagSelection:
    family: str
    filename: str
    edge_m: float

    @property
    def image_path(self) -> Path:
        return CACHE_DIR / self.family / self.filename

    @property
    def tag_id(self) -> int:
        match = re.search(r"_(\d+)\.png$", self.filename)
        return int(match.group(1)) if match else -1

    def pdf_name(self) -> str:
        return f"{self.family}_{self.tag_id}.pdf"


def create_tag_pdf(selection: TagSelection) -> io.BytesIO:
    if not selection.image_path.exists():
        raise FileNotFoundError(f"Image not cached: {selection.image_path}")

    edge_mm = selection.edge_m * 1000.0
    margin_mm = 10.0
    page_size = mm_to_points(edge_mm + (2 * margin_mm))

    buffer = io.BytesIO()
    pdf = canvas.Canvas(buffer, pagesize=(page_size, page_size))
    pdf.drawImage(
        ImageReader(selection.image_path),
        mm_to_points(margin_mm),
        mm_to_points(margin_mm),
        width=mm_to_points(edge_mm),
        height=mm_to_points(edge_mm),
        preserveAspectRatio=True,
        anchor="sw",
    )
    pdf.showPage()
    pdf.save()
    buffer.seek(0)
    return buffer


def cached_options() -> Dict[str, List[str]]:
    options: Dict[str, List[str]] = {}
    for family_dir in CACHE_DIR.iterdir():
        if not family_dir.is_dir():
            continue
        options[family_dir.name] = sorted([p.name for p in family_dir.glob("*.png")])
    return options


HOME_TEMPLATE = """
<!DOCTYPE html>
<html>
  <head>
    <title>AprilTag PDF Generator</title>
    <style>
      body { font-family: Arial, sans-serif; margin: 2rem; }
      form { margin-bottom: 2rem; }
      label { display: block; margin-top: 0.5rem; }
      select, input { padding: 0.4rem; min-width: 320px; }
      .flash { color: #c00; margin-bottom: 1rem; }
    </style>
  </head>
  <body>
    <h1>AprilTag PDF Generator</h1>
    {% with messages = get_flashed_messages() %}
      {% if messages %}
        <div class="flash">{{ messages[0] }}</div>
      {% endif %}
    {% endwith %}
    <p>Repository sync downloads all available AprilTag PNGs from the official <code>AprilRobotics/apriltag-imgs</code> repository and exposes them locally for PDF export.</p>
    <form action="{{ url_for('refresh') }}" method="post">
      <button type="submit">Refresh from official repository</button>
    </form>
    <form action="{{ url_for('generate') }}" method="post">
      <label>Tag family</label>
      <select
        name="family"
        id="family"
        required
        onchange="window.location.href='{{ url_for('home') }}?family=' + encodeURIComponent(this.value)"
      >
        {% for family in families %}
          <option value="{{ family }}" {% if family == selected_family %}selected{% endif %}>{{ family }}</option>
        {% endfor %}
      </select>
      <label>Tag image</label>
      <select name="filename" required>
        {% for name in family_files %}
          <option value="{{ name }}" {% if name == selected_file %}selected{% endif %}>{{ name }}</option>
        {% endfor %}
      </select>
      <label>Edge length (meters)</label>
      <input type="number" step="0.001" name="edge_m" value="{{ edge_m }}" min="0.01" required>
      <button type="submit">Generate PDF</button>
    </form>
    <p>Default selection uses onics-t expected tag: family {{ default_family }}, ID {{ default_tag_id }}, edge {{ edge_m }} m.</p>
  </body>
</html>
"""


@APP.route("/", methods=["GET"])
def home() -> Response:
    defaults = read_onics_defaults()
    options = cached_options()
    if not options:
        try:
            options = sync_all_tags()
        except Exception as exc:  # pragma: no cover - best effort for UX
            flash(f"Failed to sync repository: {exc}")
            options = {}

    families = sorted(options.keys())
    selected_family = request.args.get("family") or defaults["family"]
    family_files = options.get(selected_family, [])

    selected_file = request.args.get("filename")
    if selected_file is None and family_files:
        default_id = defaults["tag_id"]
        default_candidate = next(
            (name for name in family_files if f"_{default_id:05d}.png" in name or f"_{default_id}.png" in name),
            None,
        )
        selected_file = default_candidate or family_files[0]

    return render_template_string(
        HOME_TEMPLATE,
        families=families,
        selected_family=selected_family,
        family_files=family_files,
        selected_file=selected_file,
        edge_m=defaults["edge_m"],
        default_family=defaults["family"],
        default_tag_id=defaults["tag_id"],
    )


@APP.route("/refresh", methods=["POST"])
def refresh() -> Response:
    try:
        sync_all_tags()
        flash("Repository sync completed.")
    except Exception as exc:  # pragma: no cover - best effort for UX
        flash(f"Sync failed: {exc}")
    return redirect(url_for("home"))


@APP.route("/generate", methods=["POST"])
def generate() -> Response:
    family = request.form.get("family")
    filename = request.form.get("filename")
    edge_m = float(request.form.get("edge_m") or 0.0)

    if not family or not filename:
        flash("Family and tag selection are required.")
        return redirect(url_for("home"))

    selection = TagSelection(family=family, filename=filename, edge_m=edge_m)
    try:
        pdf_buffer = create_tag_pdf(selection)
    except Exception as exc:  # pragma: no cover - best effort for UX
        flash(f"PDF generation failed: {exc}")
        return redirect(url_for("home"))

    return send_file(
        pdf_buffer,
        as_attachment=True,
        download_name=selection.pdf_name(),
        mimetype="application/pdf",
    )


if __name__ == "__main__":
    APP.run(host="0.0.0.0", port=8000, debug=True)
