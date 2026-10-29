"""What the base install may import, and what branching code may name.

* LeRobot, torch and flyto-core are optional: importing the package, its
  adapter and its discovery imports none of them.
* Per-device detail is configuration: no source file names a robot model,
  an arm profile or a joint, so a second arm is a second config file.
* No address or credential literal ships.
* A source file stays under :data:`MAX_FILE_LINES`; the few that were over it
  when the limit came in may only shrink (:data:`OVER_LIMIT`), and leave the
  list once under it -- the count of large files only goes down.
"""

from __future__ import annotations

import ipaddress
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "flyto_modules_lerobot"
IPV4 = re.compile(r"(?<![\d.])(\d{1,3}(?:\.\d{1,3}){3})(?![\d.])")
SECRET_ASSIGNMENT = re.compile(r"(?i)\b(password|passwd|token|secret)\b\s*[:=]\s*['\"][^'\"]+['\"]")
# Model names, profiles and joint names belong in an arm's config file only.
DEVICE_WORDS = re.compile(
    r"(?i)am[-_]?arm|so[-_]?10[01]|so[-_]?arm|koch|alohamini|am-follower|shoulder_|elbow_|wrist_|sts3\d{3}|"
    r"ttyACM|ttyUSB|feetech|dynamixel"
)


def _sources() -> list[Path]:
    return sorted(SOURCE.rglob("*.py"))


def test_importing_the_package_imports_no_lerobot_torch_or_core():
    script = (
        "import sys\n"
        "import flyto_modules_lerobot, flyto_modules_lerobot.plugin, flyto_modules_lerobot.discovery\n"
        "import flyto_modules_lerobot.modules\n"
        "loaded = sorted(n for n in sys.modules if n.split('.')[0] in {'lerobot', 'torch', 'core'})\n"
        "assert not loaded, loaded\n"
    )
    subprocess.run([sys.executable, "-c", script], check=True, cwd=ROOT,
                   env={"PYTHONPATH": str(ROOT / "src"), "PATH": ""})


def test_no_source_names_a_model_a_profile_a_joint_or_a_port():
    found = [
        f"{path.relative_to(ROOT)}:{number}: {match.group(0)}"
        for path in _sources()
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1)
        if not line.lstrip().startswith("#")
        for match in DEVICE_WORDS.finditer(line)
    ]
    assert found == []


def test_no_address_literal_but_loopback():
    files = _sources() + [path for path in ROOT.rglob("*.json") if not {".venv", ".git"} & set(path.parts)]
    found = []
    for path in files:
        for match in IPV4.finditer(path.read_text(encoding="utf-8")):
            try:
                address = ipaddress.ip_address(match.group(1))
            except ValueError:
                continue
            if not address.is_loopback:
                found.append(f"{path.relative_to(ROOT)}: {address}")
    assert found == []


def test_no_credential_literal():
    found = [
        f"{path.relative_to(ROOT)}: {match.group(0)}"
        for path in _sources()
        for match in SECRET_ASSIGNMENT.finditer(path.read_text(encoding="utf-8"))
    ]
    assert found == []


def test_env_example_names_every_setting():
    from flyto_modules_lerobot import capabilities

    lines = (ROOT / ".env.example").read_text(encoding="utf-8").splitlines()
    settings = dict(line.split("=", 1) for line in lines if line and not line.startswith("#"))
    assert settings[capabilities.ENV_CONFIG] == ""
    assert capabilities.ENV_AUTODISCOVER in settings


#: The most lines a source file may have (flyto-cloud's atomicity ratchet uses the same).
MAX_FILE_LINES = 700
#: Files over the limit when it came in (2026-10-29), each at its length now: each may only shrink. The adapter
#: was split into looks, pick, place and teach, and the refusal codes left the capability table.
OVER_LIMIT = {
    "perception.py": 1987,
    "lerobot_adapter.py": 982,
    "capabilities.py": 744,
    "simulation.py": 793,
    "mujoco_sim.py": 760,
}


def _lines(path: Path) -> int:
    return len(path.read_text(encoding="utf-8").splitlines())


def test_no_source_file_grows_past_its_limit():
    over = {path.name: _lines(path) for path in _sources()
            if _lines(path) > OVER_LIMIT.get(path.name, MAX_FILE_LINES)}
    assert over == {}, f"split the new unit out, or shrink the file: {over}"


def test_a_file_back_under_the_limit_leaves_the_list():
    sizes = {path.name: _lines(path) for path in _sources()}
    back = [name for name in OVER_LIMIT if sizes.get(name, 0) <= MAX_FILE_LINES]
    assert back == [], f"remove from OVER_LIMIT, the limit now holds them: {back}"
