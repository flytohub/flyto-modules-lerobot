"""Read one frame from the arm's camera and say how each configured colour class matches it.

Run on the execution host with the Flyto2 host stopped (the camera is one
process's). Put the objects the arm will handle in view first. It prints each
class's matched pixels against the area it needs, and the hues of the
saturated pixels, whose peaks are where the classes' ``hue_deg`` belong
(``flyto_modules_lerobot.colour_sample``). It never edits the arm file.

Usage: python scripts/sample_colours.py --config ARM.json [--save frame.png]
       python scripts/sample_colours.py --config ARM.json --image frame.png
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flyto_modules_lerobot.arm_config import parse_arm_config  # noqa: E402
from flyto_modules_lerobot.colour_sample import sample, sample_lines  # noqa: E402
from flyto_modules_lerobot.perception import ArrayFrame, opencv_frames  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", required=True, type=Path, help="the arm file (JSON)")
    parser.add_argument("--image", type=Path, help="read this saved frame instead of the camera")
    parser.add_argument("--save", type=Path, help="also save the camera's frame here (PNG)")
    args = parser.parse_args(argv)
    config = parse_arm_config(json.loads(args.config.read_text(encoding="utf-8")), config_dir=args.config.parent)
    camera = config.perception.camera if config.perception is not None else None
    if camera is None or not hasattr(camera, "classes"):
        print("this arm file declares no camera with colour classes")
        return 1
    if args.image is not None:
        import numpy
        from PIL import Image

        frame = ArrayFrame(numpy.asarray(Image.open(args.image).convert("RGB")))
    else:
        frame = opencv_frames(camera)()
        if args.save is not None:
            import cv2

            cv2.imwrite(str(args.save), frame.image)
            print(f"saved {args.save}")
    print("\n".join(sample_lines(sample(frame, camera.classes))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
