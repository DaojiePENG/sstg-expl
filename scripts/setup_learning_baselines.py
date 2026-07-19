#!/usr/bin/env python3
"""Install/download assets required by optional learning-based baselines."""
from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT = ROOT / "models" / "checkpoints" / "ans_global_policy.pt"
GOOGLE_DRIVE_ID = "1UK2hT0GWzoTaVR5lAI6i8o27tqEmYeyY"
EXPECTED_SHA256 = "616fd1485e1f0ba9673db08340d586c050f001f171890d966809c0b9f0320314"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--install-dependencies", action="store_true",
        help="Install the project's optional [learning] dependencies first.",
    )
    args = parser.parse_args()
    if args.install_dependencies:
        subprocess.run(
            [sys.executable, "-m", "pip", "install", "torch>=2.0",
             "--index-url", "https://download.pytorch.org/whl/cpu"],
            check=True,
        )
        subprocess.run(
            [sys.executable, "-m", "pip", "install", "gdown>=5.0"],
            check=True,
        )
    try:
        import gdown
        import torch
    except ImportError as exc:
        raise SystemExit(
            "Missing learning dependencies. Re-run with --install-dependencies."
        ) from exc
    CHECKPOINT.parent.mkdir(parents=True, exist_ok=True)
    if not CHECKPOINT.exists() or sha256(CHECKPOINT) != EXPECTED_SHA256:
        gdown.download(id=GOOGLE_DRIVE_ID, output=str(CHECKPOINT), quiet=False)
    actual = sha256(CHECKPOINT)
    if actual != EXPECTED_SHA256:
        raise SystemExit(f"Checkpoint checksum mismatch: {actual}")
    print(f"ANS global checkpoint ready: {CHECKPOINT}")
    print(f"sha256: {actual}")
    print(f"torch: {torch.__version__}")


if __name__ == "__main__":
    main()
