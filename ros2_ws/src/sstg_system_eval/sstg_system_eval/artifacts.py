"""Fail-closed artifact directory primitives for system-simulation runs."""
from __future__ import annotations

from pathlib import Path
from typing import Iterable


def prepare_output_directory(
    output_dir: Path | str,
    allow_existing_output: bool = False,
    owned_artifact_names: Iterable[str] = (),
) -> Path:
    """Prepare a shared run directory and reject evaluator-owned file reuse.

    The schedule runner atomically reserves the run directory before ROS starts
    and writes its own launch manifest there.  A fresh evaluator must therefore
    allow that directory while still refusing to append to any artifact that it
    owns from an earlier process.
    """
    if not isinstance(allow_existing_output, bool):
        raise TypeError("allow_existing_output must be boolean")
    path = Path(output_dir).expanduser()
    names = tuple(str(name).strip() for name in owned_artifact_names)
    if any(
        not name
        or Path(name).name != name
        or name in {".", ".."}
        for name in names
    ):
        raise ValueError("owned artifact names must be non-empty basenames")
    path.mkdir(parents=True, exist_ok=True)
    if not allow_existing_output:
        existing = [path / name for name in names if (path / name).exists()]
        if existing:
            raise FileExistsError(
                "refusing to reuse evaluator output artifacts: "
                + ", ".join(item.name for item in existing)
            )
    return path
