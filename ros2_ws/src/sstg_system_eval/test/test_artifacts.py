from pathlib import Path

import pytest

from sstg_system_eval.artifacts import prepare_output_directory


OWNED = ("evaluation_manifest.json", "evaluation_metrics.jsonl")


def test_reserved_shared_directory_is_allowed_but_owned_reuse_is_not(tmp_path):
    output = tmp_path / "run_001"

    assert prepare_output_directory(output, owned_artifact_names=OWNED) == output
    assert output.is_dir()
    reservation = output / "run_launch_manifest.yaml"
    reservation.write_text("status: reserved\n", encoding="utf-8")
    assert prepare_output_directory(output, owned_artifact_names=OWNED) == output

    (output / OWNED[0]).write_text("{}\n", encoding="utf-8")
    with pytest.raises(FileExistsError):
        prepare_output_directory(output, owned_artifact_names=OWNED)


def test_output_directory_reuse_requires_explicit_boolean_opt_in(tmp_path):
    output = tmp_path / "run_001"
    output.mkdir()
    marker = output / "existing.txt"
    marker.write_text("preserve\n", encoding="utf-8")

    assert prepare_output_directory(output, True, OWNED) == Path(output)
    assert marker.read_text(encoding="utf-8") == "preserve\n"
    with pytest.raises(TypeError, match="must be boolean"):
        prepare_output_directory(output, "true")


def test_owned_artifact_names_must_be_safe_basenames(tmp_path):
    with pytest.raises(ValueError, match="basenames"):
        prepare_output_directory(tmp_path / "run", False, ("../escape",))
