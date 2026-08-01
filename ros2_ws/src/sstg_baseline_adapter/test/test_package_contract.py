from pathlib import Path
import py_compile

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_launch_file_compiles():
    py_compile.compile(
        str(PACKAGE_ROOT / "launch" / "frontier_mrtsp_dp.launch.py"),
        doraise=True,
    )


def test_integration_overrides_preserve_upstream_algorithm_parameters():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "frontier_mrtsp_dp.yaml").read_text(
            encoding="utf-8"
        )
    )
    upstream = config["frontier_explorer"]["ros__parameters"]
    assert upstream == {
        "use_sim_time": True,
        "autostart": False,
        "control_service_enabled": True,
        "navigate_to_pose_action_name": (
            "/baseline/frontier_mrtsp_dp/navigate_to_pose"
        ),
        "completion_event_enabled": True,
        "completion_event_topic": (
            "/baseline/frontier_mrtsp_dp/exploration_complete"
        ),
        "return_to_start_on_complete": False,
    }


def test_proxy_and_upstream_action_names_are_distinct_and_cross_wired():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "frontier_mrtsp_dp.yaml").read_text(
            encoding="utf-8"
        )
    )
    upstream = config["frontier_explorer"]["ros__parameters"]
    adapter = config["frontier_baseline_adapter"]["ros__parameters"]
    assert upstream["navigate_to_pose_action_name"] == adapter["proxy_action_name"]
    assert adapter["proxy_action_name"] != adapter["nav2_action_name"]
    assert upstream["completion_event_topic"] == adapter["completion_topic"]
    assert adapter["cancel_grace_s"] == 5.0


def test_package_declares_pinned_upstream_runtime_dependency():
    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    assert "<depend>frontier_exploration_ros2</depend>" in package_xml
    source = (
        PACKAGE_ROOT
        / "sstg_baseline_adapter"
        / "frontier_action_adapter.py"
    ).read_text(encoding="utf-8")
    assert "b0fad500e5c81ad3154f0469ca283b2702a3f90c" in source
    assert '"policy_seed_applicable": False' in source
