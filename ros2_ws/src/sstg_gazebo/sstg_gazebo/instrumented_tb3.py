"""Create a narrowly instrumented derivative of Nav2's released TB3 SDF."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
from typing import Mapping, Optional, Tuple, Union
from xml.etree import ElementTree


CONTACT_SENSOR_PREFIX = "sstg_evaluation_contact_"
CONTACT_TOPIC = "/evaluation/contacts"
GROUND_TRUTH_ODOM_TOPIC = "/evaluation/ground_truth_odom"
GROUND_TRUTH_TF_TOPIC = "/evaluation/ground_truth_tf"
ODOMETRY_PLUGIN_NAME = "gz::sim::systems::OdometryPublisher"
IMU_BACKPORT_COMMIT = "b9d523ad7ea0e98174627fdefeb4b1ae9b515063"
UPSTREAM_MODEL_NAME = "turtlebot3_waffle"
UPSTREAM_RELEASE_XACRO_SHA256 = (
    "133ebbe76997b98e43dbe03aea5a77dd6bd4117a3100343d5857b63cd3128a83"
)
DEFAULT_OUTPUT_DIRECTORY = (
    Path(tempfile.gettempdir()) / "sstg_gazebo" / "instrumented_models"
)


class InstrumentationError(RuntimeError):
    """Raised when the upstream model cannot be safely instrumented."""


@dataclass(frozen=True)
class InstrumentedTb3Sdf:
    """In-memory derivative plus an auditable narrow-change report."""

    xml: bytes
    upstream_sha256: str
    derivative_sha256: str
    collision_count: int
    contact_sensor_count: int
    imu_joint_backported: bool
    upstream_structure_preserved: bool


@dataclass(frozen=True)
class PreparedInstrumentedTb3:
    """Materialized derivative ready for the upstream spawn launch."""

    output_path: Path
    source_xacro_sha256: str
    instrumentation: InstrumentedTb3Sdf


def _model(root: ElementTree.Element) -> ElementTree.Element:
    models = root.findall("model")
    if root.tag != "sdf" or len(models) != 1:
        raise InstrumentationError(
            "rendered TB3 xacro must contain exactly one direct SDF model"
        )
    model = models[0]
    if model.get("name") != UPSTREAM_MODEL_NAME:
        raise InstrumentationError(
            "rendered model is not the audited upstream TurtleBot3 Waffle: "
            f"{model.get('name')!r}"
        )
    return model


def _signature(element: ElementTree.Element) -> Tuple[object, ...]:
    """Whitespace-insensitive structural signature for preservation checks."""
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(_signature(child) for child in list(element)),
    )


def _is_ground_truth_plugin(element: ElementTree.Element) -> bool:
    return (
        element.tag == "plugin"
        and element.get("name") == ODOMETRY_PLUGIN_NAME
        and element.findtext("odom_topic") == GROUND_TRUTH_ODOM_TOPIC
    )


def _remove_instrumentation(
    root: ElementTree.Element, *, remove_imu_backport: bool
) -> None:
    model = _model(root)
    for link in model.findall("link"):
        for sensor in list(link.findall("sensor")):
            if (sensor.get("name") or "").startswith(CONTACT_SENSOR_PREFIX):
                link.remove(sensor)
    for plugin in list(model.findall("plugin")):
        if _is_ground_truth_plugin(plugin):
            model.remove(plugin)
    if remove_imu_backport:
        joint = model.find("joint[@name='imu_joint']")
        if joint is not None:
            model.remove(joint)


def _validate_or_backport_imu_joint(model: ElementTree.Element) -> bool:
    if model.find("link[@name='base_link']") is None:
        raise InstrumentationError("upstream TB3 model has no base_link")
    if model.find("link[@name='imu_link']") is None:
        raise InstrumentationError("upstream TB3 model has no imu_link")

    imu_joint = model.find("joint[@name='imu_joint']")
    if imu_joint is not None:
        expected = {
            "type": "fixed",
            "parent": "base_link",
            "child": "imu_link",
        }
        actual = {
            "type": imu_joint.get("type"),
            "parent": imu_joint.findtext("parent"),
            "child": imu_joint.findtext("child"),
        }
        if actual != expected:
            raise InstrumentationError(
                f"upstream imu_joint contract changed: {actual!r}"
            )
        return False

    attached_imu = [
        joint for joint in model.findall("joint")
        if joint.findtext("child") == "imu_link"
    ]
    if attached_imu:
        raise InstrumentationError(
            "imu_link is already attached by an unexpected upstream joint"
        )

    # Exact narrow backport from ros-navigation commit b9d523a.  The apt
    # 1.0.1 collision geometry is intentionally left untouched.
    joint = ElementTree.Element(
        "joint", {"name": "imu_joint", "type": "fixed"}
    )
    ElementTree.SubElement(joint, "parent").text = "base_link"
    ElementTree.SubElement(joint, "child").text = "imu_link"
    ElementTree.SubElement(joint, "pose").text = "0.0 0 0.068 0 0 0"
    first_plugin = next(
        (
            index for index, child in enumerate(list(model))
            if child.tag == "plugin"
        ),
        len(model),
    )
    model.insert(first_plugin, joint)
    return True


def _sensor_suffix(collision_name: str) -> str:
    suffix = re.sub(r"[^A-Za-z0-9_]+", "_", collision_name).strip("_")
    return suffix or "collision"


def _add_contact_sensors(model: ElementTree.Element) -> Tuple[int, int]:
    collision_count = 0
    sensor_count = 0
    for link in model.findall("link"):
        link_name = link.get("name") or ""
        collisions = link.findall("collision")
        for index, collision in enumerate(collisions):
            collision_name = collision.get("name")
            if not link_name or not collision_name:
                raise InstrumentationError(
                    "every instrumented link and collision must be named"
                )
            collision_count += 1
            sensor_name = (
                f"{CONTACT_SENSOR_PREFIX}{index}_"
                f"{_sensor_suffix(collision_name)}"
            )
            if link.find(f"sensor[@name='{sensor_name}']") is not None:
                raise InstrumentationError(
                    f"upstream sensor conflicts with {link_name}/{sensor_name}"
                )
            sensor = ElementTree.SubElement(
                link,
                "sensor",
                {"name": sensor_name, "type": "contact"},
            )
            ElementTree.SubElement(sensor, "always_on").text = "true"
            ElementTree.SubElement(sensor, "update_rate").text = "50"
            contact = ElementTree.SubElement(sensor, "contact")
            ElementTree.SubElement(contact, "collision").text = collision_name
            ElementTree.SubElement(contact, "topic").text = CONTACT_TOPIC
            sensor_count += 1
    if collision_count == 0:
        raise InstrumentationError("upstream TB3 model has no collisions")
    return collision_count, sensor_count


def _add_ground_truth_odometry(model: ElementTree.Element) -> None:
    if any(_is_ground_truth_plugin(plugin) for plugin in model.findall("plugin")):
        raise InstrumentationError(
            "upstream model already defines SSTG ground-truth odometry"
        )
    plugin = ElementTree.SubElement(
        model,
        "plugin",
        {
            "filename": "gz-sim-odometry-publisher-system",
            "name": ODOMETRY_PLUGIN_NAME,
        },
    )
    ElementTree.SubElement(plugin, "odom_frame").text = "world"
    ElementTree.SubElement(plugin, "robot_base_frame").text = (
        "base_footprint_truth"
    )
    ElementTree.SubElement(plugin, "odom_topic").text = GROUND_TRUTH_ODOM_TOPIC
    ElementTree.SubElement(plugin, "tf_topic").text = GROUND_TRUTH_TF_TOPIC
    ElementTree.SubElement(plugin, "odom_publish_frequency").text = "30"
    ElementTree.SubElement(plugin, "dimensions").text = "2"


def instrument_rendered_tb3_sdf(
    rendered_sdf: Union[str, bytes],
) -> InstrumentedTb3Sdf:
    """Inject evaluator-only instrumentation into an already rendered TB3 SDF.

    The function fails if removing the three allowed additions (contact
    sensors, GT odometry, and an optional ``imu_joint`` backport) does not
    recover the complete original XML structure.
    """
    upstream_bytes = (
        rendered_sdf.encode("utf-8")
        if isinstance(rendered_sdf, str)
        else bytes(rendered_sdf)
    )
    try:
        root = ElementTree.fromstring(upstream_bytes)
    except ElementTree.ParseError as error:
        raise InstrumentationError(f"xacro produced invalid XML: {error}") from error
    model = _model(root)
    if any(
        (sensor.get("name") or "").startswith(CONTACT_SENSOR_PREFIX)
        for sensor in model.findall("link/sensor")
    ):
        raise InstrumentationError("upstream model already contains SSTG contacts")

    upstream_signature = _signature(root)
    imu_backported = _validate_or_backport_imu_joint(model)
    collision_count, contact_count = _add_contact_sensors(model)
    _add_ground_truth_odometry(model)

    stripped = deepcopy(root)
    _remove_instrumentation(
        stripped, remove_imu_backport=imu_backported
    )
    preserved = _signature(stripped) == upstream_signature
    if not preserved:
        raise InstrumentationError(
            "instrumentation changed the upstream model outside the allowlist"
        )

    ElementTree.indent(root, space="  ")
    derivative = ElementTree.tostring(
        root, encoding="utf-8", xml_declaration=True
    )
    return InstrumentedTb3Sdf(
        xml=derivative,
        upstream_sha256=hashlib.sha256(upstream_bytes).hexdigest(),
        derivative_sha256=hashlib.sha256(derivative).hexdigest(),
        collision_count=collision_count,
        contact_sensor_count=contact_count,
        imu_joint_backported=imu_backported,
        upstream_structure_preserved=preserved,
    )


def render_tb3_xacro(
    xacro_path: Union[str, Path],
    *,
    namespace: str = "",
    xacro_executable: Union[str, Path] = "xacro",
    environment: Optional[Mapping[str, str]] = None,
    timeout_seconds: float = 30.0,
) -> bytes:
    """Synchronously render the released TB3 SDF xacro."""
    source = Path(xacro_path).expanduser().resolve()
    if not source.is_file():
        raise InstrumentationError(f"upstream TB3 xacro not found: {source}")
    executable_text = os.fspath(xacro_executable)
    executable = (
        executable_text
        if os.path.isabs(executable_text)
        else shutil.which(executable_text)
    )
    if not executable or not Path(executable).is_file():
        raise InstrumentationError(
            f"xacro executable not found: {executable_text!r}"
        )
    command = [executable, str(source), f"namespace:={namespace}"]
    subprocess_environment = None
    if environment is not None:
        subprocess_environment = os.environ.copy()
        subprocess_environment.update(environment)
    try:
        completed = subprocess.run(
            command,
            check=True,
            capture_output=True,
            env=subprocess_environment,
            timeout=timeout_seconds,
        )
    except subprocess.CalledProcessError as error:
        stderr = error.stderr or b""
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", errors="replace")
        detail = stderr.strip() or "no stderr output"
        raise InstrumentationError(
            f"xacro exited with status {error.returncode}: {detail}"
        ) from error
    except subprocess.TimeoutExpired as error:
        raise InstrumentationError(
            f"xacro did not finish within {timeout_seconds:g} seconds"
        ) from error
    except OSError as error:
        raise InstrumentationError(
            f"failed to render upstream TB3 xacro with {command!r}: {error}"
        ) from error
    if not completed.stdout.strip():
        raise InstrumentationError("xacro produced an empty TB3 SDF")
    return completed.stdout


def prepare_instrumented_tb3_sdf(
    xacro_path: Union[str, Path],
    *,
    namespace: str = "",
    output_directory: Optional[Union[str, Path]] = None,
    xacro_executable: Union[str, Path] = "xacro",
    environment: Optional[Mapping[str, str]] = None,
    expected_xacro_sha256: Optional[str] = UPSTREAM_RELEASE_XACRO_SHA256,
) -> PreparedInstrumentedTb3:
    """Synchronously render, instrument and atomically materialize the TB3 SDF."""
    source = Path(xacro_path).expanduser().resolve()
    if not source.is_file():
        raise InstrumentationError(f"upstream TB3 xacro not found: {source}")
    source_xacro_sha256 = hashlib.sha256(source.read_bytes()).hexdigest()
    if (
        expected_xacro_sha256 is not None
        and source_xacro_sha256 != expected_xacro_sha256
    ):
        raise InstrumentationError(
            "installed TB3 xacro differs from the frozen upstream release: "
            f"expected {expected_xacro_sha256}, got {source_xacro_sha256}"
        )
    rendered = render_tb3_xacro(
        source,
        namespace=namespace,
        xacro_executable=xacro_executable,
        environment=environment,
    )
    result = instrument_rendered_tb3_sdf(rendered)
    directory = Path(
        output_directory if output_directory is not None
        else DEFAULT_OUTPUT_DIRECTORY
    ).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    output_path = directory / (
        f"tb3_waffle_{result.derivative_sha256[:16]}.sdf"
    )
    if not output_path.exists() or output_path.read_bytes() != result.xml:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=".tb3_waffle_", suffix=".sdf.tmp", dir=directory
        )
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(result.xml)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, output_path)
        except BaseException:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise
    return PreparedInstrumentedTb3(
        output_path=output_path,
        source_xacro_sha256=source_xacro_sha256,
        instrumentation=result,
    )
