"""Map the FR3 Duo reference station using its existing policy I/O contract.

Camera topic rewriting and named joint selection follow dataset-builder's
CameraSegment.dataset_topic and segment_decoder conventions. Robot commands
remain in the recording but are never substituted for measured state.
"""

import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from mcap.reader import make_reader
from mcap.records import Channel, Message, Schema
from mcap.writer import Writer
from mcap_protobuf.schema import register_schema
from mcap_ros2.decoder import DecoderFactory
from pipeline_configs import PolicyContract, PolicySegment, ROSMessageType

from annotations.signals import signal_type

# robotiq_gripper_client.py documents this joint's URDF range: 0 open, 0.8 closed.
KNUCKLE_MAX_RADIANS = 0.8
KNUCKLE_NAME = "robotiq_85_left_knuckle_joint"
CAMERA_ROLES = {"head": "front", "wrist_left": "wrist_left", "wrist_right": "wrist_right"}
STATE_UNITS = {
    "arm_position": "rad",
    "arm_velocity": "rad/s",
    "gripper_position": "rad",
    "arm_external_joint_torque": "N*m",
    "arm_external_wrench": None,  # Mixed units: force is N; torque is N*m.
}
STATE_FIELDS = {
    "arm_position": "position",
    "arm_velocity": "velocity",
    "gripper_position": "position",
    "arm_external_joint_torque": "effort",
    "arm_external_wrench": None,
}


@dataclass(frozen=True)
class SignalBinding:
    """One measured contract segment and its destination in the prepared copy."""

    segment: PolicySegment
    opening: bool = False

    @property
    def name(self) -> str:
        """Keep the measured angle distinct from normalized opening and legacy grip names."""
        if self.segment.policy_key.endswith("gripper_position"):
            suffix = "gripper_open" if self.opening else "gripper_angle"
            return self.segment.policy_key.replace("gripper_position", suffix)
        return self.segment.policy_key

    @property
    def channel(self) -> str:
        """Namespace converted measurements under /nomadic/labs/."""
        return f"/nomadic/labs/{self.name}"

    def declaration(self) -> dict[str, Any]:
        """Declare signal dimensions, components and measurement units."""
        declaration: dict[str, Any] = {"name": self.name, "channel": self.channel}
        if self.opening:
            return declaration | {
                "type": "float",
                "unit": "1",
                "note": "Measured Robotiq knuckle opening fraction: 1 - angle_rad / 0.8; not commanded width.",
            }
        suffix = self.segment.policy_key.split("_", 1)[1]
        components = list(self.segment.element_names)
        if self.segment.message_type == ROSMessageType.WRENCH_STAMPED:
            components = ["fx", "fy", "fz", "tx", "ty", "tz"]
        declaration.update(type="vector", dim=self.segment.width, components=components)
        if unit := STATE_UNITS[suffix]:
            declaration["unit"] = unit
        declaration["note"] = f"Measured {self.segment.topic}; field {self.segment.field or 'wrench'}."
        if suffix == "arm_external_wrench":
            declaration["note"] += " Force in N, torque in N*m; retained in the source stiffness frame."
        return declaration


def _values(segment: PolicySegment, message: Any) -> list[float]:  # noqa: ANN401 -- Decoded ROS payload.
    """Select by contract names, including the broadcaster's underscore namespace prefix."""
    if segment.resolves_by_name:
        names, values = list(message.name), list(getattr(message, str(segment.field)))
        if len(names) != len(values):
            raise ValueError(f"{segment.policy_key}: joint names and {segment.field} lengths differ.")
        result = []
        for name in segment.element_names:
            matches = [i for i, published in enumerate(names) if published == name or published.endswith(f"_{name}")]
            if len(matches) != 1:
                raise ValueError(f"{segment.policy_key}: expected exactly one recorded joint matching {name}.")
            result.append(float(values[matches[0]]))
    else:
        result = []
        for path in segment.element_paths:
            value = message
            for attribute in path.split("."):
                value = getattr(value, attribute)
            result.append(float(value))
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{segment.policy_key}: a measurement is not finite.")
    return result


class ReferenceStationMapping:
    """Convert explicitly declared FR3 Duo measurements, retaining their original clocks."""

    def __init__(self, contract_path: Path, source_topics: set[str]) -> None:
        """Read the existing reference contract; reject incompatible layouts before writing."""
        self.contract = PolicyContract.from_yaml(contract_path)
        self.bindings = self._bindings()
        cameras = {camera.policy_key: camera for camera in self.contract.cameras}
        if set(cameras) != set(CAMERA_ROLES):
            raise ValueError("The FR3 Duo mapping requires head, wrist_left and wrist_right contract cameras.")
        self.spec: dict[str, Any] = {
            "spec_version": "1.1",
            "recording": {"platform": {"kind": "manipulator", "embodiment": "franka_fr3_duo", "parts": self._parts()}},
            "primary_view": cameras["head"].dataset_topic,
            "views": [
                {"channel": camera.dataset_topic, "role": CAMERA_ROLES[camera.policy_key]}
                for camera in self.contract.cameras
            ],
            "signals": [binding.declaration() for binding in self.bindings],
        }
        required = {camera.dataset_topic for camera in self.contract.cameras}
        required.update(binding.segment.topic for binding in self.bindings)
        if missing := required - source_topics:
            raise ValueError(
                f"Recording does not match the configured LABS contract; missing channels: {', '.join(sorted(missing))}"
            )
        if source_topics & {binding.channel for binding in self.bindings}:
            raise ValueError("The recording already contains reference signal channels; use its embedded Nomadic spec.")
        self._by_topic: dict[str, list[SignalBinding]] = defaultdict(list)
        for binding in self.bindings:
            self._by_topic[binding.segment.topic].append(binding)
        self._decoder = DecoderFactory()
        self._signal: Any = signal_type()
        self._channels: dict[str, int] = {}
        self._counts: dict[str, int] = defaultdict(int)
        self._times: dict[str, int] = {}
        self._schema_id: int | None = None

    def _bindings(self) -> list[SignalBinding]:
        by_key = {segment.policy_key: segment for segment in self.contract.state}
        expected = {f"{side}_{suffix}" for side in ("left", "right") for suffix in STATE_UNITS}
        if set(by_key) != expected:
            raise ValueError("The configured contract is not the supported FR3 Duo reference state layout.")
        bindings = []
        for segment in self.contract.state:
            suffix = segment.policy_key.split("_", 1)[1]
            is_wrench = segment.policy_key.endswith("external_wrench")
            expected_type = ROSMessageType.WRENCH_STAMPED if is_wrench else ROSMessageType.JOINT_STATE
            if segment.message_type != expected_type:
                raise ValueError(f"{segment.policy_key}: unexpected ROS message type for the FR3 Duo mapping.")
            if segment.field != STATE_FIELDS[suffix]:
                raise ValueError(f"{segment.policy_key}: expected measured field {STATE_FIELDS[suffix]}.")
            if suffix in {
                "arm_position",
                "arm_velocity",
                "arm_external_joint_torque",
            } and segment.element_names != tuple(f"fr3_joint{i}" for i in range(1, 8)):
                raise ValueError(f"{segment.policy_key}: the reference arm requires its seven named FR3 joints.")
            if is_wrench and segment.element_names != (
                "force.x",
                "force.y",
                "force.z",
                "torque.x",
                "torque.y",
                "torque.z",
            ):
                raise ValueError(f"{segment.policy_key}: the full reference wrench is required.")
            if "gripper" in segment.policy_key and segment.element_names != (KNUCKLE_NAME,):
                raise ValueError("The reference gripper mapping requires the Robotiq 2F-85 knuckle joint.")
            bindings.append(SignalBinding(segment))
            if "gripper" in segment.policy_key:
                bindings.append(SignalBinding(segment, opening=True))
        return bindings

    def _parts(self) -> list[dict[str, Any]]:
        by_key = {segment.policy_key: segment for segment in self.contract.state}
        return [
            {
                "name": f"{side}_arm",
                "type": "arm",
                "side": side,
                "joints": {
                    "signal": f"{side}_arm_position",
                    "names": list(by_key[f"{side}_arm_position"].element_names),
                },
                "grip": {"signal": f"{side}_gripper_open"},
                "force": {"signal": f"{side}_arm_external_wrench"},
            }
            for side in ("left", "right")
        ]

    def copy_signals(self, writer: Writer, schema: Schema | None, channel: Channel, message: Message) -> None:
        """Add converted measurements alongside the unchanged ROS messages."""
        bindings = self._by_topic.get(channel.topic)
        if not bindings:
            return
        expected = bindings[0].segment.message_type.value.replace("/", "/msg/", 1)
        if schema is None or schema.name != expected or channel.message_encoding != "cdr":
            raise ValueError(f"{channel.topic}: expected {expected} with ROS 2 CDR encoding.")
        if message.log_time <= self._times.get(channel.topic, -1):
            raise ValueError(f"{channel.topic}: measured timestamps must be strictly increasing.")
        self._times[channel.topic] = message.log_time
        decode = self._decoder.decoder_for(channel.message_encoding, schema)
        if decode is None:
            raise ValueError(f"{channel.topic}: the recorded ROS schema cannot be decoded.")
        decoded = decode(message.data)
        if self._schema_id is None:
            self._schema_id = register_schema(writer, self._signal)
        for binding in bindings:
            values = _values(binding.segment, decoded)
            signal = self._signal()
            signal.timestamp.seconds, signal.timestamp.nanos = divmod(message.log_time, 1_000_000_000)
            if binding.opening:
                if not 0 <= values[0] <= KNUCKLE_MAX_RADIANS:
                    raise ValueError(
                        f"{channel.topic}: measured angle is outside the reference Robotiq 0-0.8 rad range."
                    )
                signal.number = 1 - values[0] / KNUCKLE_MAX_RADIANS
            else:
                signal.vector.values.extend(values)
            if binding.name not in self._channels:
                self._channels[binding.name] = writer.register_channel(binding.channel, "protobuf", self._schema_id)
            writer.add_message(
                self._channels[binding.name],
                message.log_time,
                signal.SerializeToString(),
                publish_time=message.publish_time,
                sequence=message.sequence,
            )
            self._counts[binding.name] += 1

    def validate_complete(self) -> None:
        """Require samples for every declared measurement."""
        if missing := [binding.name for binding in self.bindings if not self._counts[binding.name]]:
            raise ValueError(f"The recording has no samples for declared measurements: {', '.join(missing)}")

    def write_signals(self, writer: Writer, source: Path) -> None:
        """Read only mapped ROS channels and append measured signals in timestamp order."""
        with source.open("rb") as handle:
            for schema, channel, message in make_reader(handle).iter_messages(topics=sorted(self._by_topic)):
                self.copy_signals(writer, schema, channel, message)
        self.validate_complete()
