from __future__ import annotations

from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field


class ControllerClientSettings(BaseModel):
    host: str = "127.0.0.1"
    port: int = 8092
    request_timeout_sec: float = 1.0


class TeleopSettings(BaseModel):
    host: str = "0.0.0.0"
    port: int = 8082
    loop_hz: float = 60.0
    tracking_button_index: int = 4
    trigger_close_threshold: float = 0.5
    relative_translation_scale: float = 0.8
    max_tracking_position_error_m: float = 0.3
    use_force_control_for_gripper: bool = True
    max_gripper_width: float = 0.078
    min_gripper_width: float = 0.0
    grasp_force: float = 7.0
    gripper_velocity: float = 0.1
    gripper_stability_window: int = 30
    gripper_force_close_threshold: float = 15.0
    gripper_force_open_threshold: float = 5.0
    gripper_width_vis_precision: float = 0.001
    command_record_hz: float = 0.0
    quest_message_record_hz: float = 0.0


class QuestFeedbackSettings(BaseModel):
    quest_ip: str = "127.0.0.1"
    robot_state_udp_port: int = 10001
    tactile_udp_port: int = 10002
    image_udp_port: int = 10004
    force_udp_port: int = 10005
    state_publish_hz: float = 60.0
    force_scale_factor: float = 0.025
    record_hz: float = 0.0


class QuestImageStreamSettings(BaseModel):
    enabled: bool = False
    image_id: str = ""
    in_head_space: bool = False
    left_or_right: bool = False
    position: list[float] = Field(default_factory=lambda: [0.0, 0.4, 0.5])
    rotation: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    scale: list[float] = Field(default_factory=lambda: [0.002, 0.0015, 0.001])
    max_width: int = 320
    max_height: int = 240
    quality: int = 30
    chunk_size: int = 1024
    max_publish_hz: float = 12.0


class GelsightSettings(BaseModel):
    enabled: bool = True
    camera_name: str = "left_gripper_camera_1"
    camera_index: int = 0
    camera_path: str = ""
    device_name_contains: str = "GelSight Mini"
    device_serial_number: str = ""
    fps: int = 30
    width: int = 640
    height: int = 480
    exposure: int = -6
    contrast: int = 100
    marker_dimension: int = 2
    marker_vis_rotation_deg: float = 0.0
    vis_latency_steps: int = 5
    teleop_status_host: str = "127.0.0.1"
    teleop_status_port: int = 8082
    save_frames: bool = True
    record_hz: float = 0.0
    quest_stream: QuestImageStreamSettings = Field(default_factory=QuestImageStreamSettings)
    quest_overlay_arrow_scale: float = 12.0


class RgbCameraSettings(BaseModel):
    enabled: bool = True
    backend: Literal["orbbec"] = "orbbec"
    stream_name: str = ""
    camera_name: str = ""
    serial_number: str = ""
    color_width: int = 640
    color_height: int = 0
    color_format: str = "RGB"
    color_fps: int = 30
    frame_timeout_ms: int = 200
    save_frames: bool = True
    record_hz: float = 0.0
    quest_stream: QuestImageStreamSettings = Field(default_factory=QuestImageStreamSettings)


class RecordingSettings(BaseModel):
    enabled: bool = True
    run_root: Path = Path("./data/runs")
    image_format: str = "jpg"
    postprocess_target_hz: float = 10.0


class OperatorUiSettings(BaseModel):
    enabled: bool = True
    host: str = "127.0.0.1"
    port: int = 8083
    log_buffer_size: int = 1000
    snapshot_max_age_sec: float = 1.5


class CollectSettings(BaseModel):
    sync_hz: float = 10.0
    controller_state_poll_hz: float = 60.0
    controller_state_max_age_sec: float = 2.0
    quest_message_timeout_sec: float = 2.0
    require_quest_connection: bool = True
    start_countdown_sec: float = 2.0
    auto_postprocess: bool = True
    auto_qc: bool = True
    controller_gap_warn_sec: float = 0.5
    controller_gap_fail_sec: float = 2.0
    status_print_hz: float = 1.0
    record_raw_quest_messages: bool = False


class AutoCollectSegmentDurationsSettings(BaseModel):
    approach: float = 3.0
    descend: float = 3.0
    lift: float = 2.0
    transfer: float = 3.0
    place_descend: float = 3.0
    retreat: float = 2.0


class AutoCollectSettings(BaseModel):
    command_hz: float = 15.0
    controller_state_poll_hz: float = 15.0
    init_pose_xyz_rpy_deg: list[float] = Field(default_factory=lambda: [0.03, 0.4, 0.4, -180.0, 0.0, 45.0])
    hover_z_m: float = 0.35
    lift_z_m: float = 0.24
    table_z_m: float = 0.14
    grasp_force: float = 5.0
    pose_tolerance_m: float = 0.015
    yaw_tolerance_deg: float = 10.0
    pose_settle_timeout_sec: float = 8.0
    settle_dwell_sec: float = 0.3
    post_grasp_dwell_sec: float = 0.5
    post_release_dwell_sec: float = 0.5
    gripper_close_wait_sec: float = 1.8
    gripper_settle_timeout_sec: float = 3.0
    gripper_open_tolerance_m: float = 0.015
    auto_continue: bool = False
    segment_durations_sec: AutoCollectSegmentDurationsSettings = Field(default_factory=AutoCollectSegmentDurationsSettings)


class CalibrationSettings(BaseModel):
    calibration_dir: Path = Path("config/calibration/v6")


class RolloutPolicyInputSettings(BaseModel):
    controller_state: bool = True
    rgb_cameras: list[str] = Field(default_factory=list)
    gelsight_markers: bool = False
    gelsight_frame: bool = False
    controller_state_max_age_sec: float = 2.0
    rgb_camera_max_age_sec: float = 2.0
    gelsight_max_age_sec: float = 2.0


class RolloutPolicySettings(BaseModel):
    entrypoint: str = ""
    kwargs: dict = Field(default_factory=dict)
    inputs: RolloutPolicyInputSettings = Field(default_factory=RolloutPolicyInputSettings)


class RolloutSettings(BaseModel):
    control_hz: float = 12.0
    max_duration_sec: float = 30.0
    start_countdown_sec: float = 2.0
    status_print_hz: float = 1.0
    policy: RolloutPolicySettings = Field(default_factory=RolloutPolicySettings)


class WorkspaceSettings(BaseModel):
    controller: ControllerClientSettings = Field(default_factory=ControllerClientSettings)
    teleop: TeleopSettings = Field(default_factory=TeleopSettings)
    quest_feedback: QuestFeedbackSettings = Field(default_factory=QuestFeedbackSettings)
    gelsight: GelsightSettings = Field(default_factory=GelsightSettings)
    rgb_cameras: dict[str, RgbCameraSettings] = Field(default_factory=dict)
    recording: RecordingSettings = Field(default_factory=RecordingSettings)
    operator_ui: OperatorUiSettings = Field(default_factory=OperatorUiSettings)
    collect: CollectSettings = Field(default_factory=CollectSettings)
    auto_collect: AutoCollectSettings = Field(default_factory=AutoCollectSettings)
    calibration: CalibrationSettings = Field(default_factory=CalibrationSettings)
    rollout: RolloutSettings = Field(default_factory=RolloutSettings)
