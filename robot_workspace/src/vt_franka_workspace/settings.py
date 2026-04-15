from __future__ import annotations

from pathlib import Path

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
    force_udp_port: int = 10005
    state_publish_hz: float = 60.0
    force_scale_factor: float = 0.025
    record_hz: float = 0.0


class GelsightSettings(BaseModel):
    enabled: bool = True
    camera_name: str = "left_gripper_camera_1"
    camera_index: int = 0
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


class OrbbecSettings(BaseModel):
    enabled: bool = False
    camera_name: str = "workspace_orbbec_rgb"
    serial_number: str = ""
    color_width: int = 640
    color_height: int = 0
    color_format: str = "RGB"
    color_fps: int = 30
    frame_timeout_ms: int = 200
    save_frames: bool = True
    record_hz: float = 0.0


class RecordingSettings(BaseModel):
    enabled: bool = True
    root_dir: Path = Path("./data/episodes")
    run_root: Path = Path("./data/runs")
    image_format: str = "jpg"
    postprocess_target_hz: float = 10.0


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


class CalibrationSettings(BaseModel):
    calibration_dir: Path = Path("config/calibration/v6")


class RolloutSettings(BaseModel):
    control_hz: float = 12.0
    max_duration_sec: float = 30.0


class WorkspaceSettings(BaseModel):
    controller: ControllerClientSettings = Field(default_factory=ControllerClientSettings)
    teleop: TeleopSettings = Field(default_factory=TeleopSettings)
    quest_feedback: QuestFeedbackSettings = Field(default_factory=QuestFeedbackSettings)
    gelsight: GelsightSettings = Field(default_factory=GelsightSettings)
    orbbec: OrbbecSettings = Field(default_factory=OrbbecSettings)
    recording: RecordingSettings = Field(default_factory=RecordingSettings)
    collect: CollectSettings = Field(default_factory=CollectSettings)
    calibration: CalibrationSettings = Field(default_factory=CalibrationSettings)
    rollout: RolloutSettings = Field(default_factory=RolloutSettings)
