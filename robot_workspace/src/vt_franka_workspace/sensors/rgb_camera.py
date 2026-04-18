from __future__ import annotations

from dataclasses import dataclass

from ..settings import RgbCameraSettings


@dataclass(frozen=True)
class RgbCameraSpec:
    role: str
    stream_name: str
    settings: RgbCameraSettings


def resolve_rgb_camera_specs(
    rgb_cameras: dict[str, RgbCameraSettings],
) -> list[RgbCameraSpec]:
    specs: list[RgbCameraSpec] = []
    for role, settings in rgb_cameras.items():
        if not settings.enabled:
            continue
        stream_name = settings.stream_name or f"rgb_{role}"
        camera_name = settings.camera_name or stream_name
        specs.append(
            RgbCameraSpec(
                role=role,
                stream_name=stream_name,
                settings=settings.model_copy(update={"stream_name": stream_name, "camera_name": camera_name}),
            )
        )
    return specs


def build_rgb_camera_recorder(
    spec: RgbCameraSpec,
    *,
    recorder=None,
    live_buffer=None,
    quest_publisher=None,
    image_format: str = "jpg",
):
    if spec.settings.backend == "orbbec":
        from .orbbec import OrbbecRgbRecorder

        return OrbbecRgbRecorder(
            spec.settings,
            recorder=recorder,
            live_buffer=live_buffer,
            quest_publisher=quest_publisher,
            image_format=image_format,
        )
    raise RuntimeError(f"Unsupported RGB camera backend: {spec.settings.backend}")
