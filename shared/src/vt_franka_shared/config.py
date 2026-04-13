from __future__ import annotations

from pathlib import Path
from typing import Type, TypeVar, Union

import yaml
from pydantic import BaseModel

T = TypeVar("T", bound=BaseModel)


def load_yaml_model(path: Union[str, Path], model_type: Type[T]) -> T:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return model_type.model_validate(data)


def dump_yaml_model(path: Union[str, Path], model: BaseModel) -> None:
    config_path = Path(path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    with config_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(model.model_dump(mode="json"), handle, sort_keys=False)
