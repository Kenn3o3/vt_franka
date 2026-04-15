from .control.service import ControllerService
from .settings import ControllerSettings


def create_app(*args, **kwargs):
    from .api.app import create_app as _create_app

    return _create_app(*args, **kwargs)


__all__ = ["ControllerService", "ControllerSettings", "create_app"]
